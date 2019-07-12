/* drivers/input/misc/vcnl36863.c - vcnl36863 optical sensors driver
 *
 * Copyright (C) 2018 Vishay Capella Microsystems Limited
 * Author: Frank Hsieh <Frank.Hsieh@vishay.com>
 *                                    
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/lightsensor.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
//#include <asm/mach-types.h>
#include <linux/regulator/consumer.h>
#include <linux/vcnl36863.h>
#include <linux/capella_cm3602.h>
#include <linux/fs.h>
//#include <asm/setup.h>
#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#endif
#include <linux/jiffies.h>

#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif

#define D(x...) pr_info(x)

#define I2C_RETRY_COUNT 10

#define LS_POLLING_DELAY 600

#define REL_RED		  REL_X
#define REL_GREEN	  REL_Y
#define REL_BLUE	  REL_Z
#define REL_IR	    REL_WHEEL
#define REL_ALS	    REL_MISC

#define NEAR_DELAY_TIME ((100 * HZ) / 1000)
#define LIGHTSENSOR_ENABLE_DELAYTIME_IN_MS 100 // assume CS IT = 100ms

#define CONTROL_INT_ISR_REPORT        0x00
//#define CONTROL_ALS                   0x01
#define CONTROL_PS                    0x02
//#define CONTROL_CS_REPORT            0x03

static void sensor_irq_do_work(struct work_struct *work);
static DECLARE_WORK(sensor_irq_work, sensor_irq_do_work);

//static void report_do_work(struct work_struct *w);
//static DECLARE_DELAYED_WORK(report_work, report_do_work);

struct vcnl36863_info {
	struct class *vcnl36863_class;
	struct device *ls_dev;
	struct device *ps_dev;

	struct input_dev *ls_input_dev;
	struct input_dev *ps_input_dev;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	struct i2c_client *i2c_client;
	struct workqueue_struct *lp_wq;
  
  struct workqueue_struct *lp_als_wq;

	int intr_pin;
	int als_enable;
	int ps_enable;
 	struct mutex power_lock; //add
 	struct work_struct work_light; //add

  	struct hrtimer light_timer; //add
  	ktime_t light_poll_delay; //add    

#ifdef CONFIG_PM_SLEEP
	int als_enabled_before_suspend;
#endif

	int irq;
	
	int (*power)(int, uint8_t); /* power to the chip */

#ifdef CONFIG_HAS_WAKELOCK
	struct wake_lock ps_wake_lock;
#endif  
	int psensor_opened;
	int lightsensor_opened;
	uint8_t slave_addr;  

	uint16_t ps_close_thd_set;
	uint16_t ps_away_thd_set;	
	//uint32_t current_lux;
	//uint16_t current_adc;
	uint16_t inte_cancel_set;
	uint16_t ps_conf1_val;
	uint16_t ps_conf3_val;

	uint16_t ls_cmd;
        struct regulator *vcc_l8c_1p8;
        struct regulator *vcc_l7c_3p0;

        int enable_gpio;
};
struct vcnl36863_info *lp_info;
//static struct mutex als_enable_mutex, als_disable_mutex, als_get_adc_mutex;
//static struct mutex ps_enable_mutex, ps_disable_mutex, ps_get_adc_mutex;
static struct mutex VCNL36863_control_mutex;
static int lightsensor_enable(struct vcnl36863_info *lpi);
static int lightsensor_disable(struct vcnl36863_info *lpi);
static int initial_vcnl36863_gpio(struct vcnl36863_info *lpi);
static void psensor_initial_cmd(struct vcnl36863_info *lpi);

static int control_and_report(struct vcnl36863_info *lpi, uint8_t mode, uint16_t param);

static uint16_t vcnl36863_adc_red, vcnl36863_adc_green, vcnl36863_adc_blue, vcnl36863_adc_ir, vcnl36863_als;
static int als_calibration, ps_inf, ps_1cm, ps_2cm, ps_4cm;

static int I2C_RxData(uint16_t slaveAddr, uint8_t cmd, uint8_t *rxData, int length)
{
	uint8_t loop_i;
	int val;
	struct vcnl36863_info *lpi = lp_info;
		
	struct i2c_msg msgs[] = {
		{
		 .addr = slaveAddr,
		 .flags = 0,
		 .len = 1,
		 .buf = &cmd,
		},
		{
		 .addr = slaveAddr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxData,
		},		 
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {

		if (i2c_transfer(lp_info->i2c_client->adapter, msgs, 2) > 0)
			break;

		val = gpio_get_value(lpi->intr_pin);
		/*check intr GPIO when i2c error*/
		if (loop_i == 0 || loop_i == I2C_RETRY_COUNT -1)
			D("[PS][VCNL36863 error] %s, i2c err, slaveAddr 0x%x ISR gpio %d  = %d \n",
				__func__, slaveAddr, lpi->intr_pin, val);

		msleep(10);
	}
	if (loop_i >= I2C_RETRY_COUNT) {
		printk(KERN_ERR "[PS_ERR][VCNL36863 error] %s retry over %d\n",
			__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int I2C_TxData(uint16_t slaveAddr, uint8_t *txData, int length)
{
	uint8_t loop_i;
	int val;
	struct vcnl36863_info *lpi = lp_info;
	struct i2c_msg msg[] = {
		{
		 .addr = slaveAddr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		},
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(lp_info->i2c_client->adapter, msg, 1) > 0)
			break;

		val = gpio_get_value(lpi->intr_pin);
		/*check intr GPIO when i2c error*/
		if (loop_i == 0 || loop_i == I2C_RETRY_COUNT -1)
			D("[PS][VCNL36863 error] %s, i2c err, slaveAddr 0x%x, value 0x%x, ISR gpio%d  = %d \n",
				__func__, slaveAddr, txData[0], lpi->intr_pin, val);

		msleep(10);
	}

	if (loop_i >= I2C_RETRY_COUNT) {
		printk(KERN_ERR "[ALS+PS_ERR][VCNL36863 error] %s retry over %d\n",
			__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int _vcnl36863_I2C_Read_Word(uint16_t slaveAddr, uint8_t cmd, uint16_t *pdata)
{
	uint8_t buffer[2];
	int ret = 0;

	if (pdata == NULL)
		return -EFAULT;

	ret = I2C_RxData(slaveAddr, cmd, buffer, 2);
	if (ret < 0) {
		pr_err(
			"[ALS+PS_ERR][VCNL36863 error]%s: I2C_RxData fail [0x%x, 0x%x]\n",
			__func__, slaveAddr, cmd);
		return ret;
	}

	*pdata = (buffer[1]<<8)|buffer[0];
#if 0
	/* Debug use */
	printk(KERN_DEBUG "[VCNL36863] %s: I2C_RxData[0x%x, 0x%x] = 0x%x\n",
		__func__, slaveAddr, cmd, *pdata);
#endif
	return ret;
}

static int _vcnl36863_I2C_Write_Word(uint16_t SlaveAddress, uint8_t cmd, uint16_t data)
{
	char buffer[3];
	int ret = 0;
#if 0
	/* Debug use */
	printk(KERN_DEBUG
	"[VCNL36863] %s: _vcnl36863_I2C_Write_Word[0x%x, 0x%x, 0x%x]\n",
		__func__, SlaveAddress, cmd, data);
#endif
	buffer[0] = cmd;
	buffer[1] = (uint8_t)(data&0xff);
	buffer[2] = (uint8_t)((data&0xff00)>>8);	
	
	ret = I2C_TxData(SlaveAddress, buffer, 3);
	if (ret < 0) {
		pr_err("[ALS+PS_ERR][VCNL36863 error]%s: I2C_TxData fail\n", __func__);
		return -EIO;
	}

	return ret;
}

static int get_ps_adc_value(uint16_t *data)
{
	int ret = 0;
	struct vcnl36863_info *lpi = lp_info;

	if (data == NULL)
		return -EFAULT;	

	ret = _vcnl36863_I2C_Read_Word(lpi->slave_addr, PS_DATA, data);
	
		
	if (ret < 0) {
		pr_err(
			"[PS][VCNL36863 error]%s: _vcnl36863_I2C_Read_Word fail\n",
			__func__);
		return -EIO;
	} else {
		pr_err(
			"[PS][VCNL36863 OK]%s: _vcnl36863_I2C_Read_Word OK 0x%04x\n",
			__func__, *data);
	}

	return ret;
}

static uint16_t mid_value(uint16_t value[], uint8_t size)
{
	int i = 0, j = 0;
	uint16_t temp = 0;

	if (size < 3)
		return 0;

	for (i = 0; i < (size - 1); i++)
		for (j = (i + 1); j < size; j++)
			if (value[i] > value[j]) {
				temp = value[i];
				value[i] = value[j];
				value[j] = temp;
			}
	return value[((size - 1) / 2)];
}

static int get_stable_ps_adc_value(uint16_t *ps_adc)
{
	uint16_t value[3] = {0, 0, 0}, mid_val = 0;
	int ret = 0;
	int i = 0;
	int wait_count = 0;
	struct vcnl36863_info *lpi = lp_info;

	for (i = 0; i < 3; i++) {
		/*wait interrupt GPIO high*/
		while (gpio_get_value(lpi->intr_pin) == 0) {
			msleep(10);
			wait_count++;
			if (wait_count > 12) {
				pr_err("[PS_ERR][VCNL36863 error]%s: interrupt GPIO low,"
					" get_ps_adc_value\n", __func__);
				return -EIO;
			}
		}

		ret = get_ps_adc_value(&value[i]);
		if (ret < 0) {
			pr_err("[PS_ERR][VCNL36863 error]%s: get_ps_adc_value\n",
				__func__);
			return -EIO;
		}

		if (wait_count < 60/10) {/*wait gpio less than 60ms*/
			msleep(60 - (10*wait_count));
		}
		wait_count = 0;
	}

	/*D("Sta_ps: Before sort, value[0, 1, 2] = [0x%x, 0x%x, 0x%x]",
		value[0], value[1], value[2]);*/
	mid_val = mid_value(value, 3);
	D("Sta_ps: After sort, value[0, 1, 2] = [0x%x, 0x%x, 0x%x]",
		value[0], value[1], value[2]);
	
	return 0;
}

static int als_load_calibration(void) {
        struct file *fp = NULL;
        int value;
        int read_length = 0;
        char buf[16];
        loff_t pos = 0;
        mm_segment_t old_fs;

        fp = filp_open(ALS_CALIBRATION_50MS, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
        if (IS_ERR_OR_NULL(fp)) {
            D("[LS][VCNL36863] %s Lightsensor calibration open fail\n", __func__);
            return -ENOENT;
        }

        old_fs = get_fs();
        set_fs(KERNEL_DS);

        if (fp->f_op != NULL) {
            pos = 0;
            read_length = vfs_read(fp, buf, sizeof(buf), &pos);
            buf[read_length] = '\0';
        } else {
            D("[LS][VCNL36863] %s Lightsensor calibration f_op = NULL \n", __func__);
            set_fs(old_fs);
            filp_close(fp, NULL);
            return -ENXIO;
        }

        set_fs(old_fs);
        filp_close(fp, NULL);

        sscanf(buf, "%d", &value);
        if(value <= 0) {
            D("[LS][VCNL36863] %s Lightsensor calibration is fail or didn't calibrate\n", __func__);
            value = 1000;
        } else {
            D("[LS][VCNL36863] %s Lightsensor calibration = %d\n", __func__, value);
        }
        return value;
}

static int ps_load_inf(void) {
        struct file *fp = NULL;
        int value;
        int read_length = 0;
        char buf[8];
        loff_t pos = 0;
        mm_segment_t old_fs;

        fp = filp_open(PS_CALIBRATION_INF, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
        if (IS_ERR_OR_NULL(fp)) {
            D("[PS][VCNL36863] %s Proximity inf calibration open fail\n", __func__);
            return -ENOENT;
        }

        old_fs = get_fs();
        set_fs(KERNEL_DS);

        if (fp->f_op != NULL) {
            pos = 0;
            read_length = vfs_read(fp, buf, sizeof(buf), &pos);
            buf[read_length] = '\0';
        } else {
            D("[PS][VCNL36863] %s Proximity inf calibration f_op = NULL \n", __func__);
            set_fs(old_fs);
            filp_close(fp, NULL);
            return -ENXIO;
        }

        set_fs(old_fs);
        filp_close(fp, NULL);

        sscanf(buf, "%d", &value);
        if(value <= 0) {
            D("[PS][VCNL36863] %s Proximity inf calibration is fail or didn't calibrate\n", __func__);
            value = 0;
        } else {
            D("[PS][VCNL36863] %s Proximity inf calibration = %d\n", __func__, value);
        }
        return value;
}

static int ps_load_1cm(void) {
        struct file *fp = NULL;
        int value;
        int read_length = 0;
        char buf[8];
        loff_t pos = 0;
        mm_segment_t old_fs;

        fp = filp_open(PS_CALIBRATION_1CM, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
        if (IS_ERR_OR_NULL(fp)) {
            D("[PS][VCNL36863] %s Proximity 1cm calibration open fail\n", __func__);
            return -ENOENT;
        }

        old_fs = get_fs();
        set_fs(KERNEL_DS);

        if (fp->f_op != NULL) {
            pos = 0;
            read_length = vfs_read(fp, buf, sizeof(buf), &pos);
            buf[read_length] = '\0';
        } else {
            D("[PS][VCNL36863] %s Proximity 1cm calibration f_op = NULL \n", __func__);
            set_fs(old_fs);
            filp_close(fp, NULL);
            return -ENXIO;
        }

        set_fs(old_fs);
        filp_close(fp, NULL);

        sscanf(buf, "%d", &value);
        if(value <= 0) {
            D("[PS][VCNL36863] %s Proximity 1cm calibration is fail or didn't calibrate\n", __func__);
            value = 1024;
        } else {
            D("[PS][VCNL36863] %s Proximity 1cm calibration = %d\n", __func__, value);
        }
        return value;
}

static int ps_load_2cm(void) {
        struct file *fp = NULL;
        int value;
        int read_length = 0;
        char buf[8];
        loff_t pos = 0;
        mm_segment_t old_fs;

        fp = filp_open(PS_CALIBRATION_2CM, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
        if (IS_ERR_OR_NULL(fp)) {
            D("[PS][VCNL36863] %s Proximity 2cm calibration open fail\n", __func__);
            return -ENOENT;
        }

        old_fs = get_fs();
        set_fs(KERNEL_DS);

        if (fp->f_op != NULL) {
            pos = 0;
            read_length = vfs_read(fp, buf, sizeof(buf), &pos);
            buf[read_length] = '\0';
        } else {
            D("[PS][VCNL36863] %s Proximity 2cm calibration f_op = NULL \n", __func__);
            set_fs(old_fs);
            filp_close(fp, NULL);
            return -ENXIO;
        }

        set_fs(old_fs);
        filp_close(fp, NULL);

        sscanf(buf, "%d", &value);
        if(value <= 0) {
            D("[PS][VCNL36863] %s Proximity 2cm calibration is fail or didn't calibrate\n", __func__);
            value = 256;
        } else {
            D("[PS][VCNL36863] %s Proximity 2cm calibration = %d\n", __func__, value);
        }
        return value;
}

static int ps_load_4cm(void) {
        struct file *fp = NULL;
        int value;
        int read_length = 0;
        char buf[8];
        loff_t pos = 0;
        mm_segment_t old_fs;

        fp = filp_open(PS_CALIBRATION_4CM, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
        if (IS_ERR_OR_NULL(fp)) {
            D("[PS][VCNL36863] %s Proximity 4cm calibration open fail\n", __func__);
            return -ENOENT;
        }

        old_fs = get_fs();
        set_fs(KERNEL_DS);

        if (fp->f_op != NULL) {
            pos = 0;
            read_length = vfs_read(fp, buf, sizeof(buf), &pos);
            buf[read_length] = '\0';
        } else {
            D("[PS][VCNL36863] %s Proximity 4cm calibration f_op = NULL \n", __func__);
            set_fs(old_fs);
            filp_close(fp, NULL);
            return -ENXIO;
        }

        set_fs(old_fs);
        filp_close(fp, NULL);

        sscanf(buf, "%d", &value);
        if(value <= 0) {
            D("[PS][VCNL36863] %s Proximity 4cm calibration is fail or didn't calibrate\n", __func__);
            value = 128;
        } else {
            D("[PS][VCNL36863] %s Proximity 4cm calibration = %d\n", __func__, value);
        }
        return value;
}

static void sensor_irq_do_work(struct work_struct *work)
{
	struct vcnl36863_info *lpi = lp_info;
  
	uint16_t dummy=CONTROL_INT_ISR_REPORT;
  
	control_and_report(lpi, CONTROL_INT_ISR_REPORT, dummy);  
	  
	enable_irq(lpi->irq);
}

static irqreturn_t vcnl36863_irq_handler(int irq, void *data)
{
	struct vcnl36863_info *lpi = data;

	disable_irq_nosync(lpi->irq);
	queue_work(lpi->lp_wq, &sensor_irq_work);

	return IRQ_HANDLED;
}

static int als_get_data(uint16_t als) {
        return als * 1000 / als_calibration;
}

static void read_ls_adc_value(void)
{
	struct vcnl36863_info *lpi = lp_info;

	//mutex_lock(&als_get_adc_mutex);

	_vcnl36863_I2C_Read_Word(lpi->slave_addr, CS_R_DATA, &vcnl36863_adc_red);
	_vcnl36863_I2C_Read_Word(lpi->slave_addr, CS_G_DATA, &vcnl36863_adc_green);
	_vcnl36863_I2C_Read_Word(lpi->slave_addr, CS_B_DATA, &vcnl36863_adc_blue);
	_vcnl36863_I2C_Read_Word(lpi->slave_addr, CS_IR_DATA, &vcnl36863_adc_ir);
        vcnl36863_als =(int) als_get_data(vcnl36863_adc_green);

	D("[LS][VCNL36863] %s %x %x %x %x %x\n", __func__, 
          vcnl36863_adc_red, vcnl36863_adc_green, vcnl36863_adc_blue, vcnl36863_adc_ir, vcnl36863_als);
  
	//mutex_unlock(&als_get_adc_mutex);	
}

static void report_lsensor_input_event(struct vcnl36863_info *lpi)
{
	input_report_rel(lpi->ls_input_dev, REL_ALS, vcnl36863_als);
	input_report_rel(lpi->ls_input_dev, REL_RED,   (int) vcnl36863_adc_red+1);
	input_report_rel(lpi->ls_input_dev, REL_GREEN, (int) vcnl36863_adc_green+1);
	input_report_rel(lpi->ls_input_dev, REL_BLUE,  (int) vcnl36863_adc_blue+1);
	input_report_rel(lpi->ls_input_dev, REL_IR, (int) vcnl36863_adc_ir+1);
	
	input_sync(lpi->ls_input_dev);  
}

static void report_psensor_input_event(struct vcnl36863_info *lpi, int value)
{
	input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, value);
	input_sync(lpi->ps_input_dev);  
}

/* This function is for light sensor.  It operates every a few seconds.
 * It asks for work to be done on a thread because i2c needs a thread
 * context (slow and blocking) and then reschedules the timer to run again.
 */
static enum hrtimer_restart vcnl36863_light_timer_func(struct hrtimer *timer)
{
	struct vcnl36863_info *lpi
		= container_of(timer, struct vcnl36863_info, light_timer);
	queue_work(lpi->lp_als_wq, &lpi->work_light);
	hrtimer_forward_now(&lpi->light_timer, lpi->light_poll_delay);
	return HRTIMER_RESTART;
}

static void vcnl36863_work_func_light(struct work_struct *work)
{
	//struct vcnl36863_info *lpi = lp_info;
	struct vcnl36863_info *lpi = container_of(work, struct vcnl36863_info,
						work_light);
#if 0 
	D("[VCNL36863] %s\n", __func__);
#endif  

        read_ls_adc_value();    
        report_lsensor_input_event(lpi);
}


static int als_power(int enable)
{
	struct vcnl36863_info *lpi = lp_info;

	if (lpi->power)
		lpi->power(LS_PWR_ON, 1);

	return 0;
}

static void ls_initial_cmd(struct vcnl36863_info *lpi)
{	
	/*must disable l-sensor interrupt befrore IST create*//*disable ALS func*/
	lpi->ls_cmd &= VCNL36863_CS_INT_MASK;
	lpi->ls_cmd |= VCNL36863_CS_SD;
	_vcnl36863_I2C_Write_Word(lpi->slave_addr, CS_CONF, lpi->ls_cmd);  
}

static void psensor_initial_cmd(struct vcnl36863_info *lpi)
{
	/*must disable p-sensor interrupt befrore IST create*//*disable PS func*/		
	lpi->ps_conf1_val |= VCNL36863_PS_SD;
	lpi->ps_conf1_val &= VCNL36863_PS_INT_MASK;  
	_vcnl36863_I2C_Write_Word(lpi->slave_addr, PS_CONF1, lpi->ps_conf1_val);   
	_vcnl36863_I2C_Write_Word(lpi->slave_addr, PS_CONF3, lpi->ps_conf3_val);
	_vcnl36863_I2C_Write_Word(lpi->slave_addr, PS_THDL,  lpi->ps_away_thd_set);
	_vcnl36863_I2C_Write_Word(lpi->slave_addr, PS_THDH,  lpi->ps_close_thd_set);
	D("[PS][VCNL36863] %s, finish\n", __func__);	
}

static void vcnl36863_power_onoff(struct vcnl36863_info *lpi, int onoff)
{
	if (onoff) {
		lpi->ls_cmd |= VCNL36863_CS_STANDBY;
		_vcnl36863_I2C_Write_Word(lpi->slave_addr, CS_CONF, lpi->ls_cmd);
		lpi->ls_cmd |= VCNL36863_CS_START;
		_vcnl36863_I2C_Write_Word(lpi->slave_addr, CS_CONF, lpi->ls_cmd);
	} else {
		lpi->ls_cmd &= ~VCNL36863_CS_STANDBY;
		_vcnl36863_I2C_Write_Word(lpi->slave_addr, CS_CONF, lpi->ls_cmd);
	}
}

static int psensor_enable(struct vcnl36863_info *lpi)
{
	int ret = 0;
	mutex_lock(&lpi->power_lock);

        D("[PS][VCNL36863] %s\n", __func__);              

	if (lpi->ps_enable) {                           
	    D("[PS][VCNL36863] %s: already enabled\n", __func__);                                      
	} else {
	    if (lpi->als_enable == 0)
	    vcnl36863_power_onoff(lpi, 1);  

	    lpi->ps_conf1_val &= VCNL36863_PS_SD_MASK;
            lpi->ps_conf1_val |= VCNL36863_PS_INT_IN_AND_OUT;     
            _vcnl36863_I2C_Write_Word(lpi->slave_addr, PS_CONF1, lpi->ps_conf1_val);    

            control_and_report(lpi, CONTROL_PS, 1);

	    lpi->ps_enable=1;  
        } 

	mutex_unlock(&lpi->power_lock);
	
	return ret;
}

static int psensor_disable(struct vcnl36863_info *lpi)
{
	int ret = 0;
	
	mutex_lock(&lpi->power_lock);
	D("[PS][VCNL36863] %s\n", __func__);

	if ( lpi->ps_enable == 0 ) {
		D("[PS][VCNL36863] %s: already disabled\n", __func__);
	} else {
	    lpi->ps_conf1_val |= VCNL36863_PS_SD;
	    lpi->ps_conf1_val &= VCNL36863_PS_INT_MASK;
	    _vcnl36863_I2C_Write_Word(lpi->slave_addr, PS_CONF1, lpi->ps_conf1_val);

	    if (lpi->als_enable == 0)
	        vcnl36863_power_onoff(lpi, 0);     

	    lpi->ps_enable=0;
        }

	mutex_unlock(&lpi->power_lock);

	return ret;
}

static int psensor_open(struct inode *inode, struct file *file)
{
	struct vcnl36863_info *lpi = lp_info;

	D("[PS][VCNL36863] %s\n", __func__);

	if (lpi->psensor_opened)
		return -EBUSY;

	lpi->psensor_opened = 1;
        ps_inf = ps_load_inf();
        ps_1cm = ps_load_1cm();
        ps_2cm = ps_load_2cm();
        ps_4cm = ps_load_4cm();
        lpi->ps_close_thd_set = ps_2cm;
        lpi->ps_away_thd_set = ps_4cm;
        _vcnl36863_I2C_Write_Word(lpi->slave_addr, PS_THDH, lpi->ps_close_thd_set );
        _vcnl36863_I2C_Write_Word(lpi->slave_addr, PS_THDL, lpi->ps_away_thd_set );

	return 0;
}

static int psensor_release(struct inode *inode, struct file *file)
{
	struct vcnl36863_info *lpi = lp_info;

	D("[PS][VCNL36863] %s\n", __func__);

	lpi->psensor_opened = 0;

	return psensor_disable(lpi);
}

static long psensor_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg)
{
	int val;
	struct vcnl36863_info *lpi = lp_info;

	D("[PS][VCNL36863] %s cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {
		case CAPELLA_CM3602_IOCTL_ENABLE:
			if (get_user(val, (unsigned long __user *)arg))
				return -EFAULT;
			if (val)
				return psensor_enable(lpi);
			else
				return psensor_disable(lpi);
			break;
		case CAPELLA_CM3602_IOCTL_GET_ENABLED:
			return put_user(lpi->ps_enable, (unsigned long __user *)arg);
			break;
		default:
			pr_err("[PS][VCNL36863 error]%s: invalid cmd %d\n",
				__func__, _IOC_NR(cmd));
			return -EINVAL;
	}
}

static const struct file_operations psensor_fops = {
	.owner = THIS_MODULE,
	.open = psensor_open,
	.release = psensor_release,
	.unlocked_ioctl = psensor_ioctl
};

struct miscdevice psensor_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "proximity",
	.fops = &psensor_fops
};


static int lightsensor_enable(struct vcnl36863_info *lpi)
{
	int ret = 0;
	
	mutex_lock(&lpi->power_lock);
	D("[LS][VCNL36863] %s\n", __func__);

	if (lpi->als_enable) {
		D("[LS][VCNL36863] %s: already enabled\n", __func__);
	} else {
            if (lpi->ps_enable == 0)
		vcnl36863_power_onoff(lpi, 1);

  	    /* enable setting */
  	    lpi->ls_cmd &= VCNL36863_CS_SD_MASK;  
  	    _vcnl36863_I2C_Write_Word(lpi->slave_addr, CS_CONF, lpi->ls_cmd);

	    hrtimer_start(&lpi->light_timer, ns_to_ktime(LIGHTSENSOR_ENABLE_DELAYTIME_IN_MS * NSEC_PER_MSEC),
  		HRTIMER_MODE_REL);

            lpi->als_enable = 1;
        }

	mutex_unlock(&lpi->power_lock);

	return ret;
}

static int lightsensor_disable(struct vcnl36863_info *lpi)
{
	int ret = 0;

	mutex_lock(&lpi->power_lock);
	D("[LS][VCNL36863] %s\n", __func__);

	if ( lpi->als_enable == 0 ) {
		D("[LS][VCNL36863] %s: already disabled\n", __func__);
	} else {
   	    lpi->ls_cmd |= VCNL36863_CS_SD;	
  	    /* disable setting */
  	    _vcnl36863_I2C_Write_Word(lpi->slave_addr, CS_CONF, lpi->ls_cmd);
            lpi->als_enable = 0;

	    if (lpi->ps_enable == 0)
		vcnl36863_power_onoff(lpi, 0);
        }

	hrtimer_cancel(&lpi->light_timer);
	cancel_work_sync(&lpi->work_light);

	mutex_unlock(&lpi->power_lock);
	return ret;  	
}

static int lightsensor_open(struct inode *inode, struct file *file)
{
	struct vcnl36863_info *lpi = lp_info;
	int rc = 0;

	D("[LS][VCNL36863] %s\n", __func__);
	if (lpi->lightsensor_opened) {
		pr_err("[LS][VCNL36863 error]%s: already opened\n", __func__);
		rc = -EBUSY;
	}
	lpi->lightsensor_opened = 1;
        als_calibration = als_load_calibration();

	return rc;
}

static int lightsensor_release(struct inode *inode, struct file *file)
{
	struct vcnl36863_info *lpi = lp_info;

	D("[LS][VCNL36863] %s\n", __func__);
	lpi->lightsensor_opened = 0;
	return 0;
}

static long lightsensor_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	int rc = 0, val;
	struct vcnl36863_info *lpi = lp_info;

	/*D("[VCNL36863] %s cmd %d\n", __func__, _IOC_NR(cmd));*/

	switch (cmd) {
		case LIGHTSENSOR_IOCTL_ENABLE:
			if (get_user(val, (unsigned long __user *)arg)) {
				rc = -EFAULT;
				break;
			}
			D("[LS][VCNL36863] %s LIGHTSENSOR_IOCTL_ENABLE, value = %d\n",
				__func__, val);
			rc = val ? lightsensor_enable(lpi) : lightsensor_disable(lpi);
			break;
		case LIGHTSENSOR_IOCTL_GET_ENABLED:
			val = lpi->als_enable;
			D("[LS][VCNL36863] %s LIGHTSENSOR_IOCTL_GET_ENABLED, enabled %d\n",
				__func__, val);
			rc = put_user(val, (unsigned long __user *)arg);
			break;
		default:
			pr_err("[LS][VCNL36863 error]%s: invalid cmd %d\n",
				__func__, _IOC_NR(cmd));
			rc = -EINVAL;
	}
	return rc;
}

static const struct file_operations lightsensor_fops = {
	.owner = THIS_MODULE,
	.open = lightsensor_open,
	.release = lightsensor_release,
	.unlocked_ioctl = lightsensor_ioctl
};

static struct miscdevice lightsensor_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "lightsensor",
	.fops = &lightsensor_fops
};

static ssize_t ps_adc_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{

	uint16_t value;
	int ret;
	struct vcnl36863_info *lpi = lp_info;
	int intr_val;

	intr_val = gpio_get_value(lpi->intr_pin);

	get_ps_adc_value(&value);

	ret = sprintf(buf, "DEC ADC[%d], ENABLE = %d, intr_pin = %d\n", value, lpi->ps_enable, intr_val);

	return ret;
}

static ssize_t ps_get_proxm_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	uint16_t value;
	struct vcnl36863_info *lpi = lp_info;

        psensor_enable(lpi);
        msleep(10);

	get_ps_adc_value(&value);

        if (lpi->psensor_opened != 1) {
            psensor_disable(lpi);
        }

        return sprintf(buf, "%d", value);
}

static ssize_t ps_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{

	int ret = 0;
	struct vcnl36863_info *lpi = lp_info;

	ret = sprintf(buf, "Proximity sensor Enable = %d\n",
			lpi->ps_enable);

	return ret;
}

static ssize_t ps_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ps_en;
	struct vcnl36863_info *lpi = lp_info;

	ps_en = -1;
	sscanf(buf, "%d", &ps_en);

	if (ps_en != 0 && ps_en != 1)
		return -EINVAL;
	
	D("[PS][VCNL36863] %s: ps_en=%d\n", __func__, ps_en);
	
	if (ps_en)
		psensor_enable(lpi);
	else
		psensor_disable(lpi);

	return count;
}

static ssize_t ps_conf_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct vcnl36863_info *lpi = lp_info;
	return sprintf(buf, "PS_CONF1 = 0x%04x, PS_CONF3 = 0x%04x\n", lpi->ps_conf1_val, lpi->ps_conf3_val);
}
static ssize_t ps_conf_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int code1, code2;
	struct vcnl36863_info *lpi = lp_info;

	sscanf(buf, "0x%x 0x%x", &code1, &code2);

	D("[PS]%s: store value PS conf1 reg = 0x%04x PS conf3 reg = 0x%04x\n", __func__, code1, code2);

	lpi->ps_conf1_val = code1;
	lpi->ps_conf3_val = code2;

	_vcnl36863_I2C_Write_Word(lpi->slave_addr, PS_CONF3, lpi->ps_conf3_val );  
	_vcnl36863_I2C_Write_Word(lpi->slave_addr, PS_CONF1, lpi->ps_conf1_val );

	return count;
}

static ssize_t ps_thd_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret;
	struct vcnl36863_info *lpi = lp_info;
	ret = sprintf(buf, "[PS][VCNL36863]PS Hi/Low THD ps_close_thd_set = 0x%04x(%d), ps_away_thd_set = 0x%04x(%d)\n", lpi->ps_close_thd_set, lpi->ps_close_thd_set, lpi->ps_away_thd_set, lpi->ps_away_thd_set);
	return ret;	
}
static ssize_t ps_thd_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int code1, code2;
	int ps_en;
	struct vcnl36863_info *lpi = lp_info;

	sscanf(buf, "0x%x 0x%x", &code1, &code2);

	lpi->ps_close_thd_set = code1;	
	lpi->ps_away_thd_set = code2;
	ps_en = lpi->ps_enable;

	if(ps_en)
		psensor_disable(lpi);
	
	_vcnl36863_I2C_Write_Word(lpi->slave_addr, PS_THDH, lpi->ps_close_thd_set );
	_vcnl36863_I2C_Write_Word(lpi->slave_addr, PS_THDL, lpi->ps_away_thd_set );

	if(ps_en)
		psensor_enable(lpi);	
	
	D("[PS][VCNL36863]%s: ps_close_thd_set = 0x%04x(%d), ps_away_thd_set = 0x%04x(%d)\n", __func__, lpi->ps_close_thd_set, lpi->ps_close_thd_set, lpi->ps_away_thd_set, lpi->ps_away_thd_set);
    
	return count;
}

static ssize_t ps_canc_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct vcnl36863_info *lpi = lp_info;

	ret = sprintf(buf, "[PS][VCNL36863]PS_CANC = 0x%04x(%d)\n", lpi->inte_cancel_set,lpi->inte_cancel_set);

	return ret;
}
static ssize_t ps_canc_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int code;
	int ps_en;
	struct vcnl36863_info *lpi = lp_info;

	sscanf(buf, "0x%x", &code);

	D("[PS][VCNL36863]PS_CANC: store value = 0x%04x(%d)\n", code,code);
	
	lpi->inte_cancel_set = code;	
	ps_en = lpi->ps_enable;

	if(ps_en)
		psensor_disable(lpi);

	_vcnl36863_I2C_Write_Word(lpi->slave_addr, PS_CANC, lpi->inte_cancel_set );

	if(ps_en)
		psensor_enable(lpi);
	
	return count;
}

static ssize_t ls_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{

	int ret = 0;
	struct vcnl36863_info *lpi = lp_info;

	ret = sprintf(buf, "Light sensor Enable = %d\n",
			lpi->als_enable);

	return ret;
}

static ssize_t ls_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int ret = 0;
	int ls_auto;
	struct vcnl36863_info *lpi = lp_info;

	ls_auto = -1;
	sscanf(buf, "%d", &ls_auto);

	if (ls_auto != 0 && ls_auto != 1)
		return -EINVAL;

	if (ls_auto) {
		ret = lightsensor_enable(lpi);
	} else {
		ret = lightsensor_disable(lpi);
	}

	D("[LS][VCNL36863] %s: lpi->als_enable = %d, ls_auto=%d\n",
		__func__, lpi->als_enable, ls_auto);

	if (ret < 0)
		pr_err("[LS][VCNL36863 error]%s: set auto light sensor fail\n",
			__func__);

	return count;
}

static ssize_t ls_conf_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct vcnl36863_info *lpi = lp_info;
	return sprintf(buf, "ALS_CONF = %x\n", lpi->ls_cmd);
}
static ssize_t ls_conf_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct vcnl36863_info *lpi = lp_info;
	int value = 0;
	sscanf(buf, "0x%x", &value);

	lpi->ls_cmd = value;
	printk(KERN_INFO "[LS]set LS_CONF = %x\n", lpi->ls_cmd);
	
	_vcnl36863_I2C_Write_Word(lpi->slave_addr, CS_CONF, lpi->ls_cmd);
	return count;
}

static ssize_t ls_red_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
        struct vcnl36863_info *lpi = lp_info;

        lightsensor_enable(lpi);

        _vcnl36863_I2C_Read_Word(lpi->slave_addr, CS_R_DATA, &vcnl36863_adc_red);

        lightsensor_disable(lpi);

	return sprintf(buf, "%d\n", vcnl36863_adc_red);
}

static ssize_t ls_green_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
        struct vcnl36863_info *lpi = lp_info;

        lightsensor_enable(lpi);

        _vcnl36863_I2C_Read_Word(lpi->slave_addr, CS_G_DATA, &vcnl36863_adc_green);

        lightsensor_disable(lpi);

	return sprintf(buf, "%d\n", vcnl36863_adc_green);
}

static ssize_t ls_blue_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
        struct vcnl36863_info *lpi = lp_info;

        lightsensor_enable(lpi);

        _vcnl36863_I2C_Read_Word(lpi->slave_addr, CS_B_DATA, &vcnl36863_adc_blue);

        lightsensor_disable(lpi);

	return sprintf(buf, "%d\n", vcnl36863_adc_blue);
}

static ssize_t ls_ir_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
        struct vcnl36863_info *lpi = lp_info;

        lightsensor_enable(lpi);

        _vcnl36863_I2C_Read_Word(lpi->slave_addr, CS_IR_DATA, &vcnl36863_adc_ir);

        lightsensor_disable(lpi);

	return sprintf(buf, "%d\n", vcnl36863_adc_ir);
}

static ssize_t ls_status_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
        struct vcnl36863_info *lpi = lp_info;
        uint16_t data;
        int ret = 0;

        ret = _vcnl36863_I2C_Read_Word(lpi->slave_addr, ID_REG, &data);
        pr_info("ret(%d) addr(0x%x) ID_REG(0x%x) data(%d)\n",
                        ret, lpi->slave_addr, ID_REG, data);

        if (g_ASUS_hwID >= 5) {
            ret = (!ret && data == 0x0063) ? 1 : 0;
        } else {
            ret = (!ret && data == 0x0122) ? 1 : 0;
        }
        return sprintf(buf, "%x\n", ret);
}

static ssize_t ps_status_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
        struct vcnl36863_info *lpi = lp_info;
        uint16_t data;
        int ret = 0;

        ret = _vcnl36863_I2C_Read_Word(lpi->slave_addr, ID_REG, &data);
        pr_info("ret(%d) addr(0x%x) ID_REG(0x%x) data(%d)\n",
                        ret, lpi->slave_addr, ID_REG, data);

        if (g_ASUS_hwID >= 5) {
            ret = (!ret && data == 0x0063) ? 1 : 0;
        } else {
            ret = (!ret && data == 0x0122) ? 1 : 0;
        }
        return sprintf(buf, "%x\n", ret);
}

static ssize_t ls_adc_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
        struct vcnl36863_info *lpi = lp_info;

        lightsensor_enable(lpi);
        msleep(10);

        _vcnl36863_I2C_Read_Word(lpi->slave_addr, CS_G_DATA, &vcnl36863_adc_green);

        if (lpi->lightsensor_opened != 1) {
            lightsensor_disable(lpi);
        }

        return sprintf(buf, "%d\n", (int) vcnl36863_adc_green);
}

static struct device_attribute dev_attr_ps_adc =
__ATTR(ps_adc, 0444, ps_adc_show, NULL);

static struct device_attribute dev_attr_ps_enable =
__ATTR(ps_enable, 0664, ps_enable_show, ps_enable_store);	

static struct device_attribute dev_attr_ps_conf =	
__ATTR(ps_conf, 0664, ps_conf_show, ps_conf_store);

static struct device_attribute dev_attr_ps_thd =	
__ATTR(ps_thd, 0664, ps_thd_show, ps_thd_store);

static struct device_attribute dev_attr_ps_canc = 	
__ATTR(ps_canc, 0664, ps_canc_show, ps_canc_store);

static struct device_attribute dev_attr_ps_status = 
__ATTR(ps_status, 0444, ps_status_show, NULL);

static struct device_attribute dev_attr_ps_get_proxm = 
__ATTR(ps_get_proxm, 0444, ps_get_proxm_show, NULL);

static struct attribute *proximity_sysfs_attrs[] = {
	&dev_attr_ps_adc.attr,
	&dev_attr_ps_enable.attr,
	&dev_attr_ps_conf.attr,
	&dev_attr_ps_thd.attr,
	&dev_attr_ps_canc.attr,
	&dev_attr_ps_status.attr,
	&dev_attr_ps_get_proxm.attr,
	NULL
};

static struct attribute_group proximity_attribute_group = {
	.attrs = proximity_sysfs_attrs,
};

static struct device_attribute dev_attr_ls_enable =
__ATTR(ls_enable, 0664, ls_enable_show, ls_enable_store);

static struct device_attribute dev_attr_ls_conf =
__ATTR(ls_conf, 0664, ls_conf_show, ls_conf_store);

static struct device_attribute dev_attr_light_red =
__ATTR(ls_red, 0444, ls_red_show, NULL);

static struct device_attribute dev_attr_light_green =
__ATTR(ls_green, 0444, ls_green_show, NULL);

static struct device_attribute dev_attr_light_blue =
__ATTR(ls_blue, 0444, ls_blue_show, NULL);

static struct device_attribute dev_attr_light_ir =
__ATTR(ls_ir, 0444, ls_ir_show, NULL);

static struct device_attribute dev_attr_light_status =
__ATTR(ls_status, 0444, ls_status_show, NULL);

static struct device_attribute dev_attr_light_adc =
__ATTR(ls_adc, 0444, ls_adc_show, NULL);

static struct attribute *light_sysfs_attrs[] = {
	&dev_attr_ls_enable.attr,
	&dev_attr_ls_conf.attr,
	&dev_attr_light_red.attr,
	&dev_attr_light_green.attr,
	&dev_attr_light_blue.attr,  
	&dev_attr_light_ir.attr,
	&dev_attr_light_status.attr,
	&dev_attr_light_adc.attr,
	NULL
};

static struct attribute_group light_attribute_group = {
	.attrs = light_sysfs_attrs,
};

static int lightsensor_setup(struct vcnl36863_info *lpi)
{
	int ret;

	lpi->ls_input_dev = input_allocate_device();
	if (!lpi->ls_input_dev) {
		pr_err(
			"[LS][VCNL36863 error]%s: could not allocate ls input device\n",
			__func__);
		return -ENOMEM;
	}
	lpi->ls_input_dev->name = "vcnl36863-ls";

	input_set_capability(lpi->ls_input_dev, EV_REL, REL_ALS);
	input_set_capability(lpi->ls_input_dev, EV_REL, REL_RED);
	input_set_capability(lpi->ls_input_dev, EV_REL, REL_GREEN);
	input_set_capability(lpi->ls_input_dev, EV_REL, REL_BLUE);
	input_set_capability(lpi->ls_input_dev, EV_REL, REL_IR); 

	ret = input_register_device(lpi->ls_input_dev);
	if (ret < 0) {
		pr_err("[LS][VCNL36863 error]%s: can not register ls input device\n",
				__func__);
		goto err_free_ls_input_device;
	}

	ret = misc_register(&lightsensor_misc);
	if (ret < 0) {
		pr_err("[LS][VCNL36863 error]%s: can not register ls misc device\n",
				__func__);
		goto err_unregister_ls_input_device;
	}

	return ret;

err_unregister_ls_input_device:
	input_unregister_device(lpi->ls_input_dev);
err_free_ls_input_device:
	input_free_device(lpi->ls_input_dev);
	return ret;
}

static int psensor_setup(struct vcnl36863_info *lpi)
{
	int ret;

	lpi->ps_input_dev = input_allocate_device();
	if (!lpi->ps_input_dev) {
		pr_err("[PS][VCNL36863 error]%s: could not allocate ps input device\n",
			__func__);
		return -ENOMEM;
	}
	lpi->ps_input_dev->name = "vcnl36863-ps";
	set_bit(EV_ABS, lpi->ps_input_dev->evbit);
	input_set_abs_params(lpi->ps_input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	ret = input_register_device(lpi->ps_input_dev);
	if (ret < 0) {
		pr_err(
			"[PS][VCNL36863 error]%s: could not register ps input device\n",
			__func__);
		goto err_free_ps_input_device;
	}

	ret = misc_register(&psensor_misc);
	if (ret < 0) {
		pr_err(
			"[PS][VCNL36863 error]%s: could not register ps misc device\n",
			__func__);
		goto err_unregister_ps_input_device;
	}

	return ret;

err_unregister_ps_input_device:
	input_unregister_device(lpi->ps_input_dev);
err_free_ps_input_device:
	input_free_device(lpi->ps_input_dev);
	return ret;
}

static int initial_vcnl36863_gpio(struct vcnl36863_info *lpi)
{
	int val;

	val = gpio_get_value(lpi->intr_pin);
	D("[PS][VCNL36863] %s, INTERRUPT GPIO val = %d\n", __func__, val);
  
	return 0;
}

static int vcnl36863_power_control(struct vcnl36863_info *lpi, uint8_t enable)
{
        int ret = 0;

        pr_info("[VCNL36863] %s +\n", __func__);

        if (enable) {
                if (g_ASUS_hwID <= 5) {
                    ret = regulator_enable(lpi->vcc_l8c_1p8);
                    if (ret)
                        pr_err("[VCNL36863] Regulator enable failed vcc_l8c_1p8 ret=%d\n", ret);
                }

                ret = regulator_enable(lpi->vcc_l7c_3p0);
                if (ret)
                        pr_err("[VCNL36863] Regulator enable failed vcc_l7c_3p0 ret=%d\n", ret);

        } else {
                if (g_ASUS_hwID <= 5) {
                    ret = regulator_disable(lpi->vcc_l8c_1p8);
                    if (ret)
                        pr_err("[VCNL36863] Regulator disable failed vcc_l8c_1p8 ret=%d\n", ret);
                }

                ret = regulator_disable(lpi->vcc_l7c_3p0);
                if (ret)
                        pr_err("[VCNL36863] Regulator disable failed vcc_l7c_3p0 ret=%d\n", ret);
        }

        pr_info("[VCNL36863] %s -\n", __func__);
        return ret;
}

static int vcnl36863_power_init(struct vcnl36863_info *lpi)
{
        int ret = 0;

        pr_info("[VCNL36863] %s +\n", __func__);
        if (g_ASUS_hwID <= 5) {
            lpi->vcc_l8c_1p8 = regulator_get(&lpi->i2c_client->dev, "vcc_l8c_1p8");
            if (IS_ERR(lpi->vcc_l8c_1p8))
            {
                ret = PTR_ERR(lpi->vcc_l8c_1p8);
                pr_err("[VCNL36863] Regulator get failed vdd ret=%d", ret);
                return ret;
            }

            if (regulator_count_voltages(lpi->vcc_l8c_1p8) > 0)
            {
                ret = regulator_set_voltage(lpi->vcc_l8c_1p8, 1800000, 1800000);
                if (ret)
                {
                        pr_err("Regulator set_vtg failed vcc_l8c_1p8 ret=%d", ret);
                        goto reg_vdd_put;
                }
            }
        }

        lpi->vcc_l7c_3p0 = regulator_get(&lpi->i2c_client->dev, "vcc_l7c_3p0");
        if (IS_ERR(lpi->vcc_l7c_3p0))
        {
                ret = PTR_ERR(lpi->vcc_l7c_3p0);
                pr_err("Regulator get failed vcc_l7c_3p0 ret=%d", ret);
                goto reg_vdd_set_vtg;
        }

        if (regulator_count_voltages(lpi->vcc_l7c_3p0) > 0)
        {
                ret = regulator_set_voltage(lpi->vcc_l7c_3p0, 2856000, 3104000);
                if (ret)
                {
                        pr_err("Regulator set_vtg failed vcc_i2c ret=%d", ret);
                        goto reg_vcc_i2c_put;
                }
        }

        pr_info("[VCNL36863] %s -\n", __func__);
        return 0;
reg_vcc_i2c_put:
        regulator_put(lpi->vcc_l7c_3p0);
reg_vdd_set_vtg:
if (g_ASUS_hwID <= 5) {
        if (regulator_count_voltages(lpi->vcc_l8c_1p8) > 0)
                regulator_set_voltage(lpi->vcc_l8c_1p8, 0, 3000000);
}
reg_vdd_put:
if (g_ASUS_hwID <= 5) {
        regulator_put(lpi->vcc_l8c_1p8);
}
        return ret;
}


static int vcnl36863_setup(struct vcnl36863_info *lpi)
{
	int ret = 0;

	als_power(1);
	msleep(5);
	ret = gpio_request(lpi->intr_pin, "gpio_vcnl36863_intr");
	if (ret < 0) {
		pr_err("[PS][VCNL36863 error]%s: gpio %d request failed (%d)\n",
			__func__, lpi->intr_pin, ret);
		return ret;
	}

	ret = gpio_direction_input(lpi->intr_pin);
	if (ret < 0) {
		pr_err(
			"[PS][VCNL36863 error]%s: fail to set gpio %d as input (%d)\n",
			__func__, lpi->intr_pin, ret);
		goto fail_free_intr_pin;
	}

	//initialize gpio
	ret = initial_vcnl36863_gpio(lpi);
	if (ret < 0) {
		pr_err(
			"[PS_ERR][VCNL36863 error]%s: fail to initial vcnl36863 (%d)\n",
			__func__, ret);
		goto fail_free_intr_pin;
	}

	/*Default disable P sensor and L sensor*/
	ls_initial_cmd(lpi);
	psensor_initial_cmd(lpi);

	ret = request_any_context_irq(lpi->irq,
			vcnl36863_irq_handler,
			IRQF_TRIGGER_LOW,
			"vcnl36863",
			lpi);
	if (ret < 0) {
		pr_err(
			"[PS][VCNL36863 error]%s: req_irq(%d) fail for gpio %d (%d)\n",
			__func__, lpi->irq,
			lpi->intr_pin, ret);
		goto fail_free_intr_pin;
	}

        ret = enable_irq_wake(lpi->irq);
        if (ret) {
                pr_err("[PS_ERR][VCNL36863 error]%s: fail enable irq wake %d\n", __func__, ret);
                goto fail_free_intr_pin;
        }

	return ret;

fail_free_intr_pin:
	gpio_free(lpi->intr_pin);
	return ret;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void vcnl36863_early_suspend(struct early_suspend *h)
{
	struct vcnl36863_info *lpi = lp_info;

	D("[LS][VCNL36863] %s\n", __func__);

	if (lpi->als_enable)
		lightsensor_disable(lpi);
	
	if (lpi->ps_enable)
		psensor_disable(lpi);
}

static void vcnl36863_late_resume(struct early_suspend *h)
{
	struct vcnl36863_info *lpi = lp_info;

	D("[LS][VCNL36863] %s\n", __func__);

	if (!lpi->als_enable)
		lightsensor_enable(lpi);
		
		if (!lpi->ps_enable)
		psensor_enable(lpi);
}
#endif

#ifdef CONFIG_OF
static int vcnl36863_parse_dt(struct device *dev,
				struct vcnl36863_info *lpi)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;
	
	D("[LS][VCNL36863] %s\n", __func__);
  
	rc = of_get_named_gpio_flags(np, "capella,intrpin-gpios",
			0, NULL);
	if (rc < 0) 
	{
		dev_err(dev, "Unable to read interrupt pin number\n");
		return rc;
	} 
	else
	{
		lpi->intr_pin = rc;
 	  D("[LS][VCNL36863]%s GET INTR PIN \n", __func__);   
	}

	rc = of_property_read_u32(np, "capella,slave_address", &temp_val);
	if (rc)
	{
		dev_err(dev, "Unable to read slave_address\n");
		return rc;
	} 
	else
	{
		lpi->slave_addr = (uint8_t)temp_val;
	}
  
        rc = of_get_named_gpio_flags(np, "capella,enable-gpios", 0, NULL);
        if (rc < 0) {
                dev_err(dev, "Unable to read enable pin number\n");
                return rc;
        } else {
                lpi->enable_gpio = rc;
          D("[LS][VCNL36863]%s GET ENABLE PIN \n", __func__);
        }

	D("[PS][VCNL36863]%s PARSE OK \n", __func__);
	return 0;
}
#endif

static int vcnl36863_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;
	struct vcnl36863_info *lpi;
        uint16_t param;
#ifndef CONFIG_OF 
	struct vcnl36863_platform_data *pdata;
#endif

	D("[ALS+PS][VCNL36863] %s\n", __func__);

	lpi = kzalloc(sizeof(struct vcnl36863_info), GFP_KERNEL);
	if (!lpi)
		return -ENOMEM;

	/*D("[VCNL36863] %s: client->irq = %d\n", __func__, client->irq);*/

	lpi->i2c_client = client;

	lpi->irq = client->irq;

	i2c_set_clientdata(client, lpi);

        mutex_init(&lpi->power_lock);
#ifndef CONFIG_OF
	pdata = client->dev.platform_data;
	if (!pdata) {
		pr_err("[ALS+PS][VCNL36863 error]%s: Assign platform_data error!!\n",
			__func__);
		ret = -EBUSY;
		goto err_platform_data_null;
	}
	
	lpi->intr_pin = pdata->intr;
	
	lpi->power = pdata->power;
	
	lpi->slave_addr = pdata->slave_addr;  
	
	lpi->ps_away_thd_set = pdata->ps_away_thd_set;
	lpi->ps_close_thd_set = pdata->ps_close_thd_set;	
	lpi->ps_conf1_val = pdata->ps_conf1_val;
	lpi->ps_conf3_val = pdata->ps_conf3_val;
	lpi->ls_cmd  = pdata->ls_cmd;
#else
	if( vcnl36863_parse_dt(&client->dev, lpi) < 0 )
	{
		ret = -EBUSY;
		goto err_platform_data_null;  
	}

        ret = gpio_request_one(lpi->enable_gpio, GPIOF_OUT_INIT_HIGH, "gpio_LDO_enable");
        if (ret < 0) {
                pr_err("[PS][VCNL36863 error]%s: gpio %d request failed (%d)\n",
                        __func__, lpi->enable_gpio, ret);
                goto err_platform_data_null;
        }
        D("[VCNL36863] %s: GPIO request success value = %d\n", __func__, lpi->enable_gpio);
        ret = gpio_get_value(lpi->enable_gpio);
        D("%s: GPIO get value = %d\n", __func__, ret);
        msleep(100);

	lpi->ps_away_thd_set = 0x7F;
	lpi->ps_close_thd_set = 0xFF;
	lpi->ps_conf1_val = VCNL36863_PS_IT_4T  | VCNL36863_PS_START | VCNL36863_PS_MPS_2 | VCNL36863_PS_HG_ENABLE| VCNL36863_PS_PERS_4;
	lpi->ps_conf3_val = VCNL36863_LED_I_14mA | VCNL36863_PS_START2;
  
	lpi->ls_cmd = CS_RESERVED_1 | VCNL36863_CS_IT_50MS;
	lpi->power  = NULL;
#endif
	
		
	D("[PS][VCNL36863] %s: ls_cmd 0x%x\n",
		__func__, lpi->ls_cmd);
	
	if (lpi->ls_cmd == 0) {
		lpi->ls_cmd  = CS_RESERVED_1 | VCNL36863_CS_IT_50MS;
	}

	lp_info = lpi;
        ret = vcnl36863_power_init(lpi);
        if (ret < 0)
                    pr_err("[VCNL36863 error]%s: set regulator fail\n", __func__);

        ret = vcnl36863_power_control(lpi, 1);
        if (ret < 0)
                    pr_err("[VCNL36863 error]%s: enable regulator fail\n", __func__);

        ret = _vcnl36863_I2C_Read_Word(lpi->slave_addr, INT_FLAG, &param);
        if (ret < 0) {
            pr_err("[VCNL36863 error]%s: I2C Read fail\n", __func__);
            goto err_read_i2c_fail;
        }

	mutex_init(&VCNL36863_control_mutex);

        /* light_timer settings. we poll for light values using a timer. */
	hrtimer_init(&lpi->light_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        lpi->light_poll_delay = ns_to_ktime(LS_POLLING_DELAY * NSEC_PER_MSEC);  
	lpi->light_timer.function = vcnl36863_light_timer_func;

  lpi->lp_als_wq = create_singlethread_workqueue("vcnl36863_als_wq");
	if (!lpi->lp_als_wq) {
		pr_err("[VCNL36863 error]%s: can't create workqueue\n", __func__);
		ret = -ENOMEM;
		goto err_create_singlethread_als_workqueue;
	}

	ret = lightsensor_setup(lpi);
	if (ret < 0) {
		pr_err("[LS][VCNL36863 error]%s: lightsensor_setup error!!\n",
			__func__);
		goto err_lightsensor_setup;
	}

	/* this is the thread function we run on the work queue */
	INIT_WORK(&lpi->work_light, vcnl36863_work_func_light);

	ret = psensor_setup(lpi);
	if (ret < 0) {
		pr_err("[PS][VCNL36863 error]%s: psensor_setup error!!\n",
			__func__);
		goto err_psensor_setup;
	}
  

	lpi->lp_wq = create_singlethread_workqueue("vcnl36863_wq");
	if (!lpi->lp_wq) {
		pr_err("[PS][VCNL36863 error]%s: can't create workqueue\n", __func__);
		ret = -ENOMEM;
		goto err_create_singlethread_workqueue;
	}

#ifdef CONFIG_HAS_WAKELOCK  
	wake_lock_init(&(lpi->ps_wake_lock), WAKE_LOCK_SUSPEND, "proximity");
#endif

	ret = vcnl36863_setup(lpi);
	if (ret < 0) {
		pr_err("[PS_ERR][VCNL36863 error]%s: vcnl36863_setup error!\n", __func__);
		goto err_vcnl36863_setup;
	}
	lpi->vcnl36863_class = class_create(THIS_MODULE, "capella_sensors");
	if (IS_ERR(lpi->vcnl36863_class)) {
		ret = PTR_ERR(lpi->vcnl36863_class);
		lpi->vcnl36863_class = NULL;
		goto err_create_class;
	}

	lpi->ls_dev = device_create(lpi->vcnl36863_class,
				NULL, 0, "%s", "lightsensor");
	if (unlikely(IS_ERR(lpi->ls_dev))) {
		ret = PTR_ERR(lpi->ls_dev);
		lpi->ls_dev = NULL;
		goto err_create_ls_device;
	}

	/* register the attributes */
	ret = sysfs_create_group(&lpi->ls_input_dev->dev.kobj, &light_attribute_group);
	if (ret)
		goto err_sysfs_create_group_light;

	lpi->ps_dev = device_create(lpi->vcnl36863_class,
				NULL, 0, "%s", "proximity");
	if (unlikely(IS_ERR(lpi->ps_dev))) {
		ret = PTR_ERR(lpi->ps_dev);
		lpi->ps_dev = NULL;
		goto err_create_ps_device;
	}

	/* register the attributes */
	ret = sysfs_create_group(&lpi->ps_input_dev->dev.kobj, &proximity_attribute_group);
	if (ret)
		goto err_sysfs_create_group_proximity;

#ifdef CONFIG_HAS_EARLYSUSPEND
	lpi->early_suspend.level =
			EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	lpi->early_suspend.suspend = vcnl36863_early_suspend;
	lpi->early_suspend.resume = vcnl36863_late_resume;
	register_early_suspend(&lpi->early_suspend);
#endif

	D("[PS][VCNL36863] %s: Probe success!\n", __func__);

	return ret;

err_sysfs_create_group_proximity:
  device_destroy(lpi->vcnl36863_class, lpi->ps_dev->devt);
err_create_ps_device:
  sysfs_remove_group(&lpi->ls_input_dev->dev.kobj, &light_attribute_group);
err_sysfs_create_group_light:
	device_destroy(lpi->vcnl36863_class, lpi->ls_dev->devt);
err_create_ls_device:
	class_destroy(lpi->vcnl36863_class);
err_create_class:
	gpio_free(lpi->intr_pin); //vcnl36863_setup
err_vcnl36863_setup:
	destroy_workqueue(lpi->lp_wq);
#ifdef CONFIG_HAS_WAKELOCK  
	wake_lock_destroy(&(lpi->ps_wake_lock));
#endif  

	input_unregister_device(lpi->ls_input_dev);
	input_free_device(lpi->ls_input_dev);
	input_unregister_device(lpi->ps_input_dev);
	input_free_device(lpi->ps_input_dev);
err_create_singlethread_workqueue:
err_psensor_setup:
	mutex_destroy(&VCNL36863_control_mutex);
	misc_deregister(&lightsensor_misc); //lightsensor_setup

	destroy_workqueue(lpi->lp_als_wq);  
err_create_singlethread_als_workqueue:  
err_lightsensor_setup:
        mutex_destroy(&lpi->power_lock);
err_read_i2c_fail:
        gpio_free(lpi->enable_gpio);
err_platform_data_null:
        vcnl36863_power_control(lpi, 0);
	kfree(lpi);
	return ret;
}
   
static int control_and_report( struct vcnl36863_info *lpi, uint8_t mode, uint16_t param ) {
	int ret=0;
	uint16_t ps_data = 0;
	int val;
	
	mutex_lock(&VCNL36863_control_mutex);

	if(mode == CONTROL_INT_ISR_REPORT) {
		_vcnl36863_I2C_Read_Word(lpi->slave_addr, INT_FLAG, &param);
		D("[ALS+PS][INT][VCNL36863] param: %d\n", param);
	}
  
	/*if( mode == CONTROL_ALS ){
		if(param){
			lpi->ls_cmd &= VCNL36863_CS_SD_MASK;            
		} else {
			lpi->ls_cmd |= VCNL36863_CS_SD;
			cancel_delayed_work(&report_work);
		}
    
		if (lpi->ps_enable == 0 && param == 1)
			vcnl36863_power_onoff(lpi, 1);    
    
		_vcnl36863_I2C_Write_Word(lpi->slave_addr, CS_CONF, lpi->ls_cmd);
		lpi->als_enable=param;

 		if (lpi->ps_enable == 0 && param == 0)
			vcnl36863_power_onoff(lpi, 0);    
	} else if( mode == CONTROL_PS ){
		if(param){ 
			lpi->ps_conf1_val &= VCNL36863_PS_SD_MASK;
			lpi->ps_conf1_val |= CM36656_PS_INT_IN_AND_OUT;      
		} else {
			lpi->ps_conf1_val |= VCNL36863_PS_SD;
			lpi->ps_conf1_val &= VCNL36863_PS_INT_MASK;
		}

		if (lpi->als_enable == 0 && param == 1)
			vcnl36863_power_onoff(lpi, 1);    
    
		_vcnl36863_I2C_Write_Word(lpi->slave_addr, PS_CONF1, lpi->ps_conf1_val);    
		lpi->ps_enable=param;
    
 		if (lpi->als_enable == 0 && param == 0)
			vcnl36863_power_onoff(lpi, 0);          
	}

	if(mode == CONTROL_ALS && param==1 ){  
			msleep(200);
	}

	if( mode == CONTROL_CS_REPORT || ( (mode==CONTROL_ALS) && param ) ){
		read_ls_adc_value();
		report_lsensor_input_event(lpi);
		queue_delayed_work(lpi->lp_als_wq, &report_work, lpi->als_polling_delay);      
	}*/


#define PS_CLOSE 1
#define PS_AWAY  (1<<1)
#define PS_CLOSE_AND_AWAY PS_CLOSE+PS_AWAY
	if(lpi->ps_enable){
		int ps_status = 0;
		if( mode == CONTROL_PS )
			ps_status = PS_CLOSE_AND_AWAY;   
		else if(mode == CONTROL_INT_ISR_REPORT ){  
			if ( param & INT_FLAG_PS_IF_CLOSE )
				ps_status |= PS_CLOSE;      
			if ( param & INT_FLAG_PS_IF_AWAY )
				ps_status |= PS_AWAY;
		}

		if (ps_status!=0){
			switch(ps_status){
				case PS_CLOSE_AND_AWAY:
					get_stable_ps_adc_value(&ps_data);
					val = (ps_data >= lpi->ps_close_thd_set) ? 2 : 10;
					break;
				case PS_AWAY:
					val = 10;
					D("[PS][VCNL36863] proximity detected object away\n");
					break;
				case PS_CLOSE:
                                        get_ps_adc_value(&ps_data);
                                        if (ps_1cm > 0 && ps_data > ps_1cm) {
                                            val = 0;
					    D("[PS][VCNL36863] proximity detected object close dis < 1cm\n");
                                        } else {
					    val = 2;
					    D("[PS][VCNL36863] proximity detected object close\n");
                                        }
					break;
			};
        
			report_psensor_input_event(lpi, val);        
		}
	}

	mutex_unlock(&VCNL36863_control_mutex);
	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int vcnl36863_suspend(struct device *dev)
{
	struct vcnl36863_info *lpi;
	lpi = dev_get_drvdata(dev);

	/*
	  * Save sensor state and disable them,
	  * this is to ensure internal state flags are set correctly.
	  * device will power off after both sensors are disabled.
	  * P sensor will not be disabled because it  is a wakeup sensor.
	*/

	lpi->als_enabled_before_suspend = lpi->als_enable;  

#ifdef CONFIG_HAS_WAKELOCK
	if (lpi->ps_enable) {
		 wake_lock(&lpi->ps_wake_lock);
	}
#endif
 
#ifdef UNSUPPORT_AUTO_BACKLIGHT
	if (lpi->als_enable == 1)
		lightsensor_disable(lpi);
#endif

	return 0;
}

static int vcnl36863_resume(struct device *dev)
{
	struct vcnl36863_info *lpi;
	lpi = dev_get_drvdata(dev);

#ifdef CONFIG_HAS_WAKELOCK
	if (lpi->ps_enable ) {
	         wake_unlock(&(lpi->ps_wake_lock));
	}
#endif

	/* Don't disable light at phone calling
	  * while the automatic backlight is on.
	  */
#ifdef UNSUPPORT_AUTO_BACKLIGHT
	if (lpi->als_enabled_before_suspend)
		lightsensor_enable(lpi);
#endif	

	return 0;
}
#endif

static UNIVERSAL_DEV_PM_OPS(vcnl36863_pm, vcnl36863_suspend, vcnl36863_resume, NULL);


static const struct i2c_device_id vcnl36863_i2c_id[] = {
	{VCNL36863_I2C_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static struct of_device_id vcnl36863_match_table[] = {
	{ .compatible = "capella,vcnl36863",},
	{ },
};
#else
#define vcnl36863_match_table NULL
#endif

static struct i2c_driver vcnl36863_driver = {
	.id_table = vcnl36863_i2c_id,
	.probe = vcnl36863_probe,
	.driver = {
		.name = VCNL36863_I2C_NAME,
		.owner = THIS_MODULE,
		.pm = &vcnl36863_pm,    
		.of_match_table = of_match_ptr(vcnl36863_match_table),     
	},
};

static int __init vcnl36863_init(void)
{
	return i2c_add_driver(&vcnl36863_driver);
}

static void __exit vcnl36863_exit(void)
{
	i2c_del_driver(&vcnl36863_driver);
}

module_init(vcnl36863_init);
module_exit(vcnl36863_exit);

MODULE_AUTHOR("Frank Hsieh <Frank.Hsieh@vishay.com>");
MODULE_DESCRIPTION("VCNL36863 Optical Sensor Driver");
MODULE_LICENSE("GPL v2");
