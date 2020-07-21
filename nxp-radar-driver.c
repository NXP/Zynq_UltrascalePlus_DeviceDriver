/*
 *  nxp-radar-driver.c - Linux kernel modules for NXP driver to control the 
 *  custom PL block for radar devices 
 *   
 *
 *  Copyright (C) 2019 NXP. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 * 
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/ioctl.h>
#include <linux/poll.h>
#include <linux/errno.h>
#include <linux/atomic.h>
#include <linux/skbuff.h>
#include <linux/idr.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/sched/signal.h>

#include <linux/gpio/consumer.h>
#include <linux/ioctl.h>
#include <asm/siginfo.h>    
#include <linux/rcupdate.h> 
#include <linux/sched.h>    
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <config/gpiolib.h>
#include <linux/pinctrl/pinctrl.h>

#include "nxp-radar-driver.h"

MODULE_DESCRIPTION("NXP driver to control the custom Zynq FPGA block for radar devices communication and data capture");
MODULE_AUTHOR("Jorge Hermoso Fernandez , NXP Ltd.");
MODULE_VERSION("1.00");
MODULE_ALIAS("nxp-radar-driver");
MODULE_LICENSE("GPL v2");


#define NUM_UUTS  4							//Max number of supported UUTs
#define NUM_ADC_CHANNELS 4						//Number of ADC channels per UUT
#define NUM_DEVICES (1 + NUM_UUTS + (NUM_UUTS * NUM_ADC_CHANNELS))	//Each uut will have a spi device and a frame device (do we need a status device? for interrupts and so on ...)
#define BUF_LEN 5000 							// Max length of the message from the device
#define SUCCESS 0							//Success code
#define FRAME_LINE_LENGTH  32						//We force to read at least in blocks of 32 (4 samples in in 4 channels 2 bytes each) due to the DDR organization
#define MY_ADC_FRAME(I) ((I - NUM_UUTS -1)/NUM_ADC_CHANNELS)		//Macro to find out the corresponding frame device from adc device minor 

#define MAGIC_NUM 0x8A
#define IOCTL_SET_MSG _IOW(MAGIC_NUM, 1, uint32_t)
#define IOCTL_GET_MSG _IOR(MAGIC_NUM, 2, uint32_t)

#define IOCTL_GPIO_SET _IOW(MAGIC_NUM, 3, uint32_t)
#define IOCTL_GPIO_GET _IOR(MAGIC_NUM, 4, uint32_t)

#define IOCTL_LONG_80MSPSCAP_STORE  _IOW(MAGIC_NUM, 5, uint32_t)
#define IOCTL_LONG_CAP_STORE        _IOR(MAGIC_NUM, 6, uint32_t)
#define IOCTL_FUNC_80MSPSCAP_STORE  _IOW(MAGIC_NUM, 7, uint32_t)
#define IOCTL_FUNC_CAP_STORE        _IOR(MAGIC_NUM, 8, uint32_t)

#define GPIO0_BASE_ADDR 0xFF0A0000

#define MAX_GPIO_PINS     17
#define MAX_GPIO_INTPINS   8


/****************** Device structures *********************/
struct radar_fpga_dev {
	struct mutex radar_mutex; 
	struct cdev cdev;
};
struct radar_spi_dev {
	unsigned int *spi_data;
	unsigned long spi_buffer_size; 
	struct mutex radar_mutex; 
	struct cdev cdev;
	unsigned int slaveNum;
};
struct radar_frame_dev {
	uint64_t *frame_data;
	uint32_t frame_address;
	uint32_t frame_size;
	uint8_t  BypassOrDecimated;
};
struct radar_adc_dev{
	struct radar_frame_dev *my_frame;
	struct mutex radar_mutex; 
	struct cdev cdev;
	uint8_t adc_code;
};
/*================== Device Variables ======================== */
static dev_t radar_dev_major;						//Device identifier
static struct radar_spi_dev *radar_spi_devices = NULL;			//Collection of SPI devices
static struct radar_frame_dev *radar_frame_devices = NULL;		//Collection of Frame devices used by ADC devices
static struct radar_adc_dev *radar_adc_devices = NULL;			//Collection of ADC devices
static struct radar_fpga_dev radar_control_dev;				//Single instance of custom PL controller
static struct class *radar_class = NULL;				//Driver class
/******************** Attributes variables *********************/
static int spi_speed = 5;						// in MHz may be changed by sysfs attr 
//static int decimation = 0x2; 						// may be changed by sysfs attr 
/******************* Interrupt codes *********************/
static int SPI_INTERRUPT = 0;						// Place to store OS interrupt code for SPI HW int		
static int FRAME_INTERRUPT = 0;						// Place to store OS interrupt code for SPI HW int	
static int MCUINT_INTERRUPT = 0;					// Place to store OS interrupt code for SPI HW int	
static int ERRORN_INTERRUPT = 0;					// Place to store OS interrupt code for SPI HW int	
//static int CHIRP_START_IO = 0;					// Place to store OS interrupt code for SPI HW int	
//static int CHIRP_START_INTERRUPT = 0;					// Place to store OS interrupt code for SPI HW int	
struct siginfo info;							// Signaling structure to notify user space application
/*=================== Kernel operation variables ===================*/
static unsigned int timestamps[100] = {0};				// Storage of event timestamps
static unsigned int csTimestamps[100] = {0};				// Storage of event timestamps
static int spiCount = 0;
static ktime_t fsync_t, fsync_diff;//, cs_t;
static char msg[BUF_LEN]; 						/* The msg the device will give when asked */
static char *msg_Ptr;
static bool spiInterrupt = false;
static int captureCounter = 0;
static int frameSize = 10000000;
static int frameOffset = 0;
static unsigned int *spiPtr;
static unsigned int *framePtr;
static unsigned int detectedChirps;
//static int cs_irq_set;
static unsigned int gpio_index = 0;
static uint8_t gpioReadPin = 0;
//static uint8_t isIntrSet = 0;
struct device *gdevice = NULL;

typedef struct {
	unsigned int gpio_index;
        unsigned int GPIO_GEN_START_IO;
        unsigned int GPIO_GEN_INTERRUPT;         
	irqreturn_t (*irq_handlers)(int,void*);
        unsigned char KbuildModName[20];
        unsigned int cs_irq_set;
	} GPIO_INIT_STRUCT;

typedef struct {
	  int GpioNum;
	  int val;
} Gpio_Set_t;

                                 
static irqreturn_t gpio_BC1_Mcu_Init_isr(int irq,void* dev_id);
static irqreturn_t gpio_BC2_Mcu_Init_isr(int irq,void* dev_id);
static irqreturn_t gpio_BC3_Mcu_Init_isr(int irq,void* dev_id);
static irqreturn_t gpio_BC4_Mcu_Init_isr(int irq,void* dev_id);
static irqreturn_t gpio_BC1_Err_n_isr(int irq,void* dev_id);
static irqreturn_t gpio_BC2_Err_n_isr(int irq,void* dev_id);
static irqreturn_t gpio_BC3_Err_n_isr(int irq,void* dev_id);
static irqreturn_t gpio_BC4_Err_n_isr(int irq,void* dev_id);
static irqreturn_t gpio_non_interrupt(int irq,void* dev_id);

static void nxp_radar_request_irq(uint8_t Pin ,uint8_t TrigType);


GPIO_INIT_STRUCT gpio_init_struct[MAX_GPIO_PINS] = { 
                                          {0,416,0,gpio_BC1_Mcu_Init_isr,"BC1_Mcu_Init",0},
                                          {1,417,0,gpio_BC2_Mcu_Init_isr,"BC2_Mcu_Init",0},
                                          {2,418,0,gpio_BC3_Mcu_Init_isr,"BC3_Mcu_Init",0},
                                          {3,419,0,gpio_BC4_Mcu_Init_isr,"BC4_Mcu_Init",0},
                                          {4,420,0,gpio_BC1_Err_n_isr,"BC1_Err_n",0},
                                          {5,421,0,gpio_BC2_Err_n_isr,"BC2_Err_n",0},
                                          {6,422,0,gpio_BC3_Err_n_isr,"BC3_Err_n",0},
                                          {7,423,0,gpio_BC4_Err_n_isr,"BC4_Err_n",0},

                                          {8,424,0,gpio_non_interrupt,"gpio_pin_424",0},
                                          {9,425,0,gpio_non_interrupt,"gpio_pin_425",0},
                                          {10,426,0,gpio_non_interrupt,"gpio_pin_426",0},
                                          {11,427,0,gpio_non_interrupt,"gpio_pin_427",0},
                                          {12,428,0,gpio_non_interrupt,"gpio_pin_428",0},
                                          {13,429,0,gpio_non_interrupt,"gpio_pin_429",0},
                                          {14,430,0,gpio_non_interrupt,"gpio_pin_430",0},
                                          {15,431,0,gpio_non_interrupt,"gpio_pin_431",0},
					  {16,432,0,gpio_non_interrupt,"gpio_pin_432",0}
                                        };

typedef enum {
   IRQ_REGISTER = 0,
   IRQ_UNREGISTER
 } IrqRegisterEnum;


static int Gpio_Config_Get(uint32_t *value, uint8_t val);
static long Gpio_Config_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static int Gpio_Config_Set(uint32_t value, uint8_t val);

//static int Gpio_Config_access(unsigned long value, struct device_node * np);
static void nxp_radar_config_gpio(unsigned char index,struct device_node *np);



/******************* Helper Functions ***************************/
int hex2int(char ch){
	if (ch >= '0' && ch <= '9')
        	return ch - '0';
    	if (ch >= 'A' && ch <= 'F')
        	return ch - 'A' + 10;
    	if (ch >= 'a' && ch <= 'f')
        	return ch - 'a' + 10;
    	return 0;
}
/*
 * spi_burstRead: Returns requested Data
 * 
 */
static bool spi_burstRead(unsigned int *spiPtr, unsigned int *spi_data){
	
	int a;
	int numWords; /*Num words is fixed for read operation*/
	int requestedWords;
	int timeout;

	numWords = 2;
	timeout = 0;
	requestedWords = (spi_data[1] >> 24) + 2; /*Get the number of requested words, + 1 for header word + 1 offset (0 = 1 word,1 = 2 words, ...) */
	/*Start SPI transaction*/
	iowrite32(spi_data[0], spiPtr + (0x140/4));
	iowrite32(spi_data[1], spiPtr + (0x150/4));

	while(((ioread32(spiPtr + (0x400/4)) & 0x300) != 0x100) && (!spiInterrupt) && (timeout < 1000000)){ 
		udelay(1); /*This should be disabled if running in script mode? */
		timeout++;
	}
	if(timeout >= 1000000) printk(KERN_ALERT "Timeout on SPI operation\n");

    	for(a = 0; a < requestedWords; a++){
		spi_data[a] = ioread32(spiPtr + (0x200/4));
	}

	sprintf(msg,"00%08x", spi_data[0]);
	for(a = 1; a<requestedWords; a++){
		sprintf(msg + strlen(msg),"%08x", spi_data[a]);
	}
	sprintf(msg + strlen(msg),"\n");
	msg_Ptr = msg;

	return true;
}

/*
 * spi_burstWrite: Writes the data to specified Location
 *
 */
static bool spi_burstWrite(unsigned int *spiPtr, unsigned int *spi_data, int numWords){
	int timeout = 0;
	int a;

	/*Start SPI transaction */
	iowrite32(spi_data[0], spiPtr + (0x180/4));
	for(a = 1; a < (numWords-1); a++){
		iowrite32(spi_data[a], spiPtr + (0x120/4));	/*A read operation should not be here */
	}  
	iowrite32(spi_data[numWords-1], spiPtr + (0x110/4));

	/* Wait for SPI interrupt or status flags */
	while(((ioread32(spiPtr + (0x400/4)) & 0x300) != 0x100) && (!spiInterrupt) && (timeout < 1000000)){ 
		udelay(1); /* This should be disabled if running in script mode? */
		timeout++;
	}
	if(timeout >= 1000000) printk(KERN_ALERT "Timeout on SPI operation\n");

	/* Collect the data and print to file */
    	for(a = 0; a < numWords; a++){
		spi_data[a] = ioread32(spiPtr + (0x200/4));
	}
	
	/* TODO: Prepare file buffer done separately in case needs to be refactored for performance. */
	sprintf(msg,"%08x", spi_data[0]);
	for(a = 1; a<numWords; a++) sprintf(msg + strlen(msg),"%08x", spi_data[a]);
	sprintf(msg + strlen(msg),"\n");
	msg_Ptr = msg;

	return true;
}

/*
 * spi_access: Prepares for SPI transction
 *
 */
static int spi_access(uint32_t *data, int numWords, unsigned int slaveNum){
	uint32_t value;
	uint32_t configuration;				
	value = ioread32(spiPtr);
	configuration = value | (numWords << 16) | ((slaveNum & 0x7) << 8);

	if (spiPtr == NULL) {printk(KERN_ALERT "Error accessing AXI Interface!\n"); return -1;}
	spiInterrupt = false;/* Prepare for transaction start */
	iowrite32(configuration, spiPtr);
	if (((data[0]>>20) & 0x1) == 0x0) spi_burstRead(spiPtr, data);
	else spi_burstWrite(spiPtr, data, numWords);

	return 0;
}

/*
 * spi_Config_access:
 *
 */
static int spi_Config_access(uint32_t *data){
			
	uint32_t configuration = *data ;

	iowrite32(configuration, spiPtr);
	return 0;
}

/*
 * spi_Config_ioctl:
 *
 */
static long spi_Config_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	uint32_t value;
	value = (uint32_t)arg;
	
	
	switch(cmd) {

		case IOCTL_SET_MSG :/* IOCTL_WRITE_FROM_USER: */
			spi_Config_access(&value);
			break;

		case  IOCTL_GET_MSG :/*IOCTL_READ_TO_USER: */
			value = ioread32(spiPtr);
			break;
	}
        return value;
}

/*
 * Gpio_Config_Set: 
 *
 */
static int Gpio_Config_Set(uint32_t value,uint8_t pin){

	uint8_t err;
	uint32_t gpio_from_dt;

	gpio_from_dt = pin + 416; //of_get_named_gpio(np,"gpios",value);

        err = gpio_direction_output(gpio_from_dt, value);

         if (err < 0){
		printk(KERN_INFO "Problems Setting GPIO Direction!");
	  	return -1;
	 }

	gpio_set_value(gpio_from_dt,value);
       
	return 0;
}

/*
 * Gpio_Config_Get:
 *
 */
static int Gpio_Config_Get(uint32_t *value, uint8_t pin){

	uint8_t err;
	uint32_t gpio_from_dt;
	gpio_from_dt = pin + 416 ; //of_get_named_gpio(np,"gpios",*value);
      
      	err = gpio_direction_input(gpio_from_dt);
       	if (err < 0){
	  	printk(KERN_INFO "Problems Setting GPIO Direction!");
	  	return -1;
	}

     	*value = gpio_get_value(gpio_from_dt);	
     	return 0;
}


#if 0 // Capture_Config_ioctl
static long Capture_Config_ioctl(struct file *filp, unsigned int cmd, unsigned long arg){

	uint32_t NumSamples;

	NumSamples = (uint32_t)arg;

	switch(cmd) {

	  case IOCTL_LONG_80MSPSCAP_STORE ://IOCTL_WRITE_FROM_USER:

	  break;

	  case  IOCTL_LONG_CAP_STORE ://IOCTL_READ_TO_USER:

	  break;

	  case IOCTL_FUNC_80MSPSCAP_STORE ://IOCTL_WRITE_FROM_USER:

	  break;

	  case IOCTL_FUNC_CAP_STORE ://IOCTL_READ_TO_USER:

	  break;

	  }


  return 0;
}
#endif // Capture_Config_ioctl

/*
 * Gpio_Config_ioctl:
 *
 */
static long Gpio_Config_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	uint32_t value;
	Gpio_Set_t GpioSetV;
	value = (uint32_t)arg;

	GpioSetV = *((Gpio_Set_t*)arg);

	switch(cmd) {

		case IOCTL_GPIO_SET ://IOCTL_WRITE_FROM_USER:
			//Gpio_Config_Set(value, np);
			break;

		case  IOCTL_GPIO_GET ://IOCTL_READ_TO_USER:
			//Gpio_Config_Get(&value, np);
			copy_to_user ((void*)arg, &value,4);
			break;
	}
	return 0;
}


//====================== Custom PL Control Device Functions ========================
static int device_open(struct inode *inode, struct file *filp){
        int i;
	unsigned int mj = imajor(inode);
	unsigned int mn = iminor(inode);
	
	struct radar_fpga_dev *dev = &radar_control_dev;
	
	if (mj != radar_dev_major || mn != 0)
	{
		printk(KERN_WARNING "[target] " "No device found with minor=%d and major=%d\n", mj, mn);
		return -ENODEV; /* No such device */
	}
	
	/* store a pointer to struct cfake_dev here for other methods */
	filp->private_data = &radar_control_dev; 
	
	if (inode->i_cdev != &dev->cdev)
	{
		printk(KERN_WARNING "[target] open: internal error\n");
		return -ENODEV; /* No such device */
	}

	sprintf(msg, "Detected %d Chirps:\n", detectedChirps);
	for(i = 0; i<detectedChirps; i++){
		sprintf(msg + strlen(msg),"\tChirp %d pulse duration: %d.%03dus\n", i + 1, csTimestamps[i]/1000, csTimestamps[i]%1000 );
	}
	detectedChirps = 0;
	msg_Ptr = msg;

	try_module_get(THIS_MODULE);
	return SUCCESS;
}

static int device_release(struct inode *inode, struct file *file){
  	module_put(THIS_MODULE);
  	return SUCCESS;
}

static ssize_t device_read(struct file *filp, char __user * buffer, size_t length, loff_t * offset){
	//struct radar_spi_dev *dev = (struct radar_spi_dev *)filp->private_data;
	int bytes_read;
	//int i;

	bytes_read = 0;
	if (*offset >= strlen(msg)) return 0; /* EOF */
	if (*offset + length > strlen(msg)) length = strlen(msg) - *offset;
	
	if (*msg_Ptr == 0) return 0;
	
	while (length && *msg_Ptr) {
		put_user(*(msg_Ptr++), buffer++);
		length--;
		bytes_read++;
	}
	*offset += bytes_read;
	return bytes_read;
}

static ssize_t device_write(struct file *filp, const char __user * buffer, size_t length, loff_t * offset){
	struct radar_fpga_dev *dev;
	
	dev = (struct radar_fpga_dev *)filp->private_data;
	
	return length;
}


//====================== SPI Device Functions ========================
static int spi_open(struct inode *inode, struct file *filp){
	unsigned int mj = imajor(inode);
	unsigned int mn = iminor(inode);
	
	struct radar_spi_dev *dev = NULL;
	
	if (mj != radar_dev_major || mn < 1 || mn > NUM_UUTS)
	{
		printk(KERN_WARNING "[target] " "No device found with minor=%d and major=%d\n", mj, mn);
		return -ENODEV; /* No such device */
	}
	
	/* store a pointer to struct cfake_dev here for other methods */
	dev = &radar_spi_devices[mn-1];
	filp->private_data = dev; 
	
	if (inode->i_cdev != &dev->cdev)
	{
		printk(KERN_WARNING "[target] open: internal error\n");
		return -ENODEV; /* No such device */
	}

	dev->slaveNum = mn-1;
	/*
	sprintf(msg, "Since Last Check %d new SPI Interrupts:\n", spiCount);
	for(i = 1; i<spiCount; i++){
		sprintf(msg + strlen(msg),"\tDelay between events %d and %d:%dus\n", i-1, i, timestamps[i]);
	}
	spiCount = 0;
	msg_Ptr = msg;
	*/
	try_module_get(THIS_MODULE);
	return SUCCESS;
}

static int spi_release(struct inode *inode, struct file *file){
  	module_put(THIS_MODULE);
  	return SUCCESS;
}

static ssize_t spi_read(struct file *filp, char __user * buffer, size_t length, loff_t * offset){
	//struct radar_spi_dev *dev = (struct radar_spi_dev *)filp->private_data;
	int bytes_read = 0;
	
	printk("Iam in func %s line no %d\n",__func__, __LINE__);	
	if (*msg_Ptr == 0)
    	return 0;
	
	while (length && *msg_Ptr) {
		put_user(*(msg_Ptr++), buffer++);
		length--;
		bytes_read++;
	}
	return bytes_read;
}

/*When writing to the device we expect a contiguous character stream on hexadecimal format. 
 *The number of words will be as a result of dividing the stream in 32 bit words
 *Extra characters at the end will be discarded
*/
static ssize_t spi_write(struct file *filp, const char __user * buffer, size_t length, loff_t * offset){
	struct radar_spi_dev *dev = (struct radar_spi_dev *)filp->private_data;
	int i = 0;
	//int a = 0;
	int numWords = length/8;
	uint32_t temp = 0;

	dev->spi_data = kmalloc(numWords*sizeof(uint32_t), GFP_KERNEL);	//Decide whether it is necessary to allocate dynamically in every call
	memset(dev->spi_data, 0, numWords * sizeof(uint32_t));

	//Format the received data in the device file:
	//echo 000FF8000100002B > /dev/spi_uut0 	//Read 0x0 0xFFC
	//echo 0410580101000344 > /dev/spi_uut0		//Trigger Chirp Sequence
	for (i = 0; i<(numWords*8); i++) {
		temp = hex2int(*(buffer + i));
		dev->spi_data[i/8] |=  (temp & 0xF) << (28 - (4*(i%8)));
	}

	spi_access(dev->spi_data, numWords, dev->slaveNum);
	kfree(dev->spi_data);
	return length;
}

//============================== FRAME Device Functions ==============================
static int adc_open(struct inode *inode, struct file *filp){
	unsigned int mj = imajor(inode);
	unsigned int mn = iminor(inode);
	
	struct radar_adc_dev *dev = NULL;
	
	if (mj != radar_dev_major || mn <= NUM_UUTS || mn > (NUM_UUTS + (NUM_UUTS * NUM_ADC_CHANNELS)))
	{
		printk(KERN_WARNING "[target] " "No device found with minor=%d and major=%d\n", mj, mn);
		return -ENODEV; /* No such device */
	}
	
	/* store a pointer to struct cfake_dev here for other methods */
	dev = &radar_adc_devices[mn - (NUM_UUTS + 1)];
	filp->private_data = dev; 
	
	if (inode->i_cdev != &dev->cdev)
	{
		printk(KERN_WARNING "[target] open: internal error\n");
		return -ENODEV; /* No such device */
	}
	
		
	return 0;
}

static int adc_release(struct inode *inode, struct file *filp){
	return 0;
}

static ssize_t adc_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos){
	struct radar_adc_dev *dev;
	struct radar_frame_dev *frame_dev;
	ssize_t retval;
	int i;//,j =0,ncount;
	int frameSizeBytes;
	//size_t realCount;
	unsigned short *adc;
	dev = (struct radar_adc_dev *)filp->private_data;
	frame_dev = (struct radar_frame_dev *)dev->my_frame;
        //unsigned long usrcount =0;
	retval = 0;
	i = 0;
     	if(frame_dev-> BypassOrDecimated) {
       		frameSizeBytes = ((frame_dev->frame_size / 4) * 8) / 2; //4 samples in one 64 long long, each of those translates to 8 bytes, but we take a fourth as we only read for one adc at a time
       	}
     	else
     	{
      		frameSizeBytes = ((frame_dev->frame_size / 4) * 8) / 4; //4 samples in one 64 long long, each of those translates to 8 bytes, but we take a fourth as we only read for one adc at a time
     	}

        //printk(KERN_EMERG"First Inside %s f_pos = %d count = %d Frame_Buffer = %d\n ",__FUNCTION__,*f_pos, count,frameSizeBytes);
        //printk(KERN_EMERG"First Inside %s f_pos = %d count = %d Bypass or decimated = %d\n ",__FUNCTION__,*f_pos, count,frame_dev-> BypassOrDecimated);
        

      	if (mutex_lock_killable(&dev->radar_mutex)) {
        	printk(KERN_EMERG"Inside %s before -EINTR\n ",__FUNCTION__);
		return -EINTR;
      	}
	if (*f_pos >= frameSizeBytes) {
         	printk(KERN_EMERG"Samples collected per ADC is = %d\n", (frameSizeBytes >> 1));
         	goto out; /* EOF */
      	}
	if (*f_pos + count > frameSizeBytes) {
        	count = frameSizeBytes - *f_pos;
        	//printk(KERN_EMERG"Inside %s frameSizeBytes = %d count =%d\n ",__FUNCTION__,frameSizeBytes,count); 
     	}		
	//if (count < FRAME_LINE_LENGTH) count = FRAME_LINE_LENGTH;	//We read in the format ADC1(2Bytes) ADC2(2Bytes) ADC3(2Bytes) ADC4(2Bytes), so minimum read is 8 bytes to get full row.
	//if (count%FRAME_LINE_LENGTH > 0) count = ((count / FRAME_LINE_LENGTH) + 1) * FRAME_LINE_LENGTH; //Always ensure we get a multiple of 8 for the read portion


	//frame_dev->frame_data = (uint64_t *)memremap(frame_dev->frame_address, (frame_dev->frame_size *4), MEMREMAP_WB); //Each 64 bit long (8 bytes) contains 4 samples
        //printk(KERN_EMERG "Mem is mapped at  Frame Data = 0%x Frame size = %d", *frame_dev->frame_data,frame_dev->frame_size);

        if(frame_dev-> BypassOrDecimated) {
         	frame_dev->frame_data = (uint64_t *)memremap(frame_dev->frame_address, (frameSizeBytes *2), MEMREMAP_WB); //Each 64 bit long (8 bytes) contains 4 samples
        }
        else
        {
        	 frame_dev->frame_data = (uint64_t *)memremap(frame_dev->frame_address, (frameSizeBytes *4), MEMREMAP_WB); //Each 64 bit long (8 bytes) contains 4 samples
        }
	
        if (frame_dev->frame_data  == NULL) {
		printk(KERN_ALERT "Could not remap frame memory at 0x%x for %d samples!\n", frame_dev->frame_address, frameSizeBytes);
		goto out;
	}
        //printk(KERN_ALERT "Beofre kmalloc\n");
        //unsigned char tempAdc[65536];
	adc = (uint16_t *)vmalloc(frameSizeBytes);
        if(!adc)
        {

        	printk(KERN_ALERT "Vmalloc Failed>>>>>>>>>>>>\n");
          	memunmap(frame_dev->frame_data);
          	goto out;
        }
	if(frame_dev-> BypassOrDecimated) {
		for (i=0; i< frame_dev->frame_size/4; i+=2) {
		    	*(adc + (i *2))     = *(((uint16_t *)(frame_dev->frame_data + i + dev->adc_code)) + 3);		//S1 ADCx
		    	*(adc + (i *2) + 1) = *(((uint16_t *)(frame_dev->frame_data + i + dev->adc_code)) + 2);		//S2 ADCx
	    		*(adc + (i *2) + 2) = *(((uint16_t *)(frame_dev->frame_data + i + dev->adc_code)) + 1);		//S3 ADCx
		    	*(adc + (i *2) + 3) = *(((uint16_t *)(frame_dev->frame_data + i + dev->adc_code)) + 0);		//S4 ADCx
		}
	}

	else {  
		for (i=0; i< frame_dev->frame_size/4; i+=4) {
			*(adc + i)     = *(((uint16_t *)(frame_dev->frame_data + i + dev->adc_code)) + 3);              //S1 ADCx
                	*(adc + i + 1) = *(((uint16_t *)(frame_dev->frame_data + i + dev->adc_code)) + 2);		//S2 ADCx
			*(adc + i + 2) = *(((uint16_t *)(frame_dev->frame_data + i + dev->adc_code)) + 1);		//S3 ADCx
			*(adc + i + 3) = *(((uint16_t *)(frame_dev->frame_data + i + dev->adc_code)) + 0);		//S4 ADCx

                   }
	}


       	if (copy_to_user((void __user*)buf, ((const void *)((uint8_t*)(&adc[0]) + *f_pos)), count) != 0)
	{
                retval = -EFAULT;
                goto out1;
	}
        //printk(KERN_ALERT "END of the copy to user loop\n");

	*f_pos += count;
	retval = count;
	
       // printk(KERN_EMERG"Exit %s f_pos = %d count = %d Frame_Buffer = %d loop count = %d \n ",__FUNCTION__,*f_pos, count,frameSizeBytes,i);
out1:	
	vfree(adc);
	memunmap(frame_dev->frame_data);
out:
	mutex_unlock(&dev->radar_mutex);
	return retval;
}

//************************************* HW IO ISRs ****************************************
static irqreturn_t spi_isr(int irq,void*dev_id)	{   
	fsync_diff = ktime_sub(ktime_get(), fsync_t);
	timestamps[spiCount] = ktime_to_ns(fsync_diff) / 1000;
	spiCount++;
	fsync_t = ktime_get();
	spiInterrupt = true;
	return IRQ_HANDLED;
}

static irqreturn_t data_isr(int irq,void*dev_id) {      
	//unsigned int data;
	uint32_t irqRet;
	int i = 0;
	int captureBuffer = 0;
	int EnableLonCap = 0;
	//int NR_barracuda =0;
        int Enable_80msps;
        //int num_uut =0;
        //int num_active_uut = 0;

	//printk(KERN_ALERT "Data Interrupt Happenned! Capture num %d\n", captureCounter);
	captureBuffer = ioread32(framePtr + (0x2B00/4)); 			//Get the buffer address
	EnableLonCap  = ioread32(framePtr + (0x0300/4)); 
	Enable_80msps = ioread32(framePtr + (0x0C00/4));
        //num_uut       = ioread32(framePtr + (0x0600/4));
        //num_uut       = num_uut & 0x00000007;

        //printk(KERN_ALERT "captureBuffer %d  ture num %d\n", captureCounter);
	//Read the pointer information to know which devices have a frame captured
	switch (captureBuffer){
		case 2:
			frameOffset = 0x4000000;
			break;
		case 3:
			frameOffset = 0x8000000;
			break;
		case 0:
			frameOffset = 0xC000000;
			break;
		default:
			frameOffset = 0x0;
			break;
	}

    	/*switch (num_uut){
		case 0:
			num_active_uut = 0x1;
			break;
		case 1:
			num_active_uut = 0x4;
			break;
		case 3:
			num_active_uut = 0x3;
			break;
                case 5:
			num_active_uut = 0x2;
			break;
		default:
			num_active_uut = 0x1;
			break;
	}*/

	frameSize =  ioread32(framePtr + (0x1000/4));		//Get value from FPGA register;
        printk(KERN_ALERT "Samples Received in the Frame is  %d \n",frameSize);
        //printk(KERN_ALERT "Frame offset  %d \n",frameOffset);
	irqRet = ioread32(framePtr + (0x2C00/4));
	if (irqRet != 0x3) {
		printk(KERN_ALERT "Error During Frame Capture. IRQ: %d!!!\n", irqRet);
		goto out;
	}
        

	if(((EnableLonCap & 0x00000002) >> 1) == 1) {
		for (i=0; i<NUM_UUTS;i++) {
			radar_frame_devices[i].frame_address = 0x40000000 + (0x10000000 * i);
			radar_frame_devices[i].BypassOrDecimated = ((Enable_80msps & 0x00000008) >> 3);
                        radar_frame_devices[i].frame_size = frameSize;
		}
	}
	else {

		for (i=0; i<NUM_UUTS;i++) {
			radar_frame_devices[i].frame_address = 0x40000000 + (0x10000000 * i) + frameOffset;
                        radar_frame_devices[i].BypassOrDecimated = ((Enable_80msps & 0x00000008) >> 3);
                       radar_frame_devices[i].frame_size = frameSize;
		}
	}

out:
	captureCounter++;
  
	return IRQ_HANDLED;
}


/*
 * gpio_BC1_Mcu_Init_isr: Propagate signal to user space
 *
 */

static irqreturn_t gpio_BC1_Mcu_Init_isr(int irq,void*dev_id)	{      
	struct task_struct *t;

	if(pid!=0){
		info.si_int = 0; 	//real time signals may have 32 bits of data. MCU int: 0, Erro_n: 1
		rcu_read_lock();
		t = pid_task(find_pid_ns(pid, &init_pid_ns), PIDTYPE_PID);	// find the task with that pid
		if (t != NULL) {
			if (send_sig_info(SIGUSR1, &info, t) < 0) printk("send_sig_info error\n");
		} 
		else printk("pid_task error\n"); 
		rcu_read_unlock();      
	}
	return IRQ_HANDLED;
}


static irqreturn_t gpio_BC2_Mcu_Init_isr(int irq,void *dev_id)	{
	struct task_struct *t;

	printk(KERN_ALERT"Inside %s Interrupt received\n",__FUNCTION__);
	if(pid!=0){
		info.si_int = 1; 	//real time signals may have 32 bits of data. MCU int: 0, Erro_n: 1
		rcu_read_lock();
		t = pid_task(find_pid_ns(pid, &init_pid_ns), PIDTYPE_PID);	// find the task with that pid
		if (t != NULL) {
			if (send_sig_info(SIGUSR1, &info, t) < 0) printk("send_sig_info error\n");
		} 
	}
	return IRQ_HANDLED;
}

static irqreturn_t gpio_BC3_Mcu_Init_isr(int irq,void *dev_id)	{
	struct task_struct *t;

	printk(KERN_ALERT"Inside %s Interrupt received\n",__FUNCTION__);
	if(pid!=0){
		info.si_int = 2; 	//real time signals may have 32 bits of data. MCU int: 0, Erro_n: 1
		rcu_read_lock();
		t = pid_task(find_pid_ns(pid, &init_pid_ns), PIDTYPE_PID);	// find the task with that pid
		if (t != NULL) {
			if (send_sig_info(SIGUSR1, &info, t) < 0) printk("send_sig_info error\n");
		} 
		else printk("pid_task error\n"); 
		rcu_read_unlock();      
	}
	return IRQ_HANDLED;
}


static irqreturn_t gpio_BC4_Mcu_Init_isr(int irq,void *dev_id)	{
	struct task_struct *t;

	printk(KERN_ALERT"Inside %s Interrupt received\n",__FUNCTION__);
	if(pid!=0){
		info.si_int = 3; 	//real time signals may have 32 bits of data. MCU int: 0, Erro_n: 1
		rcu_read_lock();
		t = pid_task(find_pid_ns(pid, &init_pid_ns), PIDTYPE_PID);	// find the task with that pid
		if (t != NULL) {
			if (send_sig_info(SIGUSR1, &info, t) < 0) printk("send_sig_info error\n");
		} 
		else printk("pid_task error\n"); 
		rcu_read_unlock();      
	}
	return IRQ_HANDLED;
}

static irqreturn_t gpio_BC1_Err_n_isr(int irq,void *dev_id)	{
	struct task_struct *t;

	printk(KERN_ALERT"Inside %s Interrupt received\n",__FUNCTION__);
	if(pid!=0){
		info.si_int = 4; 	//real time signals may have 32 bits of data. MCU int: 0, Erro_n: 1
		rcu_read_lock();
		t = pid_task(find_pid_ns(pid, &init_pid_ns), PIDTYPE_PID);	// find the task with that pid
		if (t != NULL) {
			if (send_sig_info(SIGUSR1, &info, t) < 0) printk("send_sig_info error\n");
		} 
		else printk("pid_task error\n"); 
		rcu_read_unlock();      
	}
	return IRQ_HANDLED;
}

static irqreturn_t gpio_BC2_Err_n_isr(int irq,void *dev_id)	{
	struct task_struct *t;

	printk(KERN_ALERT"Inside %s Interrupt received\n",__FUNCTION__);
	if(pid!=0){
		info.si_int = 5; 	//real time signals may have 32 bits of data. MCU int: 0, Erro_n: 1
		rcu_read_lock();
		t = pid_task(find_pid_ns(pid, &init_pid_ns), PIDTYPE_PID);	// find the task with that pid
		if (t != NULL) {
			if (send_sig_info(SIGUSR1, &info, t) < 0) printk("send_sig_info error\n");
		} 
		else printk("pid_task error\n"); 
		rcu_read_unlock();      
	}
	return IRQ_HANDLED;
}

static irqreturn_t gpio_BC3_Err_n_isr(int irq,void *dev_id)	{
	struct task_struct *t;

	printk(KERN_ALERT"Inside %s Interrupt received\n",__FUNCTION__);
	if(pid!=0){
		info.si_int = 6; 	//real time signals may have 32 bits of data. MCU int: 0, Erro_n: 1
		rcu_read_lock();
		t = pid_task(find_pid_ns(pid, &init_pid_ns), PIDTYPE_PID);	// find the task with that pid
		if (t != NULL) {
			if (send_sig_info(SIGUSR1, &info, t) < 0) printk("send_sig_info error\n");
		} 
		else printk("pid_task error\n"); 
		rcu_read_unlock();      
	}
	return IRQ_HANDLED;
}

static irqreturn_t gpio_BC4_Err_n_isr(int irq,void *dev_id)	{
	struct task_struct *t;

	printk(KERN_ALERT"Inside %s Interrupt received\n",__FUNCTION__);
	if(pid!=0){
		info.si_int = 7; 	//real time signals may have 32 bits of data. MCU int: 0, Erro_n: 1
		rcu_read_lock();
		t = pid_task(find_pid_ns(pid, &init_pid_ns), PIDTYPE_PID);	// find the task with that pid
		if (t != NULL) {
			if (send_sig_info(SIGUSR1, &info, t) < 0) printk("send_sig_info error\n");
		} 
		else printk("pid_task error\n"); 
		rcu_read_unlock();      
	}
	return IRQ_HANDLED;
}


static irqreturn_t gpio_non_interrupt(int irq,void* dev_id)	{
      	return 0;	
}


#if 0
static void gpio_init_direction(void){
  //int count = 0;
  //unsigned long *gpio0;
  int error = -1;
  //int status =0;
  unsigned int index =0;//,i,j;
  //int gpioflag = 0;
 

 for (index =0;index < MAX_GPIO_PINS;index++){

  //if(!isIntrSet) {
//#ifndef GPIO_INTERRUPT_CAPABLE        
    if(gpio_init_struct[index].GPIO_GEN_START_IO - 338 <= 85)
     {
        error = gpio_direction_input(gpio_init_struct[index].GPIO_GEN_START_IO);
        
        //printk(KERN_INFO "Pin %d direction set input\n ",gpio_init_struct[index].GPIO_GEN_START_IO - 338);
        
        if (error < 0){
           printk(KERN_INFO "Problems configuring the gpio direction !");
           //goto fail;
           } 
      }
//}

  else {
 //#else   
   if (gpio_init_struct[index].GPIO_GEN_START_IO - 338 >= 87)
       { 
        error = gpio_direction_output(gpio_init_struct[index].GPIO_GEN_START_IO,0);
      
        //printk(KERN_INFO "Pin %d direction set output \n ",gpio_init_struct[index].GPIO_GEN_START_IO - 338);
       
        if (error < 0){
           printk(KERN_INFO "Problems configuring the gpio direction !");
           //goto fail;
           }

       }   
}
// #endif 
   } 

  /*Delay */ 
 }
#endif //gpio_init_direction
static void nxp_radar_config_gpio(unsigned char index, struct device_node *np)
{
 
   	int err;
  
   	if(!gpio_is_valid(gpio_init_struct[index].GPIO_GEN_START_IO)){
       		printk(KERN_INFO "test print: Invalid GPIO");
    	}
   
   	//printk(KERN_INFO "test print: The Assigned gpio is %d", gpio_init_struct[index].GPIO_GEN_START_IO);

   	err = gpio_request(gpio_init_struct[index].GPIO_GEN_START_IO, gpio_init_struct[index].KbuildModName);
    	if (err < 0){
        	printk(KERN_INFO "In Init Problems setting the gpio!");
    	}
}

static void nxp_radar_request_irq(uint8_t Pin ,uint8_t TrigType){

   	int index = 0;
   	int err;

   	index = Pin;

  	if(gpio_init_struct[index].cs_irq_set == 1)
   	{
     		printk(KERN_INFO "Can't set IRQ. IRQ Number %d is already set for the device Pin %d\n" ,gpio_init_struct[index].GPIO_GEN_INTERRUPT,gpio_init_struct[index].GPIO_GEN_START_IO - 338);
     		return;
   	} 

   	err = gpio_direction_input(gpio_init_struct[index].GPIO_GEN_START_IO);
   	if (err < 0){
     		printk(KERN_INFO "Problems configuring the gpio direction !");
    	}
   	//printk(KERN_INFO "The pin %d is configured as input ! \n",gpio_init_struct[index].GPIO_GEN_START_IO - 338);
   	gpio_init_struct[index].GPIO_GEN_INTERRUPT = gpio_to_irq(gpio_init_struct[index].GPIO_GEN_START_IO);
   	//printk(KERN_INFO "The pin has got IRQ Number %d \n" ,gpio_init_struct[index].GPIO_GEN_INTERRUPT);
   	if (gpio_init_struct[index].GPIO_GEN_INTERRUPT < 0){
     		printk(KERN_INFO "Problems configuring the gpio to irq!");
		// goto fail;
    	}
   	err = request_irq(gpio_init_struct[index].GPIO_GEN_INTERRUPT, gpio_init_struct[index].irq_handlers, TrigType,gpio_init_struct[index].KbuildModName, NULL); 
   	if (err < 0) {
     		printk(KERN_ALERT "%s: request_irq %d for module %s failed with %d\n",__func__, gpio_init_struct[index].GPIO_GEN_INTERRUPT, gpio_init_struct[index].KbuildModName, err);
     		//goto fail;
	}
	
   	/*disable_irq_nosync(gpio_init_struct[index].GPIO_GEN_INTERRUPT);
    	enable_irq(gpio_init_struct[index].GPIO_GEN_INTERRUPT); */
    	//printk(KERN_INFO "test print: GPIO Interrupt was succesfully registered for Pin: %d\n",gpio_init_struct[index].GPIO_GEN_START_IO - 338);
    	gpio_init_struct[index].cs_irq_set = 1; 
}


/*============================ File Operation Definitions =====================================*/
struct file_operations radar_control_fops = {
  	.read = device_read,
  	.write = device_write,
  	.unlocked_ioctl = Gpio_Config_ioctl,
  	.open = device_open,
  	.release = device_release, /* a.k.a. close */
};

struct file_operations radar_spi_fops = {
  	.read = spi_read,
  	.write = spi_write,
  	.unlocked_ioctl = spi_Config_ioctl,
  	.open = spi_open,
  	.release = spi_release, /* a.k.a. close */
};

struct file_operations radar_adc_fops = {
  	.read = adc_read,
  	.write = NULL,
  	.open = adc_open,
  	.release = adc_release, /* a.k.a. close */
};

//*************************** SysFs Attribute definitions ***************************************
static ssize_t spi_speed_show(struct device *dev, struct device_attribute *attr, char *buf)
{
        return snprintf(buf, PAGE_SIZE, "%d\n", spi_speed);
}

static ssize_t spi_speed_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        sscanf(buf, "%d", &spi_speed);
        return PAGE_SIZE;
}

/*
 * pid_store:
 *
 */
static ssize_t pid_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        sscanf(buf, "%d", &pid);
	//printk(KERN_INFO "Registered Interrupt Listener at PID: %d\n", pid);
        return PAGE_SIZE;
}
/*
 * capture_show:
 *
 */
static ssize_t capture_show(struct device *dev, struct device_attribute *attr, char *buf)
{
        return snprintf(buf, PAGE_SIZE, "Not implemented\n");
}

/*
 * capture_store:
 *
 */
static ssize_t capture_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	int numChirps;
	int numSamples;

	numChirps = 0;
	numSamples = 0;
        
	if (count > 0) {
		ret = sscanf(buf, "%d %d", &numChirps, &numSamples);
		if (ret >=1)
		{ //Check that either the number of chirps or the number of samples is passed
			//printk("Iam inside function %s numSamples =%#x ret =%#x numChirps =%d\n",__func__,numSamples,ret,numChirps);
			iowrite32(0x3, framePtr + (0x2D00/4)); 			//Reset Interrupts
			//iowrite32(0x1, framePtr + (0x0C00/4)); 			//Configure Polarity
			iowrite32(0x0, framePtr + (0x0300/4)); 			//Disable Test Mode
			iowrite32(numChirps, framePtr + (0x2100/4)); 	        //Nchirps
                        iowrite32(numSamples, framePtr + (0x0800/4)); 	        //Nchirps
		        //iowrite32(0x1, framePtr + (0x0400/4)); 			//Trigger
		 }
		 else 
		 {
			printk(KERN_ALERT "Incorrect Frame Triggering\n");
		 }
		return PAGE_SIZE;
	}
     	else
      	{
         	printk(KERN_ALERT "Copy From user Failed \n");
          	return -1;
      	}
}


/*
 * func_gpio_capture_show:
 *
 */
static ssize_t func_gpio_capture_show(struct device *dev, struct device_attribute *attr, char *buf){

  	//int ret;
  	uint32_t value;
  	uint8_t pin = gpioReadPin ;

  	//printk(KERN_ALERT"Inside the function %s \n",__func__);

  	//printk(KERN_ALERT"Recieved from User the pin  %d \n",pin);

  	Gpio_Config_Get(&value,pin);

  	//printk(KERN_ALERT"Read value from the Pin %d is %d \n",pin,value);

  	sprintf(buf,"%d",value);
    	return 0;
}
/*
 * func_gpio_capture_store:
 *
 */
static ssize_t func_gpio_capture_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {

    	int ret;
    	uint32_t value;
    	uint8_t pin;
   
    	//printk(KERN_ALERT"Inside function %s\n",__func__);

    	ret = sscanf(buf, "%d %d", &pin, &value);

    	//printk(KERN_ALERT"Got GPIO pin %d to set val %d From user \n",pin,value);
    
    	Gpio_Config_Set(value, pin);  
    	return 0;     
}

/*
 * func_gpio_pinread_store:
 *
 */
static ssize_t func_gpio_pinread_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){

  	//uint32_t value;
 
  	//printk(KERN_ALERT"Inside the function %s \n",__func__);

  	sscanf(buf,"%d",&gpioReadPin);
  	//printk(KERN_ALERT"Recieved from User the pin  %d \n",gpioReadPin);
     	return 0;
}

/*
 * func_intr_request_store:
 *
 */
static ssize_t func_intr_request_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){

  	//uint32_t err;
  	IrqRegisterEnum IrqReg = 0;
  	//int index = 0;
  	int pinIrq = 0;
  	int TrigType = 0;
  	//printk(KERN_ALERT"Inside the function %s \n",__func__);

  	sscanf(buf,"%d %d %d",&pinIrq,&IrqReg,&TrigType);

  	//printk(KERN_ALERT"Request from user space the Irq pin %d Register (0-Resister 1-Unregister) Type is %d  and Trig Type is %d  \n",pinIrq,IrqReg,TrigType);
  
  
  	if(TrigType == 0)
    		TrigType = IRQF_TRIGGER_RISING;
  	else
    		TrigType = IRQF_TRIGGER_FALLING;

  	if(IrqReg == IRQ_REGISTER)
  	{
    
     		printk(KERN_ALERT "Registered gpio interrupt for the pin %d \n",gpio_init_struct[pinIrq].GPIO_GEN_START_IO - 338);
    
    		if(pinIrq > 7)
     		{
       			printk(KERN_ALERT"Invalid Pin Number. Pin %d Not Interrupt capable \n",pinIrq);
       			return 0;
      		}

    		nxp_radar_request_irq(pinIrq,TrigType);
 
    		gpio_init_struct[pinIrq].cs_irq_set = 1;

   	}

  	else if(IrqReg == IRQ_UNREGISTER)
   	{
     
   		//printk(KERN_ALERT"Recieved Interrept Un-register request for the pins  %d and Regiter Value is %d \n",pinIrq,IrqReg);
   
     		if (gpio_init_struct[pinIrq].cs_irq_set) 
        	{
	  		free_irq(gpio_init_struct[pinIrq].GPIO_GEN_INTERRUPT, NULL);
          		gpio_init_struct[pinIrq].cs_irq_set = 0;
	  		printk(KERN_ALERT "Unregistered gpio interrupt for the pin %d \n",gpio_init_struct[pinIrq].GPIO_GEN_START_IO - 338);
        	} 
   	}
   	else
    	{
      		printk(KERN_ALERT " Invalid IRQ Option \n");
    	}  
    	return 0;
}


static ssize_t func_intr_request_show(struct device *dev, struct device_attribute *attr, char *buf){
     	return 0;
}

/*
 * func_80_msps_cap_show:
 *
 */
static ssize_t func_80_msps_cap_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  	return snprintf(buf, PAGE_SIZE, "functional_80msps_capture_show Not implemented\n");
}
/*
 * func_80_msps_cap_store:
 *
 */
static ssize_t func_80_msps_cap_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	int numSamples =0;
        int numChirps;

	if (count > 0) {
		ret = sscanf(buf, "%d %d", &numChirps, &numSamples);
		if (ret >=1){ //Check that the number of samples is passed
			//printk("Iam inside function %s numSamples =%#x ret =%#x numChirps =%d\n",__func__,numSamples,ret,numChirps);
			iowrite32(0x3, framePtr + (0x2D00/4)); 	       //Reset Interrupts
			//iowrite32(0x9, framePtr + (0x0C00/4)); 	        //Configure Polarity and packat data format
			iowrite32(0x0, framePtr + (0x0300/4)); 		//Disbale TestMode
			iowrite32(numChirps, framePtr + (0x2100/4)); 	//Nchirps
                        iowrite32(numSamples, framePtr + (0x0800/4)); 	//Nchirps
			//iowrite32(0x1, framePtr + (0x0400/4)); 	        //Trigger
                       
		}
		else {
			printk(KERN_ALERT "Incorrect Frame Triggering\n");
		}
          return PAGE_SIZE;
	}
       else{
          	printk(KERN_ALERT "Copy From user Failed \n");
          	return -1;
        }
}
/*
 * long_80_msps_cap_show:
 *
 */
static ssize_t long_80_msps_cap_show(struct device *dev, struct device_attribute *attr,  char *buf)
{
	//printk("Iam inside function %s\n",__func__);
	return snprintf(buf, PAGE_SIZE, "long_80msps_capture_show Not implemented\n");
}
/*
 * long_80_msps_cap_store:
 *
 */
static ssize_t long_80_msps_cap_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	int numSamples = 0;
        int numChirps;

	if (count > 0) {
		ret = sscanf(buf, "%d %d", &numChirps, &numSamples);
               	if (ret >=1){ //Check that number of samples is passed
			//printk("Iam inside function %s numSamples =%#x ret =%#x numChirps =%d\n",__func__,numSamples,ret,numChirps);
			iowrite32(0x3, framePtr + (0x2D00/4)); 			//Reset Interrupts
			//iowrite32(0x9, framePtr + (0x0C00/4)); 		//Configure Polarity and packat data format
			iowrite32(0x2, framePtr + (0x0300/4)); 			//Enable Long capture
                        iowrite32(0x1, framePtr + (0x2100/4)); 	                //Nchirps set to 1
		        iowrite32(numSamples, framePtr + (0x800/4)); 	        //Nsamples
			//iowrite32(0x1, framePtr + (0x0400/4)); 	        //Trigger
			}

		else {
			printk(KERN_ALERT "Incorrect Frame Triggering\n");
		}
		
	}
        else{
          	printk(KERN_ALERT "Copy From user Failed \n");
          	return -1;
	}
	return PAGE_SIZE;
}
/*
 * long_cap_show:
 *
 */
static ssize_t long_cap_show(struct device *dev, struct device_attribute *attr,  char *buf)
{
	//printk("Iam inside function %s\n",__func__);
	return snprintf(buf, PAGE_SIZE, "long_decimation_capture_show Not implemented\n");
}
/*
 * long_cap_store:
 *
 */
static ssize_t long_cap_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	
	int ret;
	int numSamples;
        int numChirps;
        numSamples = 0;
        

	if (count > 0) {
		ret = sscanf(buf, "%d %d", &numChirps, &numSamples);
		if (ret >=1){ //Check that number of samples is passed
			//printk("Iam inside function %s numSamples =%#x ret =%#x numChirps =%d\n",__func__,numSamples,ret,numChirps);
                        //printk("Iam inside function %s numSamples =%d ret =%d\n",__func__,numSamples,ret);
			iowrite32(0x3, framePtr + (0x2D00/4)); 			//Reset Interrupts
			//iowrite32(0x01, framePtr + (0x0C00/4));
			iowrite32(0x2, framePtr + (0x0300/4)); 	        //Enable Long capture
			iowrite32(numSamples, framePtr + (0x800/4)); 	//Nsamples
                        iowrite32(numChirps, framePtr + (0x2100/4)); 	//Nsamples
                        //iowrite32(0x1, framePtr + (0x2100/4)); 	        //Nchirps set to 1
			//iowrite32(0x1, framePtr + (0x0400/4)); 		//Trigger
			//printk("inside %s\n",__func__);
		
                 	}
		else {
			printk(KERN_ALERT "Copy From user Failed \n");
		}
             	return PAGE_SIZE;
      	}
    	else{
          	printk(KERN_ALERT "Copy From user Failed \n");
          	return -1;
         }
}


static DEVICE_ATTR(spi_speed, S_IRUSR | S_IWUSR, spi_speed_show, spi_speed_store);
//static DEVICE_ATTR(chirp_start, S_IRUSR | S_IWUSR, io_cs_show, io_cs_store);			//Select function for chirp Start (input or output)
static DEVICE_ATTR(register_pid, S_IWUSR, NULL, pid_store);							//Write pid for process to be notified for interrupt
static DEVICE_ATTR(capture, S_IWUSR | S_IRUGO, capture_show, capture_store);		//Trigger Capture and show results from capture
static DEVICE_ATTR(func_80_msps_cap, S_IWUSR | S_IRUGO, func_80_msps_cap_show, func_80_msps_cap_store);	//functional Capture and show results from long capture
static DEVICE_ATTR(long_80_msps_cap, S_IWUSR | S_IRUGO, long_80_msps_cap_show, long_80_msps_cap_store);	//long Capture 80msps and show results from long 80msps capture
static DEVICE_ATTR(long_cap, S_IWUSR | S_IRUGO, long_cap_show, long_cap_store);	//long decimation Capture and show results from long decimation capture
static DEVICE_ATTR(func_gpio_capture,S_IWUSR | S_IRUGO,func_gpio_capture_show,func_gpio_capture_store);
static DEVICE_ATTR(func_gpio_pinread,S_IWUSR,NULL,func_gpio_pinread_store);
static DEVICE_ATTR(func_intr_request,S_IWUSR | S_IRUGO,func_intr_request_show,func_intr_request_store);

/* ================================= Device Constructors/Destructors ==================================================*/
/*
 * radar_construct_fpga_device:
 *
 */
static int radar_construct_fpga_device(struct radar_fpga_dev *dev, int minor, struct class *class)
{
	int err = 0;
	dev_t devno = MKDEV(radar_dev_major, minor);
	struct device *device = NULL;
	
	BUG_ON(dev == NULL || class == NULL);

	mutex_init(&dev->radar_mutex);
	
	cdev_init(&dev->cdev, &radar_control_fops);
	dev->cdev.owner = THIS_MODULE;
	err = cdev_add(&dev->cdev, devno, 1);
	if (err)
	{
		printk(KERN_WARNING "[target] Error %d while trying to add %s%d", err, "Custom PL control device %d", minor); 
		return err;
	}
	device = device_create(class, NULL, devno, NULL, "nxp_control");
	if (IS_ERR(device)) {
		err = PTR_ERR(device);
		printk(KERN_WARNING "[target] Error %d while trying to create %s%d", err,  "Custom PL control device %d", minor-1);
		cdev_del(&dev->cdev);
		return err;
	}

       gdevice = device;
	/* device attributes on sysfs */
	err = device_create_file(device, &dev_attr_spi_speed);
	if (err < 0) {
			printk(KERN_ERR "Cant create device attribute\n");
	}
/*	err = device_create_file(device, &dev_attr_chirp_start);
	if (err < 0) {
			printk(KERN_ERR "Cant create device attribute\n");
	}
 */
	err = device_create_file(device, &dev_attr_register_pid);
	if (err < 0) {
			printk(KERN_ERR "Cant create device attribute\n");
	}
	err = device_create_file(device, &dev_attr_capture);
	if (err < 0) {
			printk(KERN_ERR "Cant create device attribute\n");
	}
	err = device_create_file(device, &dev_attr_func_80_msps_cap);
	if (err < 0) {
			printk(KERN_ERR "Cant create device attribute functional_80msps\n");
	}
	err = device_create_file(device, &dev_attr_long_80_msps_cap);
	if (err < 0) {
			printk(KERN_ERR "Cant create device attribute long_capture_80msps\n");
	}
	err = device_create_file(device, &dev_attr_long_cap);
	if (err < 0) {
			printk(KERN_ERR "Cant create device attribute long_capture_decimation\n");
	}
        err = device_create_file(device, &dev_attr_func_gpio_capture);
	if (err < 0) {
			printk(KERN_ERR "Cant create device attribute func_gpio_capture\n");
	}
        err = device_create_file(device, &dev_attr_func_gpio_pinread);
	if (err < 0) {
			printk(KERN_ERR "Cant create device attribute func_gpio_pinread\n");
	}
        err = device_create_file(device, &dev_attr_func_intr_request);
	if (err < 0) {
			printk(KERN_ERR "Cant create device attribute func_intr_request\n");
	}
  
	return 0;
}



/*
 * radar_construct_spi_device:
 *
 */
static int radar_construct_spi_device(struct radar_spi_dev *dev, int minor, struct class *class)
{
	int err = 0;
	dev_t devno = MKDEV(radar_dev_major, minor);
	struct device *device = NULL;
	
	BUG_ON(dev == NULL || class == NULL);

	/* Memory is to be allocated when the device is opened the first time */
	dev->spi_data = NULL;     
	dev->spi_buffer_size = 100 * sizeof(uint32_t);
	mutex_init(&dev->radar_mutex);
	cdev_init(&dev->cdev, &radar_spi_fops);
	dev->cdev.owner = THIS_MODULE;
	err = cdev_add(&dev->cdev, devno, 1);
	if (err)
	{
		printk(KERN_WARNING "[target] Error %d while trying to add %s%d", err, "spi_uut%d", minor - 1); 
		return err;
	}

	device = device_create(class, NULL, devno, NULL, "spi_uut%d", minor - 1);
	if (IS_ERR(device)) {
		err = PTR_ERR(device);
		printk(KERN_WARNING "[target] Error %d while trying to create %s%d", err, "spi_uut%d", minor - 1);
		cdev_del(&dev->cdev);
		return err;
	}

	return 0;
}


/*
 * radar_construct_adc_device:
 *
 */
static int radar_construct_adc_device(struct radar_adc_dev *dev, int minor, struct class *class)
{
	int err = 0;
	dev_t devno = MKDEV(radar_dev_major, minor);
	struct device *device = NULL;
	
	BUG_ON(dev == NULL || class == NULL);
	if((minor - NUM_UUTS -1)%NUM_ADC_CHANNELS == 0){	//The frame struct is shared by every NUM_ADC_CHANNELS
		radar_frame_devices[MY_ADC_FRAME(minor)].frame_data = NULL;     /* Memory is to be allocated when the device is opened the first time */
		radar_frame_devices[MY_ADC_FRAME(minor)].frame_size = 0;
	}
	dev->my_frame = &radar_frame_devices[MY_ADC_FRAME(minor)];	//Assign the correspondent shared structure
	dev->adc_code = (minor - NUM_UUTS -1)%NUM_ADC_CHANNELS; 	//Store channel number relative to uut
	mutex_init(&dev->radar_mutex);
	cdev_init(&dev->cdev, &radar_adc_fops);
	dev->cdev.owner = THIS_MODULE;
	err = cdev_add(&dev->cdev, devno, 1);
	if (err)
	{
		printk(KERN_WARNING "[target] Error %d while trying to add %s%d", err, "adc%d", (minor-NUM_UUTS)); 
		return err;
	}
	device = device_create(class, NULL, devno, NULL, "adc%d", (minor-NUM_UUTS));
	if (IS_ERR(device)) {
		err = PTR_ERR(device);
		printk(KERN_WARNING "[target] Error %d while trying to create %s%d", err, "adc%d", (minor-NUM_UUTS));
		cdev_del(&dev->cdev);
		return err;
	}
	return 0;
}

/*
 * radar_destroy_fpga_device:
 *
 */
static void radar_destroy_fpga_device(struct radar_fpga_dev *dev, int minor, struct class *class)
{
	BUG_ON(dev == NULL || class == NULL);
	device_destroy(class, MKDEV(radar_dev_major, minor));
	cdev_del(&dev->cdev);
	mutex_destroy(&dev->radar_mutex);
	return;
}

/*
 * radar_destroy_spi_device:
 *
 */
static void radar_destroy_spi_device(struct radar_spi_dev *dev, int minor,	struct class *class)
{
	BUG_ON(dev == NULL || class == NULL);
	device_destroy(class, MKDEV(radar_dev_major, minor));
	cdev_del(&dev->cdev);
	kfree(dev->spi_data);
	mutex_destroy(&dev->radar_mutex);
	return;
}
/*
 * radar_destroy_adc_device:
 *
 */
static void radar_destroy_adc_device(struct radar_adc_dev *dev, int minor,struct class *class)
{
	struct radar_frame_dev *frame_dev;

	BUG_ON(dev == NULL || class == NULL);
	device_destroy(class, MKDEV(radar_dev_major, minor));
	if((minor - NUM_UUTS -1)%NUM_ADC_CHANNELS == 0){		//The frame struct is shared by every NUM_ADC_CHANNELS
		frame_dev = dev->my_frame;
		kfree(frame_dev->frame_data);
	}
	cdev_del(&dev->cdev);
	mutex_destroy(&dev->radar_mutex);
	return;
}

static void radar_cleanup_module(int devices_to_destroy)
{
	int i;
	
	/* Get rid of character devices (if any exist) */
	if (devices_to_destroy == 0) goto out;
	radar_destroy_fpga_device(&radar_control_dev, 0, radar_class);
	if (devices_to_destroy > 1){
		for(i = 1; i<= min(NUM_UUTS, devices_to_destroy - 1); i++){
			radar_destroy_spi_device(&radar_spi_devices[i - 1], i, radar_class);
		}
		kfree(radar_spi_devices);
	}
	if (devices_to_destroy > NUM_UUTS + 1){
		for(i = NUM_UUTS + 1; i<min(NUM_DEVICES, devices_to_destroy); i++){
			radar_destroy_adc_device(&radar_adc_devices[i-(NUM_UUTS + 1)], i, radar_class);
		}
		kfree(radar_frame_devices);
		kfree(radar_adc_devices);
	}

out:	
	if (radar_class) class_destroy(radar_class);

	/* [NB] cfake_cleanup_module is never called if alloc_chrdev_region()
	 * has failed. */
	unregister_chrdev_region(MKDEV(radar_dev_major, 0), NUM_DEVICES);
	return;
}



//**************************** Module Init and Release ***************************************
static int __init nxp_pl_module_init(void)
{
        unsigned int firstReg;
	unsigned int lastReg;
	struct device_node *np;
       
	int err = 0;
	dev_t dev = 0;
	int devices_to_destroy = 0;
	int i = 0;
        
	
	detectedChirps = 0;
        pid = 0;	/* Reset the pid so it can be checked later during interrupts */

	/* Get a range of minor numbers (starting with 0) to work with */
	err = alloc_chrdev_region(&dev, 0, NUM_DEVICES, KBUILD_MODNAME);
	if (err < 0) {
		printk(KERN_WARNING "[target] alloc_chrdev_region() failed\n");
		return err;
	}
	radar_dev_major = MAJOR(dev);

	/* Create device class (before allocation of the array of devices) */
	radar_class = class_create(THIS_MODULE, KBUILD_MODNAME);
	if (IS_ERR(radar_class)) {
		err = PTR_ERR(radar_class);
		goto fail;
	}

	/* Allocate the array of spi devices */
	radar_spi_devices = (struct radar_spi_dev *)kzalloc( NUM_UUTS * sizeof(struct radar_spi_dev), GFP_KERNEL);
	if (radar_spi_devices == NULL) {
		err = -ENOMEM;
		goto fail;
	}
	/* Allocate the array of frame devices, 
         * there are only NUM_UUTS frame devices even though 
         * there are NUM_UUTS * NUM_ADC_CHANNELS devices 
         */
	radar_frame_devices = (struct radar_frame_dev *)kzalloc( NUM_UUTS * sizeof(struct radar_frame_dev), GFP_KERNEL);
	if (radar_frame_devices == NULL) {
		err = -ENOMEM;
		goto fail;
	}

	/* Allocate the array of frame devices, 
         * there are only NUM_UUTS frame devices even though 
         * there are NUM_UUTS * NUM_ADC_CHANNELS devices 
         */
	radar_adc_devices = (struct radar_adc_dev *)kzalloc( NUM_UUTS * NUM_ADC_CHANNELS * sizeof(struct radar_adc_dev), GFP_KERNEL);
	if (radar_adc_devices == NULL) {
		err = -ENOMEM;
		goto fail;
	}
	
	/* Construct devices: 0 is generic controller, 
         * 1 to NUM_UUTS is spi_devices, NUM_UUTS + 1 to NUM_UUTS + 16 
         * is adc channels
         */
	err = radar_construct_fpga_device(&radar_control_dev, 0, radar_class);
	if (err) {
		devices_to_destroy = 1;
		goto fail;
	}

	for (i = 1; i <= NUM_UUTS; i++) {
		err = radar_construct_spi_device(&radar_spi_devices[i-1], i, radar_class);
		if (err) {
			devices_to_destroy = i + 1;
			goto fail;
		}
	}

	for (i = NUM_UUTS + 1; i < NUM_DEVICES; i++){
		err = radar_construct_adc_device(&radar_adc_devices[i-(NUM_UUTS + 1)], i, radar_class);
		if (err) {
			devices_to_destroy = i + 1;
			goto fail;
		}
	}

	/* Obtain Interrupt Numbers */
	np = of_find_node_by_name(NULL,"nxp-radar-driver"); 
	if(np == NULL)
	{
		printk(KERN_ERR "Device Node is NULL \n");
		return -1;
	}
	SPI_INTERRUPT = irq_of_parse_and_map(np,0);
	FRAME_INTERRUPT = irq_of_parse_and_map(np,1);
	MCUINT_INTERRUPT = irq_of_parse_and_map(np,2);
	ERRORN_INTERRUPT = irq_of_parse_and_map(np,3);

	/* Register Interrupt for SPI driver */
	err = request_irq(SPI_INTERRUPT, spi_isr, 0, KBUILD_MODNAME ".spi", NULL); 
	if (err < 0) {
		printk(KERN_ALERT "%s: request_irq %d for module %s failed with %d\n",__func__, SPI_INTERRUPT, KBUILD_MODNAME ".spi", err);
		goto fail;
	}
	else { 
		printk(KERN_INFO "SPI Interrupt was succesfully registered: %d\n", SPI_INTERRUPT);
	}

	/* Register Interrupt for Frame Grabber (frame ready) */
	err = request_irq(FRAME_INTERRUPT, data_isr, 0, KBUILD_MODNAME ".frame", NULL);
	if (err < 0) {
		printk(KERN_ALERT "%s: request_irq %d for module %s failed with %d\n",__func__, FRAME_INTERRUPT, KBUILD_MODNAME ".frame", err);
		goto fail;
	}
	else { 
		printk(KERN_INFO "Frame Interrupt was succesfully registered: %d\n", FRAME_INTERRUPT);
	}
	

	/* Map SPI AXI addresses! */
	firstReg = 0x80015000;
	lastReg = 0x80015400;
	spiPtr =  ioremap(firstReg, (lastReg-firstReg) + 4 ); 
	if (!spiPtr) {
                printk(KERN_ERR "ioremap() failed\n");
                err = -ENOMEM;
                goto lastfail;
        }

	firstReg = 0x80010000;
	lastReg =  0x80014D00;
	framePtr =  ioremap(firstReg, (lastReg-firstReg) + 4 );
	if (!framePtr) {
                printk(KERN_ERR "ioremap() failed\n");
		iounmap(spiPtr);
                err = -ENOMEM;
                goto lastfail;
        }

	/*Prepare structure to intialize Interrupt handler 
         */
        /* This is bit of a trickery: SI_QUEUE is normally used 
         * by sigqueue from user space, and kernel space should 
         * use SI_KERNEL, but if SI_KERNEL is used the real_time data  
         * is not delivered to the user space signal handler function. 
         */        
	memset(&info, 0, sizeof(struct siginfo));
	info.si_signo = SIGUSR1;
	info.si_code = SI_QUEUE;

	for (gpio_index=0;gpio_index < (MAX_GPIO_PINS);gpio_index++) 
		nxp_radar_config_gpio(gpio_index,np);
       
        
       	//Initialize timer
	fsync_t = ktime_get();
	return 0; /* success */

lastfail:
	free_irq(SPI_INTERRUPT, NULL);
	free_irq(FRAME_INTERRUPT, NULL);
fail:
	radar_cleanup_module(devices_to_destroy);
	return err;
}


static void __exit nxp_pl_module_exit(void)
{
  
	for (gpio_index=0;gpio_index < (MAX_GPIO_INTPINS);gpio_index++) { 
   		if(gpio_init_struct[gpio_index].cs_irq_set) 
      		{
			free_irq(gpio_init_struct[gpio_index].GPIO_GEN_INTERRUPT, NULL);
			printk(KERN_INFO "unregistered gpio interrupt %d \n",gpio_init_struct[gpio_index].GPIO_GEN_INTERRUPT);
      		}
  	}

   	for (gpio_index=0;gpio_index < (MAX_GPIO_PINS);gpio_index++) {
		gpio_free(gpio_init_struct[gpio_index].GPIO_GEN_START_IO);
 	}

    	iounmap(framePtr);
    	iounmap(spiPtr);
    	free_irq(SPI_INTERRUPT, NULL);
    	free_irq(FRAME_INTERRUPT, NULL);
    	radar_cleanup_module(NUM_DEVICES);
}

module_init(nxp_pl_module_init);
module_exit(nxp_pl_module_exit);

