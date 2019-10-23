#ifndef _HEAD_H_
#define _HEAD_H_

unsigned long mem_base;
int major;

static void __iomem *fpga_base;
static void __iomem *CLK;
void gpio_configure(void);

#define GPIO_TO_PIN(bank,gpio) (32*(bank)+(gpio))

#define USER_BUFF_SIZE 				2048
#define GPMC_REVISION 		    	0x00
#define GPMC_SYSCONFIG 		    	0x10
#define GPMC_SYSSTATUS 		    	0x14
#define GPMC_IRQSTATUS 		    	0x18
#define GPMC_IRQENABLE 		    	0x1c
#define GPMC_TIMEOUT_CONTROL 		0x40
#define GPMC_ERR_ADDRESS 	    	0x44
#define GPMC_ERR_TYPE 		    	0x48
#define GPMC_CONFIG 		    	0x50
#define GPMC_STATUS 		    	0x54
#define GPMC_PREFETCH_CONFIG1 		0x1e0
#define GPMC_PREFETCH_CONFIG2 		0x1e4
#define GPMC_PREFETCH_CONTROL 		0x1ec
#define GPMC_PREFETCH_STATUS 		0x1f0
#define GPMC_ECC_CONFIG 	    	0x1f4
#define GPMC_ECC_CONTROL 	    	0x1f8
#define GPMC_ECC_SIZE_CONFIG 		0x1fc
#define GPMC_ECC1_RESULT 	    	0x200
#define GPMC_ECC_BCH_RESULT_0 		0x240
#define GPMC_BASE_ADDR          	0x50000000
#define GPMC_CS                 	1
#define GPMC_CS0                	0x60
#define GPMC_CS_SIZE            	0x30

#define STNOR_GPMC_CONFIG1			0x00001000
#define STNOR_GPMC_CONFIG2			0x000a0d01
#define STNOR_GPMC_CONFIG3			0x00020201
#define STNOR_GPMC_CONFIG4			0x0a050a01
#define STNOR_GPMC_CONFIG5			0x00101515
#define STNOR_GPMC_CONFIG6			0x05000000
#define STNOR_GPMC_CONFIG7			0x00000F41
/*
#define STNOR_GPMC_CONFIG1 0x1000
#define STNOR_GPMC_CONFIG2 0x000f0f01
#define STNOR_GPMC_CONFIG3 0x00020201
#define STNOR_GPMC_CONFIG4 0x10050801
#define STNOR_GPMC_CONFIG5 0x00101212
#define STNOR_GPMC_CONFIG6 0x05000000
#define STNOR_GPMC_CONFIG7 0x00000F41
*/

int workmode=0;//0 mohua 1shengma
int fpga1_reset;
int IOadapter_rx_int;
int RFmoudle_rx_int;

int fpga_gpio,irq_num1,irq_num2;
unsigned long mem_base;
unsigned int tx_count;
unsigned int rx_count;

static void __iomem *fpga_base;
static void __iomem *CLK;

short  audio_data[512];
unsigned int len[9];
unsigned char IOadapter_buff[8][256];

static const u32 gpmc_nor[7] = {
				STNOR_GPMC_CONFIG1,
				STNOR_GPMC_CONFIG2,
				STNOR_GPMC_CONFIG3,
				STNOR_GPMC_CONFIG4,
				STNOR_GPMC_CONFIG5,
				STNOR_GPMC_CONFIG6,
				STNOR_GPMC_CONFIG7
};
struct fpga_dev {
		dev_t devt;
		struct cdev cdev;
		struct semaphore sem_io[9];
		struct semaphore sem_RFmoudle[8];
		struct class *class;
 		unsigned char *user_buff_IO;
 		unsigned char *user_buff_RF;
 		unsigned char *user_buff_audio;
 		unsigned char *user_buff_user;
};
struct fpga_device {
	volatile unsigned short data;		    //low 16bit
	volatile unsigned short dataH;		    //high 16bit
	volatile unsigned short Ctrl;		    //control bit3脢脮脢鹿脛脺 bit2脢脮脰脨露脧 bit1路垄脰脨露脧 bit0脰脨露脧脢鹿脛脺     bit1   0mohua   1codec
	volatile unsigned short St;		        //stutus bit4脢脮fifo脗煤  bit5 脢脮fifo驴脮 bit6 路垄脗煤 bit bit7 路垄驴脮
	volatile unsigned short rx_fifo_count;	
	volatile unsigned short tx_fifo_count;	
	volatile unsigned short ByteNum;	    //1 8bit   2 16bit   4  32bit
	volatile unsigned short reset;			//00ff
};










void gpio_configure(void);




#endif
