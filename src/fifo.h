#ifndef _FIFO_H_
#define _FIFO_H_

#define	MAX_BUF_LEN	1024

typedef struct TFifo
{
	unsigned char buf[MAX_BUF_LEN];
	volatile unsigned int pread;
	volatile unsigned int pwrite;
}TFifo;

extern void fifoInit(TFifo *fifo);
extern unsigned short  fifoWrite(TFifo *fifo,unsigned char *data,unsigned short len);
extern unsigned short  fifoRead(TFifo *fifo,unsigned char *data,unsigned short len);
extern unsigned short  fifoLen(TFifo *fifo);

//extern int zynq_gpio_dir_out(unsigned int bank, unsigned int pin, unsigned int state);

//extern void zynq_gpio_set_value(unsigned int bank, unsigned int pin, int state);


#endif
