KERN_SRC=/home/zyg/kernel/linux-xlnx-master
obj-m := gpio_zynq.o

all:
	make -C $(KERN_SRC) ARCH=arm M=`pwd` modules
clean:
	make -C $(KERN_SRC) ARCH=arm M=`pwd=` clean
