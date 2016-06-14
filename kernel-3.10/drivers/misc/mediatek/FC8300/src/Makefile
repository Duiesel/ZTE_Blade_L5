include $(srctree)/drivers/misc/mediatek/Makefile.custom

OBJS = bbm.o fc8300.o fc8300_bb.o fc8300_i2c.o fc8300_isr.o fc8300_ppi.o fc8300_spi.o fc8300_tun_table.o fc8300b_tun_table.o fci_hal.o fci_hpi.o fci_oal.o fci_ringbuffer.o fci_tun.o
#single
OBJS += fc8300_tun.o fc8300b_tun.o
#2 Diversity
#OBJS += fc8300_tun.2d.o fc8300b_tun.2d.o
#4 Diversity
#OBJS += fc8300_tun.4d.o fc8300b_tun.4d.o

#fail include path
#EXTRA_CFLAGS += -I$(PWD)/../inc

#check ok include path
EXTRA_CFLAGS += -I$(srctree)/drivers/misc/mediatek/FC8300/inc

#ccflags-y += -I$(srctree)/drivers/media/FC8300/inc

#test include path by this include 
#include $(srctree)/drivers/media/FC8300/inc

KDIR = /home/mv4412/src/mv4412-kernel-3.0.15
ODIR = $(PWD)/../

obj-y = isdbt.o
isdbt-objs = $(OBJS)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules
	mv *.ko			$(ODIR)../
	mv *.o			$(ODIR)
	mv isdbt.mod.c		$(ODIR)
	mv Module.symvers	$(ODIR)
	mv modules.order	$(ODIR)
	mv .*.o.cmd		$(ODIR)
	mv .*.ko.cmd		$(ODIR)

clean:
	rm -rf $(ODIR)../*.ko
	rm -rf $(ODIR)*.o
	rm -rf $(ODIR)isdbt.mod.c
	rm -rf $(ODIR)Module.symvers
	rm -rf $(ODIR)modules.order
	rm -rf $(ODIR).*.o.cmd
	rm -rf $(ODIR).*.ko.cmd
