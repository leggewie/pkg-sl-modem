/*****************************************************************************/

/*
 *
 *   Copyright (c) 2002, Smart Link Ltd.
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *       1. Redistributions of source code must retain the above copyright
 *          notice, this list of conditions and the following disclaimer.
 *       2. Redistributions in binary form must reproduce the above
 *          copyright notice, this list of conditions and the following
 *          disclaimer in the documentation and/or other materials provided
 *          with the distribution.
 *       3. Neither the name of the Smart Link Ltd. nor the names of its
 *          contributors may be used to endorse or promote products derived
 *          from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 *
 *	amrmo_init.c  -  Smart Link Soft Modem amr,pci driver initialization.
 *
 *	Author: Sasha K (sashak@smlink.com)
 *
 */

/*****************************************************************************/

#include <linux/config.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/devfs_fs_kernel.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
#define OLD_KERNEL 1
#endif

#ifdef OLD_KERNEL
#define iminor(i) MINOR((i)->i_rdev)
#else
#include <linux/device.h>
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,13)
#define class_simple_device_add(class, dev, addr, name, i)
#define class_simple_device_remove(dev)
#define class_simple_create(module, name) (struct class_simple *)(name)
#define class_simple_destroy(class)
#else
#define pci_match_id(tbl, dev) pci_match_device(tbl, dev)
#endif

#include <modem_defs.h>

#define AMRMO_MODULE_NAME "slamr"
#define AMRMO_MAJOR 242

#define AMRMO_DBG(fmt...) if (debug > 0) { printk(KERN_DEBUG AMRMO_MODULE_NAME ": " fmt) ; }

#define MAXNUM 4


/* modem cards ids list */

/* Intel */
/* #define PCI_VENDOR_ID_INTEL                     0x8086 */
#define PCI_DEVICE_ID_ICHAA			0x2416
#define PCI_DEVICE_ID_ICHAB			0x2426
#define PCI_DEVICE_ID_BANISTER			0x7196
#define PCI_DEVICE_ID_ICH2			0x2446
#define PCI_DEVICE_ID_ICH3			0x2486
#define PCI_DEVICE_ID_ICH4			0x24c6
#define PCI_DEVICE_ID_ICH5			0x24d6

#define PCI_VENDOR_ID_STANDARD_MICROSYSTEM	0x1055	
#define PCI_DEVICE_ID_STANDARD_MICROSYSTEM	0x9178

/* AMD */
/* #define PCI_VENDOR_ID_AMD			0x1022 */
#define PCI_DEVICE_ID_AMD_ACLINK		0x7446

/* NVidia */
/* #define PCI_VENDOR_ID_NVIDIA			0x10DE */
#define PCI_DEVICE_ID_MCP			0x01C1

/* VIA */
/* #define PCI_VENDOR_ID_VIA			0x1106 */
#define PCI_DEVICE_ID_VIA			0x3068

/* SiS */
#define PCI_VENDOR_ID_SIS			0x1039
#define PCI_DEVICE_ID_SIS630			0x7013
#define PCI_DEVICE_ID_SIS960			0x7018

/* Avance */
#define PCI_VENDOR_ID_ALS300p			0x4005 // PCI_VENDOR_ID_AVANCE
#define PCI_DEVICE_ID_ALS300p			0x0308

/* ALi */
#define PCI_VENDOR_ID_ALI			0x10B9 // PCI_VENDOR_ID_AL
#define PCI_DEVICE_ID_ALI5450			0x5450
#define PCI_DEVICE_ID_ALI5451			0x5451
#define PCI_DEVICE_ID_ALI5457			0x5457
#define PCI_DEVICE_ID_ALI5459			0x5459
#define PCI_DEVICE_ID_ALI545A			0x545A

/* Philips */		
/* #define PCI_VENDOR_ID_PHILIPS			0x1131 */
#define PCI_DEVICE_ID_UCB1500			0x3400

/* RealTek */
/* #define PCI_VENDOR_ID_REALTEK			0x10EC */
#define PCI_DEVICE_ID_8101			0x8197

#define PCI_VENDOR_ID_SMARTLINK                 0x2000		
#define PCI_VENDOR_ID_SMARTLINK_1		0x163c
#define PCI_VENDOR_ID_SMARTLINK_2		0x10a5
#define PCI_VENDOR_ID_SMARTLINK_3		0x2003
#define PCI_DEVICE_ID_SL2800			0x2800
#define PCI_DEVICE_ID_SL1900			0x3052
#define PCI_DEVICE_ID_ND92XPA                   0x8800 /* ND92XPA */

#define PCI_VENDOR_ID_PCTEL                     0x134d
#define PCI_DEVICE_ID_HSP1688                   0x2189

enum {
	ALS300_CARD = 1,
	VIA3058_CARD,
	ALI5450_CARD,
	ALI5451_CARD,
	SL1800_CARD,
	SIS630_CARD,
	SIS960_CARD,
	ICH_CARD,
	ICH4_CARD,
	RTL8197_CARD,
	NVIDIA_CARD,
	SL1500_CARD,
	SL1801_CARD,
	SL1900_CARD,
	SL2800_CARD
};


struct amrmo_struct {
	unsigned id;
	const char *name;
	struct pci_dev *pci_dev;
	unsigned int irq;
	unsigned long iobase1;
	unsigned long iobase2;
	void *memaddr;
	unsigned long memlen;
	void *card;
	unsigned num;
	unsigned used;
	unsigned started;
	unsigned status;
	spinlock_t lock;
	wait_queue_head_t wait;
	char ibuf[2048];
	char obuf[2048];
};


/* extern prototypes */
asmlinkage extern void amrmo_card_interrupt(void *card);
asmlinkage extern int  amrmo_card_start(void *card);
asmlinkage extern int  amrmo_card_stop(void *card);
asmlinkage extern int  amrmo_card_ctl(void *card,unsigned cmd,unsigned long);
asmlinkage extern int  amrmo_card_read(void *card,char *buf,int n);
asmlinkage extern int  amrmo_card_write(void *card,char *buf,int n);
asmlinkage extern int  amrmo_card_enable(void *card, void *modem);
asmlinkage extern int  amrmo_card_disable(void *card);
asmlinkage extern void *amrmo_card_create(int card_id,void *amrmo);
asmlinkage extern void  amrmo_card_delete(void *card);


/* internal data */

static int debug = 0;

static const char *card_names[] = {
	0,
	"ALS300+", // ALS300_CARD
	"VIA3058",
	"ALI1535",
	"ALI1535",
	"SL1800",
	"SiS630",
	"SiS960",
	"ICH",
	"ICH4",
	"RealTek8101",
	"NvidiaMCP",
	"SL1500",
	"ALI545A",
	"SL1900",
	"SL2800"
};


static struct pci_device_id amrmo_pci_tbl [] __devinitdata = {
	{PCI_VENDOR_ID_SMARTLINK_1, PCI_DEVICE_ID_SL1900,  /* 163c:3052 */
	 PCI_ANY_ID, PCI_ANY_ID, 0, 0, SL1900_CARD},
	{PCI_VENDOR_ID_SMARTLINK_2, PCI_DEVICE_ID_SL1900,  /* 10a5:3052 */
	 PCI_ANY_ID, PCI_ANY_ID, 0, 0, SL1900_CARD},
	{PCI_VENDOR_ID_PCTEL, PCI_DEVICE_ID_HSP1688,       /* 134d:2189 */
	 PCI_ANY_ID, PCI_ANY_ID, 0, 0, SL1900_CARD},
	{PCI_VENDOR_ID_PHILIPS,  PCI_DEVICE_ID_UCB1500,    /* 1131:3400 */
	 PCI_ANY_ID, PCI_ANY_ID, 0, 0, SL1500_CARD},
	{PCI_VENDOR_ID_REALTEK,  PCI_DEVICE_ID_8101,       /* 10ec:8197 */
	 PCI_ANY_ID, PCI_ANY_ID, 0, 0, RTL8197_CARD},
	{PCI_VENDOR_ID_ALI, PCI_DEVICE_ID_ALI5459,         /* 10b9:5459 */
	 PCI_ANY_ID, PCI_ANY_ID, 0, 0, SL1800_CARD},
	{PCI_VENDOR_ID_SMARTLINK_1, PCI_DEVICE_ID_ALI5459, /* 163c:5459 */
	 PCI_ANY_ID, PCI_ANY_ID, 0, 0, SL1800_CARD},
	{PCI_VENDOR_ID_SMARTLINK_2, PCI_DEVICE_ID_ALI5459, /* 10a5:5459 */
	 PCI_ANY_ID, PCI_ANY_ID, 0, 0, SL1800_CARD},
	//{PCI_VENDOR_ID_ALS300p, PCI_DEVICE_ID_ALS300p,     /* 4005:0308 */
	// PCI_ANY_ID, PCI_ANY_ID, 0, 0, ALS300_CARD},
	{PCI_VENDOR_ID_STANDARD_MICROSYSTEM,
		PCI_DEVICE_ID_STANDARD_MICROSYSTEM,        /* 1055:9178 */
	 PCI_ANY_ID, PCI_ANY_ID, 0, 0, ICH_CARD},
	{PCI_VENDOR_ID_AMD, PCI_DEVICE_ID_AMD_ACLINK,      /* 1022:7446 */
	 PCI_ANY_ID, PCI_ANY_ID, 0, 0, ICH_CARD},
	{PCI_VENDOR_ID_NVIDIA, PCI_DEVICE_ID_MCP,          /* 10de:01c1 */
	 PCI_ANY_ID, PCI_ANY_ID, 0, 0, NVIDIA_CARD},
	{PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_ICHAA,         /* 8086:2416 */
	 PCI_ANY_ID, PCI_ANY_ID, 0, 0, ICH_CARD},
	{PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_ICHAB,         /* 8086:2426 */
	 PCI_ANY_ID, PCI_ANY_ID, 0, 0, ICH_CARD},
	{PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_BANISTER,      /* 8086:7196 */
	 PCI_ANY_ID, PCI_ANY_ID, 0, 0, ICH_CARD},
	{PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_ICH2,          /* 8086:2446 */
	 PCI_ANY_ID, PCI_ANY_ID, 0, 0, ICH_CARD},
	{PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_ICH3,          /* 8086:2486 */
	 PCI_ANY_ID, PCI_ANY_ID, 0, 0, ICH_CARD},
       	{PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_ICH4,          /* 8086:24c6 */
	 PCI_ANY_ID, PCI_ANY_ID, 0, 0, ICH4_CARD},
       	{PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_ICH5,          /* 8086:24d6 */
	 PCI_ANY_ID, PCI_ANY_ID, 0, 0, ICH4_CARD},
       	{PCI_VENDOR_ID_INTEL, 0x266d,                      /* ICH6 */
	 PCI_ANY_ID, PCI_ANY_ID, 0, 0, ICH4_CARD},
       	{PCI_VENDOR_ID_INTEL, 0x27dd,                      /* ICH7 */
	 PCI_ANY_ID, PCI_ANY_ID, 0, 0, ICH4_CARD},
	{PCI_VENDOR_ID_VIA, PCI_DEVICE_ID_VIA,             /* 1106:3068 */
	 PCI_ANY_ID, PCI_ANY_ID, 0, 0, VIA3058_CARD},
	{PCI_VENDOR_ID_SIS, PCI_DEVICE_ID_SIS630,          /* 1039:7013 */
	 PCI_ANY_ID, PCI_ANY_ID, PCI_CLASS_COMMUNICATION_MODEM<<8, 0xff00, SIS630_CARD},
	{PCI_VENDOR_ID_SIS, PCI_DEVICE_ID_SIS960,          /* 1039:7018 */
	 PCI_ANY_ID, PCI_ANY_ID, PCI_CLASS_COMMUNICATION_MODEM<<8, 0xff00, SIS960_CARD},
	//{PCI_VENDOR_ID_ALI, PCI_DEVICE_ID_ALI5450,
	// PCI_ANY_ID, PCI_ANY_ID, 0, 0, ALI5450_CARD},
	//{PCI_VENDOR_ID_ALI, PCI_DEVICE_ID_ALI5451,
	// PCI_ANY_ID, PCI_ANY_ID, 0, 0, ALI5451_CARD},
	{PCI_VENDOR_ID_ALI, PCI_DEVICE_ID_ALI5457,         /* 10b9:5457 */
	 PCI_ANY_ID, PCI_ANY_ID, 0, 0, SL1800_CARD},
	{PCI_VENDOR_ID_ALI, PCI_DEVICE_ID_ALI545A,         /* 10b9:545a */
	 PCI_ANY_ID, PCI_ANY_ID, 0, 0, SL1801_CARD},
	{PCI_VENDOR_ID_SMARTLINK, PCI_DEVICE_ID_SL2800,    /* 2000:2800 */
	 PCI_ANY_ID, PCI_ANY_ID, 0, 0, SL2800_CARD},
	{PCI_VENDOR_ID_SMARTLINK_3, PCI_DEVICE_ID_ND92XPA, /* 2003:8800 */
	 PCI_ANY_ID, PCI_ANY_ID, 0, 0, SL2800_CARD},
	{0,}
};

MODULE_DEVICE_TABLE (pci, amrmo_pci_tbl);


static struct amrmo_struct *amrmo_table[MAXNUM] = {};
#ifndef OLD_KERNEL
static struct class_simple *amrmo_class;
#endif

/*
 *    debug stuff
 *
 */

unsigned amrmo_debug_level = 0;

static int amrmo_vprintf(const char *fmt, va_list args)
{
	static char temp[4096];
	struct timeval tv;
	unsigned long len;
	char *p;
	int ret;
	if (!fmt || !(len = strlen(fmt)))
		return -1;
	do_gettimeofday(&tv);
	while (*fmt == '\n' || *fmt == '\r' || *fmt == '\t' || *fmt == ' ')
		fmt++;
	p = temp;
	len = sprintf(p, "<%c%03ld.%06ld> ",
		      in_interrupt() ? 'i' : 'p',
		      (tv.tv_sec % 1000), tv.tv_usec);
	p += len;
	ret = vsprintf(p, fmt, args);
	len += ret;
	if(temp[len-1] != '\n') {
		temp[len++] = '\n';
		temp[len]   = '\0';
	}
	return printk(KERN_DEBUG "%s",temp);
}


asmlinkage int amrmo_debug_printf(const char *fmt, ...)
{
        int ret = 0;
	va_list args;
	if(debug <= 0) return 0;
	va_start(args, fmt);
	ret = amrmo_vprintf(fmt,args);
	va_end(args);
        return ret;
}


/*
 *
 *    amrmo and char dev
 */

asmlinkage void amrmo_update_status(void *data,unsigned stat)
{
	struct amrmo_struct *amrmo = (struct amrmo_struct *)data;
	unsigned long flags;
	spin_lock_irqsave(&amrmo->lock,flags);
	amrmo->status |= stat;
	wake_up(&amrmo->wait);
	spin_unlock_irqrestore(&amrmo->lock,flags);
}

static ssize_t amrmo_read(struct file *file, char *buffer, size_t count, loff_t *ppos)
{
	struct amrmo_struct *amrmo = (struct amrmo_struct *)file->private_data;
	int ret;
	//AMRMO_DBG("amrmo_read...\n");
	if(!access_ok(VERIFY_READ, buffer, count))
		return -EFAULT;
	if (count > sizeof(amrmo->ibuf))
		count = sizeof(amrmo->ibuf);
	ret = amrmo_card_read(amrmo->card,amrmo->ibuf,count);
	if(copy_to_user(buffer,amrmo->ibuf,ret))
		return -EFAULT;
	return ret;
}

static ssize_t amrmo_write(struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
	struct amrmo_struct *amrmo = (struct amrmo_struct *)file->private_data;
	int ret;
	//AMRMO_DBG("amrmo_write...\n");
	if(!access_ok(VERIFY_WRITE, buffer, count))
		return -EFAULT;
	if (count > sizeof(amrmo->obuf))
		count = sizeof(amrmo->obuf);
	if(copy_from_user(amrmo->obuf,buffer,count))
		return -EFAULT;
	ret = amrmo_card_write(amrmo->card,amrmo->obuf,count);
	return ret;
}


static unsigned int amrmo_poll(struct file *file, poll_table *wait)
{
	struct amrmo_struct *amrmo = (struct amrmo_struct *)file->private_data;
        unsigned long flags;
        unsigned int mask = 0;
	poll_wait(file,&amrmo->wait,wait);
	spin_lock_irqsave(&amrmo->lock,flags);
	if(amrmo->status & MDMSTAT_ERROR)
		mask |= POLLERR;
	if(amrmo->status & MDMSTAT_RING)
		mask |= POLLPRI;
	if(amrmo->status & MDMSTAT_DATA) {
		mask |= POLLIN | POLLRDNORM;
		amrmo->status &= ~MDMSTAT_DATA;
	}
	spin_unlock_irqrestore(&amrmo->lock,flags);
        return mask;
}



static int amrmo_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct amrmo_struct *amrmo = (struct amrmo_struct *)file->private_data;
        unsigned long flags;
	int ret;
	unsigned stat;
	AMRMO_DBG("amrmo_ioctl: cmd %x, arg %lx...\n",cmd,arg);
        switch (cmd) 
        {
	case MDMCTL_GETSTAT:
		spin_lock_irqsave(&amrmo->lock,flags);		
		stat = amrmo->status;
		amrmo->status = 0;
		spin_unlock_irqrestore(&amrmo->lock,flags);
                if (put_user(stat, (unsigned *) arg))
                        return -EFAULT;
		return 0;
	case MDMCTL_START:
		ret = amrmo_card_start(amrmo->card);
		if(!ret) amrmo->started = 1;
		return ret;
	case MDMCTL_STOP:
		ret = amrmo_card_stop(amrmo->card);
		if(!ret) amrmo->started = 0;
		return 0;
	default:
		return amrmo_card_ctl(amrmo->card,cmd,arg);
        }
        return -EINVAL;
}

static int amrmo_open(struct inode *inode, struct file *file)
{
	struct amrmo_struct *amrmo;
	unsigned minor = iminor(inode);
	if(minor > MAXNUM)
		return -ENODEV;
	amrmo = amrmo_table[minor];
	if(!amrmo)
		return -ENODEV;
	AMRMO_DBG("amrmo_open: %d...\n",amrmo->num);
	if(amrmo->used)	// FIXME: atomic or locked is needed
		return -EBUSY;
	amrmo->used++;
	file->private_data = amrmo;
#ifdef OLD_KERNEL
	MOD_INC_USE_COUNT;
#endif
	init_waitqueue_head(&amrmo->wait);
	return 0;
}

static int amrmo_release(struct inode *inode, struct file *file)
{
	struct amrmo_struct *amrmo = file->private_data;
	AMRMO_DBG("amrmo_release...\n");
	if(amrmo->started &&
	   !amrmo_card_stop(amrmo->card))
		amrmo->started = 0;
	amrmo->used--;
#ifdef OLD_KERNEL
        MOD_DEC_USE_COUNT;	
#endif
	return 0;
}

static struct file_operations amrmo_fops = {
        .owner =   THIS_MODULE,
        .llseek =  no_llseek,
        .read =    amrmo_read,
        .write =   amrmo_write,
        .poll =    amrmo_poll,
        .ioctl =   amrmo_ioctl,
        .open =    amrmo_open,
        .release = amrmo_release,
};


/*
 *  PCI stuff
 *
 */


#ifdef OLD_KERNEL
static void amrmo_pci_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
        struct amrmo_struct *amrmo = (struct amrmo_struct *)dev_id;
	amrmo_card_interrupt(amrmo->card);
}
#else
static irqreturn_t amrmo_pci_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
        struct amrmo_struct *amrmo = (struct amrmo_struct *)dev_id;
	amrmo_card_interrupt(amrmo->card);
	return IRQ_HANDLED; 
}
#endif


static int __init amrmo_pci_probe(struct pci_dev *pci_dev, const struct pci_device_id *pci_id)
{
	struct amrmo_struct *amrmo;
	unsigned long mem_start, mem_len = 0;
	int i, ret = 0;

	printk(KERN_INFO "slamr: probe %04x:%04x %s card...\n",
	       pci_id->vendor,pci_id->device,
	       card_names[pci_id->driver_data]);

	if ( pci_enable_device(pci_dev) ||
	     pci_dev->irq == 0 )
		return -ENODEV;

	amrmo = kmalloc(sizeof(*amrmo), GFP_KERNEL);
        if (!amrmo) {
		pci_disable_device(pci_dev);
                return -ENOMEM;
	}
        memset(amrmo, 0, sizeof(*amrmo));

	amrmo->id = pci_id->driver_data;
	amrmo->name = card_names[amrmo->id];
	amrmo->pci_dev = pci_dev;
	amrmo->irq     = pci_dev->irq;
	if (pci_resource_flags(pci_dev,0)&IORESOURCE_MEM) {
		mem_start = pci_resource_start(pci_dev, 0);
		mem_len   = pci_resource_len(pci_dev, 0);
		amrmo->iobase1 = pci_resource_start(pci_dev, 1);
	}
	else if (pci_resource_flags(pci_dev,1)&IORESOURCE_MEM) {
		amrmo->iobase1 = pci_resource_start(pci_dev, 0);
		mem_start = pci_resource_start(pci_dev, 1);
		mem_len   = pci_resource_len(pci_dev, 1);
	}
	else {
		amrmo->iobase1 = pci_resource_start(pci_dev, 0);
		amrmo->iobase2 = pci_resource_start(pci_dev, 1);
		mem_start = 0;
		mem_len   = 0;
	}

	spin_lock_init(&amrmo->lock);
	init_waitqueue_head(&amrmo->wait);

	amrmo->card = amrmo_card_create(pci_id->driver_data,amrmo);
	if (!amrmo->card) {
		printk(KERN_ERR "slamr: cannot create card.\n");
		pci_disable_device(pci_dev);
		kfree(amrmo);
                return -ENOMEM;
	}

	pci_set_master(pci_dev);

	ret = pci_request_regions(pci_dev,(char*)amrmo->name);
	if(ret) {
		printk(KERN_ERR "slamr: failed request regions.\n");
		amrmo_card_delete(amrmo->card);
		pci_disable_device(pci_dev);
		kfree(amrmo);
		return ret;
	}
	if(mem_len) {
		amrmo->memaddr = ioremap(mem_start,mem_len);
		if(!amrmo->memaddr) {
			printk(KERN_ERR "slamr: failed request_irq\n");
			pci_release_regions(pci_dev);
			amrmo_card_delete(amrmo->card);
			pci_disable_device(pci_dev);
			kfree(amrmo);
			return -EIO;
		}
	}

        ret = request_irq(amrmo->irq, &amrmo_pci_interrupt,SA_SHIRQ,
			  amrmo->name,amrmo);
	if(ret) {
		printk(KERN_ERR "slamr: failed request_irq\n");
		if(amrmo->memaddr)
			iounmap(amrmo->memaddr);
		pci_release_regions(pci_dev);
		amrmo_card_delete(amrmo->card);
		pci_disable_device(pci_dev);
		kfree(amrmo);
		return ret;
	}

	ret = amrmo_card_enable(amrmo->card,NULL);
	if (ret) {
		printk(KERN_ERR "slamr: cannot init card.\n");
		free_irq(amrmo->irq, amrmo);
		if(amrmo->memaddr)
			iounmap(amrmo->memaddr);
		pci_release_regions(pci_dev);
		amrmo_card_delete(amrmo->card);
		pci_disable_device(pci_dev);
		kfree(amrmo);
		return ret;
	}

	for(i = 0 ; i < MAXNUM ; i++) {
		if(amrmo_table[i] == NULL) {
			amrmo->num = i;
			amrmo_table[i] = amrmo;
			break;
		}
	}
	if( i == MAXNUM ) {
		ret = -EBUSY;
		goto error_out;
	}

	AMRMO_DBG("amrmo_pci_probe: %d amrmo is %p. data %p, io %lx, %lx.\n",
		  amrmo->num, amrmo,amrmo->card,amrmo->iobase1,amrmo->iobase2);

	printk(KERN_INFO "slamr: slamr%d is %s card.\n",
	       amrmo->num, card_names[pci_id->driver_data]);

	pci_set_drvdata(pci_dev, amrmo);
#ifdef OLD_KERNEL
#ifdef CONFIG_DEVFS_FS
	{
		char buf[8];
		sprintf(buf, "slamr%d", i);
		devfs_register (NULL, buf, DEVFS_FL_DEFAULT, AMRMO_MAJOR, i,
				S_IFCHR|S_IRUSR|S_IWUSR, &amrmo_fops, NULL);
	}
#endif
#else
	class_simple_device_add(amrmo_class, MKDEV(AMRMO_MAJOR, i), NULL, "slamr%d", i);
	devfs_mk_cdev(MKDEV(AMRMO_MAJOR,i), S_IFCHR|S_IRUSR|S_IWUSR, "slamr%d", i);
#endif
	return 0;

 error_out:
	free_irq(amrmo->irq, amrmo);
	if(amrmo->memaddr)
		iounmap(amrmo->memaddr);
	pci_release_regions(pci_dev);
	amrmo_card_delete(amrmo->card);
	pci_disable_device(pci_dev);
	kfree(amrmo);
	return ret;
}

static void __exit amrmo_pci_remove(struct pci_dev *pci_dev)
{
	struct amrmo_struct *amrmo = pci_get_drvdata(pci_dev);
	AMRMO_DBG("amrmo: remove %p...\n", amrmo);
#ifdef OLD_KERNEL
#ifdef CONFIG_DEVFS_FS
	{
		char buf[8];
		void * handle;
		sprintf(buf, "slamr%d", amrmo->num);
		handle = devfs_find_handle (NULL, buf, AMRMO_MAJOR, amrmo->num,
					    DEVFS_SPECIAL_CHR, 0);
		devfs_unregister (handle);
	}
#endif
#else
	class_simple_device_remove(MKDEV(AMRMO_MAJOR, amrmo->num));
	devfs_remove("slamr%d", amrmo->num);
#endif
	amrmo_table[amrmo->num] = NULL;
	amrmo_card_disable(amrmo->card);
        free_irq(amrmo->irq, amrmo);
	if(amrmo->memaddr)
		iounmap(amrmo->memaddr);
	pci_release_regions(pci_dev);
	amrmo_card_delete(amrmo->card);
	pci_disable_device(pci_dev);
	pci_set_drvdata(pci_dev, NULL);
	kfree(amrmo);
	return;
}


static struct pci_driver amrmo_pci_driver = {
	.name	  = AMRMO_MODULE_NAME,
	.id_table = amrmo_pci_tbl,
	.probe	  = amrmo_pci_probe,
	.remove   = amrmo_pci_remove,
};


/*
 *  module stuff
 */

MODULE_PARM(debug,"i");
MODULE_PARM_DESC(debug,"debug level: 0-3 (default=0)");

MODULE_AUTHOR("Smart Link Ltd.");
MODULE_DESCRIPTION("SmartLink HAMR5600,SmartPCI56/561 based modem driver");
MODULE_LICENSE("Smart Link Ltd.");


static int __init amrmo_init(void)
{
	struct pci_dev *dev = NULL;
	int err;
#ifdef OLD_KERNEL
	if (!pci_present())
		return -ENODEV;
#endif

	printk(KERN_INFO AMRMO_MODULE_NAME ": " "SmartLink AMRMO modem.\n");

	amrmo_debug_level = debug;

	/* fix me: how to prevent modem cards grabing by
	   serial driver? */
#ifdef OLD_KERNEL
	pci_for_each_dev(dev) {
#else
        while ((dev = pci_find_device(PCI_ANY_ID, PCI_ANY_ID, dev)) != NULL) {
#endif
		if(pci_match_id(amrmo_pci_tbl, dev) &&
		   pci_dev_driver(dev)) {
#ifdef OLD_KERNEL
			AMRMO_DBG("device %04x:%04x is used by %s: remove\n",
				  dev->vendor,dev->device,
				  dev->driver?dev->driver->name:"");
			if (dev->driver && dev->driver->remove)
				dev->driver->remove(dev);
			dev->driver = NULL;
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
			struct device *reldev = get_device(&dev->dev);
			AMRMO_DBG("device %04x:%04x is grabbed by driver %s: try to release\n",
				  dev->vendor,dev->device,
				  (reldev&&reldev->driver)?
				   reldev->driver->name:"unknown");
			if (reldev) {
				device_release_driver(reldev);
				put_device(reldev);
			}
#else
			printk("slamr: device %04x:%04x is grabbed by another driver\n",
			       dev->vendor,dev->device);
#endif
		}
	}

#ifndef OLD_KERNEL
	amrmo_class = class_simple_create(THIS_MODULE, "slamr");
	if (IS_ERR(amrmo_class)) {
		int err = PTR_ERR(amrmo_class);
		printk(KERN_ERR "slamr: failure creating simple class, error %d\n", err);
		return err;
	}
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
	if (!pci_register_driver(&amrmo_pci_driver)) {
#else
	if ((err = pci_register_driver(&amrmo_pci_driver)) < 0) {
#endif
		pci_unregister_driver(&amrmo_pci_driver);
#ifndef OLD_KERNEL
		class_simple_destroy(amrmo_class);
#endif
                return err;
	}

	if(register_chrdev(AMRMO_MAJOR, "slamr", &amrmo_fops) < 0) {
		pci_unregister_driver(&amrmo_pci_driver);
#ifndef OLD_KERNEL
		class_simple_destroy(amrmo_class);
#endif
		return -ENOMEM;
	}

	return 0;
}

static void __exit amrmo_exit(void)
{
	AMRMO_DBG("slamr: exit...\n");
	unregister_chrdev(AMRMO_MAJOR,"slamr");
	pci_unregister_driver(&amrmo_pci_driver);
#ifndef OLD_KERNEL
	class_simple_destroy(amrmo_class);
#endif
}

module_init(amrmo_init);
module_exit(amrmo_exit);

#ifdef OLD_KERNEL
EXPORT_NO_SYMBOLS;
#endif


