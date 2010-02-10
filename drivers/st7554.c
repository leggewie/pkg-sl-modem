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
 *	usb_st7554.c  --  ST7554 USB Smart Link Soft Modem driver
 *
 *	Author: SashaK (sashak@smlink.com)
 *
 *
 */

/*****************************************************************************/

#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/usb.h>
#include <linux/device.h>
#include <linux/devfs_fs_kernel.h>

#include <modem_defs.h>

#define MAX_MODEMS 16

#define USB_INFO(fmt...) printk(KERN_INFO fmt)
#define USB_ERR(fmt...)  printk(KERN_ERR fmt)
#define USB_DBG(fmt...) { if(debug) printk(KERN_DEBUG fmt); }

#define DEBUG_URB_PRINT 0
#define USB_DBG_URB(fmt...) // USB_DBG(fmt)

static int debug = 0;
MODULE_PARM(debug,"i");
MODULE_PARM_DESC(debug,"Debug level: 0-3 (default=0)");

/* st7554 IDs */
#define ST7554_VENDOR_ID   0x0483
#define ST7554_PRODUCT_ID  0x7554

/* st7554 interfaces */
#define ST7554_IFACE_CTRL_ALTSETTING         0
#define ST7554_IFACE_MODEM_ALTSETTING        1

/* comm class defined requests */
#define USB_COMM_CLASS_AUX_LINE_STATE 0x0a
#define USB_COMM_CLASS_HOOK_STATE 0x0b
#define USB_COMM_CLASS_PULSE_STATE 0x0c

/* st7554 vendor defined requests */
#define ST7554_SSI1_CONTROL    0x1
#define ST7554_SSI2_CONTROL    0x2
#define ST7554_FIFO_MASK       0x3
#define ST7554_SSI1_COUNTER    0x4
#define ST7554_SSI2_COUNTER    0x6
#define ST7554_GPIO_DIR        0x8
#define ST7554_GPIO_OUT        0xA
#define ST7554_GPIO_MASK       0xC
#define ST7554_GPIO_INV        0xE
#define ST7554_GPIO_STATUS     0x15
#define ST7554_SSI1_CWORD      0x17
#define ST7554_FIFO_CONTROL1   0x19
#define ST7554_FIFO_CONTROL2   0x1a

/* requests' values */
/* SSI control */
#define SSI1_POWERDOWN       0x0000 
#define SSI1_POWERUP         0x0001
/* FIFO mask */
#define SSI1_UNDERRUN        0x0040
#define SSI1_OVERRUN         0x0020
/* GPIO mask */
#define GPIO_RING1           0x0001
#define GPIO_HANDSET         0x0004
#define GPIO_HOOK1           0x0008
#define GPIO_CID1            0x0020
#define GPIO_LED_CARRIER     0x0040
#define GPIO_LED_HOOK        0x0080
#define GPIO_RFC             0x0100
#define GPIO_DISHS           0x0200
#define GPIO_DP              0x0400  /* used for Off Hook */
#define GPIO_BUZEN           0x0800
#define GPIO_DC              0x1000
#define GPIO_LED_RW          0x2000
#define GPIO_MONOLITHINC     0x4000
/* FIFO control */
#define BYTE_ORDER_LE        0x8000

/* gpio hook off bit mask */
#define GPIO_HOOK1_OFF   (GPIO_HOOK1)
//#define GPIO_HOOK1_OFF   (GPIO_HOOK1|GPIO_DP|GPIO_DC)

/* st7554 hw parameters */
#define ST7554_FIFO_SIZE  128
#define ST7554_HW_IODELAY 48

/* urb size */
#define DESCFRAMES 5

/* control message timeout */
#define CONTROL_MSG_TMO HZ

/* run state bits */
#define MO0_RUNNING 1
#define MO1_RUNNING 2
#define MI0_RUNNING 3
#define MI1_RUNNING 4
#define RUNNING     5

/* data type definitions */

struct st7554_state {
	struct usb_device    *usbdev; 
	struct usb_interface *iface ;
	const char *name;
	unsigned minor;
	struct file *file;
	spinlock_t lock;
	wait_queue_head_t wait;
	unsigned status;

	unsigned int ctrl_ep;  /* control endpoint */
	u32 intr_status;       /* interrupt status */
	struct urb *intr_urb;  /* interrupt urb ptr */

	unsigned int format;       /* sample format */
	unsigned int srate;        /* sample rate */
	unsigned int fragsize;     /* fragsize in bytes */

	u16 gpio;             /* gpio_out register shadow */

	int delay;                 /* i/o delay */
	int disbalance;            /* output disbalance */

	unsigned long run_mask;    /* run states mask: started+urbs */
	struct completion start_comp;
	struct completion stop_comp;

	struct usb_modem_channel { /* modem in/out channels */
		unsigned int maxsz;  /* max packet size */
		unsigned int interval; /* interval */
		unsigned int pipe;   /* usb data pipe */
		struct dmabuf {	     /* "dma" ring buffer */
			int          count;    /* byte count */
			unsigned int head;     /* head(write) pointer */
			unsigned int tail;     /* tail(read) pointer */
			unsigned int size;     /* buffer size */
			unsigned char *buf;    /* data buffer */
			unsigned int error;    /* over/underrun */
		} dma;
		struct urb *urb[2];  /* isoc urb */
	} mo, mi;

	struct codec {             /* installed codec */
		const char *name;
		int (*set_srate )(struct st7554_state *,unsigned);
		int (*set_format)(struct st7554_state *,unsigned);
	} codec;

	/* register access proc */
	int (*get_reg)(struct st7554_state *s, u8 reg, u16 *value);
	int (*set_reg)(struct st7554_state *s, u8 reg, u16  value);

	struct semaphore sem;
};




static struct st7554_state *st7554_table[MAX_MODEMS] = {};
static struct class_simple *st7554_class;

static DECLARE_MUTEX(open_sem);

/* --------------------------------------------------------------------- */

static int dma_init (struct dmabuf *db)
{
	db->buf = (unsigned char *)__get_free_pages(GFP_KERNEL, 1);
	if (!db->buf)
		return -ENOMEM;
	db->head = db->tail = db->count = 0;
	db->size = 1UL<<(PAGE_SHIFT + 1) ;
	return 0;
}


static void dma_free(struct dmabuf *db)
{
	int size = db->size;
	db->head = db->tail = db->count = db->size = 0;
	free_pages((unsigned long)db->buf, get_order(size));
	db->buf = NULL;
}


static int dmabuf_copyin(struct dmabuf *db, void *buffer, unsigned int size)
{
        int ret = 0, cnt;
        while (size) {
		cnt = db->size - db->head;
		if (cnt > size )
			cnt = size;
		if (cnt > db->size - db->count)
			cnt = db->size - db->count;
		if (cnt <= 0) {      /* overflow */
			db->error++;
			USB_ERR("dmabuf_copyin: overrun: ret %d.\n", ret);
			return ret;
		}
		memcpy(db->buf + db->head, buffer, cnt);
		
                buffer += cnt;
		db->count += cnt;
                db->head += cnt ;
                if (db->head >= db->size)
                        db->head = 0;
		size -= cnt;
		ret += cnt;
        }
	return ret;
}


/* --------------------------------------------------------------------- */

#define arrsize(a) (sizeof(a)/sizeof((a)[0]))

#define NUM_OF_URBS(ch) (sizeof((ch)->urb)/sizeof((ch)->urb[0]))
#define MO_URB_NO(s,u) ((u) == (s)->mo.urb[1])
#define MI_URB_NO(s,u) ((u) == (s)->mi.urb[1])

#define BYTES_IN_FRAMES(s,n) ((((s)->srate*(n))/1000)<<(MFMT_BYTESSHIFT((s)->format)))

#define FILL_URB(state,ch,u) { \
	(u)->dev = (state)->usbdev;  \
	(u)->pipe = (ch)->pipe; \
	(u)->context  = (state);          \
	(u)->number_of_packets = DESCFRAMES;  \
        (u)->interval = (ch)->interval; \
        (u)->status   = 0;              \
	(u)->transfer_flags |= URB_ISO_ASAP|URB_ASYNC_UNLINK;   }

#define FILL_DESC_OUT(state,ch,u,count) { int i; \
	unsigned shft = MFMT_BYTESSHIFT((state)->format); \
	unsigned len = count;  \
	for (i = 0 ; i < DESCFRAMES ; i++) { \
		(u)->iso_frame_desc[i].actual_length = 0; \
                (u)->iso_frame_desc[i].offset = 0; \
		(u)->iso_frame_desc[i].length = (len/(DESCFRAMES-i))&(~shft); \
		len -= (u)->iso_frame_desc[i].length; \
        } }

#define FILL_DESC_IN(state,ch,u,count) { int i, offs; \
        for ( i=0 , offs=0 ; i < DESCFRAMES; i++, offs += (ch)->maxsz) { \
	     (u)->iso_frame_desc[i].length = (ch)->maxsz; \
	     (u)->iso_frame_desc[i].offset = offs; } }

#define FILL_URB_OUT(state,ch,u,len) \
                 { FILL_URB(state,ch,u); FILL_DESC_OUT(state,ch,u,len);}
#define FILL_URB_IN(state,ch,u,len)  \
                 { FILL_URB(state,ch,u); FILL_DESC_IN(state,ch,u,len); }


/* --------------------------------------------------------------------- */


static int mi_free(struct st7554_state *s)
{
	struct usb_modem_channel *ch = &s->mi;
	int i;
 	for( i = 0 ; i < NUM_OF_URBS(ch) ; i++) {
		if(ch->urb[i]) {
			if(ch->urb[i]->transfer_buffer)
				kfree(ch->urb[i]->transfer_buffer);
			usb_free_urb(ch->urb[i]);
			ch->urb[i] = NULL;
		}
	}
	dma_free(&ch->dma);
	return 0;
}

static int mi_init (struct st7554_state *s)
{
	struct usb_modem_channel *ch = &s->mi;
	int i;
	if ( dma_init (&ch->dma) )
		return -ENOMEM;
	for (i = 0 ; i < NUM_OF_URBS(ch) ; i++) {
		struct urb *u = usb_alloc_urb(DESCFRAMES,GFP_KERNEL);
		if (!u)
			goto error;
		ch->urb[i] = u;
		u->transfer_buffer = kmalloc(ch->maxsz*DESCFRAMES,GFP_KERNEL);
		if(!u->transfer_buffer)
			goto error;
		u->transfer_buffer_length = ch->maxsz*DESCFRAMES;
		memset(u->transfer_buffer,0,u->transfer_buffer_length);
		FILL_URB_IN(s,ch,u,ch->maxsz*DESCFRAMES);
	}
	return 0;
 error:
 	mi_free(s);
	return -ENOMEM;
}



static int mo_free(struct st7554_state *s)
{
	struct usb_modem_channel *ch = &s->mo;
	int i;
	for (i = 0 ; i < NUM_OF_URBS(ch) ; i++) {
		usb_free_urb(ch->urb[i]);
		ch->urb[i] = NULL;
	}
	dma_free(&ch->dma);
	return 0;
}

static int mo_init (struct st7554_state *s)
{
	struct usb_modem_channel *ch = &s->mo;
	int i;

	if( dma_init (&ch->dma) )
		return -ENOMEM;

	for (i = 0 ; i < NUM_OF_URBS(ch) ; i++) {
		struct urb *u = usb_alloc_urb(DESCFRAMES,GFP_KERNEL);
		if (!u)
			goto error;
		ch->urb[i] = u;
		u->transfer_buffer_length = ch->maxsz*DESCFRAMES;
		u->transfer_buffer = ch->dma.buf;
		FILL_URB_OUT(s,ch,u,ch->maxsz*DESCFRAMES);
	}
	return 0;
 error:
 	mo_free(s);
	return -ENOMEM;
}



/* ----------------------------------------------------------------------- */


static void st7554_interrupt(struct urb *urb, struct pt_regs* regs)
{
	struct st7554_state *s = urb->context;
	u32 *status = urb->transfer_buffer;
	u16 fifo_status;
	u16 gpio_status;

	if (urb->status) {
		USB_DBG("st7554 interrupt: status = %d\n", urb->status);
		return;
	}

	fifo_status = *status &0xffff;
	gpio_status = *status >> 16;
#if 1
	USB_DBG("interrupt: fifo %04x, gpio %04x...\n",
		fifo_status, gpio_status);
#endif

	if (fifo_status & SSI1_UNDERRUN ) {
		USB_ERR("st7554: fifo underrun!\n");
		s->status |= MDMSTAT_ERROR;
	}
	if (fifo_status & SSI1_OVERRUN) {
		USB_ERR("st7554: fifo overrun!\n");
		s->status |= MDMSTAT_ERROR;
	}

	if (gpio_status & GPIO_RING1) {
		s->status |= MDMSTAT_RING;
	}

	if(s->status)
		wake_up_interruptible(&s->wait);
	urb->dev = s->usbdev;
	usb_submit_urb(urb,GFP_ATOMIC);
}

/* --------------------------------------------------------------------- */


static void mo_complete(struct urb *u, struct pt_regs* regs)
{
	struct st7554_state *s = u->context;
	struct dmabuf *db = &s->mo.dma;
	struct usb_iso_packet_descriptor *p;
	unsigned long flags;
	int i, mybit = MO_URB_NO(s,u) ? MO1_RUNNING : MO0_RUNNING ;

	if (u->status)
		goto finish;

	spin_lock_irqsave(&s->lock, flags);
	for (i = 0 ; i < u->number_of_packets ; i++) {
		p = &u->iso_frame_desc[i];
		if (p->status)
			USB_ERR("mo_complete %d: err: fr.%d status %d.\n",
				MO_URB_NO(s,u), i, p->status);
		if (s->disbalance + (int)p->length > 0) {
			p->length += s->disbalance;
			s->disbalance = 0;
		}
		else {
			/* FIXME: striping may optimize case recovery,
			   but fully stripped urb will cause mem leak in
			   usb controller driver (usb-uhci.o) */
			s->disbalance += p->length - 2 ;
			p->length = 2;
		}

		if (p->length > s->mo.maxsz) {
			s->disbalance += p->length - s->mo.maxsz;
			p->length = s->mo.maxsz;
		}
		if (p->length > db->size - db->tail) {
			s->disbalance += p->length - (db->size - db->tail);
			p->length = db->size - db->tail;
		}
		p->offset = db->tail;
		db->tail = (db->tail + p->length)%db->size ;
		db->count -= p->length;
	}
	spin_unlock_irqrestore(&s->lock, flags);

	USB_DBG_URB("mo_complete %d: %d: sent %d.\n",
		    MO_URB_NO(s,u), u->start_frame, u->actual_length);
	u->dev = s->usbdev;
	if(!test_bit(RUNNING,&s->run_mask) ||
	   usb_submit_urb(u,GFP_ATOMIC) )
		goto finish;

	return;
 finish:
	USB_DBG("mo_complete %d: finished, status %d\n",
		MO_URB_NO(s,u),u->status);
	clear_bit(mybit,&s->run_mask);
	complete(&s->stop_comp);
}


static void mo_startup_complete(struct urb *u, struct pt_regs* regs)
{
	struct st7554_state *s = u->context;
	USB_DBG("mo_startup_complete %d: %d: sent %d.\n",
		MO_URB_NO(s,u), u->start_frame, u->actual_length);
	FILL_DESC_OUT(s,&s->mo,u,BYTES_IN_FRAMES(s,DESCFRAMES));
	u->complete = mo_complete;
	mo_complete(u,regs);
	complete(&s->start_comp);
}


/* ----------------------------------------------------------------------- */


static void mi_complete(struct urb *u, struct pt_regs* regs)
{
	struct st7554_state *s = u->context;
	struct urb *next;
	struct usb_iso_packet_descriptor *p;
	unsigned long flags;
	int i, mybit = MI_URB_NO(s,u) ? MI1_RUNNING : MI0_RUNNING ;

	if (u->status)
		goto finish;

	next = s->mo.urb[!MI_URB_NO(s,u)];
	spin_lock_irqsave(&s->lock, flags);
	for (i = 0 ; i < u->number_of_packets ; i++) {
		p = &u->iso_frame_desc[i];
		if (p->status) {
			USB_ERR("mi_complete %d: err: fr.%d status %d.\n",
				MI_URB_NO(s,u), i, p->status);
		}
		dmabuf_copyin(&s->mi.dma, u->transfer_buffer + p->offset, p->actual_length);
		/* set length of out urb (in driven) */
		next->iso_frame_desc[i].length = p->actual_length;
	}

	if(s->mi.dma.count > 0)
		wake_up_interruptible(&s->wait);

	spin_unlock_irqrestore(&s->lock, flags);

	USB_DBG_URB("mi_complete %d: %d: recv %d.\n",
		    MI_URB_NO(s,u), u->start_frame, u->actual_length);
	u->dev = s->usbdev;
	if(!test_bit(RUNNING,&s->run_mask) ||
	   usb_submit_urb(u,GFP_ATOMIC))
		goto finish;

	return;
 finish:
	USB_DBG("mi_complete %d: finished, status = %d\n",
		MI_URB_NO(s,u),u->status);
	clear_bit(mybit,&s->run_mask);
	complete(&s->stop_comp);
}


static void mi_startup_complete(struct urb *u, struct pt_regs* regs)
{
	struct st7554_state *s = u->context;
	struct usb_iso_packet_descriptor *p;
	unsigned long flags;
	int mybit = MI_URB_NO(s,u) ? MI1_RUNNING : MI0_RUNNING ;
	int i;

	if (u->status)
		goto finish;

	spin_lock_irqsave(&s->lock, flags);

	if(u->actual_length > 0) {
		for (i = 0 ; i < u->number_of_packets ; i++) {
			p = &u->iso_frame_desc[i];
			if (p->status) {
				USB_ERR("mi_startup_complete %d: err: fr.%d status %d.\n",
					MI_URB_NO(s,u), i, p->status);
			}
			dmabuf_copyin(&s->mi.dma, u->transfer_buffer + p->offset, p->actual_length);
		}
		s->mi.urb[0]->complete = mi_complete;
		s->mi.urb[1]->complete = mi_complete;
	}

	i = BYTES_IN_FRAMES(s,DESCFRAMES) - u->actual_length;
	USB_DBG("mi_startup: advance mo head +%d...\n",i);
	s->mo.dma.count += i;
	s->mo.dma.head = (s->mo.dma.head + i)%s->mo.dma.size;

	if(s->mi.dma.count > 0)
		wake_up_interruptible(&s->wait);

	spin_unlock_irqrestore(&s->lock, flags);

	USB_DBG("mi_startup_complete %d: %d: recv %d.\n",
		MI_URB_NO(s,u), u->start_frame, u->actual_length);
	u->dev = s->usbdev;
	if(!test_bit(RUNNING,&s->run_mask) ||
	   usb_submit_urb(u,GFP_ATOMIC))
		goto finish;

	return;
 finish:
	USB_DBG("mi_startup_complete %d: finished, status %d\n",
		MI_URB_NO(s,u),u->status);
	clear_bit(mybit,&s->run_mask);
	complete(&s->stop_comp);
}


/* --------------------------------------------------------------------- */

/*
 * Start process brief scheme:
 *
 *       |<----DESCFRAMES--->||<----DESCFRAMES--->|
 * frame:| 0 | 1 | 2 | 3 | 4 || 5 | 6 | 7 | 8 | 9 |
 * -----------------------------------------------------------
 *       |      in urb 0     ||      in urb 1     |
 *   in: |+++|+++|+++|+++|+++||+++|+++|+++|+++|+++|
 *       |     out urb 0     ||     out urb 1     |
 *  out: |+++|+++|+++|+++|+++||+++|+++|+++|+++|+++|
 * -----------------------------------------------------------
 *           fill out fifo        start fifo
 *       |<----------------->||<----------------->|
 *
 *
 */


static int st7554_stop (struct st7554_state *s)
{
	USB_DBG ("st7554 stop...\n");

	down(&s->sem);
	if(!test_and_clear_bit(RUNNING,&s->run_mask))
		goto out;

	/* stop fifo */
	s->set_reg(s, ST7554_FIFO_MASK, 0);
	s->set_reg(s, ST7554_SSI1_COUNTER, 0);

	if(test_bit(MO0_RUNNING,&s->run_mask))
		usb_unlink_urb(s->mo.urb[0]);
	if(test_bit(MI0_RUNNING,&s->run_mask))
		usb_unlink_urb(s->mi.urb[0]);
	if(test_bit(MO1_RUNNING,&s->run_mask))
		usb_unlink_urb(s->mo.urb[1]);
	if(test_bit(MI1_RUNNING,&s->run_mask))
		usb_unlink_urb(s->mi.urb[1]);

	while(test_bit(MO0_RUNNING,&s->run_mask) ||
	      test_bit(MI0_RUNNING,&s->run_mask) ||
	      test_bit(MO1_RUNNING,&s->run_mask) ||
	      test_bit(MI1_RUNNING,&s->run_mask) )
		wait_for_completion(&s->stop_comp);

	/* flush buffers */
	s->mi.dma.count = s->mi.dma.head = s->mi.dma.tail = 0;
	s->mo.dma.count = s->mo.dma.head = s->mo.dma.tail = 0;
	USB_DBG ("st7554 stopped - delay %d.\n", s->delay);
 out:
	up(&s->sem);
	return 0;
}


static int st7554_start (struct st7554_state *s)
{
	int len, ret = 0;

	USB_DBG ("st7554 start...\n");

	down(&s->sem);
	if(test_and_set_bit(RUNNING,&s->run_mask))
		goto out;

	init_completion(&s->start_comp);
	init_completion(&s->stop_comp);

	/* setup run params */
	s->disbalance = 0;
	len = 32;
	memset(s->mo.dma.buf,0,s->mo.dma.size);
	s->mo.dma.count = len;
	s->mo.dma.head  = len;
	s->delay = len + ST7554_FIFO_SIZE + BYTES_IN_FRAMES(s,DESCFRAMES)*2;	

	/* prepare urbs */
	FILL_URB_IN(s, &s->mi, s->mi.urb[0], s->mi.maxsz*DESCFRAMES);
	FILL_URB_IN(s, &s->mi, s->mi.urb[1], s->mi.maxsz*DESCFRAMES);

	FILL_URB_OUT(s, &s->mo, s->mo.urb[0], s->mo.maxsz*DESCFRAMES);
	FILL_URB_OUT(s, &s->mo, s->mo.urb[1], BYTES_IN_FRAMES(s,DESCFRAMES));

	s->mi.urb[0]->complete = mi_startup_complete;
	s->mo.urb[0]->complete = mo_startup_complete;
	s->mi.urb[1]->complete = mi_startup_complete;
	s->mo.urb[1]->complete = mo_complete;

	/* submit all urbs */

	ret = usb_submit_urb(s->mo.urb[0],GFP_KERNEL);
	if(ret) goto out;
	set_bit(MO0_RUNNING,&s->run_mask);

	ret = usb_submit_urb(s->mi.urb[0],GFP_KERNEL);
	if(ret) goto out;
	set_bit(MI0_RUNNING,&s->run_mask);

	ret = usb_submit_urb(s->mo.urb[1],GFP_KERNEL);
	if(ret) goto out;
	set_bit(MO1_RUNNING,&s->run_mask);

	ret = usb_submit_urb(s->mi.urb[1],GFP_KERNEL);
	if(ret) goto out;
	set_bit(MI1_RUNNING,&s->run_mask);

	USB_DBG("st7554 start: submitted urbs: "
		"mo0(%p) %d, mi0(%p) %d, mo1(%p) %d, mi1(%p) %d.\n",
		s->mo.urb[0], s->mo.urb[0]->start_frame, 
		s->mi.urb[0], s->mi.urb[0]->start_frame,
		s->mo.urb[1], s->mo.urb[1]->start_frame,
		s->mi.urb[1], s->mi.urb[1]->start_frame);

	wait_for_completion(&s->start_comp);

	USB_DBG("st7554 start: starting...\n");
	/* start fifo */
	ret = s->set_reg(s, ST7554_SSI1_COUNTER, s->fragsize);
	if (ret < 0)
		goto out;
	/* set fifo mask */
	ret = s->set_reg(s, ST7554_FIFO_MASK, SSI1_UNDERRUN|SSI1_OVERRUN);

 out:
	up(&s->sem);
	if(ret) {
		USB_ERR("st7554: start failed = %d (mask %lx)\n", ret,
			s->run_mask);
		st7554_stop(s);
	}
	return ret;
}



static int st7554_set_srate (struct st7554_state *s, unsigned srate)
{
	unsigned long flags;
	if (s->srate == srate)
		return 0;
	if(s->codec.set_srate(s, srate))
		return -EINVAL;
	spin_lock_irqsave(&s->lock,flags);
	s->srate = srate;
	spin_unlock_irqrestore(&s->lock,flags);
	return 0;
}

static int st7554_set_format (struct st7554_state *s, unsigned format)
{
	unsigned long flags;
	if(format == MFMT_QUERY)
		return s->codec.set_format(s, MFMT_QUERY);
	if (s->format == format)
		return 0;
	if(s->codec.set_format(s, format))
		return -EINVAL;
	spin_lock_irqsave(&s->lock,flags);
	s->format = format;
	spin_unlock_irqrestore(&s->lock,flags);
	return 0;
}

static int st7554_set_frag (struct st7554_state *s, unsigned frag)
{
	unsigned long flags;
	spin_lock_irqsave(&s->lock,flags);
	s->fragsize = frag;
	spin_unlock_irqrestore(&s->lock,flags);
	return 0;
}

static int st7554_set_hook(struct st7554_state *s, unsigned hook)
{
	unsigned long flags;
	u16 val = s->gpio;
	if (hook == MODEM_HOOK_OFF)
		val |=  (GPIO_HOOK1_OFF|GPIO_LED_HOOK);
	else if (hook == MODEM_HOOK_ON)
		val &= ~(GPIO_HOOK1_OFF|GPIO_LED_HOOK);
	else
		return -EINVAL;
	if(s->set_reg(s, ST7554_GPIO_OUT, val))
		return -EIO;
	spin_lock_irqsave(&s->lock,flags);
	s->gpio = val;
	spin_unlock_irqrestore(&s->lock,flags);
	return 0;
}



/*
 *    file operations
 *
 */


static ssize_t st7554_read(struct file *file, char *buffer, size_t count, loff_t *ppos)
{
	struct st7554_state *s = (struct st7554_state *)file->private_data;
	struct dmabuf *db;
	unsigned long flags;
	int cnt, ret = 0;
	if(!s) return -ENODEV;
	db = &s->mi.dma;
	while (count) {
		cnt = count;
		spin_lock_irqsave(&s->lock,flags);
		if ( cnt > db->count )
			cnt = db->count;
		if ( cnt > db->size - db->tail )
			cnt = db->size - db->tail;
		spin_unlock_irqrestore(&s->lock,flags);
		if ( cnt <= 0 )
			break;
		if(copy_to_user(buffer, db->buf + db->tail, cnt))
			return -EFAULT;

		spin_lock_irqsave(&s->lock,flags);
		db->count -= cnt;
		db->tail = (db->tail + cnt)%db->size;
		spin_unlock_irqrestore(&s->lock,flags);
		buffer += cnt;
                count -= cnt;
		ret += cnt;
        }
	return ret;
}


static ssize_t st7554_write(struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
	struct st7554_state *s = (struct st7554_state *)file->private_data;
	struct dmabuf *db;
	unsigned long flags;
	int cnt, ret = 0;
	if(!s) return -ENODEV;
	db = &s->mo.dma;
	while (count) {
		cnt = count;
		spin_lock_irqsave(&s->lock,flags);
		if ( cnt > db->size - db->count )
			cnt = db->size - db->count;
		if ( cnt > db->size - db->head )
			cnt = db->size - db->head;
		spin_unlock_irqrestore(&s->lock,flags);
		if ( cnt <= 0 )
			return ret;
		if(copy_from_user(db->buf + db->head, buffer, cnt))
			return -EFAULT;

		spin_lock_irqsave(&s->lock,flags);
		db->count += cnt;
		db->head = (db->head + cnt)%db->size;
		spin_unlock_irqrestore(&s->lock,flags);
		buffer += cnt;
                count -= cnt;
		ret += cnt;
        }
	return ret;
}


static unsigned int st7554_poll(struct file *file, poll_table *wait)
{
	struct st7554_state *s = (struct st7554_state *)file->private_data;
        unsigned long flags;
        unsigned int mask = 0;
	if(!s) return POLLERR;
	poll_wait(file,&s->wait,wait);
	spin_lock_irqsave(&s->lock,flags);
	if(s->status & MDMSTAT_ERROR)
		mask |= POLLERR;
	if(s->status & MDMSTAT_RING)
		mask |= POLLPRI;
	if(s->mi.dma.count > 0) {
		mask |= POLLIN | POLLRDNORM;
	}
	spin_unlock_irqrestore(&s->lock,flags);
        return mask;
}


static int st7554_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct st7554_state *s = (struct st7554_state *)file->private_data;
	USB_DBG ("st7554 ioctl: cmd %x...\n", cmd);
	if (!s || !s->usbdev)
		return -ENODEV;

	switch (cmd) {
	case MDMCTL_CAPABILITIES:
		return -EINVAL;
	case MDMCTL_HOOKSTATE:
		return st7554_set_hook(s, arg);
	case MDMCTL_SPEED:
		return st7554_set_srate(s, arg);
	case MDMCTL_GETFMTS:
		return st7554_set_format(s, MFMT_QUERY);
	case MDMCTL_SETFMT:
		return st7554_set_format(s, arg);
	case MDMCTL_SETFRAGMENT:
		return st7554_set_frag(s, arg);
	case MDMCTL_CODECTYPE:
		return CODEC_STLC7550;
	case MDMCTL_IODELAY:
		{ int val;
		val = s->delay + ST7554_HW_IODELAY;
		USB_DBG("st7554 ioctl: IODELAY = %d.\n", val);
		return val;
		}
	case MDMCTL_START:
		return st7554_start(s);
	case MDMCTL_STOP:
		return st7554_stop(s);
	case MDMCTL_GETSTAT:
		USB_DBG ("st7554 ioctl: GETSTAT...\n");
		{ unsigned long flags;
		unsigned stat;
		spin_lock_irqsave(&s->lock,flags);
		stat = s->status;
		s->status = 0;
		spin_unlock_irqrestore(&s->lock,flags);
                if (put_user(stat, (unsigned *) arg))
                        return -EFAULT;
		}
		return 0;
	default:
		break;
	}
	return -ENOIOCTLCMD;
}

static int st7554_open(struct inode *inode, struct file *file)
{
	struct st7554_state *s;
	unsigned minor = MINOR(inode->i_rdev);
	USB_DBG("st7554 open...\n");
	if(minor > arrsize(st7554_table))
		return -ENODEV;
	down(&open_sem);
	s = st7554_table[minor];
	if(!s || !s->usbdev) {
		up(&open_sem);
		return -ENODEV;
	}
	if(s->file) {
		up(&open_sem);
		return -EBUSY;
	}
	s->file = file;
	up(&open_sem);
	file->private_data = s;
	return 0;
}


static int st7554_close(struct inode *inode, struct file *file)
{
	struct st7554_state *s = (struct st7554_state *)file->private_data;
	USB_DBG("st7554 close...\n");
	if(s) {
		st7554_stop(s);
		s->file = NULL;
	}
	return 0;
}



static struct file_operations st7554_fops = {
        .owner =   THIS_MODULE,
        .llseek =  no_llseek,
        .read =    st7554_read,
        .write =   st7554_write,
        .poll =    st7554_poll,
        .ioctl =   st7554_ioctl,
        .open =    st7554_open,
        .release = st7554_close,
};


/* --------------------------------------------------------------------- */


static int st7554_get_reg (struct st7554_state *s, u8 reg, u16 *val)
{
	int ret;
	ret = usb_control_msg(s->usbdev,
			      usb_rcvctrlpipe(s->usbdev,s->ctrl_ep),
			      reg|USB_DIR_IN,
			      USB_TYPE_VENDOR|USB_RECIP_DEVICE|USB_DIR_IN,  
			      0, 0, val, sizeof(*val),
			      CONTROL_MSG_TMO);
	if ( ret < 0 )
		USB_ERR("st7554_get_reg: error: reg %x, ret = %d\n",
		    reg, ret);
	return ret;
}

static int st7554_set_reg(struct st7554_state *s, u8 reg, u16 value)
{
	int ret =
		usb_control_msg(s->usbdev,
				usb_sndctrlpipe(s->usbdev,s->ctrl_ep),
				reg,
				USB_TYPE_VENDOR|USB_RECIP_DEVICE,
				value, 0, NULL, 0,
				CONTROL_MSG_TMO);
	if (ret < 0 )
		USB_ERR("st7554_set_reg: error: reg %x, val %x, ret = %d\n",
			reg, value, ret);
	return ret;
}

static int stlc7550_set_srate(struct st7554_state *s, unsigned int srate)
{
	if ( srate == 8000 )
		return s->set_reg(s, ST7554_SSI1_CWORD,0x3c8);
	else if (srate == 9600 )
		return s->set_reg(s, ST7554_SSI1_CWORD,0x3c0);
	else if (srate == 16000 )
		return s->set_reg(s, ST7554_SSI1_CWORD,0x3f0);
	else if (srate == 24000 )
	        return s->set_reg(s, ST7554_SSI1_CWORD,0x3e8);
	else
		return -EINVAL;
}

static int stlc7550_set_format(struct st7554_state *s, unsigned int format)
{
	if (format == MFMT_QUERY)
		return MFMT_U16_LE | MFMT_S16_LE;
	if (!MFMT_IS_16BIT(format))
		return -EINVAL;
	if (!MFMT_IS_LE(format))
		return -EINVAL;
	return 0;
}

#if 0
static int st75951_set_srate(struct st7554_state *s, unsigned int srate)
{
	return -EINVAL;
}

static int st75951_set_format(struct st7554_state *s, unsigned int format){
	return -EINVAL;
}
#endif

/* ---------------------------------------------------------------------- */

#if 0 /* ifdef DEBUG */
#define PRINT_REG(s ,reg, name) { u16 val; int ret; \
                   ret = s->get_reg(s,reg,&val);\
                   USB_DBG("st7554: vendor reg %s (%x) = %x , ret %d.\n", \
                   name, reg, val, ret); }

static void print_all_regs(struct st7554_state *s) {
	PRINT_REG(s, ST7554_REVISION,"REVISION");
	PRINT_REG(s, ST7554_SSI1_CONTROL,"SSI1_CONTROL");
	PRINT_REG(s, ST7554_SSI2_CONTROL,"SSI2_CONTROL");
	PRINT_REG(s, ST7554_FIFO_MASK,"FIFO_MASK");
	PRINT_REG(s, ST7554_FIFO_SSI1_COUNTER,"FIFO_SSI1_COUNTER");
	PRINT_REG(s, ST7554_FIFO_SSI2_COUNTER,"FIFO_SSI2_COUNTER");
	PRINT_REG(s, ST7554_GPIO_DIR,"GPIO_DIR");
	PRINT_REG(s, ST7554_GPIO_OUT,"GPIO_OUT");
	PRINT_REG(s, ST7554_GPIO_MASK,"GPIO_MASK");
	PRINT_REG(s, ST7554_GPIO_INV,"GPIO_INV");
	PRINT_REG(s, ST7554_GPIO_STATUS,"GPIO_STATUS");
	PRINT_REG(s, ST7554_FIFO_CONTROL1,"FIFO_CONTROL1");
	PRINT_REG(s, ST7554_FIFO_CONTROL2,"FIFO_CONTROL2");
}
#endif /* DEBUG */

/* ---------------------------------------------------------------------- */

#define SET_REG(s,reg,val) { ret = s->set_reg(s,reg,val); if (ret < 0) { USB_ERR("st7554: failed to set reg %x.\n", reg); ; return ret;} }


static int st7554_init  (struct st7554_state *s)
{
	int ret;
	s->gpio = 0;
	SET_REG(s, ST7554_GPIO_DIR, 0x3ff8);
	SET_REG(s, ST7554_GPIO_OUT, 0x00);
	/* SET_REG(s, ST7554_GPIO_MASK, GPIO_HANDSET); */
	SET_REG(s, ST7554_GPIO_MASK, GPIO_RING1);
	SET_REG(s, ST7554_FIFO_CONTROL1, 0x2828|BYTE_ORDER_LE);
	SET_REG(s, ST7554_FIFO_CONTROL2, 0x2828);
	SET_REG(s, ST7554_FIFO_MASK, 0x00);
	SET_REG(s, ST7554_FIFO_MASK, SSI1_UNDERRUN|SSI1_OVERRUN);
	SET_REG(s, ST7554_SSI1_COUNTER, 0x00);
	SET_REG(s, ST7554_SSI2_COUNTER, 0x00);
	/* power up */
	SET_REG(s, ST7554_SSI1_CONTROL, SSI1_POWERUP);
	/* control word */
	SET_REG(s, ST7554_SSI1_CWORD, 0x3c0);
	/* no inversion */
	SET_REG(s, ST7554_GPIO_INV, 0);

	/* clear usb ep */
	usb_clear_halt(s->usbdev, s->mi.pipe);
	usb_clear_halt(s->usbdev, s->mo.pipe);

	return 0;
}

static int st7554_release (struct st7554_state *s)
{
	int ret;
	USB_DBG("st7554_release: clear regs...\n");
	/* clear fifo & gpio  masks */
	SET_REG(s, ST7554_FIFO_MASK, 0);
	SET_REG(s, ST7554_GPIO_MASK, 0);
	/* hook on && all */
	s->gpio = 0;
	SET_REG(s, ST7554_GPIO_OUT, 0x00);
	/* power down */
	SET_REG(s, ST7554_SSI1_CONTROL, SSI1_POWERDOWN);
	return 0;
} 

/* --------------------------------------------------------------------- */

static int st7554_probe(struct usb_interface *interface,
			const struct usb_device_id *id);
static void st7554_disconnect(struct usb_interface *interface);


static struct usb_device_id st7554_ids [] = {
	{ USB_DEVICE(ST7554_VENDOR_ID,ST7554_PRODUCT_ID) },
	{ 0 }			   /* Terminating entry */
};

MODULE_DEVICE_TABLE (usb, st7554_ids);

static struct usb_driver st7554_usb_driver = {
	.owner =       THIS_MODULE,
	.name =	       "ST7554 USB Modem",
	.probe =       st7554_probe,
	.disconnect =  st7554_disconnect,
	.id_table =    st7554_ids,
};

/* --------------------------------------------------------------------- */


static int st7554_probe(struct usb_interface *interface,
			const struct usb_device_id *id)
{
	struct usb_device *usbdev = interface_to_usbdev(interface);
	struct st7554_state *s;
	struct usb_host_interface *iface_desc;
	int i , ret;
	u16 val;

        if ((usbdev->descriptor.idVendor  != ST7554_VENDOR_ID) ||
            (usbdev->descriptor.idProduct != ST7554_PRODUCT_ID))
                return -ENODEV;

	USB_DBG("st7554 usb: probe...\n");

	s = kmalloc(sizeof(*s), GFP_KERNEL);
	if (!s) {
		USB_ERR("st7554 probe: no memory.\n");
		return -ENOMEM;
	}
	memset(s, 0, sizeof(*s));

	spin_lock_init(&s->lock);
	init_MUTEX (&s->sem);
	init_waitqueue_head(&s->wait);

	s->name = "ST7554 USB Modem";
	s->usbdev = usbdev;
	s->iface  = interface;
	s->ctrl_ep = 0;

	iface_desc = &interface->altsetting[1];
	usb_set_interface(usbdev, 0, 1);

	/* interrupt init */
	s->intr_urb = usb_alloc_urb(0,GFP_KERNEL);
        if (!s->intr_urb) {
                USB_ERR("cannot alloc intr urb\n");
		kfree(s);
		return -ENOMEM;
        }
	usb_fill_int_urb(s->intr_urb, usbdev,
			 usb_rcvintpipe(usbdev,iface_desc->endpoint[0].desc.bEndpointAddress),
			 &s->intr_status,
			 sizeof(s->intr_status),
			 st7554_interrupt, s,
			 iface_desc->endpoint[0].desc.bInterval);
	ret = usb_submit_urb(s->intr_urb,GFP_KERNEL);
	if (ret < 0) {
		USB_ERR("st7554_init: cannot submit intr urb: %d.\n", ret);
		goto error;
	}

	s->mo.pipe = usb_sndisocpipe(usbdev,iface_desc->endpoint[1].desc.bEndpointAddress);
	s->mo.maxsz = iface_desc->endpoint[1].desc.wMaxPacketSize;
	s->mo.interval = iface_desc->endpoint[1].desc.bInterval;
	s->mi.pipe = usb_rcvisocpipe(usbdev,iface_desc->endpoint[2].desc.bEndpointAddress);
	s->mi.maxsz = iface_desc->endpoint[2].desc.wMaxPacketSize;
	s->mi.interval = iface_desc->endpoint[2].desc.bInterval;

	USB_DBG("probe: int interval %d, maxsize mo %d, mi %d\n",
		iface_desc->endpoint[0].desc.bInterval,
		iface_desc->endpoint[1].desc.wMaxPacketSize,
		iface_desc->endpoint[2].desc.wMaxPacketSize );

	s->get_reg = st7554_get_reg;
	s->set_reg = st7554_set_reg;

	/* SSI1 codec detection */
	if (s->get_reg(s,ST7554_GPIO_STATUS,&val) < 0) {
		USB_ERR("st7554 probe: cannot detect codec type.\n");
		goto error;
	}

	if (val&GPIO_MONOLITHINC) { /* st75951/2 silicon DAA codec */
		USB_ERR("st7554 probe: unsupported codec st75951/2.\n");
		s->codec.name = "stlc75971/2";
		goto error;
	}
	else {
		USB_DBG("codec stlc7550 detected.\n");
		s->codec.name = "stlc7550";
		s->codec.set_srate  = stlc7550_set_srate ;
		s->codec.set_format = stlc7550_set_format;
	}

	if(st7554_init(s)) {
		USB_ERR("st7554 probe: cannot initialize device.\n");
		goto error;
	}

	st7554_set_hook  (s, MODEM_HOOK_ON);
	st7554_set_srate (s, 9600);
	st7554_set_format(s, MFMT_S16_LE);
	st7554_set_frag  (s, 96);

	if (mo_init(s) < 0 ) {
		USB_ERR("st7554: cannot init out channel.\n");
		goto error1;
	}
	if (mi_init(s) < 0 ) {
		USB_ERR("st7554: cannot init in channel.\n");
		mo_free(s);
		goto error1;
	}

	down(&open_sem);
	for(i = 0 ; i < arrsize(st7554_table) ; i++) {
		if(st7554_table[i] == NULL) {
			st7554_table[i] = s;
			s->minor = i;
			break;
		}
	}
	up(&open_sem);
	if (i == arrsize(st7554_table)) {
		USB_ERR("no more states\n");
		mo_free(s); mi_free(s);
		goto error1;
	}

	usb_set_intfdata(interface, s );
	class_simple_device_add(st7554_class, MKDEV(243, i), NULL, "slusb%d", i);
	devfs_mk_cdev(MKDEV(243,i),S_IFCHR|S_IRUSR|S_IWUSR,"slusb%d",i);

	USB_INFO(KERN_INFO "slusb: slusb%d is found.\n", s->minor);

	return 0;

 error1:
	st7554_release(s);
 error:
	if (s->intr_urb) {
		usb_unlink_urb(s->intr_urb);
		usb_free_urb(s->intr_urb);
	}
	kfree(s);
	return -ENOMEM;
}


static void st7554_disconnect(struct usb_interface *interface)
{
	struct st7554_state *s = usb_get_intfdata(interface);
	usb_set_intfdata(interface, NULL );
	USB_DBG("st7554 disconnect...\n");
        if (!s || !s->usbdev) {
                USB_DBG("st7554 disconnect: no dev.\n");
                return;
        }

	class_simple_device_remove(MKDEV(243, s->minor));
 	devfs_remove("slusb%d",s->minor);

	st7554_stop(s);
	down(&open_sem);
	if(s->file) {
		/* TBD: notify disconnect */
		s->file->private_data = NULL;
		s->file = NULL;
	}
	USB_DBG("unlink intr...\n");
	if (s->intr_urb) {
		usb_unlink_urb(s->intr_urb);
		usb_free_urb(s->intr_urb);
	}

	st7554_release(s);
	s->usbdev = NULL;
	st7554_table[s->minor] = NULL;
	up(&open_sem);
	USB_DBG("st7554 mo/mi free...\n");
	mo_free(s);
	mi_free(s);
	kfree(s);
}

/* ---------------------------------------------------------------------- */


static int __init st7554_modem_init(void)
{
	int ret;
	USB_INFO ("ST7554 USB Modem.\n");

	st7554_class = class_simple_create(THIS_MODULE, "slusb");
	if (IS_ERR(st7554_class)) {
		ret = PTR_ERR(st7554_class);
		USB_ERR("st7554_modem_init: failed to create sysfs class, error %d\n", ret);
		return ret;
	}

	ret = usb_register(&st7554_usb_driver);
	if ( ret ) {
		USB_ERR ("st7554_modem_init: cannot register usb device.\n");
		class_simple_destroy(st7554_class);
		return ret;
	}

	if(register_chrdev(243, "slusb", &st7554_fops) < 0) {
		usb_deregister(&st7554_usb_driver);
		class_simple_destroy(st7554_class);
		return -ENOMEM;
	}
	return 0;
}


static void __exit st7554_modem_exit(void)
{
	USB_DBG ("st7554: exit...\n");
	unregister_chrdev(243,"slusb");
	usb_deregister(&st7554_usb_driver);
	class_simple_destroy(st7554_class);
}


module_init(st7554_modem_init);
module_exit(st7554_modem_exit);


MODULE_AUTHOR("Smart Link Ltd.");
MODULE_DESCRIPTION("ST7554 USB Smart Link Soft Modem driver.");
MODULE_LICENSE("Smart Link Ltd.");

