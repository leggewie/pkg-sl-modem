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
 *	sysdep_amr.c  --  pci driver sysdep routines.
 *
 *	Author: Seva (seva@smlink.com)
 *
 *
 */

/*****************************************************************************/


#include <linux/version.h>
#include <linux/module.h>

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <asm/io.h>

/* memory allocations */
asmlinkage void *sysdep_malloc (unsigned int size)
{
        void *mem;
        mem = kmalloc(size, in_interrupt() ? GFP_ATOMIC : GFP_KERNEL);
        if(!mem)
                printk(KERN_ERR "slmdm: cannot alloc %db mem\n", size);
        return mem;
}

asmlinkage void sysdep_free (void *mem)
{
	kfree(mem);
}

/* strings */
asmlinkage void *sysdep_memset(void *d,int c,size_t l)
{
	return memset(d,c,l);
}

asmlinkage void *sysdep_memcpy(void *d,const void *s,size_t l)
{
	return memcpy(d,s,l);
}

asmlinkage size_t sysdep_strlen(const char *s)
{
	return strlen(s);
}

asmlinkage char *sysdep_strncpy(char *d,const char *s, size_t l)
{
	return strncpy(d,s,l);
}

asmlinkage int sysdep_strcmp(const char *s1,const char *s2)
{
	return strcmp(s1, s2);
}

asmlinkage int sysdep_sprintf(char *buf, const char *fmt, ...)
{
	va_list args;
	int i;
	va_start(args, fmt);
	i=vsprintf(buf,fmt,args);
	va_end(args);
	return i;
}

/* getting times */
asmlinkage unsigned long sysdep_get_utime(void)
{
    struct timeval t;
    do_gettimeofday (&t);
    return t.tv_sec * 1000000 + t.tv_usec ;
}

/* cli/sti handling */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,5,28))
asmlinkage void sysdep_save_flags   (unsigned long *flags) {save_flags(*flags); }
asmlinkage void sysdep_restore_flags(unsigned long *flags) {restore_flags(*flags);}
asmlinkage void sysdep_cli(void) {cli();}
#else
asmlinkage void sysdep_save_flags   (unsigned long *flags) {local_irq_save(*flags); }
asmlinkage void sysdep_restore_flags(unsigned long *flags) {local_irq_restore(*flags);}
asmlinkage void sysdep_cli(void) {}
#endif


/* dmesg print */
asmlinkage void sysdep_dmesg_print(const char *s)
{
	printk("%s\n",s);
}

/* page allocations */
asmlinkage unsigned long sysdep_get_free_pages(unsigned long order)
{
	return __get_free_pages(GFP_KERNEL, order);
}

asmlinkage void sysdep_free_pages(unsigned long addr, unsigned long order)
{
	free_pages(addr, order);
}

asmlinkage unsigned long sysdep_virt_to_phys(volatile void * address)
{
	return virt_to_phys (address);
}

/* delay */
asmlinkage void sysdep_udelay(unsigned long usecs)
{
	udelay(usecs);
}

/* from linux/pci.h */
asmlinkage int sysdep_pci_read_config_byte (void *dev, unsigned char where, unsigned char *val)
{	
        return pci_read_config_byte (dev, where, val);
}

asmlinkage int sysdep_pci_read_config_word (void *dev, unsigned char where, unsigned short *val)
{
        return pci_read_config_word (dev, where, val);
}

asmlinkage int sysdep_pci_read_config_dword (void *dev, unsigned char where, unsigned int *val)
{
        return pci_read_config_dword (dev, where, val);
}

asmlinkage int sysdep_pci_write_config_byte (void *dev, unsigned char where, unsigned char val)
{
        return pci_write_config_byte (dev, where, val);
}

asmlinkage int sysdep_pci_write_config_word (void *dev, unsigned char where, unsigned short val)
{
        return pci_write_config_word (dev, where, val);
}

asmlinkage int sysdep_pci_write_config_dword (void *dev, unsigned char where, unsigned int val)
{
	return pci_write_config_dword (dev, where, val);
}

/* io */
asmlinkage unsigned char sysdep_inb(unsigned short port)
{
	return inb(port);
}
asmlinkage void sysdep_outb(unsigned char data,unsigned short port)
{
	outb(data,port);
}
asmlinkage unsigned short sysdep_inw(unsigned short port) 
{
	return inw(port);
}

asmlinkage void sysdep_outw(unsigned short data,unsigned short port)
{
	outw(data,port);
}
asmlinkage unsigned long sysdep_inl(unsigned short port) 
{
	return inl(port);
}
asmlinkage void sysdep_outl(unsigned long data,unsigned short port)
{
	outl(data,port);
}


