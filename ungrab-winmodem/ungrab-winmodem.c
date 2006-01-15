/*
 * Copyright (c) 2005, linmodems.org
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Sasha K (sashak@smlink.com)
 *
 * ungrab-winmodem.c - try to release softmodem device when grabbed by others.
 *
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/kernel.h>

#define PCI_DEVICE_ID_ALI5457			0x5457
#define PCI_DEVICE_ID_ALI5459			0x5459
#define PCI_DEVICE_ID_ALI545A			0x545A

#define PCI_VENDOR_ID_SMARTLINK                 0x2000		
#define PCI_VENDOR_ID_SMARTLINK_1		0x163c
#define PCI_VENDOR_ID_SMARTLINK_2		0x10a5
#define PCI_VENDOR_ID_SMARTLINK_3		0x2003
#define PCI_DEVICE_ID_SL2800			0x2800
#define PCI_DEVICE_ID_SL1900			0x3052
#define PCI_DEVICE_ID_ND92XPA                   0x8800 /* ND92XPA */


static struct pci_device_id softmodem_pci_tbl [] __devinitdata = {
	/* 163c:3052 */
	{PCI_VENDOR_ID_SMARTLINK_1, PCI_DEVICE_ID_SL1900, PCI_ANY_ID, PCI_ANY_ID },
	/* 10a5:3052 */
	{PCI_VENDOR_ID_SMARTLINK_2, PCI_DEVICE_ID_SL1900, PCI_ANY_ID, PCI_ANY_ID },
	/* 10b9:5459 */
	{PCI_VENDOR_ID_AL, PCI_DEVICE_ID_ALI5459, PCI_ANY_ID, PCI_ANY_ID },
	/* 10b9:5457 */
	{PCI_VENDOR_ID_AL, PCI_DEVICE_ID_ALI5457, PCI_ANY_ID, PCI_ANY_ID },
	/* 10b9:545a */
	{PCI_VENDOR_ID_AL, PCI_DEVICE_ID_ALI545A, PCI_ANY_ID, PCI_ANY_ID },
	/* 2000:2800 */
	{PCI_VENDOR_ID_SMARTLINK, PCI_DEVICE_ID_SL2800, PCI_ANY_ID, PCI_ANY_ID },
	/* 2003:8800 */
	{PCI_VENDOR_ID_SMARTLINK_3, PCI_DEVICE_ID_ND92XPA, PCI_ANY_ID, PCI_ANY_ID },
	/* pctel HSP1688 */
	{ 0x134d, 0x2189, PCI_ANY_ID, PCI_ANY_ID },
	/* phillips pci modem */
	{ 0x1131, 0x3400, PCI_ANY_ID, PCI_ANY_ID },
	{0,}
};


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13)
#define pci_match_id(tbl, dev) pci_match_device(tbl, dev)
#endif

static int __init softmodem_release_init(void)
{
	struct pci_dev *dev = NULL;
	struct device *reldev;
        while ((dev = pci_find_device(PCI_ANY_ID, PCI_ANY_ID, dev)) != NULL) {
		if(pci_match_id(softmodem_pci_tbl, dev) && pci_dev_driver(dev) &&
		   (reldev = get_device(&dev->dev)) ) {
			printk(KERN_INFO "device %04x:%04x is grabbed by driver %s: try to release\n",
				  dev->vendor, dev->device,
				  (reldev&&reldev->driver) ? reldev->driver->name : "unknown");
			device_release_driver(reldev);
			put_device(reldev);
		}
	}
	return 0;
}

static void __exit softmodem_release_exit(void)
{
}

module_init(softmodem_release_init);
module_exit(softmodem_release_exit);

MODULE_AUTHOR("linmodems.org");
MODULE_DESCRIPTION("No driver - just try release softmodem device");
MODULE_LICENSE("GPL");
