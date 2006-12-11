/*
 *    kernel-ver.c - prints Linux kernel version to stdout and exit.
 *
 */

#include <stdio.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18)
#include <linux/utsrelease.h>
#endif

int main()
{
        printf ("%s\n",UTS_RELEASE);
        return 0;
}

