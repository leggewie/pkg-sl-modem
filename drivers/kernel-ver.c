/*
 *    kernel-ver.c - prints Linux kernel version to stdout and exit.
 *
 */

#include <stdio.h>
#include <linux/version.h>

int main()
{
        printf ("%s\n",UTS_RELEASE);
        return 0;
}

