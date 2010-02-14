---------------

Ungrab-Winmodem

---------------

This loadable module was written by Sasha Khapyorsky  - April 14, 2005

Purpose:
=======
Release slmodem devices which have been detected by Linux as if they were true
hardware modems and assigned by Linux to work with the standard serial driver.

When is it helpful?
===================
When you see something like:
here the "modprobe slamr debug=3" outout:
slamr: unsupported module, tainting kernel.
slamr: module license 'Smart Link Ltd.' taints kernel.
slamr: SmartLink AMRMO modem.
slamr: device 10b9:5457 is grabbed by driver serial

or the same after two commands   modprobe slamr
                                 dmesg | grep slamr

Installation:
=============

You must be root or superuser

make
make install

Usage:
======

modprobe ungrab-winmodem

The modprobe command MUST be given BEFORE loading slamr

This text written by Jacques Goldberg
Questions: discuss@linmodems.org    , Subject: ungrab-winmodem
