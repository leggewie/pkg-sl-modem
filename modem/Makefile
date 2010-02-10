###########################################################################
#
#
#       Makefile  --  modem application Makefile.
#
#       Copyright(c) 2003, Smart Link Ltd. (www.smlink.com)
#	All rights reserved.
#
#       Author: Sasha K (sashak@smlink.com)
#
#
###########################################################################
#
###########################################################################

CC:= gcc
RM:= rm -f

CFLAGS:= -Wall -g -O -I. -DCONFIG_DEBUG_MODEM


modem-objs:= \
	modem.o modem_datafile.o modem_at.o modem_timer.o \
	modem_pack.o modem_ec.o modem_comp.o \
	modem_param.o modem_debug.o homolog_data.o
dp-objs:= dp_sinus.o dp_dummy.o
sysdep-objs:= sysdep_common.o

all: slmodemd modem_test

slmodemd: modem_main.o modem_cmdline.o $(modem-objs) $(dp-objs) dsplibs.o $(sysdep-objs)
modem_test: modem_test.o modem_cmdline.o $(modem-objs) $(dp-objs) dsplibs.o $(sysdep-objs)

#SUPPORT_ALSA:=1
ifdef SUPPORT_ALSA
slmodemd: -lasound
CFLAGS+= -DSUPPORT_ALSA=1
endif

slmodemd modem_test:
	$(CC) -o $@ $^

clean:
	$(RM) slmodemd modem_test modem_main.o modem_cmdline.o modem_test.o $(modem-objs) $(dp-objs) $(sysdep-objs)
	$(RM) *~ *.orig *.rej

.PHONY: all dep generic-dep clean clean-build-profile


#
# rules
#

%.o: %.c .build_profile
	$(CC) $(CFLAGS) $(EXTRA_CFLAGS) $(CFLAGS_$@) -o $@ -c $<

dep: generic-dep
generic-dep:
	$(RM) .depend
	$(foreach src,$(wildcard *.c),$(CC) -M $(CFLAGS) $(EXTRA_CFLAGS) $(CFLAGS_$(src:.c=.o)) -c $(src) >> .depend ;)

-include .depend

# build_profile
-include .build_profile

.build_profile::
ifneq ($(CFLAGS),$(PROFILE_CFLAGS))
	@if [ "$(CFLAGS) $(EXTRA_CFLAGS)" != "$(PROFILE_CFLAGS)" ] ; then \
	  echo "rebuild profile..." ; \
	  echo "PROFILE_CFLAGS=$(CFLAGS) $(EXTRA_CFLAGS)" > $(@) ; fi
endif

clean: clean-build-profile
clean-build-profile:
	@$(RM) -f .build_profile

