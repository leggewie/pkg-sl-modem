#!/bin/sh

if test "$1" = ""
then
  if test `ps aux | grep -c -e "[s]lmodemd"` = "1"
  then
    killall -9 slmodemd
  fi
  rm -f /dev/modem
  rm -f /etc/wvdial.conf
  exit
fi

if test "$1" = "sm56"
then
  if test `ps aux | grep -c -e "[s]lmodemd"` = "1"
  then
    killall -9 slmodemd
  fi
  /usr/sbin/slmodemd -c FRANCE --alsa hw:0,6 &
  cd /etc
  rm -f /etc/wvdial.conf
  ln -s wvdial/wvdial.conf.sm56 wvdial.conf
  cd /dev
  rm -f /dev/modem  
  sleep 1
  FILE=`ls -al ttySL0 | cut -d">" -f 2`
  FILE=`echo $FILE`
  echo "chmod a+rw $FILE"
  chmod a+rw $FILE ; ln -s ttySL0 modem
  exit
fi

if test "$1" = "olitec"
then
  if test `ps aux | grep -c -e "[s]lmodemd"` = "1"
  then
    killall -9 slmodemd
  fi
  cd /etc
  rm -f /etc/wvdial.conf
  ln -s wvdial/wvdial.conf.olitec wvdial.conf
  cd /dev
  rm -f /dev/modem  
  chmod a+rw ttyUSB0 ; ln -s ttyUSB0 modem
  exit
fi

if test "$1" = "ppp"
then
  /sbin/ifconfig wlan0 down
fi

  