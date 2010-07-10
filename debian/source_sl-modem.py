
def add_info(report, ui):
  Dialup = ui.yesno("Is there a problem with dialup ?")
  if Dialup == None: #user cancelled
    raise StopIteration
  Fax = ui.yesno("Is there a problem with fax ?")
  if Fax == None: #user cancelled
    raise StopIteration

  if Fax == True:
    tags.append("fax")
    SLModemdDevice = command_output(['sh', '-c', '[ ! -r /etc/default/sl-modem-daemon ] || (. /etc/default/sl-modem-daemon ; echo $SLMODEMD_DEVICE)'])
    SLModemdOpts = command_output(['sh', '-c', '[ ! -r /etc/default/sl-modem-daemon ] || (. /etc/default/sl-modem-daemon ; echo $OPTS)'])
    if ':' in SLModemdDevice or 'alsa' in SLModemdOpts:
      ui.information("sl-modem is configured for ALSA mode, yet fax does not work with ALSA mode")
      # If there isn't a problem with dialup, then abort the bug report:
      if Dialup != True:
        raise StopIteration

    lsmod_out = command_output(['lsmod'])
    if not 'slusb' in lsmod_out and not 'slamr' in lsmod_out:
      ui.information("SLMODEM_DEVICE is set to auto, and neither slamr nor slusb is loaded, please explicitly set SLMODEM_DEVICE to slamr ( or slusb if you are using a USB modem), and try again before continuing with this bug report.")


  if Dialup == True:
    tags.append("dialup")
    response = ui.yesno("It is recommended to test with wvdial before continuing with this bug report. Did you try dialup using wvdial utility ?")
    if response == None: #user cancelled
      raise StopIteration

    if response == True:
      WvDialConf = ui.question_file("Please enter the path to wvdial configuration file, by default it is /etc/wvdial.conf, note that this file will be included in the bug report after removing Username and Password fields")
    if WvDialConf == None: #user cancelled
      if os.path.isfile('/etc/wvdial.conf'):
        WvDialConf = '/etc/wvdial.conf'

  attach_conffiles(report, 'sl-modem-daemon')
  attach_alsa(report)
  attach_related_packages(report, [
    "sl-modem-source"
    ])
  report['Lsmod'] = command_output(['lsmod'])
  attach_file_if_exists(report, "/var/log/slmodemd/slmodem.log")
  report['Lspci'] = command_output(['lspci','-vvnn'])
  report['Lsusb'] = command_output(['lsusb'])
  attach_file(report, '/proc/version_signature', 'ProcVersionSignature')
  report['DevNodes'] = command_output(['sh', '-c',
    'ls -l /dev/ttySL*  /dev/pts/*  /dev/slamr* /dev/slusb*'])

  if Dialup == True and WvDialConf != None:
    report['WvDialConf'] = command_output(['sed', '-e',
      '/^\s*\(Password\|Username\)/ d', WvDialConf])
