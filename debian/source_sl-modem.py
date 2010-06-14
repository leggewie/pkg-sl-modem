
def add_info(report):
  attach_conffiles(report, 'sl-modem-daemon')
  attach_alsa(report)
	attach_related_packages(report, [
		"sl-modem-source",
		] )
  report['Lsmod'] = command_output(['lsmod'])
  attach_file_if_exists(report, "/var/log/slmodemd/slmodem.log")
  report['Lspci'] = command_output(['lspci','-vvnn'])
  report['Lsusb'] = command_output(['lsusb'])
	attach_file(report, '/proc/version_signature', 'ProcVersionSignature')
