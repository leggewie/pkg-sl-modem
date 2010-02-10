# Generate version information for kernel module
%define kname %(echo `uname -r`)
%define kver  %(echo %{kname} | sed -e 's/smp//' -e 's/bigmem//' -e 's/enterprise//')
%define ktype %(echo kernel-%{kname}|sed -e 's/%{kver}//' -e 's/-$//')
%define krel  %(echo %{kname} | sed -e 's/-/_/g')

###############################################################################
#
# Common Package Information
#
###############################################################################
Name: slmodem
Version: 2.9.4
Release: 2
Distribution: Unknown
License: SmartLink
URL: ftp://ftp.smlink.com/linux/unsupported/
# Alternative: URL: http://linmodems.technion.ac.il/packages/smartlink/
Vendor: Smart Link Ltd.

#
# Source files (taken from $RPM_SOURCE_DIR)
#
Source0: %{name}-%{version}.tar.gz
Source1: slmodemd

#
# Patches (taken from $RPM_SOURCE_DIR))
#
#Patch0: XYZ.patch

#
# Build Requirements
#
#buildarch: noarch
BuildRequires: kernel-source = %{kver}

# Package preparation area
BuildRoot: %{_tmppath}/%{name}

###############################################################################
#
# Package Information (Main)
#
###############################################################################
Summary: Driver for Smart Link HAMR5600 winmodem (user space)
Group: System Environment/Driver

AutoReq: 1
#PreReq:   Describe dependencies to install this package (see %pre/%post)
#Requires: Describe dependencies to use this package
AutoProv: 1
#Provides: Describe what this package provides

%description
This is the Linux driver for the Smart Link Soft Modem HAMR5600 hardware.
It provides a full-featured 56K Voice Fax Modem.

This winmodem hardware is used e.g. in the IBM ThinkPad T30.

This package includes the generic application (slmodemd) and the init script
for it.

%package -n kernel-module-%{name}
###############################################################################
#
# Subpackage Information (kernel module)
#
###############################################################################
Summary: Driver for Smart Link HAMR5600 winmodem (kernel space)
Group: System Environment/Kernel
Release: %{krel}

AutoReq: 0
#PreReq:   Describe dependencies to install this subpackage (see %pre/%post)
Requires: %{name} = %{version}
Requires: %{ktype} = %{kver}
AutoProv: 0
#Provides: Describe what this subpackage provides

%description -n kernel-module-%{name}
This is the Linux driver for the Smart Link Soft Modem HAMR5600 hardware.
It provides a full-featured 56K Voice Fax Modem.

This winmodem hardware is used e.g. in the IBM ThinkPad T30.

This package includes the hardware specific kernel-space drivers (slamr,
slusb) and the device nodes for them.

###############################################################################
#
# Build Phase Information
#
###############################################################################
# Define %_make_cmd in $HOME/.rpmmacros to override the standard make
%if !%{?_make_cmd:1}0
%define _make_cmd make
%endif

# Use '--define "_skip_build 1"' during package testing to skip build phases
%if !%{?_skip_build:1}0

%prep
#
# Preparation (-bp): Unpack the source files to $RPM_BUILD_DIR...
#
%setup -q

# ... apply patches
#%patch0 -p1 -b .XYZ

%build
#
# Compilation (-bc): Configure and build sources in $RPM_BUILD_DIR
#
export KERNEL_VER=%{kver}
%{_make_cmd} KERNEL_DIR=/lib/modules/%{kname}/build

%install
#
# Installation (-bi): Copy data from build $RPM_BUILD_DIR to
#                     package preparation area %{buildroot}
#
# "install" from Makefile is broken, so do everything by hand...
rm -rf %{buildroot}
install -D -m 755 modem/slmodemd %{buildroot}/usr/sbin/slmodemd
install -D -m 755 -d %{buildroot}/var/lib/slmodemd
install -D -m 644 drivers/slamr.o %{buildroot}/lib/modules/%{kname}/misc/slamr.o
install -D -m 644 drivers/slusb.o %{buildroot}/lib/modules/%{kname}/misc/slusb.o

# Hack device for kppp
mkdir -p %{buildroot}/dev
ln -sf /dev/ttySL0 %{buildroot}/dev/modem

# Install service file
install -D -m 755 %{SOURCE1} %{buildroot}/etc/init.d/slmodemd


%clean
#
# Installation cleanup: Delete package preparation area %{buildroot}
#
rm -rf %{buildroot}

%else # !%{_skip_build}
# Fix environment when build phases are skipped...
%define _builddir %{_topdir}/BUILD/%{name}-%{version}
%endif # !%{_skip_build}

%files
###############################################################################
#
# Packaging Phase Information (Main)
#
###############################################################################
#
# Package Content
#
%defattr (-,root,root)

# Directories owned by this package
%dir /var/lib/slmodemd

# Files owned by this package (taken from %{buildroot}
/usr/sbin/slmodemd
/etc/init.d/slmodemd

# Documentation for this package (taken from $RPM_BUILD_DIR)
%doc README
%doc README.1st

#
# Package Installation Scripts
#
# NOTE: During "update" of a package the order of execution is
#
#        %pre       (NEW)
#        %post      (NEW)
#        %preun     (OLD)
#        %postun    (OLD)
#
#%pre
# Commands to execute before package is installed (see also Prereq:)
# Parameter $1:  1 = install, 2 = update

#%post
# Commands to execute after package is installed (see also Prereq:)
# Parameter $1:  1 = install, 2 = update

#%preun
# Commands to execute before package is uninstalled
# Parameter $1:  0 = remove, 1 = update

#%postun
# Commands to execute after package is uninstalled
# Parameter $1:  0 = remove, 1 = update

%files -n kernel-module-%{name}
###############################################################################
#
# Packaging Phase Information (Subpackage kernel module)
#
###############################################################################
#
# Package Content
#
%defattr (-,root,root)

# Directories owned by this package
#%dir %{_datadir}/...

# Files owned by this package (taken from %{buildroot})
/lib/modules/%{kname}/misc/slamr.o
/lib/modules/%{kname}/misc/slusb.o

# Devices
%defattr (600,root,root)
%dev (c, 212, 0) /dev/slamr0
%dev (c, 212, 1) /dev/slamr1
%dev (c, 212, 2) /dev/slamr2
%dev (c, 212, 3) /dev/slamr3
%dev (c, 213, 0) /dev/slusb0
%dev (c, 213, 1) /dev/slusb1
%dev (c, 213, 2) /dev/slusb2
%dev (c, 213, 3) /dev/slusb3
/dev/modem

# Documentation for this package (taken from $RPM_BUILD_DIR)
#%doc ChangeLog

#
# Package Installation Scripts
#
# NOTE: During "update" of a package the order of execution is
#
#        %pre       (NEW)
#        %post      (NEW)
#        %preun     (OLD)
#        %postun    (OLD)
#
#%pre -n kernel-module-%{name}
# Commands to execute before package is installed (see also Prereq:)
# Parameter $1:  1 = install, 2 = update

%post -n kernel-module-%{name}
# Commands to execute after package is installed (see also Prereq:)
# Parameter $1:  1 = install, 2 = update
if [ $1 = 1 ]; then
	cat << EOF >> /etc/modules.conf
# added by kernel-module-slmodem package
alias char-major-212 slamr
alias char-major-213 slusb
EOF
fi
/sbin/depmod -A

%preun -n kernel-module-%{name}
# Commands to execute before package is uninstalled
# Parameter $1:  0 = remove, 1 = update
service slmodemd stop
modprobe -r slamr slusb
if [ $1 = 0 ]; then
	rm -f /dev/ttySL*
	egrep -ve 'added by kernel-module-slmodem package|alias char-major-212 slamr|alias char-major-213 slusb' /etc/modules.conf >/tmp/modules.conf.slmodem$$
	mv /tmp/modules.conf.slmodem$$ /etc/modules.conf
fi
/sbin/depmod -A

#%postun -n kernel-module-%{name}
# Commands to execute after package is uninstalled
# Parameter $1:  0 = remove, 1 = update

%changelog
###############################################################################
#
# Package Change History
#
###############################################################################
* Tue Nov 05 2003 Stefan Becker <Stefan.Becker@nokia.com>
- The HW doesn't seem to like suspend. Added removal of the kernel modules
  when slmodemd is stopped, so HW can be kicked by restarting the service.

* Mon Nov 04 2003 Stefan Becker <Stefan.Becker@nokia.com>
- Initial version based on slmodem-2.9.2

