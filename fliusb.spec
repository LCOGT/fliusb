Name:           fliusb
Version:        2.2
Release:        lcogt
Summary:        A kernel module source package for fliusb

License:        GPL
Source0:        %{name}-%{version}.tar.gz

Requires:       make, gcc, kernel-devel
BuildArch:      noarch

%description
This package contains the source code for the kernel module fliusb and a Makefile. After installing this package, the kernel module can be built on the target system.

%prep
%setup -q

%build
# Do nothing here because we will build on the target machine

%install
# Install the source files and Makefile in /usr/src/appname
install -d %{buildroot}/usr/src/%{name}
cp -r * %{buildroot}/usr/src/%{name}

%files
/usr/src/%{name}


