Introduction
============

This directory contains a native driver for FLI USB devices that
should work with any Linux kernel 2.6 and higher.  The driver creates a
character special file `/dev/fliusbi<i>`, where i=0,1,2... is the device
number, when a recognized FLI USB device is attached.  The character
special file provides two ways to communicate with FLI USB devices;
through `read()`/`write()` system calls or by using `ioctl()`.

Modifications
=============

This driver has been modified by the Las Cumbres Observatory (LCO) to make
it work with newer Linux kernel versions with more modern features.

The modifications have fixed several bugs, the most prominent of which is that
the driver no longer crashes if the camera is unplugged/power cycled while in
use.

Compile-time options which were not enabled by default have been removed to
keep the code simple and easy to understand.

**Please note that this source code repository is not associated with FLI.**

Compiling
=========

Use the `make` command to compile the driver.  This will create the
kernel module `fliusb.ko` that can then be loaded using the `insmod`
command.

Compile Options
===============

Compile-time options are controlled by defining the following
preprocessor macros.  Each option is briefly described below.

`DEBUG`		This option makes the driver more verbose and causes
		it to print a basic description of what it's doing as
		various operations are performed.  This has a negative
		impact on performance but might be useful for
		debugging purposes.

Module Parameters
=================

`buffersize`	The default size of the driver buffer (per FLI USB
		device).  This also acts as the threshold for when
		scatter-gather reads are done, if enabled.

`timeout`	The default timeout, in milliseconds, for `read()` and
		write() system calls.

`ioctl()` Commands
==================

The supported `ioctl()` commands are defined in `fliusb_ioctl.h`.
Parameters for each command are passed by pointer using the third
argument to the `ioctl()` system call.  See `fliusb_ioctl.h` for the
pointer type each command expects.  The commands listed below are
recognized.

`FLIUSB_GETRDEPADDR`		Get the current USB endpoint address
				used when a `read()` is performed.

`FLIUSB_SETRDEPADDR`		Set the USB endpoint address used when
				a `read()` is performed.

`FLIUSB_GETWREPADDR`		Get the current USB endpoint address
				used when a `write()` is performed.

`FLIUSB_SETWREPADDR` 		Set the USB endpoint address used when
				a `write()` is performed.

`FLIUSB_GETBUFFERSIZE`		Get the current buffer size.

`FLIUSB_SETBUFFERSIZE`		Set the buffer size.

`FLIUSB_GETTIMEOUT`		Get the current timeout, in milliseconds.

`FLIUSB_SETTIMEOUT`		Set the current timeout, in milliseconds.

`FLIUSB_BULKREAD`		Perform a bulk read.

`FLIUSB_BULKWRITE`		Perform a bulk write.

`FLIUSB_GET_DEVICE_DESCRIPTOR`	Get the USB device descriptor.
