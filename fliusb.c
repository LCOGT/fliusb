/*

  Copyright (c) 2005 Finger Lakes Instrumentation (FLI), L.L.C.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

        Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.

        Redistributions in binary form must reproduce the above
        copyright notice, this list of conditions and the following
        disclaimer in the documentation and/or other materials
        provided with the distribution.

        Neither the name of Finger Lakes Instrumentation (FLI), L.L.C.
        nor the names of its contributors may be used to endorse or
        promote products derived from this software without specific
        prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.

  ======================================================================

  Finger Lakes Instrumentation, L.L.C. (FLI)
  web: http://www.fli-cam.com
  email: support@fli-cam.com

*/

#include <linux/version.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kref.h>
#include <linux/errno.h>
#include <linux/usb.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/slab.h>
#include <asm/uaccess.h>

#ifdef SGREAD
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/scatterlist.h>
#include <asm/scatterlist.h>
#endif

#include "fliusb_ioctl.h"

#define FLIUSB_NAME "fliusb"
#define FLIUSB_MINOR_BASE 240	/* This is arbitrary */

#define FLIUSB_VENDORID			0x0f18

#define FLIUSB_PRODID_MAXCAM		0x0002
#define FLIUSB_PRODID_STEPPER		0x0005
#define FLIUSB_PRODID_FOCUSER		0x0006
#define FLIUSB_PRODID_FILTERWHEEL	0x0007
#define FLIUSB_PRODID_PROLINECAM	0x000a

/* Default values (module parameters override these) */
#define FLIUSB_TIMEOUT 5000	/* milliseconds */
#ifndef SGREAD
	#define FLIUSB_BUFFERSIZE 65536
#else
	#define FLIUSB_BUFFERSIZE PAGE_SIZE
#endif /* SGREAD */

/* Model-specific parameters */
#define FLIUSB_RDEPADDR 0x82
#define FLIUSB_WREPADDR 0x02
#define FLIUSB_PROLINE_RDEPADDR 0x81
#define FLIUSB_PROLINE_WREPADDR 0x01

#ifdef SGREAD

#define NUMSGPAGE 32

struct fliusb_sg {
	struct page *userpg[NUMSGPAGE];
	struct scatterlist slist[NUMSGPAGE];
	unsigned int maxpg;
	struct usb_sg_request sgreq;
	struct timer_list timer;
	struct semaphore sem;
};

#endif /* SGREAD */

struct fliusb_dev {
	/* Bulk transfer pipes used for read()/write() */
	unsigned int rdbulkpipe;
	unsigned int wrbulkpipe;

	/* Kernel buffer used for bulk reads */
	void *buffer;
	unsigned int buffersize;
	struct semaphore buffsem;

#ifdef SGREAD
	struct fliusb_sg usbsg;
#endif

	unsigned int timeout;	/* timeout for bulk transfers in milliseconds */

	struct usb_device *usbdev;
	struct usb_interface *interface;

	struct kref kref;
};

#define FLIUSB_ERR(fmt, args...) \
	printk(KERN_ERR "%s[%d]: " fmt "\n", __FUNCTION__, __LINE__ , ##args)

#define FLIUSB_WARN(fmt, args...) \
	printk(KERN_WARNING "%s[%d]: " fmt "\n", __FUNCTION__, __LINE__ , ##args)

#define FLIUSB_INFO(fmt, args...) \
	printk(KERN_NOTICE "%s[%d]: " fmt "\n", __FUNCTION__, __LINE__ , ##args)

#ifdef DEBUG
	#define FLIUSB_DBG(fmt, args...) FLIUSB_INFO(fmt , ##args)
#else
	#define FLIUSB_DBG(fmt, args...)  do {		\
		if (0) {				\
			FLIUSB_INFO(fmt, ##args);	\
		}					\
	} while (0)
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 36) && !defined(init_MUTEX)
	#define init_MUTEX(sem) sema_init(sem, 1)
#endif

/* Module parameters */
static unsigned int param_buffersize = FLIUSB_BUFFERSIZE;
module_param_named(buffersize, param_buffersize, uint, S_IRUGO);
MODULE_PARM_DESC(buffersize, "USB bulk transfer buffer size");

static unsigned int param_timeout = FLIUSB_TIMEOUT;
module_param_named(timeout, param_timeout, uint, S_IRUGO);
MODULE_PARM_DESC(timeout, "USB bulk transfer timeout (msec)");

/* Forward declarations */
static struct usb_driver fliusb_driver;

static void fliusb_delete(struct kref *kref)
{
	struct fliusb_dev *dev = container_of(kref, struct fliusb_dev, kref);

	usb_put_dev(dev->usbdev);

	if (dev->buffer)
		kfree(dev->buffer);

	kfree(dev);
}

static int fliusb_allocbuffer(struct fliusb_dev *dev, unsigned int size)
{
	void *tmp;
	int err = 0;

	if (size == 0)
		return -EINVAL;

	if (down_interruptible(&dev->buffsem))
		return -ERESTARTSYS;

	if ((tmp = kmalloc(size, GFP_KERNEL)) == NULL) {
		FLIUSB_WARN("kmalloc() failed: could not allocate %d byte buffer", size);
		err = -ENOMEM;
		goto done;
	}

	if (dev->buffer != NULL)
		kfree(dev->buffer);

	dev->buffer = tmp;
	dev->buffersize = size;

done:
	up(&dev->buffsem);
	return err;
}

static int fliusb_open(struct inode *inode, struct file *file)
{
	struct fliusb_dev *dev;
	struct usb_interface *interface;
	int minor = iminor(inode);

	if ((interface = usb_find_interface(&fliusb_driver, minor)) == NULL) {
		FLIUSB_ERR("no interface found for minor number %d", minor);
		return -ENODEV;
	}

	if ((dev = usb_get_intfdata(interface)) == NULL) {
		FLIUSB_WARN("no device data for minor number %d", minor);
		return -ENODEV;
	}

	/* increment usage count */
	kref_get(&dev->kref);

	/* save a pointer to the device structure for later use */
	file->private_data = dev;

	return 0;
}

static int fliusb_release(struct inode *inode, struct file *file)
{
	struct fliusb_dev *dev = file->private_data;

	if (dev == NULL) {
		FLIUSB_ERR("no device data for minor number %d", iminor(inode));
		return -ENODEV;
	}

	/* decrement usage count */
	kref_put(&dev->kref, fliusb_delete);

	return 0;
}

static int fliusb_simple_bulk_read(struct fliusb_dev *dev, unsigned int pipe,
				   char __user *userbuffer, size_t count,
				   unsigned int timeout)
{
	int err, cnt;

	if (count > dev->buffersize)
		count = dev->buffersize;

	if (!access_ok(VERIFY_WRITE, userbuffer, count))
		return -EFAULT;

	if (down_interruptible(&dev->buffsem))
		return -ERESTARTSYS;

	/* a simple blocking bulk read */
	if ((err = usb_bulk_msg(dev->usbdev, pipe, dev->buffer, count, &cnt, timeout))) {
		cnt = err;
		goto done;
	}

	if (__copy_to_user(userbuffer, dev->buffer, cnt))
		cnt = -EFAULT;

done:
	up(&dev->buffsem);
	return cnt;
}

#ifdef SGREAD

static void fliusb_sg_bulk_read_timeout(unsigned long data)
{
	struct fliusb_dev *dev = (struct fliusb_dev *)data;

	FLIUSB_ERR("bulk read timed out");
	usb_sg_cancel(&dev->usbsg.sgreq);
}

static int fliusb_sg_bulk_read(struct fliusb_dev *dev, unsigned int pipe,
			       char __user *userbuffer, size_t count,
			       unsigned int timeout)
{
	int err, i;
	unsigned int numpg;
	size_t pgoffset;

	/*
	 * userbuffer must be aligned to a multiple of the endpoint's
	 * maximum packet size
	 */
	if ((size_t)userbuffer % usb_maxpacket(dev->usbdev, pipe, 0)) {
		FLIUSB_ERR("user buffer is not properly aligned: 0x%p %% 0x%04x", userbuffer, usb_maxpacket(dev->usbdev, pipe, 0));
		return -EINVAL;
	}

	pgoffset = (size_t)userbuffer & (PAGE_SIZE - 1);
	numpg = ((count + pgoffset - 1) >> PAGE_SHIFT) + 1;

	if (numpg > dev->usbsg.maxpg) {
		numpg = dev->usbsg.maxpg;
		count = PAGE_SIZE - pgoffset + PAGE_SIZE * (dev->usbsg.maxpg - 1);
	}

	if (down_interruptible(&dev->usbsg.sem))
		return -ERESTARTSYS;

	down_read(&current->mm->mmap_sem);
	numpg = get_user_pages(current, current->mm, (size_t)userbuffer & PAGE_MASK,
			       numpg, 1, 0, dev->usbsg.userpg, NULL);
	up_read(&current->mm->mmap_sem);

	if (numpg <= 0) {
		FLIUSB_ERR("get_user_pages() failed: %d", numpg);
		err = numpg;
		goto done;
	}

#if (LINUX_VERSION_CODE < KERNEL_VERSION (2,6,24))
	dev->usbsg.slist[0].page = dev->usbsg.userpg[0];
	dev->usbsg.slist[0].offset = pgoffset;
	dev->usbsg.slist[0].length = min(count, (size_t)(PAGE_SIZE - pgoffset));
#else
	sg_set_page(&dev->usbsg.slist[0], dev->usbsg.userpg[0], min(count, (size_t)(PAGE_SIZE - pgoffset)), pgoffset);
#endif

	if (numpg > 1) {
		for (i = 1; i < numpg - 1; i++) {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,24))
			dev->usbsg.slist[i].page = dev->usbsg.userpg[i];
			dev->usbsg.slist[i].offset = 0;
			dev->usbsg.slist[i].length = PAGE_SIZE;
		}

		dev->usbsg.slist[i].page = dev->usbsg.userpg[i];
		dev->usbsg.slist[i].offset = 0;
		dev->usbsg.slist[i].length = ((size_t)userbuffer + count) & (PAGE_SIZE - 1);
		if (dev->usbsg.slist[i].length == 0)
			dev->usbsg.slist[i].length = PAGE_SIZE;
#else
			sg_set_page(&dev->usbsg.slist[i], dev->usbsg.userpg[i], PAGE_SIZE, 0);
		}

		if ((((size_t)userbuffer + count) & (PAGE_SIZE - 1)) == 0) {
			sg_set_page(&dev->usbsg.slist[i], dev->usbsg.userpg[i], PAGE_SIZE, 0);
		} else {
			sg_set_page(&dev->usbsg.slist[i], dev->usbsg.userpg[i], ((size_t)userbuffer + count) & (PAGE_SIZE - 1), 0);
		}
#endif
	}

	if ((err = usb_sg_init(&dev->usbsg.sgreq, dev->usbdev, pipe, 0,
			 dev->usbsg.slist, numpg, 0, GFP_KERNEL))) {
		FLIUSB_ERR("usb_sg_init() failed: %d", err);
		goto done;
	}

	dev->usbsg.timer.expires = jiffies + (timeout * HZ + 500) / 1000;
	dev->usbsg.timer.data = (unsigned long)dev;
	dev->usbsg.timer.function = fliusb_sg_bulk_read_timeout;
	add_timer(&dev->usbsg.timer);

	/* wait for the transfer to complete */
	usb_sg_wait(&dev->usbsg.sgreq);

	del_timer_sync(&dev->usbsg.timer);

	if (dev->usbsg.sgreq.status) {
		FLIUSB_ERR("bulk read error %d; transfered %d bytes", (int) dev->usbsg.sgreq.status, (int) dev->usbsg.sgreq.bytes);
		err = dev->usbsg.sgreq.status;
		goto done;
	}

	err = dev->usbsg.sgreq.bytes;

done:
	up(&dev->usbsg.sem);

	for (i = 0; i < numpg; i++) {
		if (!PageReserved(dev->usbsg.userpg[i]))
			SetPageDirty(dev->usbsg.userpg[i]);

		page_cache_release(dev->usbsg.userpg[i]);
	}

	return err;
}

#endif /* SGREAD */

static int fliusb_bulk_read(struct fliusb_dev *dev, unsigned int pipe,
			    char __user *userbuffer, size_t count,
			    unsigned int timeout)
{
	FLIUSB_DBG("pipe: 0x%08x; userbuffer: %p; count: %zu; timeout: %u",
			pipe, userbuffer, count, timeout);

#ifdef SGREAD
	if (count > dev->buffersize)
		return fliusb_sg_bulk_read(dev, pipe, userbuffer, count, timeout);
	else
#endif /* SGREAD */
		return fliusb_simple_bulk_read(dev, pipe, userbuffer, count, timeout);
}

#ifndef ASYNCWRITE

static int fliusb_bulk_write(struct fliusb_dev *dev, unsigned int pipe,
			     const char __user *userbuffer, size_t count,
			     unsigned int timeout)
{
	int err, cnt;

	FLIUSB_DBG("pipe: 0x%08x; userbuffer: %p; count: %zu; timeout: %u",
			pipe, userbuffer, count, timeout);

	if (count > dev->buffersize)
		count = dev->buffersize;

	if (down_interruptible(&dev->buffsem))
		return -ERESTARTSYS;

	if (copy_from_user(dev->buffer, userbuffer, count)) {
		cnt = -EFAULT;
		goto done;
	}

	/* a simple blocking bulk write */
	if ((err = usb_bulk_msg(dev->usbdev, pipe, dev->buffer, count, &cnt, timeout))) {
		cnt = err;
		// reset USB in case of an error
		err = usb_reset_configuration (dev->usbdev);
		FLIUSB_DBG("configuration return: %d", err);
	}

done:
	up(&dev->buffsem);
	return cnt;
}

#else

static void fliusb_bulk_write_callback(struct urb *urb, struct pt_regs *regs)
{
	switch (urb->status) {
	case 0:
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		break;			/* These aren't noteworthy */
	default:
		FLIUSB_WARN("bulk write error %d; transfered %d bytes",
		urb->status, urb->actual_length);
		break;
	}

	usb_buffer_free(urb->dev, urb->transfer_buffer_length,
	urb->transfer_buffer, urb->transfer_dma);
}

static int fliusb_bulk_write(struct fliusb_dev *dev, unsigned int pipe,
			     const char __user *userbuffer, size_t count,
			     unsigned int timeout)
{
	int err;
	struct urb *urb = NULL;
	char *buffer = NULL;

	FLIUSB_DBG("pipe: 0x%08x; userbuffer: %p; count: %u; timeout: %u",
			pipe, userbuffer, count, timeout);

	if (!access_ok(VERIFY_READ, userbuffer, count))
		return -EFAULT;

	if ((urb = usb_alloc_urb(0, GFP_KERNEL)) == NULL)
		return -ENOMEM;

	if ((buffer = usb_buffer_alloc(dev->usbdev, count, GFP_KERNEL, &urb->transfer_dma)) == NULL) {
		err = -ENOMEM;
		goto error;
	}

	if (__copy_from_user(buffer, userbuffer, count)) {
		err = -EFAULT;
		goto error;
	}

	usb_fill_bulk_urb(urb, dev->usbdev, pipe, buffer, count, fliusb_bulk_write_callback, dev);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	if ((err = usb_submit_urb(urb, GFP_KERNEL)))
		goto error;

	usb_free_urb(urb);

	return count;

error:
	if (buffer)
		usb_buffer_free(dev->usbdev, count, buffer, urb->transfer_dma);

	if (urb)
		usb_free_urb(urb);

	return err;
}

#endif /* ASYNCWRITE */

static ssize_t fliusb_read(struct file *file, char __user *userbuffer,
			   size_t count, loff_t *ppos)
{
	struct fliusb_dev *dev = file->private_data;

	if (count == 0)
		return 0;

	if (*ppos)
		return -ESPIPE;

	return fliusb_bulk_read(dev, dev->rdbulkpipe, userbuffer, count, dev->timeout);
}

static ssize_t fliusb_write(struct file *file, const char __user *userbuffer,
			    size_t count, loff_t *ppos)
{
	struct fliusb_dev *dev = file->private_data;

	if (count == 0)
		return 0;

	if (*ppos)
		return -ESPIPE;

	return fliusb_bulk_write(dev, dev->wrbulkpipe, userbuffer, count, dev->timeout);
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 36)
static long fliusb_ioctl(struct file *file,
                        unsigned int cmd, unsigned long arg)
#else
static int fliusb_ioctl(struct inode *inode, struct file *file,
			unsigned int cmd, unsigned long arg)
#endif
{
	struct fliusb_dev *dev = file->private_data;

	FLIUSB_DBG("cmd: 0x%08x; arg: 0x%08lx", cmd, arg);

	if (_IOC_TYPE(cmd) != FLIUSB_IOC_TYPE || _IOC_NR(cmd) > FLIUSB_IOC_MAX)
		return -ENOTTY;

	switch (cmd) {
	case FLIUSB_GETBUFFERSIZE:
		return put_user(dev->buffersize, (unsigned int __user *)arg);

	case FLIUSB_GETTIMEOUT:
		return put_user(dev->timeout, (unsigned int __user *)arg);

	case FLIUSB_GETRDEPADDR: {
		u8 tmp = usb_pipeendpoint(dev->rdbulkpipe) | USB_DIR_IN;
		return put_user(tmp, (u8 __user *)arg);
	}

	case FLIUSB_GETWREPADDR: {
		u8 tmp = usb_pipeendpoint(dev->wrbulkpipe) | USB_DIR_OUT;
		return put_user(tmp, (u8 __user *)arg);
	}

	case FLIUSB_SETRDEPADDR: {
		unsigned int pipe;
		u8 tmp;

		if (get_user(tmp, (u8 __user *)arg))
			return -EFAULT;

		pipe = usb_rcvbulkpipe(dev->usbdev, tmp);
		if (usb_maxpacket(dev->usbdev, pipe, 0) == 0) {
			FLIUSB_ERR("invalid read USB bulk transfer endpoint address: 0x%02x", tmp);
			return -EINVAL;
		}

		dev->rdbulkpipe = pipe;
		return 0;
	}

	case FLIUSB_SETWREPADDR: {
		unsigned int pipe;
		u8 tmp;

		if (get_user(tmp, (u8 __user *)arg))
			return -EFAULT;

		pipe = usb_sndbulkpipe(dev->usbdev, tmp);

		if (usb_maxpacket(dev->usbdev, pipe, 1) == 0) {
			FLIUSB_ERR("invalid write USB bulk transfer endpoint address: 0x%02x", tmp);
			return -EINVAL;
		}

		dev->wrbulkpipe = pipe;
		return 0;
	}

	case FLIUSB_SETBUFFERSIZE: {
		unsigned int tmp;

		if (get_user(tmp, (unsigned int __user *)arg))
			return -EFAULT;

		return fliusb_allocbuffer(dev, tmp);
	}

	case FLIUSB_SETTIMEOUT: {
		unsigned int tmp;

		if (get_user(tmp, (unsigned int __user *)arg))
			return -EFAULT;

		if (tmp == 0) {
			FLIUSB_ERR("invalid timeout: %u", tmp);
			return -EINVAL;
		}

		dev->timeout = tmp;
		return 0;
	}

	case FLIUSB_BULKREAD: {
		fliusb_bulktransfer_t xfer;
		unsigned int pipe;

		if (copy_from_user(&xfer, (fliusb_bulktransfer_t __user *)arg, sizeof(xfer)))
			return -EFAULT;

		pipe = usb_rcvbulkpipe(dev->usbdev, xfer.ep);
		return fliusb_bulk_read(dev, pipe, xfer.buf, xfer.count, xfer.timeout);
	}

	case FLIUSB_BULKWRITE: {
		fliusb_bulktransfer_t xfer;
		unsigned int pipe;

		if (copy_from_user(&xfer, (fliusb_bulktransfer_t __user *)arg, sizeof(xfer)))
			return -EFAULT;

		pipe = usb_sndbulkpipe(dev->usbdev, xfer.ep);
		return fliusb_bulk_write(dev, pipe, xfer.buf, xfer.count, xfer.timeout);
	}

	case FLIUSB_GET_DEVICE_DESCRIPTOR: {
		struct usb_device_descriptor *desc = &dev->usbdev->descriptor;

		if (copy_to_user((void *)arg, desc, sizeof(*desc)))
			return -EFAULT;

		return 0;
	}

	case FLIUSB_GET_STRING_DESCRIPTOR: {
		fliusb_string_descriptor_t desc;
		int err;

		if (copy_from_user(&desc, (fliusb_string_descriptor_t __user *)arg, sizeof(desc)))
			return -EFAULT;

		memset(desc.buf, 0, sizeof(desc.buf));

		err = usb_string(dev->usbdev, desc.index, desc.buf, sizeof(desc.buf));
		if (err < 0)
			FLIUSB_WARN("usb_string() failed: %d", err);

		if (copy_to_user((void *)arg, &desc, sizeof(desc)))
			return -EFAULT;

		return 0;
	}

	default:
		FLIUSB_ERR("invalid ioctl request: %d", cmd);
		return -ENOTTY;
	}

	return -ENOTTY;		/* shouldn't get here */
}

static int fliusb_initdev(struct fliusb_dev *dev, struct usb_interface *interface,
			  const struct usb_device_id *id)
{
	char prodstr[64] = "unknown";
	int err;

	dev->usbdev = usb_get_dev(interface_to_usbdev(interface));
	dev->interface = interface;
	dev->timeout = param_timeout;

	switch (id->idProduct) {
	case FLIUSB_PRODID_MAXCAM:
	case FLIUSB_PRODID_STEPPER:
	case FLIUSB_PRODID_FOCUSER:
	case FLIUSB_PRODID_FILTERWHEEL:
		dev->rdbulkpipe = usb_rcvbulkpipe(dev->usbdev, FLIUSB_RDEPADDR);
		dev->wrbulkpipe = usb_sndbulkpipe(dev->usbdev, FLIUSB_WREPADDR);
		break;

	case FLIUSB_PRODID_PROLINECAM:
		dev->rdbulkpipe = usb_rcvbulkpipe(dev->usbdev, FLIUSB_PROLINE_RDEPADDR);
		dev->wrbulkpipe = usb_sndbulkpipe(dev->usbdev, FLIUSB_PROLINE_WREPADDR);
		break;

	default:
		FLIUSB_WARN("unsupported FLI USB device");
		return -EINVAL;
	}

	/* Check that the endpoints exist */
	if (usb_maxpacket(dev->usbdev, dev->rdbulkpipe, 0) == 0) {
		FLIUSB_ERR("invalid read USB bulk transfer endpoint address: 0x%02x",
				usb_pipeendpoint(dev->rdbulkpipe) | USB_DIR_IN);
		return -ENXIO;
	}

	if (usb_maxpacket(dev->usbdev, dev->wrbulkpipe, 1) == 0) {
		FLIUSB_ERR("invalid write USB bulk transfer endpoint address: 0x%02x",
				usb_pipeendpoint(dev->wrbulkpipe) | USB_DIR_OUT);
		return -ENXIO;
	}

	init_MUTEX(&dev->buffsem);
	if ((err = fliusb_allocbuffer(dev, param_buffersize)))
		return err;

#ifdef SGREAD
	dev->usbsg.maxpg = NUMSGPAGE;
	init_timer(&dev->usbsg.timer);
	init_MUTEX(&dev->usbsg.sem);
#endif /* SGREAD */

	if ((err = usb_string(dev->usbdev, dev->usbdev->descriptor.iProduct, prodstr, sizeof(prodstr))) < 0)
		FLIUSB_WARN("usb_string() failed: %d", err);

	FLIUSB_INFO("FLI USB device found: '%s'", prodstr);
	return 0;
}

static struct file_operations fliusb_fops = {
	.owner		= THIS_MODULE,
	.read		= fliusb_read,
	.write		= fliusb_write,
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 36)
	.unlocked_ioctl	= fliusb_ioctl,
#else
	.ioctl		= fliusb_ioctl,
#endif
	.open		= fliusb_open,
	.release	= fliusb_release,
};

static struct usb_class_driver fliusb_class = {
	.name		= "usb/" FLIUSB_NAME "%d",
	.fops		= &fliusb_fops,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,14))
	.mode		= S_IFCHR | S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP,
#endif
	.minor_base	= FLIUSB_MINOR_BASE,
};

static int fliusb_probe(struct usb_interface *interface,
			const struct usb_device_id *id)
{
	struct fliusb_dev *dev;
	int err;

	/* allocate our internal device structure */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (dev == NULL)
		return -ENOMEM;

	kref_init(&dev->kref);

	if ((err = fliusb_initdev(dev, interface, id))) {
		kref_put(&dev->kref, fliusb_delete);
		return err;
	}

	/* save a pointer to the device structure in this interface */
	usb_set_intfdata(interface, dev);

	/* register the device */
	if ((err = usb_register_dev(interface, &fliusb_class))) {
		FLIUSB_ERR("usb_register_dev() failed: %d", err);
		usb_set_intfdata(interface, NULL);
		kref_put(&dev->kref, fliusb_delete);
		return err;
	}

	FLIUSB_INFO("FLI USB device attached; rdepaddr: 0x%02x; wrepaddr: 0x%02x; buffersize: %d; timeout: %d",
		usb_pipeendpoint(dev->rdbulkpipe) | USB_DIR_IN,
		usb_pipeendpoint(dev->wrbulkpipe) | USB_DIR_OUT,
		dev->buffersize, dev->timeout);

	return 0;
}

static void fliusb_disconnect(struct usb_interface *interface)
{
	struct fliusb_dev *dev = usb_get_intfdata(interface);

	usb_set_intfdata(interface, NULL);

	/* give back the minor number we were using */
	usb_deregister_dev(interface, &fliusb_class);

	/* decrement usage count */
	kref_put(&dev->kref, fliusb_delete);

	FLIUSB_INFO("FLI USB device disconnected");
}

/* Devices supported by this driver */
static struct usb_device_id fliusb_table[] = {
	{ USB_DEVICE(FLIUSB_VENDORID, FLIUSB_PRODID_MAXCAM) },
	{ USB_DEVICE(FLIUSB_VENDORID, FLIUSB_PRODID_STEPPER) },
	{ USB_DEVICE(FLIUSB_VENDORID, FLIUSB_PRODID_FOCUSER) },
	{ USB_DEVICE(FLIUSB_VENDORID, FLIUSB_PRODID_FILTERWHEEL) },
	{ USB_DEVICE(FLIUSB_VENDORID, FLIUSB_PRODID_PROLINECAM) },
	{}, /* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, fliusb_table);

static struct usb_driver fliusb_driver = {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,14))
	.owner		= THIS_MODULE,
#endif
	.name		= FLIUSB_NAME,
	.probe		= fliusb_probe,
	.disconnect	= fliusb_disconnect,
	.id_table	= fliusb_table,
};

static int __init fliusb_init(void)
{
	int err;

	if ((err = usb_register(&fliusb_driver)))
		FLIUSB_ERR("usb_register() failed: %d", err);

	FLIUSB_INFO(FLIUSB_NAME " module loaded");

	return err;
}

static void __exit fliusb_exit(void)
{
	usb_deregister(&fliusb_driver);

	FLIUSB_INFO(FLIUSB_NAME " module unloaded");
}

module_init(fliusb_init);
module_exit(fliusb_exit);

MODULE_AUTHOR("Finger Lakes Instrumentation, L.L.C. <support@flicamera.com>");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION("1.3");
