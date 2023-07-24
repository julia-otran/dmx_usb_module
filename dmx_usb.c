/*
 * DMX USB driver
 *
 * Copyright (C) 2004,2006,2010,2019 Erwin Rol (erwin@erwinrol.com)
 *
 * This driver is based on the usb-skeleton driver;
 *
 * Copyright (C) 2001-2003 Greg Kroah-Hartman (greg@kroah.com)
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/completion.h>
#include <linux/usb.h>
#include <linux/version.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>
#include <linux/time64.h>
#include <linux/kthread.h>
#include <linux/sched.h>

#define init_MUTEX(LOCKNAME) sema_init(LOCKNAME,1);

#include "dmx_usb.h"

#define CONFIG_USB_DEBUG 1

#ifdef CONFIG_USB_DEBUG
	static int debug = 1;
#else
	static int debug = 0;
#endif

/* Use our own dbg macro */
#undef dbg
#define dbg(format, arg...) do { if (debug) printk(KERN_DEBUG __FILE__ ": " format "\n" , ## arg); } while (0)

#ifndef info
#define info(format, arg...) do { printk(KERN_INFO __FILE__ ": " format "\n" , ## arg); } while (0)
#endif

#ifndef err
#define err(format, arg...) do { printk(KERN_ERR __FILE__ ": " format "\n" , ## arg); } while (0)
#endif

/* Version Information */
#define DRIVER_VERSION "V19.12.1"
#define DRIVER_AUTHOR "Erwin Rol, erwin@erwinrol.com"
#define DRIVER_DESC "DMX USB Driver"

/* Module parameters */
module_param(debug, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Debug enabled or not");

static struct usb_device_id dmx_usb_table [] = {
	{ USB_DEVICE_VER(FTDI_VID, FTDI_8U232AM_PID, 0x400, 0xffff) },
	{ USB_DEVICE_VER(FTDI_VID, FTDI_8U232AM_ALT_PID, 0x400, 0xffff) },
	{ }                                             /* Terminating entry */
};

MODULE_DEVICE_TABLE (usb, dmx_usb_table);

/* Get a minor range for your devices from the usb maintainer */
#define DMX_USB_MINOR_BASE	192

/* Structure to hold all of our device specific stuff */
struct dmx_usb_device {
	struct usb_device *	udev;			/* save off the usb device pointer */
	struct usb_interface *	interface;		/* the interface for this device */
	unsigned char		minor;			/* the starting minor number for this device */
	unsigned char		num_ports;		/* the number of ports this device has */
	char			num_interrupt_in;	/* number of interrupt in endpoints we have */
	char			num_bulk_in;		/* number of bulk in endpoints we have */
	char			num_bulk_out;		/* number of bulk out endpoints we have */

	unsigned char *		bulk_in_buffer;		/* the buffer to receive data */
	size_t			bulk_in_size;		/* the size of the receive buffer */
	__u8			bulk_in_endpointAddr;	/* the address of the bulk in endpoint */

	unsigned char *		bulk_out_buffer;	/* the buffer to send data */
	size_t			bulk_out_size;		/* the size of the send buffer */
	size_t			bulk_out_alloc_size;
	struct urb *		write_urb;		/* the urb used to send data */
	__u8			bulk_out_endpointAddr;	/* the address of the bulk out endpoint */
	atomic_t		write_busy;		/* true iff write urb is busy */
	struct completion	write_finished;		/* wait for the write to finish */

	int			open;			/* if the port is open or not */
	int			present;		/* if the device is not disconnected */
	struct semaphore	sem;			/* locks this structure */
	struct task_struct *ktask;
	atomic64_t write_buffer;
	atomic_t write_buffer_size;
	struct completion async_write_completion;
	struct semaphore	write_sem;
	struct completion write_exit_completion;

	__u8 write_thread_run;
};


static ktime_t test_cb_time;
static __u8 all_write_thread_run;

/* prevent races between open() and disconnect() */
	static DEFINE_SEMAPHORE(disconnect_sem);

// 30 FPS (writes seems to take a max of 32ms)
#define DMX_NS_PER_FRAME_NORMAL (33 * 1000 * 1000)
#define DMX_NS_PER_FRAME_FAST (21 * 1000 * 1000)

/* local function prototypes */
static int dmx_set_scheduler(struct task_struct *ts);
static int dmx_sleep(unsigned long ns);
static int dmx_write_thread(void *data);
static ssize_t dmx_usb_write_internal(struct dmx_usb_device *dev, const char *buffer, size_t count, ktime_t *urb_time);
static int dmx_usb_wait_temt(struct dmx_usb_device *dev);
static ssize_t dmx_usb_write	(struct file *file, const char *buffer, size_t count, loff_t *ppos);
static long dmx_usb_ioctl	(struct file *file, unsigned int cmd, unsigned long arg);
static int dmx_usb_open		(struct inode *inode, struct file *file);
static int dmx_usb_release	(struct inode *inode, struct file *file);

static int dmx_usb_probe	(struct usb_interface *interface, const struct usb_device_id *id);
static void dmx_usb_disconnect	(struct usb_interface *interface);

static void dmx_usb_write_bulk_callback	(struct urb *urb);

static struct file_operations dmx_usb_fops = {
	/*
	 * The owner field is part of the module-locking
	 * mechanism. The idea is that the kernel knows
	 * which module to increment the use-counter of
	 * BEFORE it calls the device's open() function.
	 * This also means that the kernel can decrement
	 * the use-counter again before calling release()
	 * or should the open() function fail.
	 */
	.owner =	THIS_MODULE,

	.write =		dmx_usb_write,
	.unlocked_ioctl =	dmx_usb_ioctl,
	.open =			dmx_usb_open,
	.release =		dmx_usb_release,
};

/*
 * usb class driver info in order to get a minor number from the usb core,
 * and to have the device registered with devfs and the driver core
 */
static struct usb_class_driver dmx_usb_class = {
	.name =		"usb/dmx%d",
	.fops =		&dmx_usb_fops,
	.minor_base =	DMX_USB_MINOR_BASE,
};

/* usb specific object needed to register this driver with the usb subsystem */
static struct usb_driver dmx_usb_driver = {
	.name =		"dmx_usb",
	.probe =	dmx_usb_probe,
	.disconnect =	dmx_usb_disconnect,
	.id_table =	dmx_usb_table,
};

static int dmx_set_scheduler(struct task_struct *ts)
{
	int result = 0;

	sched_set_fifo(ts);

	if (result) {
		err("[DMX USB] Setting scheduler failed. Timing will not be constant");
	}

	return result;
}

static int dmx_sleep(unsigned long ns)
{
	unsigned long us = ns / 1000;
	unsigned long ms = us / 1000;

	ktime_t delta;
	ktime_t time = ktime_get();

	if (ns <= 10000) {
		do {
			// Best way found, get stuck at a loop
			delta = ktime_get() - time;
		} while (delta < ns);
	} else if (ms > 10) {
		msleep(ms);
	} else if (us > 5000) {
		usleep_range(us - 2500, us);
	} else if (us > 2000) {
		usleep_range(us - 1000, us);
	} else if (us > 1000) {
		usleep_range(us - 500, us);
	} else {
		usleep_range(0, us);
	}

	time = ktime_get() - time;

	/*
	if (time > ns + 1000000) {
		dbg("[DMX USB] Sleep took too long. should be %lu but was %lu", ns, time);
	}
	*/

	return 0;
}

/**
 */
static inline void dmx_usb_debug_data (const char *function, int size, const unsigned char *data)
{
	int i;

	if (!debug)
		return;

	printk (KERN_DEBUG __FILE__": %s - length = %d, data = ",
		function, size);
	for (i = 0; i < size; ++i) {
		printk ("%.2x ", data[i]);
	}
	printk ("\n");
}

static __u32 dmx_usb_baud_to_divisor(int baud)
{
	static const unsigned char divfrac[8] = { 0, 3, 2, 4, 1, 5, 6, 7 };
	__u32 divisor;
	int divisor3 = 48000000 / 2 / baud; // divisor shifted 3 bits to the left
	divisor = divisor3 >> 3;
	divisor |= (__u32)divfrac[divisor3 & 0x7] << 14;
	/* Deal with special cases for highest baud rates. */
	if (divisor == 1) divisor = 0; else     // 1.0
	if (divisor == 0x4001) divisor = 1;     // 1.5
	return divisor;
}

static int dmx_usb_set_speed(struct dmx_usb_device* dev)
{
	__u16 urb_value;
	__u16 urb_index;
	__u32 urb_index_value;
	int rv;

	urb_index_value = dmx_usb_baud_to_divisor(250000);
	urb_value = (__u16)urb_index_value;
	urb_index = (__u16)(urb_index_value >> 16);

	rv = usb_control_msg(dev->udev,
				usb_sndctrlpipe(dev->udev, 0),
				FTDI_SIO_SET_BAUDRATE_REQUEST,
				FTDI_SIO_SET_BAUDRATE_REQUEST_TYPE,
				urb_value, urb_index,
				NULL, 0, HZ*10);

	return rv;
}

static int dmx_usb_setup(struct dmx_usb_device* dev)
{
	__u16 urb_value;

	urb_value = FTDI_SIO_SET_DATA_STOP_BITS_2 | FTDI_SIO_SET_DATA_PARITY_NONE;
	urb_value |= 8; // number of data bits

	if (usb_control_msg(dev->udev, usb_sndctrlpipe(dev->udev, 0),
				FTDI_SIO_SET_DATA_REQUEST,
				FTDI_SIO_SET_DATA_REQUEST_TYPE,
				urb_value , 0,
				NULL, 0, HZ*10) < 0) {
		err("%s FAILED to set databits/stopbits/parity", __FUNCTION__);
	}

	if (usb_control_msg(dev->udev, usb_sndctrlpipe(dev->udev, 0),
				FTDI_SIO_SET_FLOW_CTRL_REQUEST,
				FTDI_SIO_SET_FLOW_CTRL_REQUEST_TYPE,
				0, 0,
				NULL, 0, HZ*10) < 0) {
		err("%s error from disable flowcontrol urb", __FUNCTION__);
	}

	dmx_usb_set_speed(dev);
	return 0;
}

static void dmx_usb_set_break(struct dmx_usb_device* dev, int break_state)
{
	int ctrl_result;

	__u16 urb_value = FTDI_SIO_SET_DATA_STOP_BITS_2 | FTDI_SIO_SET_DATA_PARITY_NONE | 8;

	if (break_state) {
		urb_value |= FTDI_SIO_SET_BREAK;
	}

	ctrl_result = usb_control_msg(dev->udev, usb_sndctrlpipe(dev->udev, 0),
				FTDI_SIO_SET_DATA_REQUEST,
				FTDI_SIO_SET_DATA_REQUEST_TYPE,
				urb_value , 0,
				NULL, 0, HZ*10);

	if (ctrl_result < 0) {
		err("%s FAILED to enable/disable break state (state was %d) errno: %d", __FUNCTION__, break_state, ctrl_result);
	}

	// dbg("%s break state is %d - urb is %d", __FUNCTION__,break_state, urb_value);
}

/**
 */
static inline void dmx_usb_delete (struct dmx_usb_device *dev)
{
	void *tmp_frame = NULL;

	if (dev->ktask) {
		dev->present = 0;
		dev->write_thread_run = 0;
		wait_for_completion(&dev->write_exit_completion);
	}

	tmp_frame = (void*) atomic64_read(&dev->write_buffer);
	atomic64_set(&dev->write_buffer, 0);

	if (tmp_frame) {
		kfree(tmp_frame);
	}

	kfree (dev->bulk_in_buffer);
	usb_free_coherent (dev->udev, dev->bulk_out_alloc_size,
				dev->bulk_out_buffer,
				dev->write_urb->transfer_dma);
	usb_free_urb (dev->write_urb);
	kfree (dev);
}

/**
 */
static int dmx_usb_open (struct inode *inode, struct file *file)
{
	struct dmx_usb_device *dev = NULL;
	struct usb_interface *interface;
	int subminor;
	int retval = 0;

	dbg("%s", __FUNCTION__);

	subminor = iminor(inode);

	/* prevent disconnects */
	down (&disconnect_sem);

	interface = usb_find_interface (&dmx_usb_driver, subminor);
	if (!interface) {
		err ("%s - error, can't find device for minor %d",
		     __FUNCTION__, subminor);
		retval = -ENODEV;
		goto exit_no_device;
	}

	dev = usb_get_intfdata(interface);
	if (!dev) {
		retval = -ENODEV;
		goto exit_no_device;
	}

	/* lock this device */
	down (&dev->sem);

	/* increment our usage count for the driver */
	++dev->open;

	/* save our object in the file's private structure */
	file->private_data = dev;

	/* unlock this device */
	up (&dev->sem);

exit_no_device:
	up (&disconnect_sem);
	return retval;
}


/**
 */
static int dmx_usb_release (struct inode *inode, struct file *file)
{
	struct dmx_usb_device *dev;
	int retval = 0;

	dev = (struct dmx_usb_device *)file->private_data;
	if (dev == NULL) {
		dbg ("%s - object is NULL", __FUNCTION__);
		return -ENODEV;
	}

	dbg("%s - minor %d", __FUNCTION__, dev->minor);

	/* lock our device */
	down(&dev->write_sem);
	down(&dev->sem);

	if (dev->open <= 0) {
		dbg ("%s - device not opened", __FUNCTION__);
		retval = -ENODEV;
		goto exit_not_opened;
	}

	/* wait for any bulk writes that might be going on to finish up */
	if (atomic_read (&dev->write_busy))
		wait_for_completion (&dev->write_finished);

	--dev->open;

	if (!dev->present && !dev->open) {
		/* the device was unplugged before the file was released */
		up(&dev->sem);
		up(&dev->write_sem);
		dmx_usb_delete (dev);
		return 0;
	}

exit_not_opened:
	up (&dev->sem);
	up(&dev->write_sem);

	return retval;
}

static __u16 dmx_usb_get_status(struct dmx_usb_device* dev)
{
	__u16 status = 0;
	int count = 0;
	__u16 *buf = NULL;
	size_t buf_size = sizeof(__u16);
	int bulk_retval;

	buf = kmalloc(buf_size, GFP_KERNEL);
	if (buf == NULL)
		goto error;

	bulk_retval = usb_bulk_msg (dev->udev,
				usb_rcvbulkpipe (dev->udev, dev->bulk_in_endpointAddr),
				buf, buf_size, &count, HZ*10);

	if (bulk_retval)
		goto error;

	if (buf_size != count)
		goto error;

	status = *buf;
error:
	kfree(buf);

	return status;
}

static int dmx_write_thread(void *data)
{
	struct dmx_usb_device *dev;
	void *last_frame;
	void *tmp_frame;
	size_t last_frame_size;
	int lost_frames = 0;
	ssize_t bytes_written;
	ktime_t aux_time;
	__u32 frame_transfer_time_ns;

	dev = (struct dmx_usb_device*)data;

	down (&dev->sem);
	dmx_set_scheduler(dev->ktask);
	up(&dev->sem);

	last_frame = NULL;
	last_frame_size = 0;

	while(dev->present && dev->write_thread_run && all_write_thread_run) {
		/* lock this object */
		down (&dev->sem);

		tmp_frame = (void*) atomic64_read(&dev->write_buffer);

		if (!dev->present) {
			up(&dev->sem);
			continue;
		}

		if (tmp_frame) {
			if (last_frame) {
				kfree(last_frame);
				last_frame = NULL;
			}

			lost_frames = 0;
			last_frame = tmp_frame;
			last_frame_size = atomic_read(&dev->write_buffer_size);

			atomic_set(&dev->write_buffer_size, 0);
			atomic64_set(&dev->write_buffer, 0);
		} else if (last_frame) {
			lost_frames++;

			if (lost_frames > 100) {
				lost_frames = 0;
				kfree(last_frame);
				last_frame = NULL;
			}
		}

		if (last_frame) {
			bytes_written = dmx_usb_write_internal(dev, last_frame, last_frame_size, &aux_time);

			if (tmp_frame) {
				complete(&dev->async_write_completion);
			}

			if (bytes_written > 0) {
				frame_transfer_time_ns = (92 + 12 + (44 * bytes_written)) * 1000;
				dmx_sleep((frame_transfer_time_ns - aux_time) + 1000000);
			}

			dmx_usb_wait_temt(dev);
			// dbg("[DMX USB] Write took %lu", aux_time / 1000000);
		} else {
			up (&dev->sem);
			// No need intelligent sleeps
			dmx_sleep(DMX_NS_PER_FRAME_NORMAL);
			continue;
		}

		/* unlock the device */
		up (&dev->sem);
	}

	if (last_frame) {
		kfree(last_frame);
		last_frame = NULL;
	}

	complete(&dev->write_exit_completion);

	return 0;
}

static ssize_t dmx_usb_write_internal(struct dmx_usb_device *dev, const char *buffer, size_t count, ktime_t *urb_time)
{
	ktime_t aux_time;
	ssize_t bytes_written = 0;
	int retval = 0;
	__u8 *transfer_buffer;

	// dbg("%s - minor %d, count = %d", __FUNCTION__, dev->minor, count);

	/* verify that the device wasn't unplugged */
	if (!dev->present) {
		retval = -ENODEV;
		goto exit;
	}

	/* verify that we actually have some data to write */
	if (count == 0) {
		dbg("%s - write request of 0 bytes", __FUNCTION__);
		goto exit;
	}

	atomic_set(&dev->write_busy, 1);
	init_completion(&dev->write_finished);

	/* we can only write as much as our buffer will hold */
	bytes_written = min (dev->bulk_out_size, count+1);

	/* copy the data from userspace into our transfer buffer;
	 * this is the only copy required.
	 */
	
	transfer_buffer = (__u8*) dev->write_urb->transfer_buffer;
	transfer_buffer[0] = (__u8)0;
	memcpy(&transfer_buffer[1], buffer, bytes_written);

	// dmx_usb_debug_data (__FUNCTION__, bytes_written,
	//		     dev->write_urb->transfer_buffer);

	/* this urb was already set up, except for this write size */
	dev->write_urb->transfer_buffer_length = bytes_written;

	/* the transmit buffer is empty, now toggle the break */
	dmx_usb_set_break(dev, 1);
	dmx_usb_set_break(dev, 0);

	aux_time = ktime_get();

	retval = usb_submit_urb(dev->write_urb, GFP_KERNEL);

	if (retval) {
		atomic_set(&dev->write_busy, 0);
		complete(&dev->write_finished);

		err("%s - failed submitting write urb, error %d",
		    __FUNCTION__, retval);
	} else {
		wait_for_completion(&dev->write_finished);
		retval = bytes_written;
	}

	aux_time = ktime_get() - aux_time;
	(*urb_time) = aux_time;
	
	//dbg("[DMX USB] URB callback time %lu ms", aux_time / 1000000);

	

exit:
	return retval;
}

static int dmx_usb_wait_temt(struct dmx_usb_device *dev) {
	__u16 stat;
	int retval = 0;
	int count = 0;

	/* Poll the device to see if the transmit buffer is empty */
	while (count < 5) {
		stat = dmx_usb_get_status(dev);

		if (stat == 0) {
			retval = -EFAULT;
			break;
		}

		if ((stat >> 8) & FTDI_RS_TEMT) {
			break;
		}

		count++;
	}

	//dbg("[DMX USB] FTDI TEMT %lu ms", aux_time / 1000000);

	if (count >= 5) {
		return -1;
	}

	return retval;
}

/**
 *	dmx_usb_write
 *
 *	A device driver has to decide how to report I/O errors back to the
 *	user.  The safest course is to wait for the transfer to finish before
 *	returning so that any errors will be reported reliably.  dmx_usb_read()
 *	works like this.  But waiting for I/O is slow, so many drivers only
 *	check for errors during I/O initiation and do not report problems
 *	that occur during the actual transfer.  That's what we will do here.
 *
 *	A driver concerned with maximum I/O throughput would use double-
 *	buffering:  Two urbs would be devoted to write transfers, so that
 *	one urb could always be active while the other was waiting for the
 *	user to send more data.
 */
static ssize_t dmx_usb_write (struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
	struct dmx_usb_device *dev;
	int retval = 0;
	char *from_user;

	dev = (struct dmx_usb_device *)file->private_data;

	down(&dev->write_sem);

	if (!dev->present) {
		up(&dev->write_sem);
		return -EFAULT;
	}

	from_user  = (char*)kmalloc(count, GFP_KERNEL);

	if (!from_user) {
		return -ENOMEM;
	}

	if (count > dev->bulk_out_size - 1) {
		retval = dev->bulk_out_size - 1;
	} else {
		retval = count;
	}

	if (copy_from_user(from_user, buffer, retval)) {
		return -EFAULT;
	}

	if (dev->ktask != NULL) {
		if (atomic64_read(&dev->write_buffer) == 0) {
			init_completion(&dev->async_write_completion);

			atomic_set(&dev->write_buffer_size, retval);
			atomic64_set(&dev->write_buffer, (__u64) from_user);

			wait_for_completion_interruptible_timeout(&dev->async_write_completion, 1*HZ);
		} else {
			kfree(from_user);
			// err("[DMX USB] Frame dropped.");

			// Wait some time, maybe the frame get's consumed
			dmx_sleep(2000000);
		}
	} else {
		kfree(from_user);
		err("[DMx USB] Write thread is running? Dropping frame.");
		retval = -EFAULT;
	}

	up(&dev->write_sem);
	return retval;
}


/**
 */
static long dmx_usb_ioctl (struct file *file, unsigned int cmd, unsigned long arg)
{
	struct dmx_usb_device *dev;

	dev = (struct dmx_usb_device *)file->private_data;

	/* lock this object */
	down (&dev->sem);

	/* verify that the device wasn't unplugged */
	if (!dev->present) {
		up (&dev->sem);
		return -ENODEV;
	}

	dbg("%s - minor %d, cmd 0x%.4x, arg %ld", __FUNCTION__,
	    dev->minor, cmd, arg);

	/* fill in your device specific stuff here */

	/* unlock the device */
	up (&dev->sem);

	/* return that we did not understand this ioctl call */
	return -ENOTTY;
}


/**
 */
static void dmx_usb_write_bulk_callback (struct urb *urb)
{
	struct dmx_usb_device *dev = (struct dmx_usb_device *)urb->context;

	// dbg("%s - minor %d", __FUNCTION__, dev->minor);

	/* sync/async unlink faults aren't errors */
	if (urb->status && !(urb->status == -ENOENT ||
				urb->status == -ECONNRESET)) {
		dbg("%s - nonzero write bulk status received: %d",
		    __FUNCTION__, urb->status);
	}

	test_cb_time = ktime_get() - test_cb_time;

	// dbg("[DMX USB] Callback took %lu", test_cb_time / 1000000);

	/* notify anyone waiting that the write has finished */
	atomic_set (&dev->write_busy, 0);
	complete(&dev->write_finished);
}

/**
 *
 *	Called by the usb core when a new device is connected that it thinks
 *	this driver might be interested in.
 */
static int dmx_usb_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(interface);
	struct dmx_usb_device *dev = NULL;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	size_t buffer_size;
	int i;
	int retval = -ENOMEM;

	/* See if the device offered us matches what we can accept */
	if ((le16_to_cpu(udev->descriptor.idVendor) != FTDI_VID) ||
	    (le16_to_cpu(udev->descriptor.idProduct) != FTDI_8U232AM_PID)) {
		return -ENODEV;
	}

	/* allocate memory for our device state and initialize it */
	dev = kmalloc (sizeof(struct dmx_usb_device), GFP_KERNEL);
	if (dev == NULL) {
		err ("Out of memory");
		return -ENOMEM;
	}
	memset (dev, 0x00, sizeof (*dev));

	init_MUTEX(&dev->sem);
	init_MUTEX(&dev->write_sem);
	init_completion(&dev->write_exit_completion);

	dev->udev = udev;
	dev->interface = interface;

	/* set up the endpoint information */
	/* check out the endpoints */
	/* use only the first bulk-in and bulk-out endpoints */
	iface_desc = &interface->altsetting[0];
	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
		endpoint = &iface_desc->endpoint[i].desc;

		if (!dev->bulk_in_endpointAddr &&
		    (endpoint->bEndpointAddress & USB_DIR_IN) &&
		    ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
					== USB_ENDPOINT_XFER_BULK)) {
			/* we found a bulk in endpoint */
			buffer_size = endpoint->wMaxPacketSize;
			dev->bulk_in_size = buffer_size;
			dev->bulk_in_endpointAddr = endpoint->bEndpointAddress;
			dev->bulk_in_buffer = kmalloc (buffer_size, GFP_KERNEL);
			if (!dev->bulk_in_buffer) {
				err("Couldn't allocate bulk_in_buffer");
				goto error;
			}
		}

		if (!dev->bulk_out_endpointAddr &&
		    !(endpoint->bEndpointAddress & USB_DIR_IN) &&
		    ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
					== USB_ENDPOINT_XFER_BULK)) {
			/* we found a bulk out endpoint */
			/* a probe() may sleep and has no restrictions on memory allocations */
			dev->write_urb = usb_alloc_urb(0, GFP_KERNEL);
			if (!dev->write_urb) {
				err("No free urbs available");
				goto error;
			}
			dev->bulk_out_endpointAddr = endpoint->bEndpointAddress;

			/* on some platforms using this kind of buffer alloc
			 * call eliminates a dma "bounce buffer".
			 *
			 * NOTE: you'd normally want i/o buffers that hold
			 * more than one packet, so that i/o delays between
			 * packets don't hurt throughput.
			 */
			buffer_size = endpoint->wMaxPacketSize;
			dev->bulk_out_size = 513;
			dev->bulk_out_alloc_size = buffer_size;
			dev->write_urb->transfer_flags = URB_NO_TRANSFER_DMA_MAP;
			dev->bulk_out_buffer = usb_alloc_coherent (udev,
					buffer_size, GFP_KERNEL,
					&dev->write_urb->transfer_dma);
			if (!dev->bulk_out_buffer) {
				err("Couldn't allocate bulk_out_buffer");
				goto error;
			}
			usb_fill_bulk_urb(dev->write_urb, udev,
				      usb_sndbulkpipe(udev,
						      endpoint->bEndpointAddress),
				      dev->bulk_out_buffer, buffer_size,
				      dmx_usb_write_bulk_callback, dev);
		}
	}
	if (!(dev->bulk_in_endpointAddr && dev->bulk_out_endpointAddr)) {
		err("Couldn't find both bulk-in and bulk-out endpoints");
		goto error;
	}

	dmx_usb_setup(dev);

	/* allow device read, write and ioctl */
	dev->present = 1;

	/* we can register the device now, as it is ready */
	usb_set_intfdata (interface, dev);
	retval = usb_register_dev (interface, &dmx_usb_class);
	if (retval) {
		/* something prevented us from registering this driver */
		err ("Not able to get a minor for this device.");
		usb_set_intfdata (interface, NULL);
		goto error;
	}

	dev->minor = interface->minor;

	/* let the user know what node this device is now attached to */
	info ("DMX USB device now attached to dmx%d", dev->minor);

	atomic64_set(&dev->write_buffer, 0);

	dev->ktask = kthread_run(dmx_write_thread, dev, "[DMX USB Write Thread]");

	if (IS_ERR(dev->ktask)) {
		err("[DMX USB] Failed to start DMX USB Write Thread. Will write data sync");
		dev->ktask = NULL;
	}

	dev->write_thread_run = 1;

	return 0;

error:
	dmx_usb_delete (dev);
	return retval;
}


/**
 *
 *	Called by the usb core when the device is removed from the system.
 *
 *	This routine guarantees that the driver will not submit any more urbs
 *	by clearing dev->udev.  It is also supposed to terminate any currently
 *	active urbs.  Unfortunately, usb_bulk_msg(), used in dmx_usb_read(), does
 *	not provide any way to do this.  But at least we can cancel an active
 *	write.
 */
static void dmx_usb_disconnect(struct usb_interface *interface)
{
	struct dmx_usb_device *dev;
	int minor;

	/* prevent races with open() */
	down (&disconnect_sem);

	dev = usb_get_intfdata (interface);
	usb_set_intfdata (interface, NULL);

	dev->write_thread_run = 0;

	down(&dev->write_sem);
	down(&dev->sem);

	minor = dev->minor;

	/* give back our minor */
	usb_deregister_dev (interface, &dmx_usb_class);

	/* terminate an ongoing write */
	if (atomic_read (&dev->write_busy)) {
		usb_unlink_urb (dev->write_urb);
		wait_for_completion (&dev->write_finished);
	}

	/* prevent device read, write and ioctl */
	dev->present = 0;

	up(&dev->sem);
	up(&dev->write_sem);

	/* if the device is opened, dmx_usb_release will clean this up */
	if (!dev->open)
		dmx_usb_delete (dev);

	up (&disconnect_sem);

	info("DMX USB #%d now disconnected", minor);
}



/**
 *	dmx_usb_init
 */
static int __init dmx_usb_init(void)
{
	int result;

	all_write_thread_run = 1;

	/* register this driver with the USB subsystem */
	result = usb_register(&dmx_usb_driver);
	if (result) {
		err("usb_register failed. Error number %d",
		    result);
		return result;
	}

	info(DRIVER_DESC " " DRIVER_VERSION);
	return 0;
}


/**
 *	dmx_usb_exit
 */
static void __exit dmx_usb_exit(void)
{
	all_write_thread_run = 0;

	/* deregister this driver with the USB subsystem */
	usb_deregister(&dmx_usb_driver);
}


module_init (dmx_usb_init);
module_exit (dmx_usb_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

