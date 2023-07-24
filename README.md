# Linux JSLC DMX USB Driver

The DMX USB driver is a Linux 6.X kernel driver for the Enttec
DMX USB dongle ( http://www.enttec.com/dmxusb.php )

- It runs very smooth, 
- It does not bugs under high I/O (the default driver under such case will send something that makes everything flash)
- Can reach 41 DMX FPS. 

## Beware!
This driver expects at most 512 bytes of data.

The original driver expected that the wrote data includes startCode at a total of 513 bytes.

**Do not write the the DMX start code when using this driver fork**

If you do want to use with it with DMX Samples, remember to remove the extra first byte of the examples.

## Building the driver

*For x64 platforms, you may need sign the module*


As far as I know, most people use secure boot. This prevents unsigned modules from running. However you can create your own certificate and install it on the bios. Also, there's many documentation about how to do this process, so take a look on the internet.


Before the driver can be build, the kernel sourcecode needs to be 
installed. To build the driver just call make, this should build a
`dmx_usb.ko` (the kernel module) and a `dmx_usb_test` (a small test
program).

## Building on a RPi4 running Raspbian

First update your RPi with the following commands;

```
apt-get update -y
apt-get upgrade -y
```

After that you might want to reboot to make sure you are running the
maybe updated kernel.

Than get the kernel source code;

```
# Get rpi-source
sudo wget https://raw.githubusercontent.com/notro/rpi-source/master/rpi-source -O /usr/bin/rpi-source

# Make it executable
sudo chmod +x /usr/bin/rpi-source

# Tell the update mechanism that this is the latest version of the script
/usr/bin/rpi-source -q --tag-update

# Get the kernel files thingies.
rpi-source
```

Now you should be ready to build the DMX USB module.

## Problems loading the right driver.

A "small" problem to solve was to make the kernel clear that it should 
use my driver for the dongle and not the standard usbserial driver. 
Since I don't have any other USBserial devices with the samechip type I 
just removed the `ftdi_sio.ko` module, and changed the `/etc/hotplug/usb.distmap` 
to point to my module. To support both the `ftdi_sio.o` module and the 
`dmx_usb.o` module the `ftdi_sio.o` module should be patched and the Product ID 
for the FT232BM should be removed from it, this will mean all other 
serial-usb devices are still supported just not the one with FT232BM chips, 
since it isn't possible to differentiate DMX-USB and normal-serial versions.

