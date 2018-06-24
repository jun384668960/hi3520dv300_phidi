#!/bin/sh

echo "mount usb1"
umount /mnt/usb1
mount -t vfat /dev/sda1 /mnt/usb1
