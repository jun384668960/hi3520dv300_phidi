#!/bin/sh

echo "mount usb1"
@umount -f1 /mnt/usb1
mount -t vfat /dev/sda1 /mnt/usb1
