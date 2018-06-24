#!/bin/sh

echo "ismod btn_drv.ko"
cd /ko
insmod btn_drv.ko
insmod led_drv.ko

echo "start hotplug listen"
cd /var/shell
./hotplug.sh&

echo "start phidi mp4file"
/var/bin/phidi&
/var/bin/mp4file&

echo "done"

