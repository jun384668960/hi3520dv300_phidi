#!/bin/sh

echo /sbin/mdev > /proc/sys/kernel/hotplug
mdev -s
