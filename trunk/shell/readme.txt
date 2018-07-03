如何制作自己的文件系统镜像

make OSDRV_CROSS=arm-hisiv300-linux CHIP=hi3520dv300 all

1. SDK编译后，osdrv/pub/目录下有rootfs_uclibc.tgz文件
2. 解压rootfs_uclibc.tgz后将本文件夹下的内容复制到相应位置
   ko文件夹  		--> rootfs_uclibc目录的根目录
   shell脚本 		--> rootfs_uclibc/var/shell/
   mdev.conf 		--> rootfs_uclibc/etc/
   rcS       		--> rootfs_uclibc/etc/init.d/
   mp4file和phidi 	--> rootfs_uclibc/var/bin/
3. 创建rootfs_uclibc/mnt/usb1目录
4. 创建设备节点
   在rootfs_uclibc/dev/目录
	mknod console c 5 1
	mknod null c 1 3
	chmod 666 *
5. 使用mkfs.jffs2生成镜像
   mkfs.jffs2 -d ./rootfs_uclibc -l -U -e 0x10000 -o rootfs_hi3520dv300_64k.jffs2