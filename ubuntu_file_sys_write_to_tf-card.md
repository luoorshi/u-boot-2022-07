全志 H3 TF 卡一键启动制作手册  
（以现有路径为例，直接复制即用）

------------------------------------------------
一、准备物料  
1. TF 卡 ≥ 8 GB，读卡器  
2. 已编译好的文件（路径保持不动，仅做示例）  
   - U-Boot：  
     `/home/test/work/H5/uboot/u-boot-2022-07/u-boot-2022.07/u-boot-sunxi-with-spl.bin`  
   - 内核与设备树：  
     `/home/test/work/H5/linux/linux5.19.6_Quarkn/arch/arm/boot/zImage`  
     `/home/test/work/H5/linux/linux5.19.6_Quarkn/arch/arm/boot/dts/sun8i-h3-quark-luoorshi.dtb`  
3. 宿主机：Ubuntu 18.04/20.04 x86_64，已安装 `qemu-user-static`

------------------------------------------------
二、整卡分区（一次性操作）

```bash
# 找到 TF 卡，假设为 /dev/sdf
lsblk
sudo umount /dev/sdf* 2>/dev/null || true

# 清空前 10 MB
sudo dd if=/dev/zero of=/dev/sdf bs=1M count=10

# 分区：p1 100 M FAT32，其余 ext4
sudo fdisk /dev/sdf <<EOF
o
n
p
1

+100M
t
c
n
p
2


w
EOF

# 格式化
sudo mkfs.vfat -n BOOT /dev/sdf1
sudo mkfs.ext4 -L rootfs /dev/sdf2
```

------------------------------------------------
三、挂载点创建

```bash
sudo mkdir -p /media/boot /media/rootfs
```

------------------------------------------------
四、写入启动文件

```bash
# 1. 写入 U-Boot（偏移 8 KB）
sudo dd if=/home/test/work/H5/uboot/u-boot-2022-07/u-boot-2022.07/u-boot-sunxi-with-spl.bin of=/dev/sdf bs=1024 seek=8

# 2. 挂载分区
sudo mount /dev/sdf1 /media/boot
sudo mount /dev/sdf2 /media/rootfs

# 3. 拷贝内核与设备树
sudo cp /home/test/work/H5/linux/linux5.19.6_Quarkn/arch/arm/boot/zImage /media/boot/
sudo cp /home/test/work/H5/linux/linux5.19.6_Quarkn/arch/arm/boot/dts/sun8i-h3-quark-luoorshi.dtb /media/boot/

# 4. 生成 boot.scr
cat >/tmp/boot.cmd <<'EOF'
setenv bootargs console=ttyS0,115200 root=/dev/mmcblk0p2 rw rootwait
fatload mmc 0:1 0x43000000 sun8i-h3-quark-luoorshi.dtb
fatload mmc 0:1 0x42000000 zImage
bootz 0x42000000 - 0x43000000
EOF
sudo mkimage -A arm -O linux -T script -C none -n "Boot script" -d /tmp/boot.cmd /media/boot/boot.scr
```

------------------------------------------------
五、根文件系统部署（Ubuntu-base）

```bash
# 1. 下载并解压
cd /tmp
wget http://cdimage.ubuntu.com/ubuntu-base/releases/20.04/release/ubuntu-base-20.04.5-base-armhf.tar.gz
sudo tar -xpf ubuntu-base-20.04.5-base-armhf.tar.gz -C /media/rootfs/

# 2. 准备 qemu 桥
sudo cp /usr/bin/qemu-arm-static /media/rootfs/usr/bin/

# 检查版本，如果版本较低需要升级
qemu-arm-static --version 

# 升级到4.2
## 下载
wget http://archive.ubuntu.com/ubuntu/pool/universe/q/qemu/qemu-user-static_4.2-3ubuntu6_amd64.deb

## 安装（会覆盖 2.11.1）
sudo dpkg -i qemu-user-static_4.2-3ubuntu6_amd64.deb

## 验证
qemu-arm-static --version        # 应显示 4.2.0

sudo update-binfmts --disable qemu-arm
sudo update-binfmts --enable qemu-arm


sudo cp /usr/bin/qemu-arm-static /media/rootfs/usr/bin/

然后检查 ls /proc/sys/fs/binfmt_misc/qemu-arm*

# 3. 挂载虚拟文件系统
sudo mount -t proc none /media/rootfs/proc
sudo mount -t sysfs none /media/rootfs/sys
sudo mount -o bind /dev /media/rootfs/dev

# 4. 进入 chroot
sudo chroot /media/rootfs /usr/bin/qemu-arm-static /bin/bash
```

------------------------------------------------
六、在 chroot 内一次性初始化（复制即可）

```bash
# 1. DNS
echo nameserver 8.8.8.8 > /etc/resolv.conf

# 2. 换清华源
cat >/etc/apt/sources.list <<'EOF'
deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports focal main restricted universe multiverse
deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports focal-updates main restricted universe multiverse
deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports focal-security main restricted universe multiverse
EOF

# 3. 装常用包
apt update
apt install -y sudo openssh-server netplan.io iproute2 net-tools vim kmod

# 4. 建用户
useradd -m -s /bin/bash user
echo 'user:user' | chpasswd
echo 'user ALL=(ALL) NOPASSWD:ALL' >/etc/sudoers.d/user
chmod 440 /etc/sudoers.d/user

# 5. root 密码
passwd

# 6. ssh 允许密码登录
sed -i 's/^#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config

# 7. 网络 DHCP
mkdir -p /etc/netplan
cat >/etc/netplan/01-dhcp.yaml <<'EOF'
network:
  version: 2
  ethernets:
    eth0:
      dhcp4: true
EOF

# 8. 退出
exit
```

------------------------------------------------
七、卸载 & 清理

```bash
sudo umount /media/rootfs/dev
sudo umount /media/rootfs/proc
sudo umount /media/rootfs/sys
sudo umount /media/boot /media/rootfs
sudo eject /dev/sdf
```

------------------------------------------------
八、上电验证

1. 插卡 → H3 上电 → 串口 115200  
2. 看到 **Ubuntu 20.04.5 LTS** 启动信息  
3. 串口最终出现：
   ```
   localhost login:
   ```
   输入 `root` + 你刚才设置的密码即可登录；  
   或 DHCP 拿到 IP 后：
   ```
   ssh user@<IP>
   密码 user
   ```

------------------------------------------------
九、后续可选优化

- 禁用串口 90 s 等待：
  ```bash
  systemctl mask serial-getty@ttyS0.service
  ```
- 换静态 IP、添加 Wi-Fi、更新内核只需替换 `/media/boot/zImage` 与 `*.dtb` 并重新生成 `boot.scr`。

至此，TF 卡已可一键启动全志 H3 + Ubuntu 20.04 最小系统。