#!/usr/bin/env bash
# 在 Windows 主机上用 VMware 命令行工具创建 RoboCup-Ubuntu20 虚拟机（仅建磁盘+配置，不含系统安装）。
# 运行环境：Git Bash（Windows）。不删除/移动/覆盖任何现有工程文件。
set -euo pipefail

VMWARE_DIR="C:/Program Files/VMware/VMware Workstation"
VDISK="$VMWARE_DIR/vmware-vdiskmanager.exe"
VM_DIR="D:/VM/RoboCup-Ubuntu20"
ISO="D:/a/.robocup/installers/ubuntu-20.04.6-desktop-amd64.iso"
VMX="$VM_DIR/RoboCup-Ubuntu20.vmx"
VMDK="$VM_DIR/RoboCup-Ubuntu20.vmdk"

log() { printf '[create-vm] %s\n' "$*"; }
fail() { log "ERROR: $*" >&2; exit 1; }

[ -x "$VDISK" ] || fail "未找到 vmware-vdiskmanager.exe: $VDISK"
[ -f "$ISO" ] || fail "未找到 Ubuntu ISO: $ISO"

mkdir -p "$VM_DIR"
log "VM 目录: $VM_DIR"

# 1) 创建 80 GB 动态分配磁盘（lsilogic；-t 0 = growable 单文件）
if [ -f "$VMDK" ]; then
  log "磁盘已存在，跳过创建: $VMDK"
else
  log "创建 80GB 动态磁盘..."
  "$VDISK" -c -s 80GB -a lsilogic -t 0 "$VMDK"
fi

# 2) 写入 .vmx 配置
cat > "$VMX" <<'EOF'
.encoding = "UTF-8"
config.version = "8"
virtualHW.version = "21"
vcpu.hotadd = "FALSE"
memsize = "8192"
numvcpus = "6"
cpuid.coresPerSocket = "3"
guestOS = "ubuntu-64"
nvram = "RoboCup-Ubuntu20.nvram"
virtualHW.productCompatibility = "hosted"
powerType.powerOff = "soft"
powerType.powerOn = "soft"
powerType.suspend = "soft"
powerType.reset = "soft"
displayName = "RoboCup-Ubuntu20"
annotation = "RoboCup 2026 无人机仿真开发环境 (Ubuntu 20.04 + ROS Noetic + Gazebo 11 + PX4 v1.13.2 + MAVROS + XTDrone)"
mks.enable3d = "TRUE"
svga.vramSize = "134217728"
svga.graphicsMemoryKB = "131072"
svga.autodetect = "FALSE"
scsi0.present = "TRUE"
scsi0.virtualDev = "lsilogic"
scsi0:0.present = "TRUE"
scsi0:0.fileName = "RoboCup-Ubuntu20.vmdk"
scsi0:0.mode = "persistent"
scsi0:0.deviceType = "disk"
scsi0:0.redo = ""
ide1:0.present = "TRUE"
ide1:0.fileName = "D:/a/.robocup/installers/ubuntu-20.04.6-desktop-amd64.iso"
ide1:0.deviceType = "cdrom-image"
ide1:0.autodetect = "FALSE"
ide1:0.startConnected = "TRUE"
floppy0.present = "FALSE"
ethernet0.present = "TRUE"
ethernet0.connectionType = "nat"
ethernet0.virtualDev = "e1000"
ethernet0.wakeOnPkgRcv = "FALSE"
ethernet0.addressType = "generated"
usb.present = "TRUE"
usb.generic.autoconnect = "FALSE"
tools.syncTime = "TRUE"
sound.present = "FALSE"
serial0.present = "FALSE"
parallel0.present = "FALSE"
pciBridge0.present = "TRUE"
hpet0.present = "TRUE"
isolation.tools.hgfs.disable = "FALSE"
sharedFolder0.present = "FALSE"
EOF

log "已写入: $VMX"
log "VM 创建完成（仅磁盘+配置）。系统安装请用 VMware 打开此 .vmx 后启动，并在 GRUB 编辑追加 preseed 参数。"
