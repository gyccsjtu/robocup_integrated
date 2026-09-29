# VMware Ubuntu 20.04 baseline

This is the native-Linux alternative to the Docker single-UAV baseline. It is
for development and validation, not a claim about the eventual official
competition image.

## VM settings

Create `RoboCup-Ubuntu20` at `D:\VM\RoboCup-Ubuntu20` with Ubuntu 20.04.6
Desktop, 6 vCPU, 8 GB RAM, an 80 GB growable disk, NAT networking, and 3D
acceleration enabled. Do not work from a VMware shared folder: clone or copy
the repository into `~/robocup/robocup_ws` inside the Linux ext4 filesystem.

## Bootstrap

After Ubuntu is installed and the repository is in the VM, run:

```bash
cd ~/robocup/robocup_ws
chmod +x scripts/vm/*.sh
scripts/vm/bootstrap_ubuntu20.sh
```

The bootstrap verifies and pins PX4 `46a12a09bf11c8cbafc5ad905996645b4fe1a9df`
and XTDrone `62339a816ef815113a0366a62e8aca4be3000f80`, applies the existing
overlay, builds PX4 SITL and builds the XTDrone Gazebo ROS packages.

Start the GUI scene with:

```bash
scripts/vm/start_single_uav_native.sh
```

The Windows PowerShell Docker launchers must not be run inside the VM.

## Snapshot points

Take snapshots only while the VM is shut down or the UAV is on the ground:

1. `01-ubuntu-clean` after Ubuntu and VMware Tools.
2. `02-ros-px4-ready` after the bootstrap script and an iris/MAVROS health check.
3. `03-robocup-baseline` after the project single-UAV demo passes.
