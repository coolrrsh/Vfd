# Raspberry Pi: CH432 Kernel Driver Setup

This guide provides instructions for disabling the default SPI interface (`spidev0.0`), applying a custom device tree overlay, and installing the `ch432` kernel module on a Raspberry Pi.

## 📋 Prerequisites

- Raspberry Pi running Raspberry Pi OS (with access to `/boot/firmware/`)
- A cross-compiled `ch432.ko` kernel module compatible with your Raspberry Pi's kernel version
- Access to modify the device tree overlays and system configuration files

## 🔧 Setup Instructions

### 1. Disable `spidev0.0` and Apply Overlay

To disable the default SPI device (`spidev0.0`) and enable the custom CH432 overlay:

1. **Patch the Device Tree**  
   Apply your custom overlay to the device tree. This assumes you have a `ch432_overlay.dtb` file ready.

   Copy the patched overlay to the firmware overlays directory:

   ```bash
   sudo cp ch432_overlay.dtb /boot/firmware/overlays/
   
  Modify the config.txt 
  
  ```bash
  sudo echo "dtoverlay=ch432_overlay" >> /boot/firmware/config.txt

  Reboot and insert the kernel modules
  
  ```bash
  insmod ch432.ko

  Finally, communicate /dev/ttyWC* file for Modbus communication
