# 🧩 Persistent USB Naming for SOLO Motor Controllers

## 📘 Overview

When multiple **SOLO motor controllers** are connected via USB, Linux assigns them names like `/dev/ttyACM0`, `/dev/ttyACM1`, etc.  
These names can **change after every reboot**, breaking your ROS2 configuration.

This guide shows how to create **persistent names**:

```bash
/dev/solo_left
/dev/solo_right
```

These symlinks will always map to the correct physical controller regardless of connection order.

## ⚙️ Steps

### 1. Identify connected devices

```bash
ls -l /dev/ttyACM*
```

### 2. Get each device’s USB port path

```bash
udevadm info -a -n /dev/ttyACM1 | grep KERNELS
udevadm info -a -n /dev/ttyACM2 | grep KERNELS
```

You’ll see something like:

```
KERNELS=="1-2.1"
KERNELS=="1-2.2"
```

Write these down — they represent the physical USB ports.

---

### 3. Create udev rule file

```bash
sudo nano /etc/udev/rules.d/99-solo.rules
```

Add the following (replace `1-2.1` and `1-2.2` with your actual values):

```bash
# SOLO Motor Controllers - Persistent Device Names
SUBSYSTEM=="tty", KERNELS=="1-2.2", SYMLINK+="solo_left", MODE="0666"
SUBSYSTEM=="tty", KERNELS=="1-2.1", SYMLINK+="solo_right", MODE="0666"
```

---

### 4. Reload and apply rules

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

---

### 5. Verify the symlinks

```bash
ls -l /dev/solo_*
```

Expected output:

```
/dev/solo_left -> ttyACM1
/dev/solo_right -> ttyACM2
```

---

## 🧠 Notes

- Keep each controller plugged into the same USB port.
- If you rearrange USB connections, rerun the `udevadm info` commands and update the rule.
- You can safely reference these symlinks in ROS2 configs:

```yaml
serial_port_left: "/dev/solo_left"
serial_port_right: "/dev/solo_right"
```

---

✅ **Result:**
Your motor controllers now have stable, human-readable device names that remain consistent across reboots.

```

```
