**IMPORTANT DISCLAIMER**

> [!CAUTION]
> This software is provided for testing and educational purposes only. Use at your own risk. The developers are not responsible for any damage, data loss, or issues that may occur. Please ensure you have proper backups before installation.

# MomenToMoiX GKI Kernel

**By:** @koneko_dev [t.me/Koneko_dev](https://t.me/Koneko_dev)

# Features
- [KernelSU-Next](#kernelsu-next)
- [SUSFS](#susfs)
- [MomenToMoiX Driver](#momentomoix-driver)
- [Memory Management](#memory-management)
- [Power Management](#power-management)
- [Scheduler & I/O](#scheduler--io)

## [KernelSU-Next](https://github.com/pershoot/KernelSU-Next)

A kernel-based root solution for Android devices.

> [!WARNING]
> This release uses the [pershoot/KernelSU-Next](https://github.com/pershoot/KernelSU-Next) fork. The fork maintainer has said it is not ready for production use, so treat it as use at your own risk.

## [SUSFS](https://gitlab.com/simonpunk/susfs4ksu)

A KSU addon for hiding root using kernel patches and a userspace module.

## MomenToMoiX Driver

A smart, adaptive kernel-level optimization driver. It dynamically scales CPU behavior, frequencies, and I/O scheduling in real-time based on Screen State (ON/OFF) and Charging Status.

**How it works:**

- **Screen OFF (Battery Saver):** Isolates background apps to power-efficient cores (CPU 0), caps max frequency (with a separate bias for charging vs. non-charging states), switches the I/O scheduler to a low-overhead mode, and arms a thermal hold if the device is running hot — maximizing deep sleep and eliminating idle battery drain.
- **Screen ON (Instant):** Restores stock kernel profiles within milliseconds for a fluid, lag-free experience the moment the screen turns on. If a thermal hold is active, frequency restrictions are extended briefly until temperatures cool down.

**Requirements:**

MomenToMoiX detects screen state via `fb_notifier` (event-driven) with automatic fallback to sysfs polling. For the sysfs polling fallback to work, your device needs one of these paths:

- DPMS: `/sys/class/drm/card0-DSI-1/dpms`
- Backlight: `/sys/class/backlight/panel0-backlight/brightness`

**Verify it's running:**

```bash
su -c 'dmesg -w | grep momx'
```
after the device finishes booting.

Based on `@KanagawaYamadaVTeacher`'s Tenebrion logic and SELinux rules.

## Memory Management

- MGLRU (Multi-Generational LRU)
- ZRAM with ZSTD compression support
- ZSMALLOC
- KSM (Kernel Samepage Merging)
- Compaction & Migration
- Transparent Huge Pages (THP)

## Power Management

- TEO Idle Governor
- WQ_POWER_EFFICIENT
- SCHED_MC
- NO_HZ_IDLE
- Strict 100 max wakelocks limit with automated GC

## Scheduler & I/O

- Full Preemption (CONFIG_PREEMPT)
- BFQ I/O Scheduler (with Group IOSCHED)
- MQ-Deadline
- TCP FastOpen

## Other Features

- Full LTO (Link Time Optimization) builds
- Google Common Kernel LTS tracking

## Big Thanks

https://github.com/LoggingNewMemory/SuiKernel-Release — Tenebrion logic and SELinux rules

## Recommended Tools

[Kernel Flasher](https://github.com/fatalcoder524/KernelFlasher)
- Recommended flashing utility

[PixelFlasher by badabing2005](https://github.com/badabing2005/PixelFlasher)
- Pixel phone flashing GUI utility with features.

## Installation Instructions

### Prerequisites
- Unlocked bootloader.
- Backup your current boot image.
- Have root access using Magisk / KernelSU / Apatch (Any forks).

### Via Kernel Flasher
Download the correct AnyKernel3 ZIP for your device.
If you previously used another root method, clean it up first:
a. Magisk: perform a complete uninstall after flashing the AnyKernel3 ZIP.
b. KSU LKM (boot/init_boot/vendor_boot‑patched): Flash back the stock boot/init_boot/vendor_boot depending on what you patched.
c. KSU GKI: if you are 100% sure you already flashed stock init_boot/boot/vendor_boot, no action is needed; otherwise, follow the same steps as KSU LKM.
d. APatch: remove /data/adb contents to avoid leftover root conflicts after flashing the AnyKernel3 ZIP.
Flash the ZIP to the active slot using Kernel Flasher.
Install the KernelSU‑Next Manager APK, same version as mentioned in the release notes.
Open the KernelSU‑Next app.
Reboot the device if you performed any cleanup in step 2

---

MomenToMoiX Kernel is based on and developed from [WildKernels/GKI_KernelSU_SUSFS](https://github.com/WildKernels/GKI_KernelSU_SUSFS)

