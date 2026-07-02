#!/usr/bin/env bash
# selinux.sh
# SELinux rule injection for Tenebrion module
# Adapted from SuiKernel pattern for KernelSU-Next (pershoot fork, dev-susfs)
# Author: Kanagawa Yamada

set -euo pipefail

if [[ -z "${SELINUX_RULES_C:-}" ]]; then
    echo "selinux.sh: ERROR — variabel SELINUX_RULES_C belum di-set oleh caller"
    exit 1
fi

if [[ ! -f "$SELINUX_RULES_C" ]]; then
    echo "selinux.sh: ERROR — $SELINUX_RULES_C tidak ditemukan"
    exit 1
fi

MARKER='rcu_assign_pointer(selinux_state.policy, pol);'

if ! grep -qF "$MARKER" "$SELINUX_RULES_C"; then
    echo "selinux.sh: ERROR — marker line tidak ditemukan di $SELINUX_RULES_C"
    exit 1
fi

if ! grep -q "ksu_allow" "$SELINUX_RULES_C"; then
    echo "selinux.sh: ERROR — fungsi ksu_allow tidak ditemukan di $SELINUX_RULES_C"
    exit 1
fi

BEFORE_LINES=$(wc -l < "$SELINUX_RULES_C")

echo "selinux.sh: Injecting Tenebrion SELinux rules ke $SELINUX_RULES_C..."

sed -i "/${MARKER//\//\\/}/i\\
    ksu_allow(db, \"kernel\", \"sysfs_leds\", \"dir\", \"search\");\\
    ksu_allow(db, \"kernel\", \"sysfs_leds\", \"dir\", \"getattr\");\\
    ksu_allow(db, \"kernel\", \"sysfs_leds\", \"file\", \"read\");\\
    ksu_allow(db, \"kernel\", \"sysfs_leds\", \"file\", \"open\");\\
    ksu_allow(db, \"kernel\", \"sysfs_leds\", \"file\", \"getattr\");\\
    ksu_allow(db, \"kernel\", \"sysfs_backlight\", \"dir\", \"search\");\\
    ksu_allow(db, \"kernel\", \"sysfs_backlight\", \"dir\", \"getattr\");\\
    ksu_allow(db, \"kernel\", \"sysfs_backlight\", \"file\", \"read\");\\
    ksu_allow(db, \"kernel\", \"sysfs_backlight\", \"file\", \"open\");\\
    ksu_allow(db, \"kernel\", \"sysfs_backlight\", \"file\", \"getattr\");\\
    ksu_allow(db, \"kernel\", \"sysfs_drm\", \"dir\", \"search\");\\
    ksu_allow(db, \"kernel\", \"sysfs_drm\", \"dir\", \"getattr\");\\
    ksu_allow(db, \"kernel\", \"sysfs_drm\", \"file\", \"read\");\\
    ksu_allow(db, \"kernel\", \"sysfs_drm\", \"file\", \"open\");\\
    ksu_allow(db, \"kernel\", \"sysfs_drm\", \"file\", \"getattr\");\\
    ksu_allow(db, \"kernel\", \"sysfs_type\", \"dir\", \"search\");\\
    ksu_allow(db, \"kernel\", \"sysfs_type\", \"dir\", \"getattr\");\\
    ksu_allow(db, \"kernel\", \"sysfs_type\", \"file\", \"read\");\\
    ksu_allow(db, \"kernel\", \"sysfs_type\", \"file\", \"open\");\\
    ksu_allow(db, \"kernel\", \"sysfs_type\", \"file\", \"getattr\");\\
    ksu_allow(db, \"kernel\", \"sysfs\", \"dir\", \"search\");\\
    ksu_allow(db, \"kernel\", \"sysfs\", \"dir\", \"getattr\");\\
    ksu_allow(db, \"kernel\", \"sysfs\", \"file\", \"read\");\\
    ksu_allow(db, \"kernel\", \"sysfs\", \"file\", \"open\");\\
    ksu_allow(db, \"kernel\", \"sysfs\", \"file\", \"getattr\");\\
    ksu_allow(db, \"kernel\", \"cgroup\", \"dir\", \"search\");\\
    ksu_allow(db, \"kernel\", \"cgroup\", \"dir\", \"write\");\\
    ksu_allow(db, \"kernel\", \"cgroup\", \"dir\", \"getattr\");\\
    ksu_allow(db, \"kernel\", \"cgroup\", \"file\", \"read\");\\
    ksu_allow(db, \"kernel\", \"cgroup\", \"file\", \"write\");\\
    ksu_allow(db, \"kernel\", \"cgroup\", \"file\", \"open\");\\
    ksu_allow(db, \"kernel\", \"cgroup\", \"file\", \"getattr\");\\
    ksu_allow(db, \"kernel\", \"sysfs_memory\", \"dir\", \"search\");\\
    ksu_allow(db, \"kernel\", \"sysfs_memory\", \"dir\", \"getattr\");\\
    ksu_allow(db, \"kernel\", \"sysfs_memory\", \"file\", \"read\");\\
    ksu_allow(db, \"kernel\", \"sysfs_memory\", \"file\", \"write\");\\
    ksu_allow(db, \"kernel\", \"sysfs_memory\", \"file\", \"open\");\\
    ksu_allow(db, \"kernel\", \"sysfs_memory\", \"file\", \"getattr\");\\
    ksu_allow(db, \"kernel\", \"sysfs_thermal\", \"dir\", \"search\");\\
    ksu_allow(db, \"kernel\", \"sysfs_thermal\", \"dir\", \"getattr\");\\
    ksu_allow(db, \"kernel\", \"sysfs_thermal\", \"file\", \"read\");\\
    ksu_allow(db, \"kernel\", \"sysfs_thermal\", \"file\", \"open\");\\
    ksu_allow(db, \"kernel\", \"sysfs_thermal\", \"file\", \"getattr\");\\
    ksu_allow(db, \"kernel\", \"sysfs_battery_supply\", \"dir\", \"search\");\\
    ksu_allow(db, \"kernel\", \"sysfs_battery_supply\", \"dir\", \"getattr\");\\
    ksu_allow(db, \"kernel\", \"sysfs_battery_supply\", \"file\", \"read\");\\
    ksu_allow(db, \"kernel\", \"sysfs_battery_supply\", \"file\", \"open\");\\
    ksu_allow(db, \"kernel\", \"sysfs_battery_supply\", \"file\", \"getattr\");" \
    "$SELINUX_RULES_C"

AFTER_LINES=$(wc -l < "$SELINUX_RULES_C")

if [[ "$AFTER_LINES" -le "$BEFORE_LINES" ]]; then
    echo "selinux.sh: ERROR — injeksi GAGAL, jumlah baris tidak bertambah ($BEFORE_LINES → $AFTER_LINES)"
    exit 1
fi

echo "selinux.sh: ✅ Tenebrion SELinux rules berhasil di-inject ($BEFORE_LINES → $AFTER_LINES baris)"
