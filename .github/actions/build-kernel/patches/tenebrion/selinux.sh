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
BLOCK_TAG='/* tenebrion selinux rules */'

if ! grep -qF "$MARKER" "$SELINUX_RULES_C"; then
    echo "selinux.sh: ERROR — marker line tidak ditemukan di $SELINUX_RULES_C"
    exit 1
fi

if ! grep -q "ksu_allow" "$SELINUX_RULES_C"; then
    echo "selinux.sh: ERROR — fungsi ksu_allow tidak ditemukan di $SELINUX_RULES_C"
    exit 1
fi

# Sudah pernah di-inject? Jangan dobel.
if grep -qF "$BLOCK_TAG" "$SELINUX_RULES_C"; then
    echo "selinux.sh: rules sudah ter-inject sebelumnya, skip"
    exit 0
fi

RULES_TMP="$(mktemp)"
OUT_TMP="$(mktemp)"
trap 'rm -f "$RULES_TMP" "$OUT_TMP"' EXIT

# emit <target_type> <class> <perm...>   (source selalu "kernel")
emit() {
    local tgt="$1" cls="$2" p
    shift 2
    for p in "$@"; do
        printf '    ksu_allow(db, "kernel", "%s", "%s", "%s");\n' \
            "$tgt" "$cls" "$p" >> "$RULES_TMP"
    done
}

printf '    %s\n' "$BLOCK_TAG" >> "$RULES_TMP"

# Read-only sysfs
for T in sysfs_leds sysfs_backlight sysfs_drm sysfs_type sysfs \
         sysfs_thermal sysfs_battery_supply; do
    emit "$T" dir  search getattr
    emit "$T" file read open getattr
done

# cgroup (cpuset isolation dari momx_watch)
emit cgroup dir  search write getattr
emit cgroup file read write open getattr

# sysfs memory: AOSP label + vendor label.
# Denial di log: tcontext=vendor_sysfs_memory, name="scheduler" -> write.
for T in sysfs_memory vendor_sysfs_memory; do
    emit "$T" dir  search getattr
    emit "$T" file read write open getattr
done

# Denial di log: { dac_override } tclass=capability, scontext=kernel
emit kernel capability dac_override

BEFORE_LINES=$(wc -l < "$SELINUX_RULES_C")

echo "selinux.sh: Injecting Tenebrion SELinux rules ke $SELINUX_RULES_C..."

# Sisipkan blok rules tepat sebelum setiap baris marker
awk -v marker="$MARKER" -v rules="$RULES_TMP" '
    index($0, marker) {
        while ((getline line < rules) > 0) print line
        close(rules)
    }
    { print }
' "$SELINUX_RULES_C" > "$OUT_TMP"

cat "$OUT_TMP" > "$SELINUX_RULES_C"

AFTER_LINES=$(wc -l < "$SELINUX_RULES_C")

if [[ "$AFTER_LINES" -le "$BEFORE_LINES" ]]; then
    echo "selinux.sh: ERROR — injeksi GAGAL, jumlah baris tidak bertambah ($BEFORE_LINES → $AFTER_LINES)"
    exit 1
fi

for CHECK in '"vendor_sysfs_memory", "file", "write"' '"capability", "dac_override"'; do
    if ! grep -qF "$CHECK" "$SELINUX_RULES_C"; then
        echo "selinux.sh: ERROR — rule $CHECK tidak ada setelah injeksi"
        exit 1
    fi
done

echo "selinux.sh: ✅ Tenebrion SELinux rules berhasil di-inject ($BEFORE_LINES → $AFTER_LINES baris)"
