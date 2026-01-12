# meta-ecu-150a1/recipes-bsp/imx-mkimage/files/boot.cmd
#
# ECU-150A1 RAUC A/B Boot Script
# Supports both eMMC (mmc 0 / mmcblk0) and SD Card (mmc 1 / mmcblk1)
# Architecture: Kernel and DTB are stored in rootfs /boot directory
# Compile command: mkimage -A arm64 -T script -C none -n "RAUC Boot" -d boot.cmd boot.scr

echo "=== ECU-150A1 RAUC A/B Boot Script ==="

# ═══════════════════════════════════════════════════════════════════
# Setup Boot Device: Using U-Boot's default mmcdev variable
# ═══════════════════════════════════════════════════════════════════
# ECU-150A1's U-Boot automatically sets mmcdev based on boot source from SPL:
#   - Boot from eMMC: mmcdev=0 → /dev/mmcblk0
#   - Boot from SD Card: mmcdev=1 → /dev/mmcblk1
# See: meta-ecu-150a1/recipes-bsp/u-boot/files/0001-Add-ECU150A1-U-Boot-support.patch

echo ">>> Boot device: mmc ${mmcdev}"

# Set Linux kernel root device name based on mmcdev
if test "${mmcdev}" = "0"; then
    setenv root_dev "mmcblk0"
    echo ">>> Using eMMC: /dev/mmcblk0"
else
    setenv root_dev "mmcblk1"
    echo ">>> Using SD Card: /dev/mmcblk1"
fi

# Fallback: if mmcdev is not set (should not happen in theory, as U-Boot sets it)
if test -z "${mmcdev}"; then
    echo "!!! WARNING: mmcdev not set, defaulting to mmc 1 (SD Card)"
    setenv mmcdev 1
    setenv root_dev "mmcblk1"
fi

# ═══════════════════════════════════════════════════════════════════
# Initialize RAUC environment variables
# ═══════════════════════════════════════════════════════════════════
test -n "${BOOT_ORDER}" || setenv BOOT_ORDER "system0 system1"
test -n "${BOOT_system0_LEFT}" || setenv BOOT_system0_LEFT 3
test -n "${BOOT_system1_LEFT}" || setenv BOOT_system1_LEFT 3

echo "BOOT_ORDER=${BOOT_ORDER}"
echo "BOOT_system0_LEFT=${BOOT_system0_LEFT}"
echo "BOOT_system1_LEFT=${BOOT_system1_LEFT}"

# ═══════════════════════════════════════════════════════════════════
# Dynamic Partition Mapping (based on detected device)
# ═══════════════════════════════════════════════════════════════════
# Slot system0: partition 2, Slot system1: partition 3
setenv root_part_system0 "/dev/${root_dev}p2"
setenv root_part_system1 "/dev/${root_dev}p3"

echo ">>> Slot system0: ${root_part_system0}"
echo ">>> Slot system1: ${root_part_system1}"

# DTB filename
setenv fdt_file "fsl-imx8mq-ecu150a1.dtb"

# ═══════════════════════════════════════════════════════════════════
# A/B Slot Selection Logic
# ═══════════════════════════════════════════════════════════════════
for BOOT_SLOT in "${BOOT_ORDER}"; do
    
    if test "x${BOOT_SLOT}" = "xsystem0"; then
        if test ${BOOT_system0_LEFT} -gt 0; then
            echo ">>> Trying Slot system0 (${BOOT_system0_LEFT} attempts left)..."
            
            setexpr BOOT_system0_LEFT ${BOOT_system0_LEFT} - 1
            saveenv
            
            # Load kernel and dtb from rootfs_a (partition 2) /boot directory
            if load mmc ${mmcdev}:2 ${loadaddr} /boot/Image; then
                if load mmc ${mmcdev}:2 ${fdt_addr} /boot/${fdt_file}; then
                    setenv bootargs "console=ttymxc0,115200 rauc.slot=system0 root=${root_part_system0} rootwait rw"
                    echo ">>> Booting Slot system0 from /dev/${root_dev}..."
                    booti ${loadaddr} - ${fdt_addr}
                fi
            fi
            
            echo ">>> Slot system0: Failed to load kernel or dtb"
        else
            echo ">>> Slot system0: no attempts left, skipping"
        fi
    fi
    
    if test "x${BOOT_SLOT}" = "xsystem1"; then
        if test ${BOOT_system1_LEFT} -gt 0; then
            echo ">>> Trying Slot system1 (${BOOT_system1_LEFT} attempts left)..."
            
            setexpr BOOT_system1_LEFT ${BOOT_system1_LEFT} - 1
            saveenv
            
            # Load kernel and dtb from rootfs_b (partition 3) /boot directory
            if load mmc ${mmcdev}:3 ${loadaddr} /boot/Image; then
                if load mmc ${mmcdev}:3 ${fdt_addr} /boot/${fdt_file}; then
                    setenv bootargs "console=ttymxc0,115200 rauc.slot=system1 root=${root_part_system1} rootwait rw"
                    echo ">>> Booting Slot system1 from /dev/${root_dev}..."
                    booti ${loadaddr} - ${fdt_addr}
                fi
            fi
            
            echo ">>> Slot system1: Failed to load kernel or dtb"
        else
            echo ">>> Slot system1: no attempts left, skipping"
        fi
    fi
    
done

# All slots failed
echo "!!! FATAL: No bootable slot found!"
reset