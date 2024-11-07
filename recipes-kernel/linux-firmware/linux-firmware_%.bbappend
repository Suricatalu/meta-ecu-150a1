FILESEXTRAPATHS:prepend := "${THISDIR}/linux-firmware:"

SRC_URI += "\
    file://rtl_nic \
"

do_install:append(){
    install -d ${D}$${nonarch_base_libdir}/firmware/rtl_nic
    install -m 0644 ${WORKDIR}/rtl_nic/*.fw* ${D}${nonarch_base_libdir}/firmware/rtl_nic/

    find ${D}${nonarch_base_libdir}/firmware -type f -exec chmod 644 '{}' ';'
    find ${D}${nonarch_base_libdir}/firmware -type f -exec chown root:root '{}' ';'
}

# NOTE: Use "=+" instead of "+=". Otherwise file is placed into the linux-firmware package.
PACKAGES =+ " ${PN}-rtl-nic"

FILES:${PN}-rtl-nic = "\
    ${nonarch_base_libdir}/firmware/rtl_nic/*.fw* \
"
