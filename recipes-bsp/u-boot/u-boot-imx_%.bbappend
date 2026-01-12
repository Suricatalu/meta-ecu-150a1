FILESEXTRAPATHS:prepend := "${THISDIR}/files:"

SRC_URI:append:imx8mq-ecu150a1 = " file://0001-Add-ECU150A1-U-Boot-support.patch \
                                   file://0002-rauc_env_defconfig.patch \
                                 "
