ADDON_FILES_DIR:="${THISDIR}/files"

do_compile:append () {
    case ${MACHINE} in
    imx8mq-ecu150a1)
        mkimage -A arm64 -T script -C none -n "Boot script" -d ${ADDON_FILES_DIR}/boot.cmd ${ADDON_FILES_DIR}/boot.scr
        install -m 0644 ${ADDON_FILES_DIR}/boot.scr ${DEPLOY_DIR_IMAGE}/
        ;;
    esac
}

