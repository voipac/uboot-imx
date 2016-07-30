# uboot-imx-v2009.08
## Supported modules:
* Rex mx6pro (production)
* Rex mx6dl (prototypes)

## Supported OS:
* Android 4.4.2

## Download repository
    git clone -b uboot-imx-v2009.08 --single-branch https://github.com/voipac/uboot-imx uboot-imx-v2009.08-rex
    cd uboot-imx-v2009.08-rex

## Setup cross compiler
    export PATH="/opt/gcc-linaro-arm-linux-gnueabihf-4.8-2014.04_linux/bin:~/workdir/bin:$PATH"
    export CROSS_COMPILE=arm-linux-gnueabihf-
    export ARCH=arm

## Build
#### Build (imx6q rexpro) (production)
    make distclean
    make mx6q_rex_config
    make
    cp u-boot.bin /srv/tftp/imx6/u-boot-0x27800000_imx6q_$(date "+%Y%m%d").bin

#### Build (mx6dl rex) (prototype)
    make distclean
    make mx6dl_rex_config
    make
    cp u-boot.bin /srv/tftp/imx6/u-boot-0x27800000_imx6dl_$(date "+%Y%m%d").bin
