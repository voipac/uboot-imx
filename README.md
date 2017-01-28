# uboot-imx-v2015.04
## Supported modules:
* TinyRex mx6ultra/max/pro/basic (production)
* Rex mx6ultra/pro/basic (production)

## Supported OS:
* Linux

## Download repository
    git clone -b uboot-imx-v2015.04 --single-branch https://github.com/voipac/uboot-imx uboot-imx-v2015.04-rex
    cd uboot-imx-v2015.04-rex

## Setup cross compiler
    export PATH="/opt/gcc-linaro-arm-linux-gnueabihf-4.8-2014.04_linux/bin:~/workdir/bin:$PATH"
    export CROSS_COMPILE=arm-linux-gnueabihf-
    export ARCH=arm

## Build
#### Build (imx6s tinyrexlite) (production)
    make distclean
    make mx6tinyrexlite_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-tinyrexlite.imx

    make distclean
    make mx6tinyrexliterecovery_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-tinyrexliterecovery.imx

#### Build (imx6s tinyrexbasic) (production)
    make distclean
    make mx6tinyrexbasic_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-tinyrexbasic.imx

    make distclean
    make mx6tinyrexbasicrecovery_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-tinyrexbasicrecovery.imx

#### Build (imx6d tinyrexpro) (production)
    make distclean
    make mx6tinyrexpro_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-tinyrexpro.imx

    make distclean
    make mx6tinyrexprorecovery_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-tinyrexprorecovery.imx

#### Build (imx6q tinyrexmax) (production)
    make distclean
    make mx6tinyrexmax_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-tinyrexmax.imx

    make distclean
    make mx6tinyrexmaxrecovery_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-tinyrexmaxrecovery.imx

#### Build (imx6q tinyrexmax4g) (production)
    make distclean
    make mx6tinyrexmax4g_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-tinyrexmax4g.imx

    make distclean
    make mx6tinyrexmax4grecovery_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-tinyrexmax4grecovery.imx

#### Build (imx6qp tinyrexultra) (production)
    make distclean
    make mx6tinyrexultra_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-tinyrexultra.imx

    make distclean
    make mx6tinyrexultrarecovery_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-tinyrexultrarecovery.imx

#### Build (imx6d rexbasic) (production)
    make distclean
    make mx6rexbasic_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-rexbasic.imx
    cp u-boot.bin /srv/tftp/imx6/u-boot-imx6-rexbasic.bin

#### Build (imx6q rexpro) (production)
    make distclean
    make mx6rexpro_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-rexpro.imx
    cp u-boot.bin /srv/tftp/imx6/u-boot-imx6-rexpro.bin

#### Build (imx6q rexpro) (SD card version)
    make distclean
    make mx6rexprosd_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-rexprosd.imx
    cp u-boot.bin /srv/tftp/imx6/u-boot-imx6-rexprosd.bin

#### Build (imx6q rexultra) (production)
    make distclean
    make mx6rexultra_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-rexultra.imx
    cp u-boot.bin /srv/tftp/imx6/u-boot-imx6-rexultra.bin

## IMPORTANT
    u-boot-imx6*-rex.imx must be flashed into spi flash at offset 0x400.

## Some lines that might be helpful
    setenv ipaddr 192.168.0.150
    setenv serverip 192.168.0.1
    tftp 0x17800000 imx6/u-boot-imx6q-rex.bin
    go 0x17800000

    mw.b 0x10800000 0xFF 0x80000;
    tftp 0x10800000 imx6/u-boot-imx6q-rex.imx;
    sf probe 2:2;sf erase 0x0 0x80000;sf write 0x10800000 0x400 0x7fc00
