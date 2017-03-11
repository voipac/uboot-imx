# uboot-imx-v2015.04
## Supported modules:
* OpenRex ultra/max4g/max/basic
* TinyRex ultra/max4g/max/pro/basic
* Rex ultra/pro/basic

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
#### Build (imx6s openrexbasic)
    make distclean
    make mx6openrexbasic_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-openrexbasic.imx

#### Build (imx6q openrexmax)
    make distclean
    make mx6openrexmax_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-openrexmax.imx

#### Build (imx6q openrexmax4g)
    make distclean
    make mx6openrexmax4g_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-openrexmax4g.imx

#### Build (imx6qp openrexultra)
    make distclean
    make mx6openrexultra_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-openrexultra.imx

#### Build (imx6s tinyrexlite)
    make distclean
    make mx6tinyrexlite_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-tinyrexlite.imx

    make distclean
    make mx6tinyrexliterecovery_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-tinyrexliterecovery.imx

#### Build (imx6s tinyrexbasic)
    make distclean
    make mx6tinyrexbasic_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-tinyrexbasic.imx

    make distclean
    make mx6tinyrexbasicrecovery_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-tinyrexbasicrecovery.imx

#### Build (imx6d tinyrexpro)
    make distclean
    make mx6tinyrexpro_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-tinyrexpro.imx

    make distclean
    make mx6tinyrexprorecovery_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-tinyrexprorecovery.imx

#### Build (imx6q tinyrexmax)
    make distclean
    make mx6tinyrexmax_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-tinyrexmax.imx

    make distclean
    make mx6tinyrexmaxrecovery_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-tinyrexmaxrecovery.imx

#### Build (imx6q tinyrexmax4g)
    make distclean
    make mx6tinyrexmax4g_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-tinyrexmax4g.imx

    make distclean
    make mx6tinyrexmax4grecovery_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-tinyrexmax4grecovery.imx

#### Build (imx6qp tinyrexultra)
    make distclean
    make mx6tinyrexultra_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-tinyrexultra.imx

    make distclean
    make mx6tinyrexultrarecovery_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-tinyrexultrarecovery.imx

#### Build (imx6d rexbasic)
    make distclean
    make mx6rexbasic_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-rexbasic.imx
    cp u-boot.bin /srv/tftp/imx6/u-boot-imx6-rexbasic.bin

#### Build (imx6q rexpro)
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

#### Build (imx6q rexultra)
    make distclean
    make mx6rexultra_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-rexultra.imx
    cp u-boot.bin /srv/tftp/imx6/u-boot-imx6-rexultra.bin

## IMPORTANT
    u-boot-imx6-openrex*.imx must be flashed into spi flash at offset 0x400.
	u-boot-imx6-rex*.imx must be flashed into SD card at offset 0x400.
    u-boot-imx6-rex*.imx must be flashed into spi flash at offset 0x400.
