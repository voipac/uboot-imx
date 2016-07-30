# uboot-imx-v2014.10
## Supported modules:
* TinyRex mx6max/pro/basic (production)
* TinyRex mx6q/dl/s (prototypes)
* Rex mx6pro/basic (production)
* Rex mx6dl (prototypes)

## Supported OS:
* Linux

## Download repository
    git clone -b uboot-imx-v2014.10 --single-branch https://github.com/voipac/uboot-imx uboot-imx-v2014.10-rex
    cd uboot-imx-v2014.10-rex

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
    cp u-boot.bin /srv/tftp/imx6/u-boot-imx6-tinyrexlite.bin

#### Build (imx6s tinyrexbasic) (production)
    make distclean
    make mx6tinyrexbasic_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-tinyrexbasic.imx
    cp u-boot.bin /srv/tftp/imx6/u-boot-imx6-tinyrexbasic.bin

#### Build (imx6d tinyrexpro) (production)
    make distclean
    make mx6tinyrexpro_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-tinyrexpro.imx
    cp u-boot.bin /srv/tftp/imx6/u-boot-imx6-tinyrexpro.bin

#### Build (imx6q tinyrexmax) (production)
    make distclean
    make mx6tinyrexmax_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-tinyrexmax.imx
    cp u-boot.bin /srv/tftp/imx6/u-boot-imx6-tinyrexmax.bin

#### Build (imx6q tinyrexultra) (production)
    make distclean
    make mx6tinyrexultra_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6-tinyrexultra.imx
    cp u-boot.bin /srv/tftp/imx6/u-boot-imx6-tinyrexultra.bin

#### Build (imx6s tinyrex) (prototype)
    make distclean
    make mx6stinyrex_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6s-tinyrex.imx
    cp u-boot.bin /srv/tftp/imx6/u-boot-imx6s-tinyrex.bin
    
#### Build (imx6dl tinyrex) (prototype)
    make distclean
    make mx6dltinyrex_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6dl-tinyrex.imx
    cp u-boot.bin /srv/tftp/imx6/u-boot-imx6dl-tinyrex.bin

#### Build (imx6q tinyrex) (prototype)
    make distclean
    make mx6qtinyrex_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6q-tinyrex.imx
    cp u-boot.bin /srv/tftp/imx6/u-boot-imx6q-tinyrex.bin

#### Build (imx6dl rex) (prototype)
    make distclean
    make mx6dlrex_config
    make
    cp u-boot.imx /srv/tftp/imx6/u-boot-imx6dl-rex.imx
    cp u-boot.bin /srv/tftp/imx6/u-boot-imx6dl-rex.bin

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
    u-boot-imx6q-rex.imx must be flashed into spi flash at offset 0x400.
    
## Some lines that might be helpful
    setenv ipaddr 192.168.0.150
    setenv serverip 192.168.0.1
    tftp 0x17800000 imx6/u-boot-imx6q-rex.bin
    go 0x17800000

    mw.b 0x10800000 0xFF 0x80000;
    tftp 0x10800000 imx6/u-boot-imx6q-rex.imx;
    sf probe 2:2;sf erase 0x0 0x80000;sf write 0x10800000 0x400 0x7fc00

  
