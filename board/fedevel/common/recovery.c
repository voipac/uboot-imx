/*
 * Copyright (C) 2015-2017 Voipac.
 *
 * Author: support <support@voipac.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <malloc.h>
#include <recovery.h>

#if 0
void setup_recovery_env(void)
{
    board_recovery_setup();
}

/* export to lib_arm/board.c */
void check_recovery_mode(void)
{
    if (check_recovery_cmd_file()) {
        puts("Fastboot: Recovery command file found!\n");
        setup_recovery_env();
    } else {
        puts("Fastboot: Normal\n");
    }
}
#endif
