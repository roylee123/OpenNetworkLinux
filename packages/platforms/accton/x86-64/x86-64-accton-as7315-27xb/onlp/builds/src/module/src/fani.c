/************************************************************
 * <bsn.cl fy=2014 v=onl>
 *
 *           Copyright 2014 Big Switch Networks, Inc.
 *           Copyright 2014 Accton Technology Corporation.
 *
 * Licensed under the Eclipse Public License, Version 1.0 (the
 * "License"); you may not use this file except in compliance
 * with the License. You may obtain a copy of the License at
 *
 *        http://www.eclipse.org/legal/epl-v10.html
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the
 * License.
 *
 * </bsn.cl>
 ************************************************************
 *
 * Fan Platform Implementation Defaults.
 *
 ***********************************************************/
#include <onlplib/onie.h>
#include <onlp/sys.h>
#include <onlp/platformi/sysi.h>
#include <onlplib/i2c.h>
#include <onlplib/file.h>
#include <onlp/platformi/fani.h>
#include "platform_lib.h"


#define MAX_FAN_SPEED     25300

#define FAN_BOARD_PATH   "/sys/bus/i2c/devices/50-0066/"

#define CHASSIS_FAN_INFO(fid)           \
    { \
        { ONLP_FAN_ID_CREATE(FAN_##fid##_ON_FAN_BOARD), "Chassis Fan - "#fid, 0 },\
        0x0,\
        ONLP_FAN_CAPS_SET_PERCENTAGE | ONLP_FAN_CAPS_GET_RPM | ONLP_FAN_CAPS_GET_PERCENTAGE,\
        0,\
        0,\
        ONLP_FAN_MODE_INVALID,\
    }

enum fan_id {
    FAN_1_ON_FAN_BOARD = 1,
    FAN_2_ON_FAN_BOARD,
    FAN_3_ON_FAN_BOARD,
    FAN_4_ON_FAN_BOARD,
    FAN_5_ON_FAN_BOARD,
};

/* Static fan information */
onlp_fan_info_t finfo[] = {
    { }, /* Not used */
    CHASSIS_FAN_INFO(1),
    CHASSIS_FAN_INFO(2),
    CHASSIS_FAN_INFO(3),
    CHASSIS_FAN_INFO(4),
    CHASSIS_FAN_INFO(5),
};

#define VALIDATE(_id)                           \
    do {                                        \
        if(!ONLP_OID_IS_FAN(_id)) {             \
            return ONLP_STATUS_E_INVALID;       \
        }                                       \
    } while(0)


static bool
_read_syseeprom(onlp_onie_info_t *rv)
{
    uint8_t* ma = NULL;
    int size;

    if(onlp_sysi_onie_data_get(&ma, &size) == 0) {
        if(ma) {
            onlp_onie_decode(rv, ma, -1);
            aim_free(ma);
            return ONLP_STATUS_OK;
        }
    }
    return ONLP_STATUS_E_UNSUPPORTED;
}

static bool
_has_fan_control(void)
{
    static char board_rev[32] = {0};
    int rv;
    onlp_onie_info_t info;

    /*Read rev from sys api if none.*/
    if (!AIM_STRLEN(board_rev)) {
        rv = _read_syseeprom(&info);
        if( rv < 0) {
            AIM_LOG_ERROR("Failed: %d @_read_syseeprom", rv);
            return 0;
        }
        AIM_STRNCPY(board_rev, info.label_revision, sizeof(board_rev));
    }

    if(!AIM_STRCMP(board_rev, "R0A")) {
        return 0;
    }
    return 1;
}

/*
 * This function will be called prior to all of onlp_fani_* functions.
 */
int
onlp_fani_init(void)
{

    _has_fan_control();
    return ONLP_STATUS_OK;
}

int
fani_enable_fan(int fid, bool enable)
{
    char path[256];

    ONLPLIB_SNPRINTF(path, sizeof(path)-1,
                     FAN_BOARD_PATH"fan%d_enable", fid);

    if (onlp_file_write_int(!!enable, path) < 0) {
        AIM_LOG_ERROR("Unable to write data to file (%s)\r\n", path);
        return ONLP_STATUS_E_INTERNAL;
    }
    return ONLP_STATUS_OK;
}

int
fani_is_fan_enabled(int fid, bool *enable)
{
    char path[256];
    int val;

    ONLPLIB_SNPRINTF(path, sizeof(path)-1,
                     FAN_BOARD_PATH"fan%d_enable", fid);
    if (onlp_file_read_int(&val, path) < 0) {
        AIM_LOG_ERROR("Unable to get data from file (%s)\r\n", path);
        return ONLP_STATUS_E_INTERNAL;
    }

    *enable = val;
    return ONLP_STATUS_OK;
}



static int
_onlp_fani_info_get_fan(int fid, onlp_fan_info_t* info)
{
    bool has_fan_control, enable;

    has_fan_control = _has_fan_control();

    if (!has_fan_control) {
        info->status |= ONLP_FAN_STATUS_PRESENT;
        info->status |=  ONLP_FAN_STATUS_F2B;
        info->rpm = 11500;
        info->percentage = (info->rpm * 100)/MAX_FAN_SPEED;
    } else {
        int   value;
        /* get fan present status
         */
        if (onlp_file_read_int(&value, FAN_BOARD_PATH"fan%d_present", fid) < 0) {
            AIM_LOG_ERROR("Unable to read status from (%s)\r\n", FAN_BOARD_PATH);
            return ONLP_STATUS_E_INTERNAL;
        }

        if (value == 0) {
            return ONLP_STATUS_OK; /* fan is not present */
        }
        info->status |= ONLP_FAN_STATUS_PRESENT;


        /* get fan fault status (turn on when any one fails)
         */
        if (onlp_file_read_int(&value, FAN_BOARD_PATH"fan%d_fault", fid) < 0) {
            AIM_LOG_ERROR("Unable to read status from (%s)\r\n", FAN_BOARD_PATH);
            return ONLP_STATUS_E_INTERNAL;
        }

        /*True fault if fan is enabled.*/
        if( fani_is_fan_enabled(fid,&enable) != ONLP_STATUS_OK) {
            return ONLP_STATUS_E_INTERNAL;
        }

        if (value && enable) {
            info->status |= ONLP_FAN_STATUS_FAILED;
            return ONLP_STATUS_OK;
        }

        /* get fan speed
         */
        if (onlp_file_read_int(&value, FAN_BOARD_PATH"fan%d_input", fid) < 0) {
            AIM_LOG_ERROR("Unable to read status from (%s)\r\n", FAN_BOARD_PATH);
            return ONLP_STATUS_E_INTERNAL;
        }
        info->rpm = value;
        /* get speed percentage from rpm
             */
        info->percentage = (info->rpm * 100)/MAX_FAN_SPEED;

    }
    return ONLP_STATUS_OK;
}



int
onlp_fani_info_get(onlp_oid_t id, onlp_fan_info_t* info)
{
    int rc = 0;
    int fid;
    VALIDATE(id);

    fid = ONLP_OID_ID_GET(id);
    if (fid >= AIM_ARRAYSIZE(finfo) || fid == 0) {
        return ONLP_STATUS_E_INVALID;
    }
    *info = finfo[fid];

    switch (fid)
    {
    case FAN_1_ON_FAN_BOARD ... FAN_5_ON_FAN_BOARD:
        rc =_onlp_fani_info_get_fan(fid, info);
        break;
    default:
        rc = ONLP_STATUS_E_INVALID;
        break;
    }

    return rc;
}

/*
 * This function sets the fan speed of the given OID as a percentage.
 *
 * This will only be called if the OID has the PERCENTAGE_SET
 * capability.
 *
 * It is optional if you have no fans at all with this feature.
 */
int
onlp_fani_percentage_set(onlp_oid_t id, int p)
{
    bool has_fan_control;

    has_fan_control = _has_fan_control();
    if (!has_fan_control) {
        return ONLP_STATUS_E_INTERNAL;
    } else {
        int  fid;
        char path[256];

        VALIDATE(id);
        fid = ONLP_OID_ID_GET(id);
        /* Cannot set 0, CPLD treat 0 as full speed.
         * Contrast to set duty, turn the fan off.
         */
        if (p == 0) {
            return fani_enable_fan(fid, 0);
        }

        switch (fid)
        {
        case FAN_1_ON_FAN_BOARD ... FAN_5_ON_FAN_BOARD:
            ONLPLIB_SNPRINTF(path, sizeof(path)-1,
                             FAN_BOARD_PATH"fan%d_pwm", fid);
            break;
        default:
            return ONLP_STATUS_E_INVALID;
        }

        if (onlp_file_write_int(p, path) < 0) {
            AIM_LOG_ERROR("Unable to write data to file (%s)\r\n", path);
            return ONLP_STATUS_E_INTERNAL;
        }

        return ONLP_STATUS_OK;

    }
}
