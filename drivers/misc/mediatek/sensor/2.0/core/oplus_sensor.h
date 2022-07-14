/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

/*
 * Copyright(C)2014 MediaTek Inc.
 * Modification based on code covered by the below mentioned copyright
 * and/or permission notice(S).
 */

#ifndef __OPLUSSENSOR_H__
#define __OPLUSSENSOR_H__
#include <linux/types.h>
#define SENSOR_TYPE_CAMERA_PROTECT                     (OPLUS_VIRTAUL_SENSOR_START)
#define SENSOR_TYPE_FREE_FALL                          (OPLUS_VIRTAUL_SENSOR_START + 1)
#define SENSOR_TYPE_PICKUP_DETECT                      (OPLUS_VIRTAUL_SENSOR_START + 2)
#define SENSOR_TYPE_FP_DISPLAY                         (OPLUS_VIRTAUL_SENSOR_START + 3)
#define SENSOR_TYPE_SAR_MODEM                          (OPLUS_VIRTAUL_SENSOR_START + 4)
#define SENSOR_TYPE_LUX_AOD                            (OPLUS_VIRTAUL_SENSOR_START + 5)
#define SENSOR_TYPE_PEDO_MINUTE                        (OPLUS_VIRTAUL_SENSOR_START + 6)
//#ifdef OPLUS_FEATURE_ACTIVITY_RECOGNITION
#define SENSOR_TYPE_OPLUS_ACTIVITY_RECOGNITION          (OPLUS_VIRTAUL_SENSOR_START + 8)
//#endif //OPLUS_FEATURE_ACTIVITY_RECOGNITION
//#ifdef OPLUS_FEATURE_ELEVATOR_DETECT
#define SENSOR_TYPE_ELEVATOR_DETECT                    (OPLUS_VIRTAUL_SENSOR_START + 10)
//#endif //OPLUS_FEATURE_ELEVATOR_DETECT
/* end sensor type */
#define SENSOR_TYPE_SENSOR_MONITOR                     (OPLUS_VIRTAUL_SENSOR_START + 11)
/* end sensor type */
#define VIRTUAL_SENSOR_TYPE_MAX                         (SENSOR_TYPE_SENSOR_MONITOR +1)


#define ID_OPLUS_BASE             (0)
#define ID_CAMERA_PROTECT       (ID_OPLUS_BASE + SENSOR_TYPE_CAMERA_PROTECT - 1)
#define ID_FREE_FALL            (ID_OPLUS_BASE + SENSOR_TYPE_FREE_FALL - 1)
#define ID_PICKUP_DETECT        (ID_OPLUS_BASE + SENSOR_TYPE_PICKUP_DETECT- 1)
#define ID_FP_DISPLAY           (ID_OPLUS_BASE + SENSOR_TYPE_FP_DISPLAY - 1)
#define ID_SAR_MODEM            (ID_OPLUS_BASE + SENSOR_TYPE_SAR_MODEM - 1)
#define ID_LUX_AOD              (ID_OPLUS_BASE + SENSOR_TYPE_LUX_AOD - 1)
#define ID_PEDO_MINUTE          (ID_OPLUS_BASE + SENSOR_TYPE_PEDO_MINUTE - 1)
//#ifdef OPLUS_FEATURE_ACTIVITY_RECOGNITION
#define ID_OPLUS_ACTIVITY_RECOGNITION (ID_OPLUS_BASE + SENSOR_TYPE_OPLUS_ACTIVITY_RECOGNITION - 1)
//#endif //OPLUS_FEATURE_ACTIVITY_RECOGNITION

//#ifdef OPLUS_FEATURE_ELEVATOR_DETECT
#define ID_ELEVATOR_DETECT      (ID_OPLUS_BASE + SENSOR_TYPE_ELEVATOR_DETECT - 1)
//#endif //OPLUS_FEATURE_ELEVATOR_DETECT
#define ID_SENSOR_MONITOR       (ID_OPLUS_BASE + SENSOR_TYPE_SENSOR_MONITOR - 1)
/* end sensor ID */
#define ID_VIRTUAL_SENSOR_MAX    (ID_SENSOR_MONITOR + 1)
typedef struct {
    uint32_t value;
    uint16_t report_count;
}camera_protect_event_t;

typedef struct {
    uint32_t free_fall_time;
    uint32_t angle;
    uint16_t report_count;
}free_fall_event_t;

typedef struct {
    uint32_t value;
    uint16_t report_count;
}pickup_detect_event_t;

typedef struct {
    uint32_t value;
    uint16_t report_count;
}fp_display_event_t;

typedef struct {
    int32_t state;
}sar_modem_event_t;

typedef struct {
    int16_t state;
    uint16_t report_count;
}lux_aod_event_t;

typedef struct {
    int16_t state;
    uint16_t report_count;
}sensor_monitor_event_t;

typedef struct {
    uint32_t step_count;
    uint16_t report_count;
    uint32_t move_status;
    uint16_t time_gap;
    uint32_t step_run_count;
    uint32_t step_walk_count;
}pedo_minute_event_t;

//#ifdef OPLUS_FEATURE_ACTIVITY_RECOGNITION
typedef struct {
    uint16_t motion_count;
    uint16_t motion_ith;
    uint16_t incar_state;
    uint16_t activity_mode;
    uint32_t delta_time;
}oplus_activity_recognition_event_t;
//#endif //OPLUS_FEATURE_ACTIVITY_RECOGNITION

//#ifdef OPLUS_FEATURE_ELEVATOR_DETECT
typedef struct {
    uint16_t value;
    uint16_t report_count;
} elevator_detect_event_t;
//#endif //OPLUS_FEATURE_ELEVATOR_DETECT

union oplus_data_unit_t {
    camera_protect_event_t camera_protect_data_t;
    free_fall_event_t free_fall_data_t;
    pickup_detect_event_t pickup_detect_data_t;
    fp_display_event_t fp_display_data_t;
    sar_modem_event_t sar_modem_event;
    lux_aod_event_t lux_aod_event;
    sensor_monitor_event_t sensor_monitor_event;
    pedo_minute_event_t pedo_minute_event;
    //#ifdef OPLUS_FEATURE_ACTIVITY_RECOGNITION
    oplus_activity_recognition_event_t oplus_activity_recognition_event;
    //#endif //OPLUS_FEATURE_ACTIVITY_RECOGNITION
    //#ifdef OPLUS_FEATURE_ELEVATOR_DETECT
    elevator_detect_event_t elevator_detect_event;
    //#endif //OPLUS_FEATURE_ELEVATOR_DETECT
};

#endif /*__OPLUSSENSOR_H*/