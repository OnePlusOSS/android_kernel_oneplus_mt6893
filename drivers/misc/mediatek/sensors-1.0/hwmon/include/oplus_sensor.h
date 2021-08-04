/* SPDX-License-Identifier: GPL-2.0-only  */
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#ifndef __OPLUSSENSOR_H__
#define __OPLUSSENSOR_H__

#define SENSOR_TYPE_CAMERA_PROTECT                     73
#define SENSOR_TYPE_FREE_FALL                          74
#define SENSOR_TYPE_PICKUP_DETECT                      75
#define SENSOR_TYPE_FP_DISPLAY                         76
#define SENSOR_TYPE_LUX_AOD                            78
#define SENSOR_TYPE_PEDO_MINUTE                        79
//#ifdef OPLUS_FEATURE_ACTIVITY_RECOGNITION
#define SENSOR_TYPE_OPLUS_ACTIVITY_RECOGNITION          81
//#endif //OPLUS_FEATURE_ACTIVITY_RECOGNITION
/* end sensor type */
#define SENSOR_TYPE_MAX_NUM                            SENSOR_TYPE_OPLUS_ACTIVITY_RECOGNITION


#define ID_OPLUS_BASE 			(0)
#define ID_CAMERA_PROTECT       (ID_OPLUS_BASE + SENSOR_TYPE_CAMERA_PROTECT - 1)
#define ID_FREE_FALL            (ID_OPLUS_BASE + SENSOR_TYPE_FREE_FALL - 1)
#define ID_PICKUP_DETECT        (ID_OPLUS_BASE + SENSOR_TYPE_PICKUP_DETECT- 1)
#define ID_FP_DISPLAY           (ID_OPLUS_BASE + SENSOR_TYPE_FP_DISPLAY - 1)
#define ID_LUX_AOD              (ID_OPLUS_BASE + SENSOR_TYPE_LUX_AOD - 1)
#define ID_PEDO_MINUTE          (ID_OPLUS_BASE + SENSOR_TYPE_PEDO_MINUTE - 1)
//#ifdef OPLUS_FEATURE_ACTIVITY_RECOGNITION
#define ID_OPLUS_ACTIVITY_RECOGNITION (ID_OPLUS_BASE + SENSOR_TYPE_OPLUS_ACTIVITY_RECOGNITION - 1)
//#endif //OPLUS_FEATURE_ACTIVITY_RECOGNITION
/* end sensor ID */
#define ID_SENSOR_MAX_HANDLE    (ID_OPLUS_ACTIVITY_RECOGNITION)
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

union oplus_data_unit_t {
    camera_protect_event_t camera_protect_data_t;
    free_fall_event_t free_fall_data_t;
    pickup_detect_event_t pickup_detect_data_t;
    fp_display_event_t fp_display_data_t;
    sar_modem_event_t sar_modem_event;
    lux_aod_event_t lux_aod_event;
    pedo_minute_event_t pedo_minute_event;
    //#ifdef OPLUS_FEATURE_ACTIVITY_RECOGNITION
    oplus_activity_recognition_event_t oplus_activity_recognition_event;
    //#endif //OPLUS_FEATURE_ACTIVITY_RECOGNITION
};

#endif /*__OPLUSSENSOR_H*/