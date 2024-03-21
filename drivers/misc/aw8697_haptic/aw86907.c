/*
 * aw86907.c   aw86907 haptic module
 *
 * Version: v0.0.1
 *
 * Copyright (c) 2018 AWINIC Technology CO., LTD
 *
 *  Author: Nick Li <liweilei@awinic.com.cn>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/syscalls.h>
#include <linux/power_supply.h>
#include <linux/mman.h>
#include "aw86907.h"
#include "aw86907_reg.h"
#include "aw86907_config.h"
#ifdef OPLUS_FEATURE_CHG_BASIC
#include <linux/proc_fs.h>
#include <linux/pm_qos.h>
#include <linux/vmalloc.h>
#include <soc/oplus/device_info.h>
#include <soc/oplus/system/oplus_project.h>
#endif

/******************************************************
 *
 * Marco
 *
 ******************************************************/
#define AW86907_I2C_NAME "aw86907_haptic"
#define AW86907_HAPTIC_NAME "awinic_haptic"

#define AW86907_VERSION "v0.0.1"

#define AWINIC_RAM_UPDATE_DELAY

#define AW_I2C_RETRIES 10
#define AW_I2C_RETRY_DELAY 2
#define AW_READ_CHIPID_RETRIES 5
#define AW_READ_CHIPID_RETRY_DELAY 2

#define AW86907MAX_DSP_START_TRY_COUNT 10

#define AW86907MAX_FIRMWARE_LOAD_CNT 20
#define OP_AW_DEBUG

#define SCORE_MODE
#ifdef SCORE_MODE
#define FIRST_SCOREMODE
//#define SECOND_SCOREMODE
#else
#define AISCAN_CTRL
#endif

#define TRUST_LEVEL 3
static int s_tz_init = 0;
#define ABS(x) ((x) < 0 ? (-x) : (x))

/* add haptic audio tp mask */
//extern struct shake_point record_point[10];
/* add haptic audio tp mask end */
/******************************************************
 *
 * variable
 *
 ******************************************************/
#define PM_QOS_VALUE_VB 400
struct pm_qos_request pm_qos_req_vb_aw86907;

static uint8_t AW86907_HAPTIC_RAM_VBAT_COMP_GAIN;

#define AW86907_RTP_NAME_MAX 64
//static char *aw86907_ram_name = "aw86907_haptic.bin";
//
static char aw86907_ram_name[5][30] = {
	{ "aw8697_haptic_170.bin" }, { "aw8697_haptic_170.bin" },
	{ "aw8697_haptic_170.bin" }, { "aw8697_haptic_170.bin" },
	{ "aw8697_haptic_170.bin" },
};

static char aw86907_long_sound_rtp_name[5][30] = {
	{ "aw8697_long_sound_168.bin" },
	{ "aw8697_long_sound_170.bin" },
	{ "aw8697_long_sound_173.bin" },
	{ "aw8697_long_sound_175.bin" },
};

static char aw86907_old_steady_test_rtp_name_0815[11][60] = {
	{ "aw8697_old_steady_test_RTP_52_160Hz.bin" },
	{ "aw8697_old_steady_test_RTP_52_162Hz.bin" },
	{ "aw8697_old_steady_test_RTP_52_164Hz.bin" },
	{ "aw8697_old_steady_test_RTP_52_166Hz.bin" },
	{ "aw8697_old_steady_test_RTP_52_168Hz.bin" },
	{ "aw8697_old_steady_test_RTP_52_170Hz.bin" },
	{ "aw8697_old_steady_test_RTP_52_172Hz.bin" },
	{ "aw8697_old_steady_test_RTP_52_174Hz.bin" },
	{ "aw8697_old_steady_test_RTP_52_176Hz.bin" },
	{ "aw8697_old_steady_test_RTP_52_178Hz.bin" },
	{ "aw8697_old_steady_test_RTP_52_180Hz.bin" },
};

static char aw86907_high_temp_high_humidity_0815[11][60] = {
	{ "aw8697_high_temp_high_humidity_channel_RTP_51_160Hz.bin" },
	{ "aw8697_high_temp_high_humidity_channel_RTP_51_162Hz.bin" },
	{ "aw8697_high_temp_high_humidity_channel_RTP_51_164Hz.bin" },
	{ "aw8697_high_temp_high_humidity_channel_RTP_51_166Hz.bin" },
	{ "aw8697_high_temp_high_humidity_channel_RTP_51_168Hz.bin" },
	{ "aw8697_high_temp_high_humidity_channel_RTP_51_170Hz.bin" },
	{ "aw8697_high_temp_high_humidity_channel_RTP_51_172Hz.bin" },
	{ "aw8697_high_temp_high_humidity_channel_RTP_51_174Hz.bin" },
	{ "aw8697_high_temp_high_humidity_channel_RTP_51_176Hz.bin" },
	{ "aw8697_high_temp_high_humidity_channel_RTP_51_178Hz.bin" },
	{ "aw8697_high_temp_high_humidity_channel_RTP_51_180Hz.bin" },
};

static char aw86907_old_steady_test_rtp_name_0832[11][60] = {
	{ "aw8697_old_steady_test_RTP_52_225Hz.bin" },
	{ "aw8697_old_steady_test_RTP_52_226Hz.bin" },
	{ "aw8697_old_steady_test_RTP_52_227Hz.bin" },
	{ "aw8697_old_steady_test_RTP_52_228Hz.bin" },
	{ "aw8697_old_steady_test_RTP_52_229Hz.bin" },
	{ "aw8697_old_steady_test_RTP_52_230Hz.bin" },
	{ "aw8697_old_steady_test_RTP_52_231Hz.bin" },
	{ "aw8697_old_steady_test_RTP_52_232Hz.bin" },
	{ "aw8697_old_steady_test_RTP_52_233Hz.bin" },
	{ "aw8697_old_steady_test_RTP_52_234Hz.bin" },
	{ "aw8697_old_steady_test_RTP_52_235Hz.bin" },
};

static char aw86907_high_temp_high_humidity_0832[11][60] = {
	{ "aw8697_high_temp_high_humidity_channel_RTP_51_225Hz.bin" },
	{ "aw8697_high_temp_high_humidity_channel_RTP_51_226Hz.bin" },
	{ "aw8697_high_temp_high_humidity_channel_RTP_51_227Hz.bin" },
	{ "aw8697_high_temp_high_humidity_channel_RTP_51_228Hz.bin" },
	{ "aw8697_high_temp_high_humidity_channel_RTP_51_229Hz.bin" },
	{ "aw8697_high_temp_high_humidity_channel_RTP_51_230Hz.bin" },
	{ "aw8697_high_temp_high_humidity_channel_RTP_51_231Hz.bin" },
	{ "aw8697_high_temp_high_humidity_channel_RTP_51_232Hz.bin" },
	{ "aw8697_high_temp_high_humidity_channel_RTP_51_233Hz.bin" },
	{ "aw8697_high_temp_high_humidity_channel_RTP_51_234Hz.bin" },
	{ "aw8697_high_temp_high_humidity_channel_RTP_51_235Hz.bin" },
};
static char aw86907_ringtone_rtp_f0_170_name[][AW86907_RTP_NAME_MAX] = {
	{ "aw8697_rtp.bin" },
	{ "aw8697_Hearty_channel_RTP_1_170.bin" },
	{ "aw8697_Instant_channel_RTP_2_170.bin" },
	{ "aw8697_Music_channel_RTP_3_170.bin" },
	{ "aw8697_Percussion_channel_RTP_4_170.bin" },
	{ "aw8697_Ripple_channel_RTP_5_170.bin" },
	{ "aw8697_Bright_channel_RTP_6_170.bin" },
	{ "aw8697_Fun_channel_RTP_7_170.bin" },
	{ "aw8697_Glittering_channel_RTP_8_170.bin" },
	{ "aw8697_Granules_channel_RTP_9_170.bin" },
	{ "aw8697_Harp_channel_RTP_10_170.bin" },
	{ "aw8697_Impression_channel_RTP_11_170.bin" },
	{ "aw8697_Ingenious_channel_RTP_12_170.bin" },
	{ "aw8697_Joy_channel_RTP_13_170.bin" },
	{ "aw8697_Overtone_channel_RTP_14_170.bin" },
	{ "aw8697_Receive_channel_RTP_15_170.bin" },
	{ "aw8697_Splash_channel_RTP_16_170.bin" },

	{ "aw8697_About_School_RTP_17_170.bin" },
	{ "aw8697_Bliss_RTP_18_170.bin" },
	{ "aw8697_Childhood_RTP_19_170.bin" },
	{ "aw8697_Commuting_RTP_20_170.bin" },
	{ "aw8697_Dream_RTP_21_170.bin" },
	{ "aw8697_Firefly_RTP_22_170.bin" },
	{ "aw8697_Gathering_RTP_23_170.bin" },
	{ "aw8697_Gaze_RTP_24_170.bin" },
	{ "aw8697_Lakeside_RTP_25_170.bin" },
	{ "aw8697_Lifestyle_RTP_26_170.bin" },
	{ "aw8697_Memories_RTP_27_170.bin" },
	{ "aw8697_Messy_RTP_28_170.bin" },
	{ "aw8697_Night_RTP_29_170.bin" },
	{ "aw8697_Passionate_Dance_RTP_30_170.bin" },
	{ "aw8697_Playground_RTP_31_170.bin" },
	{ "aw8697_Relax_RTP_32_170.bin" },
	{ "aw8697_Reminiscence_RTP_33_170.bin" },
	{ "aw8697_Silence_From_Afar_RTP_34_170.bin" },
	{ "aw8697_Silence_RTP_35_170.bin" },
	{ "aw8697_Stars_RTP_36_170.bin" },
	{ "aw8697_Summer_RTP_37_170.bin" },
	{ "aw8697_Toys_RTP_38_170.bin" },
	{ "aw8697_Travel_RTP_39_170.bin" },
	{ "aw8697_Vision_RTP_40_170.bin" },

	{ "aw8697_reserved.bin" },
	{ "aw8697_reserved.bin" },
	{ "aw8697_reserved.bin" },
	{ "aw8697_reserved.bin" },
	{ "aw8697_reserved.bin" },
	{ "aw8697_reserved.bin" },

	{ "aw8697_reserved.bin" },
	{ "aw8697_Simple_channel_RTP_48_170.bin" },
	{ "aw8697_Pure_RTP_49_170.bin" },
	{ "barca_alarm_ring_RTP_120_170.bin" },
	{ "barca_incoming_ring_RTP_121_170.bin" },
	{ "barca_notice_ring_RTP_122_170.bin" },
};

#ifdef OPLUS_FEATURE_CHG_BASIC
static char aw86907_rtp_name_165Hz[][AW86907_RTP_NAME_MAX] = {
	{ "aw8697_rtp.bin" },
	{ "aw8697_Hearty_channel_RTP_1.bin" },
	{ "aw8697_Instant_channel_RTP_2_165Hz.bin" },
	{ "aw8697_Music_channel_RTP_3.bin" },
	{ "aw8697_Percussion_channel_RTP_4.bin" },
	{ "aw8697_Ripple_channel_RTP_5.bin" },
	{ "aw8697_Bright_channel_RTP_6.bin" },
	{ "aw8697_Fun_channel_RTP_7.bin" },
	{ "aw8697_Glittering_channel_RTP_8.bin" },
	{ "aw8697_Granules_channel_RTP_9_165Hz.bin" },
	{ "aw8697_Harp_channel_RTP_10.bin" },
	{ "aw8697_Impression_channel_RTP_11.bin" },
	{ "aw8697_Ingenious_channel_RTP_12_165Hz.bin" },
	{ "aw8697_Joy_channel_RTP_13_165Hz.bin" },
	{ "aw8697_Overtone_channel_RTP_14.bin" },
	{ "aw8697_Receive_channel_RTP_15_165Hz.bin" },
	{ "aw8697_Splash_channel_RTP_16_165Hz.bin" },

	{ "aw8697_About_School_RTP_17_165Hz.bin" },
	{ "aw8697_Bliss_RTP_18.bin" },
	{ "aw8697_Childhood_RTP_19_165Hz.bin" },
	{ "aw8697_Commuting_RTP_20_165Hz.bin" },
	{ "aw8697_Dream_RTP_21.bin" },
	{ "aw8697_Firefly_RTP_22_165Hz.bin" },
	{ "aw8697_Gathering_RTP_23.bin" },
	{ "aw8697_Gaze_RTP_24_165Hz.bin" },
	{ "aw8697_Lakeside_RTP_25_165Hz.bin" },
	{ "aw8697_Lifestyle_RTP_26.bin" },
	{ "aw8697_Memories_RTP_27_165Hz.bin" },
	{ "aw8697_Messy_RTP_28_165Hz.bin" },
	{ "aw8697_Night_RTP_29_165Hz.bin" },
	{ "aw8697_Passionate_Dance_RTP_30_165Hz.bin" },
	{ "aw8697_Playground_RTP_31_165Hz.bin" },
	{ "aw8697_Relax_RTP_32_165Hz.bin" },
	{ "aw8697_Reminiscence_RTP_33.bin" },
	{ "aw8697_Silence_From_Afar_RTP_34_165Hz.bin" },
	{ "aw8697_Silence_RTP_35_165Hz.bin" },
	{ "aw8697_Stars_RTP_36_165Hz.bin" },
	{ "aw8697_Summer_RTP_37_165Hz.bin" },
	{ "aw8697_Toys_RTP_38_165Hz.bin" },
	{ "aw8697_Travel_RTP_39.bin" },
	{ "aw8697_Vision_RTP_40.bin" },

	{ "aw8697_waltz_channel_RTP_41_165Hz.bin" },
	{ "aw8697_cut_channel_RTP_42_165Hz.bin" },
	{ "aw8697_clock_channel_RTP_43_165Hz.bin" },
	{ "aw8697_long_sound_channel_RTP_44_165Hz.bin" },
	{ "aw8697_short_channel_RTP_45_165Hz.bin" },
	{ "aw8697_two_error_remaind_RTP_46_165Hz.bin" },

	{ "aw8697_kill_program_RTP_47_165Hz.bin" },
	{ "aw8697_Simple_channel_RTP_48.bin" },
	{ "aw8697_Pure_RTP_49_165Hz.bin" },
	{ "aw8697_reserved_sound_channel_RTP_50.bin" },

	{ "aw8697_high_temp_high_humidity_channel_RTP_51.bin" },

	{ "aw8697_old_steady_test_RTP_52.bin" },
	{ "aw8697_listen_pop_53.bin" },
	{ "aw8697_desk_7_RTP_54_165Hz.bin" },
	{ "aw8697_nfc_10_RTP_55_165Hz.bin" },
	{ "aw8697_vibrator_remain_12_RTP_56_165Hz.bin" },
	{ "aw8697_notice_13_RTP_57.bin" },
	{ "aw8697_third_ring_14_RTP_58.bin" },
	{ "aw8697_emergency_warning_RTP_59_165Hz.bin" },

	{ "aw8697_honor_fisrt_kill_RTP_60_165Hz.bin" },
	{ "aw8697_honor_two_kill_RTP_61_165Hz.bin" },
	{ "aw8697_honor_three_kill_RTP_62_165Hz.bin" },
	{ "aw8697_honor_four_kill_RTP_63_165Hz.bin" },
	{ "aw8697_honor_five_kill_RTP_64_165Hz.bin" },
	{ "aw8697_honor_three_continu_kill_RTP_65_165Hz.bin" },
	{ "aw8697_honor_four_continu_kill_RTP_66_165Hz.bin" },
	{ "aw8697_honor_unstoppable_RTP_67_165Hz.bin" },
	{ "aw8697_honor_thousands_kill_RTP_68_165Hz.bin" },
	{ "aw8697_honor_lengendary_RTP_69_165Hz.bin" },

	{ "aw8697_Freshmorning_RTP_70_165Hz.bin" },
	{ "aw8697_Peaceful_RTP_71_165Hz.bin" },
	{ "aw8697_Cicada_RTP_72_165Hz.bin" },
	{ "aw8697_Electronica_RTP_73_165Hz.bin" },
	{ "aw8697_Holiday_RTP_74_165Hz.bin" },
	{ "aw8697_Funk_RTP_75_165Hz.bin" },
	{ "aw8697_House_RTP_76_165Hz.bin" },
	{ "aw8697_Temple_RTP_77_165Hz.bin" },
	{ "aw8697_Dreamyjazz_RTP_78_165Hz.bin" },
	{ "aw8697_Modern_RTP_79_165Hz.bin" },

	{ "aw8697_Round_RTP_80_165Hz.bin" },
	{ "aw8697_Rising_RTP_81_165Hz.bin" },
	{ "aw8697_Wood_RTP_82_165Hz.bin" },
	{ "aw8697_Heys_RTP_83_165Hz.bin" },
	{ "aw8697_Mbira_RTP_84_165Hz.bin" },
	{ "aw8697_News_RTP_85_165Hz.bin" },
	{ "aw8697_Peak_RTP_86_165Hz.bin" },
	{ "aw8697_Crisp_RTP_87_165Hz.bin" },
	{ "aw8697_Singingbowls_RTP_88_165Hz.bin" },
	{ "aw8697_Bounce_RTP_89_165Hz.bin" },

	{ "aw8697_reserved_90.bin" },
	{ "aw8697_reserved_91.bin" },
	{ "aw8697_reserved_92.bin" },
	{ "aw8697_reserved_93.bin" },
	{ "aw8697_ALCloudscape_94_165HZ.bin" },
	{ "aw8697_ALGoodenergy_95_165HZ.bin" },
	{ "aw8697_NTblink_96_165HZ.bin" },
	{ "aw8697_NTwhoop_97_165HZ.bin" },
	{ "aw8697_Newfeeling_98_165HZ.bin" },
	{ "aw8697_nature_99_165HZ.bin" },

	{ "aw8697_soldier_first_kill_RTP_100_165Hz.bin" },
	{ "aw8697_soldier_second_kill_RTP_101_165Hz.bin" },
	{ "aw8697_soldier_third_kill_RTP_102_165Hz.bin" },
	{ "aw8697_soldier_fourth_kill_RTP_103_165Hz.bin" },
	{ "aw8697_soldier_fifth_kill_RTP_104_165Hz.bin" },
	{ "aw8697_stepable_regulate_RTP_105_165Hz.bin" },
	{ "aw8697_voice_level_bar_edge_RTP_106_165Hz.bin" },
	{ "aw8697_strength_level_bar_edge_RTP_107_165Hz.bin" },
	{ "aw8697_charging_simulation_RTP_108_165Hz.bin" },
	{ "aw8697_fingerprint_success_RTP_109_165Hz.bin" },

	{ "aw8697_fingerprint_effect1_RTP_110_165Hz.bin" },
	{ "aw8697_fingerprint_effect2_RTP_111_165Hz.bin" },
	{ "aw8697_fingerprint_effect3_RTP_112_165Hz.bin" },
	{ "aw8697_fingerprint_effect4_RTP_113_165Hz.bin" },
	{ "aw8697_fingerprint_effect5_RTP_114_165Hz.bin" },
	{ "aw8697_fingerprint_effect6_RTP_115_165Hz.bin" },
	{ "aw8697_fingerprint_effect7_RTP_116_165Hz.bin" },
	{ "aw8697_fingerprint_effect8_RTP_117_165Hz.bin" },
	{ "aw8697_breath_simulation_RTP_118_165Hz.bin" },
	{ "aw8697_reserved_119.bin" },

	{ "aw8697_Miss_RTP_120.bin" },
	{ "aw8697_Scenic_RTP_121_165Hz.bin" },
	{ "aw8697_voice_assistant_RTP_122_165Hz.bin" },
	/* used for 7 */
	{ "aw8697_Appear_channel_RTP_123_165Hz.bin" },
	{ "aw8697_Miss_RTP_124_165Hz.bin" },
	{ "aw8697_Music_channel_RTP_125_165Hz.bin" },
	{ "aw8697_Percussion_channel_RTP_126_165Hz.bin" },
	{ "aw8697_Ripple_channel_RTP_127_165Hz.bin" },
	{ "aw8697_Bright_channel_RTP_128_165Hz.bin" },
	{ "aw8697_Fun_channel_RTP_129_165Hz.bin" },
	{ "aw8697_Glittering_channel_RTP_130_165Hz.bin" },
	{ "aw8697_Harp_channel_RTP_131_165Hz.bin" },
	{ "aw8697_Overtone_channel_RTP_132_165Hz.bin" },
	{ "aw8697_Simple_channel_RTP_133_165Hz.bin" },

	{ "aw8697_Seine_past_RTP_134_165Hz.bin" },
	{ "aw8697_Classical_ring_RTP_135_165Hz.bin" },
	{ "aw8697_Long_for_RTP_136_165Hz.bin" },
	{ "aw8697_Romantic_RTP_137_165Hz.bin" },
	{ "aw8697_Bliss_RTP_138_165Hz.bin" },
	{ "aw8697_Dream_RTP_139_165Hz.bin" },
	{ "aw8697_Relax_RTP_140_165Hz.bin" },
	{ "aw8697_Joy_channel_RTP_141_165Hz.bin" },
	{ "aw8697_weather_wind_RTP_142_165Hz.bin" },
	{ "aw8697_weather_cloudy_RTP_143_165Hz.bin" },
	{ "aw8697_weather_thunderstorm_RTP_144_165Hz.bin" },
	{ "aw8697_weather_default_RTP_145_165Hz.bin" },
	{ "aw8697_weather_sunny_RTP_146_165Hz.bin" },
	{ "aw8697_weather_smog_RTP_147_165Hz.bin" },
	{ "aw8697_weather_snow_RTP_148_165Hz.bin" },
	{ "aw8697_weather_rain_RTP_149_165Hz.bin" },

	/* used for 7 end*/
	{ "aw8697_rtp_lighthouse.bin" },
	{ "aw8697_rtp_silk.bin" },
	{ "aw8697_reserved_152.bin" },
	{ "aw8697_reserved_153.bin" },
	{ "aw8697_reserved_154.bin" },
	{ "aw8697_reserved_155.bin" },
	{ "aw8697_reserved_156.bin" },
	{ "aw8697_reserved_157.bin" },
	{ "aw8697_reserved_158.bin" },
	{ "aw8697_reserved_159.bin" },
	{ "aw8697_reserved_160.bin" },
	{ "aw8697_reserved_161.bin" },
	{ "aw8697_reserved_162.bin" },
	{ "aw8697_reserved_163.bin" },
	{ "aw8697_reserved_164.bin" },
	{ "aw8697_reserved_165.bin" },
	{ "aw8697_reserved_166.bin" },
	{ "aw8697_reserved_167.bin" },
	{ "aw8697_reserved_168.bin" },
	{ "aw8697_reserved_169.bin" },
	{ "aw8697_reserved_170.bin" },
	{ "aw8697_reserved_171.bin" },
	{ "aw8697_reserved_172.bin" },
	{ "aw8697_reserved_173.bin" },
	{ "aw8697_reserved_174.bin" },
	{ "aw8697_reserved_175.bin" },
	{ "aw8697_reserved_176.bin" },
	{ "aw8697_reserved_177.bin" },
	{ "aw8697_reserved_178.bin" },
	{ "aw8697_reserved_179.bin" },
	{ "aw8697_reserved_180.bin" },
	{ "aw8697_reserved_181.bin" },
	{ "aw8697_reserved_182.bin" },
	{ "aw8697_reserved_183.bin" },
	{ "aw8697_reserved_184.bin" },
	{ "aw8697_reserved_185.bin" },
	{ "aw8697_reserved_186.bin" },
	{ "aw8697_reserved_187.bin" },
	{ "aw8697_reserved_188.bin" },
	{ "aw8697_reserved_189.bin" },
	{ "aw8697_reserved_190.bin" },
	{ "aw8697_reserved_191.bin" },
	{ "aw8697_reserved_192.bin" },
	{ "aw8697_reserved_193.bin" },
	{ "aw8697_reserved_194.bin" },
	{ "aw8697_reserved_195.bin" },
	{ "aw8697_reserved_196.bin" },
	{ "aw8697_reserved_197.bin" },
	{ "alarm_Pacman_RTP_198_165Hz.bin" },
	{ "notif_Pacman_RTP_199_165Hz.bin" },
	{ "ringtone_Pacman_RTP_200_165Hz.bin" },
	{ "ringtone_Alacrity_RTP_201_165Hz.bin" },
	{ "ring_Amenity_RTP_202_165Hz.bin" },
	{ "ringtone_Blues_RTP_203_165Hz.bin" },
	{ "ring_Bounce_RTP_204_165Hz.bin" },
	{ "ring_Calm_RTP_205_165Hz.bin" },
	{ "ringtone_Cloud_RTP_206_165Hz.bin" },
	{ "ringtone_Cyclotron_RTP_207_165Hz.bin" },
	{ "ringtone_Distinct_RTP_208_165Hz.bin" },
	{ "ringtone_Dynamic_RTP_209_165Hz.bin" },
	{ "ringtone_Echo_RTP_210_165Hz.bin" },
	{ "ringtone_Expect_RTP_211_165Hz.bin" },
	{ "ringtone_Fanatical_RTP_212_165Hz.bin" },
	{ "ringtone_Funky_RTP_213_165Hz.bin" },
	{ "ringtone_Guitar_RTP_214_165Hz.bin" },
	{ "ringtone_Harping_RTP_215_165Hz.bin" },
	{ "ringtone_Highlight_RTP_216_165Hz.bin" },
	{ "ringtone_Idyl_RTP_217_165Hz.bin" },
	{ "ringtone_Innocence_RTP_218_165Hz.bin" },
	{ "ringtone_Journey_RTP_219_165Hz.bin" },
	{ "ringtone_Joyous_RTP_220_165Hz.bin" },
	{ "ring_Lazy_RTP_221_165Hz.bin" },
	{ "ringtone_Marimba_RTP_222_165Hz.bin" },
	{ "ring_Mystical_RTP_223_165Hz.bin" },
	{ "ringtone_Old_telephone_RTP_224_165Hz.bin" },
	{ "ringtone_Oneplus_tune_RTP_225_165Hz.bin" },
	{ "ringtone_Rhythm_RTP_226_165Hz.bin" },
	{ "ringtone_Optimistic_RTP_227_165Hz.bin" },
	{ "ringtone_Piano_RTP_228_165Hz.bin" },
	{ "ring_Whirl_RTP_229_165Hz.bin" },
	{ "VZW_Alrwave_RTP_230_165Hz.bin" },
	{ "t-jingle_RTP_231_165Hz.bin" },
	{ "ringtone_Eager_232_165Hz.bin" },
	{ "ringtone_Ebullition_233_165Hz.bin" },
	{ "ringtone_Friendship_234_165Hz.bin" },
	{ "ringtone_Jazz_life_RTP_235_165Hz.bin" },
	{ "ringtone_Sun_glittering_RTP_236_165Hz.bin" },
	{ "notif_Allay_RTP_237_165Hz.bin" },
	{ "notif_Allusion_RTP_238_165Hz.bin" },
	{ "notif_Amiable_RTP_239_165Hz.bin" },
	{ "notif_Blare_RTP_240_165Hz.bin" },
	{ "notif_Blissful_RTP_241_165Hz.bin" },
	{ "notif_Brisk_RTP_242_165Hz.bin" },
	{ "notif_Bubble_RTP_243_165Hz.bin" },
	{ "notif_Cheerful_RTP_244_165Hz.bin" },
	{ "notif_Clear_RTP_245_165Hz.bin" },
	{ "notif_Comely_RTP_246_165Hz.bin" },
	{ "notif_Cozy_RTP_247_165Hz.bin" },
	{ "notif_Ding_RTP_248_165Hz.bin" },
	{ "notif_Effervesce_RTP_249_165Hz.bin" },
	{ "notif_Elegant_RTP_250_165Hz.bin" },
	{ "notif_Free_RTP_251_165Hz.bin" },
	{ "notif_Hallucination_RTP_252_165Hz.bin" },
	{ "notif_Inbound_RTP_253_165Hz.bin" },
	{ "notif_Light_RTP_254_165Hz.bin" },
	{ "notif_Meet_RTP_255_165Hz.bin" },
	{ "notif_Naivety_RTP_256_165Hz.bin" },
	{ "notif_Quickly_RTP_257_165Hz.bin" },
	{ "notif_Rhythm_RTP_258_165Hz.bin" },
	{ "notif_Surprise_RTP_259_165Hz.bin" },
	{ "notif_Twinkle_RTP_260_165Hz.bin" },
	{ "Version_Alert_RTP_261_165Hz.bin" },
	{ "alarm_Alarm_clock_RTP_262_165Hz.bin" },
	{ "alarm_Beep_RTP_263_165Hz.bin" },
	{ "alarm_Breeze_RTP_264_165Hz.bin" },
	{ "alarm_Dawn_RTP_265_165Hz.bin" },
	{ "alarm_Dream_RTP_266_165Hz.bin" },
	{ "alarm_Fluttering_RTP_267_165Hz.bin" },
	{ "alarm_Flyer_RTP_268_165Hz.bin" },
	{ "alarm_Interesting_RTP_269_165Hz.bin" },
	{ "alarm_Leisurely_RTP_270_165Hz.bin" },
	{ "alarm_Memory_RTP_271_165Hz.bin" },
	{ "alarm_Relieved_RTP_272_165Hz.bin" },
	{ "alarm_Ripple_RTP_273_165Hz.bin" },
	{ "alarm_Slowly_RTP_274_165Hz.bin" },
	{ "alarm_spring_RTP_275_165Hz.bin" },
	{ "alarm_Stars_RTP_276_165Hz.bin" },
	{ "alarm_Surging_RTP_277_165Hz.bin" },
	{ "alarm_tactfully_RTP_278_165Hz.bin" },
	{ "alarm_The_wind_RTP_279_165Hz.bin" },
	{ "alarm_Walking_in_the_rain_RTP_280_165Hz.bin" },
	{ "BoHaoPanAnJian_281_165Hz.bin" },
	{ "BoHaoPanAnNiu_282_165Hz.bin" },
	{ "BoHaoPanKuaiJie_283_165Hz.bin" },
	{ "DianHuaGuaDuan_284_165Hz.bin" },
	{ "DianJinMoShiQieHuan_285_165Hz.bin" },
	{ "HuaDongTiaoZhenDong_286_165Hz.bin" },
	{ "LeiShen_287_165Hz.bin" },
	{ "XuanZhongYouXi_288_165Hz.bin" },
	{ "YeJianMoShiDaZi_289_165Hz.bin" },
	{ "YouXiSheZhiKuang_290_165Hz.bin" },
	{ "ZhuanYeMoShi_291_165Hz.bin" },
	{ "Climber_RTP_292_165Hz.bin" },
	{ "Chase_RTP_293_165Hz.bin" },
	{ "shuntai24k_rtp_294_165Hz.bin" },
	{ "wentai24k_rtp_295_165Hz.bin" },
	{ "20ms_RTP_296_165Hz.bin" },
	{ "40ms_RTP_297_165Hz.bin" },
	{ "60ms_RTP_298_165Hz.bin" },
	{ "80ms_RTP_299_165Hz.bin" },
	{ "100ms_RTP_300_165Hz.bin" },
	{ "120ms_RTP_301_165Hz.bin" },
	{ "140ms_RTP_302_165Hz.bin" },
	{ "160ms_RTP_303_165Hz.bin" },
	{ "180ms_RTP_304_165Hz.bin" },
	{ "200ms_RTP_305_165Hz.bin" },
	{ "220ms_RTP_306_165Hz.bin" },
	{ "240ms_RTP_307_165Hz.bin" },
	{ "260ms_RTP_308_165Hz.bin" },
	{ "280ms_RTP_309_165Hz.bin" },
	{ "300ms_RTP_310_165Hz.bin" },
	{ "320ms_RTP_311_165Hz.bin" },
	{ "340ms_RTP_312_165Hz.bin" },
	{ "360ms_RTP_313_165Hz.bin" },
	{ "380ms_RTP_314_165Hz.bin" },
	{ "400ms_RTP_315_165Hz.bin" },
	{ "420ms_RTP_316_165Hz.bin" },
	{ "440ms_RTP_317_165Hz.bin" },
	{ "460ms_RTP_318_165Hz.bin" },
	{ "480ms_RTP_319_165Hz.bin" },
	{ "500ms_RTP_320_165Hz.bin" },
	{ "AT500ms_RTP_321_165Hz.bin" },
};
#endif /* OPLUS_FEATURE_CHG_BASIC */

static char aw86907_rtp_name[][AW86907_RTP_NAME_MAX] = {
	{ "aw8697_rtp.bin" },
#ifdef OPLUS_FEATURE_CHG_BASIC
	{ "aw8697_Hearty_channel_RTP_1.bin" },
	{ "aw8697_Instant_channel_RTP_2.bin" },
	{ "aw8697_Music_channel_RTP_3.bin" },
	{ "aw8697_Percussion_channel_RTP_4.bin" },
	{ "aw8697_Ripple_channel_RTP_5.bin" },
	{ "aw8697_Bright_channel_RTP_6.bin" },
	{ "aw8697_Fun_channel_RTP_7.bin" },
	{ "aw8697_Glittering_channel_RTP_8.bin" },
	{ "aw8697_Granules_channel_RTP_9.bin" },
	{ "aw8697_Harp_channel_RTP_10.bin" },
	{ "aw8697_Impression_channel_RTP_11.bin" },
	{ "aw8697_Ingenious_channel_RTP_12.bin" },
	{ "aw8697_Joy_channel_RTP_13.bin" },
	{ "aw8697_Overtone_channel_RTP_14.bin" },
	{ "aw8697_Receive_channel_RTP_15.bin" },
	{ "aw8697_Splash_channel_RTP_16.bin" },

	{ "aw8697_About_School_RTP_17.bin" },
	{ "aw8697_Bliss_RTP_18.bin" },
	{ "aw8697_Childhood_RTP_19.bin" },
	{ "aw8697_Commuting_RTP_20.bin" },
	{ "aw8697_Dream_RTP_21.bin" },
	{ "aw8697_Firefly_RTP_22.bin" },
	{ "aw8697_Gathering_RTP_23.bin" },
	{ "aw8697_Gaze_RTP_24.bin" },
	{ "aw8697_Lakeside_RTP_25.bin" },
	{ "aw8697_Lifestyle_RTP_26.bin" },
	{ "aw8697_Memories_RTP_27.bin" },
	{ "aw8697_Messy_RTP_28.bin" },
	{ "aw8697_Night_RTP_29.bin" },
	{ "aw8697_Passionate_Dance_RTP_30.bin" },
	{ "aw8697_Playground_RTP_31.bin" },
	{ "aw8697_Relax_RTP_32.bin" },
	{ "aw8697_Reminiscence_RTP_33.bin" },
	{ "aw8697_Silence_From_Afar_RTP_34.bin" },
	{ "aw8697_Silence_RTP_35.bin" },
	{ "aw8697_Stars_RTP_36.bin" },
	{ "aw8697_Summer_RTP_37.bin" },
	{ "aw8697_Toys_RTP_38.bin" },
	{ "aw8697_Travel_RTP_39.bin" },
	{ "aw8697_Vision_RTP_40.bin" },

	{ "aw8697_waltz_channel_RTP_41.bin" },
	{ "aw8697_cut_channel_RTP_42.bin" },
	{ "aw8697_clock_channel_RTP_43.bin" },
	{ "aw8697_long_sound_channel_RTP_44.bin" },
	{ "aw8697_short_channel_RTP_45.bin" },
	{ "aw8697_two_error_remaind_RTP_46.bin" },

	{ "aw8697_kill_program_RTP_47.bin" },
	{ "aw8697_Simple_channel_RTP_48.bin" },
	{ "aw8697_Pure_RTP_49.bin" },
	{ "aw8697_reserved_sound_channel_RTP_50.bin" },

	{ "aw8697_high_temp_high_humidity_channel_RTP_51.bin" },

	{ "aw8697_old_steady_test_RTP_52.bin" },
	{ "aw8697_listen_pop_53.bin" },
	{ "aw8697_desk_7_RTP_54.bin" },
	{ "aw8697_nfc_10_RTP_55.bin" },
	{ "aw8697_vibrator_remain_12_RTP_56.bin" },
	{ "aw8697_notice_13_RTP_57.bin" },
	{ "aw8697_third_ring_14_RTP_58.bin" },
	{ "aw8697_emergency_warning_RTP_59.bin" },

	{ "aw8697_honor_fisrt_kill_RTP_60.bin" },
	{ "aw8697_honor_two_kill_RTP_61.bin" },
	{ "aw8697_honor_three_kill_RTP_62.bin" },
	{ "aw8697_honor_four_kill_RTP_63.bin" },
	{ "aw8697_honor_five_kill_RTP_64.bin" },
	{ "aw8697_honor_three_continu_kill_RTP_65.bin" },
	{ "aw8697_honor_four_continu_kill_RTP_66.bin" },
	{ "aw8697_honor_unstoppable_RTP_67.bin" },
	{ "aw8697_honor_thousands_kill_RTP_68.bin" },
	{ "aw8697_honor_lengendary_RTP_69.bin" },

	{ "aw8697_Freshmorning_RTP_70_170Hz.bin" },
	{ "aw8697_Peaceful_RTP_71_170Hz.bin" },
	{ "aw8697_Cicada_RTP_72_170Hz.bin" },
	{ "aw8697_Electronica_RTP_73_170Hz.bin" },
	{ "aw8697_Holiday_RTP_74_170Hz.bin" },
	{ "aw8697_Funk_RTP_75_170Hz.bin" },
	{ "aw8697_House_RTP_76_170Hz.bin" },
	{ "aw8697_Temple_RTP_77_170Hz.bin" },
	{ "aw8697_Dreamyjazz_RTP_78_170Hz.bin" },
	{ "aw8697_Modern_RTP_79_170Hz.bin" },

	{ "aw8697_Round_RTP_80_170Hz.bin" },
	{ "aw8697_Rising_RTP_81_170Hz.bin" },
	{ "aw8697_Wood_RTP_82_170Hz.bin" },
	{ "aw8697_Heys_RTP_83_170Hz.bin" },
	{ "aw8697_Mbira_RTP_84_170Hz.bin" },
	{ "aw8697_News_RTP_85_170Hz.bin" },
	{ "aw8697_Peak_RTP_86_170Hz.bin" },
	{ "aw8697_Crisp_RTP_87_170Hz.bin" },
	{ "aw8697_Singingbowls_RTP_88_170Hz.bin" },
	{ "aw8697_Bounce_RTP_89_170Hz.bin" },

	{ "aw8697_reserved_90.bin" },
	{ "aw8697_reserved_91.bin" },
	{ "aw8697_reserved_92.bin" },
	{ "aw8697_reserved_93.bin" },
	{ "aw8697_ALCloudscape_94_170HZ.bin" },
	{ "aw8697_ALGoodenergy_95_170HZ.bin" },
	{ "aw8697_NTblink_96_170HZ.bin" },
	{ "aw8697_NTwhoop_97_170HZ.bin" },
	{ "aw8697_Newfeeling_98_170HZ.bin" },
	{ "aw8697_nature_99_170HZ.bin" },

	{ "aw8697_soldier_first_kill_RTP_100.bin" },
	{ "aw8697_soldier_second_kill_RTP_101.bin" },
	{ "aw8697_soldier_third_kill_RTP_102.bin" },
	{ "aw8697_soldier_fourth_kill_RTP_103.bin" },
	{ "aw8697_soldier_fifth_kill_RTP_104.bin" },
	{ "aw8697_stepable_regulate_RTP_105.bin" },
	{ "aw8697_voice_level_bar_edge_RTP_106.bin" },
	{ "aw8697_strength_level_bar_edge_RTP_107.bin" },
	{ "aw8697_charging_simulation_RTP_108.bin" },
	{ "aw8697_fingerprint_success_RTP_109.bin" },

	{ "aw8697_fingerprint_effect1_RTP_110.bin" },
	{ "aw8697_fingerprint_effect2_RTP_111.bin" },
	{ "aw8697_fingerprint_effect3_RTP_112.bin" },
	{ "aw8697_fingerprint_effect4_RTP_113.bin" },
	{ "aw8697_fingerprint_effect5_RTP_114.bin" },
	{ "aw8697_fingerprint_effect6_RTP_115.bin" },
	{ "aw8697_fingerprint_effect7_RTP_116.bin" },
	{ "aw8697_fingerprint_effect8_RTP_117.bin" },
	{ "aw8697_breath_simulation_RTP_118.bin" },
	{ "aw8697_reserved_119.bin" },

	{ "aw8697_Miss_RTP_120.bin" },
	{ "aw8697_Scenic_RTP_121.bin" },
	{ "aw8697_voice_assistant_RTP_122.bin" },
	/* used for 7 */
	{ "aw8697_Appear_channel_RTP_123.bin" },
	{ "aw8697_Miss_RTP_124.bin" },
	{ "aw8697_Music_channel_RTP_125.bin" },
	{ "aw8697_Percussion_channel_RTP_126.bin" },
	{ "aw8697_Ripple_channel_RTP_127.bin" },
	{ "aw8697_Bright_channel_RTP_128.bin" },
	{ "aw8697_Fun_channel_RTP_129.bin" },
	{ "aw8697_Glittering_channel_RTP_130.bin" },
	{ "aw8697_Harp_channel_RTP_131.bin" },
	{ "aw8697_Overtone_channel_RTP_132.bin" },
	{ "aw8697_Simple_channel_RTP_133.bin" },

	{ "aw8697_Seine_past_RTP_134.bin" },
	{ "aw8697_Classical_ring_RTP_135.bin" },
	{ "aw8697_Long_for_RTP_136.bin" },
	{ "aw8697_Romantic_RTP_137.bin" },
	{ "aw8697_Bliss_RTP_138.bin" },
	{ "aw8697_Dream_RTP_139.bin" },
	{ "aw8697_Relax_RTP_140.bin" },
	{ "aw8697_Joy_channel_RTP_141.bin" },
	{ "aw8697_weather_wind_RTP_142.bin" },
	{ "aw8697_weather_cloudy_RTP_143.bin" },
	{ "aw8697_weather_thunderstorm_RTP_144.bin" },
	{ "aw8697_weather_default_RTP_145.bin" },
	{ "aw8697_weather_sunny_RTP_146.bin" },
	{ "aw8697_weather_smog_RTP_147.bin" },
	{ "aw8697_weather_snow_RTP_148.bin" },
	{ "aw8697_weather_rain_RTP_149.bin" },

/* used for 7 end*/
#endif
	{ "aw8697_rtp_lighthouse.bin" },
	{ "aw8697_rtp_silk.bin" },
	{ "aw8697_reserved_152.bin" },
	{ "aw8697_reserved_153.bin" },
	{ "aw8697_reserved_154.bin" },
	{ "aw8697_reserved_155.bin" },
	{ "aw8697_reserved_156.bin" },
	{ "aw8697_reserved_157.bin" },
	{ "aw8697_reserved_158.bin" },
	{ "aw8697_reserved_159.bin" },
	{ "aw8697_reserved_160.bin" },

	/* oplus ringtone start */
	{ "aw8697_reserved_161.bin" },
	{ "aw8697_reserved_162.bin" },
	{ "aw8697_reserved_163.bin" },
	{ "aw8697_reserved_164.bin" },
	{ "aw8697_reserved_165.bin" },
	{ "aw8697_reserved_166.bin" },
	{ "aw8697_reserved_167.bin" },
	{ "aw8697_reserved_168.bin" },
	{ "aw8697_reserved_169.bin" },
	{ "aw8697_reserved_170.bin" },
	/* oplus ringtone end */
	{ "aw8697_reserved_171.bin" },
	{ "aw8697_reserved_172.bin" },
	{ "aw8697_reserved_173.bin" },
	{ "aw8697_reserved_174.bin" },
	{ "aw8697_reserved_175.bin" },
	{ "aw8697_reserved_176.bin" },
	{ "aw8697_reserved_177.bin" },
	{ "aw8697_reserved_178.bin" },
	{ "aw8697_reserved_179.bin" },
	{ "aw8697_reserved_180.bin" },
	{ "aw8697_reserved_181.bin" },
	{ "aw8697_reserved_182.bin" },
	{ "aw8697_reserved_183.bin" },
	{ "aw8697_reserved_184.bin" },
	{ "aw8697_reserved_185.bin" },
	{ "aw8697_reserved_186.bin" },
	{ "aw8697_reserved_187.bin" },
	{ "aw8697_reserved_188.bin" },
	{ "aw8697_reserved_189.bin" },
	{ "aw8697_reserved_190.bin" },
	{ "aw8697_reserved_191.bin" },
	{ "aw8697_reserved_192.bin" },
	{ "aw8697_reserved_193.bin" },
	{ "aw8697_reserved_194.bin" },
	{ "aw8697_reserved_195.bin" },
	{ "aw8697_reserved_196.bin" },
	{ "aw8697_reserved_197.bin" },
	{ "alarm_Pacman_RTP_198.bin" },
	{ "notif_Pacman_RTP_199.bin" },
	{ "ringtone_Pacman_RTP_200.bin" },
	{ "ringtone_Alacrity_RTP_201.bin" },
	{ "ring_Amenity_RTP_202.bin" },
	{ "ringtone_Blues_RTP_203.bin" },
	{ "ring_Bounce_RTP_204.bin" },
	{ "ring_Calm_RTP_205.bin" },
	{ "ringtone_Cloud_RTP_206.bin" },
	{ "ringtone_Cyclotron_RTP_207.bin" },
	{ "ringtone_Distinct_RTP_208.bin" },
	{ "ringtone_Dynamic_RTP_209.bin" },
	{ "ringtone_Echo_RTP_210.bin" },
	{ "ringtone_Expect_RTP_211.bin" },
	{ "ringtone_Fanatical_RTP_212.bin" },
	{ "ringtone_Funky_RTP_213.bin" },
	{ "ringtone_Guitar_RTP_214.bin" },
	{ "ringtone_Harping_RTP_215.bin" },
	{ "ringtone_Highlight_RTP_216.bin" },
	{ "ringtone_Idyl_RTP_217.bin" },
	{ "ringtone_Innocence_RTP_218.bin" },
	{ "ringtone_Journey_RTP_219.bin" },
	{ "ringtone_Joyous_RTP_220.bin" },
	{ "ring_Lazy_RTP_221.bin" },
	{ "ringtone_Marimba_RTP_222.bin" },
	{ "ring_Mystical_RTP_223.bin" },
	{ "ringtone_Old_telephone_RTP_224.bin" },
	{ "ringtone_Oneplus_tune_RTP_225.bin" },
	{ "ringtone_Rhythm_RTP_226.bin" },
	{ "ringtone_Optimistic_RTP_227.bin" },
	{ "ringtone_Piano_RTP_228.bin" },
	{ "ring_Whirl_RTP_229.bin" },
	{ "VZW_Alrwave_RTP_230.bin" },
	{ "t-jingle_RTP_231.bin" },
	{ "ringtone_Eager_232.bin" },
	{ "ringtone_Ebullition_233.bin" },
	{ "ringtone_Friendship_234.bin" },
	{ "ringtone_Jazz_life_RTP_235.bin" },
	{ "ringtone_Sun_glittering_RTP_236.bin" },
	{ "notif_Allay_RTP_237.bin" },
	{ "notif_Allusion_RTP_238.bin" },
	{ "notif_Amiable_RTP_239.bin" },
	{ "notif_Blare_RTP_240.bin" },
	{ "notif_Blissful_RTP_241.bin" },
	{ "notif_Brisk_RTP_242.bin" },
	{ "notif_Bubble_RTP_243.bin" },
	{ "notif_Cheerful_RTP_244.bin" },
	{ "notif_Clear_RTP_245.bin" },
	{ "notif_Comely_RTP_246.bin" },
	{ "notif_Cozy_RTP_247.bin" },
	{ "notif_Ding_RTP_248.bin" },
	{ "notif_Effervesce_RTP_249.bin" },
	{ "notif_Elegant_RTP_250.bin" },
	{ "notif_Free_RTP_251.bin" },
	{ "notif_Hallucination_RTP_252.bin" },
	{ "notif_Inbound_RTP_253.bin" },
	{ "notif_Light_RTP_254.bin" },
	{ "notif_Meet_RTP_255.bin" },
	{ "notif_Naivety_RTP_256.bin" },
	{ "notif_Quickly_RTP_257.bin" },
	{ "notif_Rhythm_RTP_258.bin" },
	{ "notif_Surprise_RTP_259.bin" },
	{ "notif_Twinkle_RTP_260.bin" },
	{ "Version_Alert_RTP_261.bin" },
	{ "alarm_Alarm_clock_RTP_262.bin" },
	{ "alarm_Beep_RTP_263.bin" },
	{ "alarm_Breeze_RTP_264.bin" },
	{ "alarm_Dawn_RTP_265.bin" },
	{ "alarm_Dream_RTP_266.bin" },
	{ "alarm_Fluttering_RTP_267.bin" },
	{ "alarm_Flyer_RTP_268.bin" },
	{ "alarm_Interesting_RTP_269.bin" },
	{ "alarm_Leisurely_RTP_270.bin" },
	{ "alarm_Memory_RTP_271.bin" },
	{ "alarm_Relieved_RTP_272.bin" },
	{ "alarm_Ripple_RTP_273.bin" },
	{ "alarm_Slowly_RTP_274.bin" },
	{ "alarm_spring_RTP_275.bin" },
	{ "alarm_Stars_RTP_276.bin" },
	{ "alarm_Surging_RTP_277.bin" },
	{ "alarm_tactfully_RTP_278.bin" },
	{ "alarm_The_wind_RTP_279.bin" },
	{ "alarm_Walking_in_the_rain_RTP_280.bin" },
	{ "BoHaoPanAnJian_281.bin" },
	{ "BoHaoPanAnNiu_282.bin" },
	{ "BoHaoPanKuaiJie_283.bin" },
	{ "DianHuaGuaDuan_284.bin" },
	{ "DianJinMoShiQieHuan_285.bin" },
	{ "HuaDongTiaoZhenDong_286.bin" },
	{ "LeiShen_287.bin" },
	{ "XuanZhongYouXi_288.bin" },
	{ "YeJianMoShiDaZi_289.bin" },
	{ "YouXiSheZhiKuang_290.bin" },
	{ "ZhuanYeMoShi_291.bin" },
	{ "Climber_RTP_292.bin" },
	{ "Chase_RTP_293.bin" },
	{ "shuntai24k_rtp_294.bin" },
	{ "wentai24k_rtp_295.bin" },
	{ "20ms_RTP_296.bin" },
	{ "40ms_RTP_297.bin" },
	{ "60ms_RTP_298.bin" },
	{ "80ms_RTP_299.bin" },
	{ "100ms_RTP_300.bin" },
	{ "120ms_RTP_301.bin" },
	{ "140ms_RTP_302.bin" },
	{ "160ms_RTP_303.bin" },
	{ "180ms_RTP_304.bin" },
	{ "200ms_RTP_305.bin" },
	{ "220ms_RTP_306.bin" },
	{ "240ms_RTP_307.bin" },
	{ "260ms_RTP_308.bin" },
	{ "280ms_RTP_309.bin" },
	{ "300ms_RTP_310.bin" },
	{ "320ms_RTP_311.bin" },
	{ "340ms_RTP_312.bin" },
	{ "360ms_RTP_313.bin" },
	{ "380ms_RTP_314.bin" },
	{ "400ms_RTP_315.bin" },
	{ "420ms_RTP_316.bin" },
	{ "440ms_RTP_317.bin" },
	{ "460ms_RTP_318.bin" },
	{ "480ms_RTP_319.bin" },
	{ "500ms_RTP_320.bin" },
	{ "AT500ms_RTP_321.bin" },
};

#ifdef OPLUS_FEATURE_CHG_BASIC
static char aw86907_rtp_name_175Hz[][AW86907_RTP_NAME_MAX] = {
	{ "aw8697_rtp.bin" },
	{ "aw8697_Hearty_channel_RTP_1.bin" },
	{ "aw8697_Instant_channel_RTP_2_175Hz.bin" },
	{ "aw8697_Music_channel_RTP_3.bin" },
	{ "aw8697_Percussion_channel_RTP_4.bin" },
	{ "aw8697_Ripple_channel_RTP_5.bin" },
	{ "aw8697_Bright_channel_RTP_6.bin" },
	{ "aw8697_Fun_channel_RTP_7.bin" },
	{ "aw8697_Glittering_channel_RTP_8.bin" },
	{ "aw8697_Granules_channel_RTP_9_175Hz.bin" },
	{ "aw8697_Harp_channel_RTP_10.bin" },
	{ "aw8697_Impression_channel_RTP_11.bin" },
	{ "aw8697_Ingenious_channel_RTP_12_175Hz.bin" },
	{ "aw8697_Joy_channel_RTP_13_175Hz.bin" },
	{ "aw8697_Overtone_channel_RTP_14.bin" },
	{ "aw8697_Receive_channel_RTP_15_175Hz.bin" },
	{ "aw8697_Splash_channel_RTP_16_175Hz.bin" },

	{ "aw8697_About_School_RTP_17_175Hz.bin" },
	{ "aw8697_Bliss_RTP_18.bin" },
	{ "aw8697_Childhood_RTP_19_175Hz.bin" },
	{ "aw8697_Commuting_RTP_20_175Hz.bin" },
	{ "aw8697_Dream_RTP_21.bin" },
	{ "aw8697_Firefly_RTP_22_175Hz.bin" },
	{ "aw8697_Gathering_RTP_23.bin" },
	{ "aw8697_Gaze_RTP_24_175Hz.bin" },
	{ "aw8697_Lakeside_RTP_25_175Hz.bin" },
	{ "aw8697_Lifestyle_RTP_26.bin" },
	{ "aw8697_Memories_RTP_27_175Hz.bin" },
	{ "aw8697_Messy_RTP_28_175Hz.bin" },
	{ "aw8697_Night_RTP_29_175Hz.bin" },
	{ "aw8697_Passionate_Dance_RTP_30_175Hz.bin" },
	{ "aw8697_Playground_RTP_31_175Hz.bin" },
	{ "aw8697_Relax_RTP_32_175Hz.bin" },
	{ "aw8697_Reminiscence_RTP_33.bin" },
	{ "aw8697_Silence_From_Afar_RTP_34_175Hz.bin" },
	{ "aw8697_Silence_RTP_35_175Hz.bin" },
	{ "aw8697_Stars_RTP_36_175Hz.bin" },
	{ "aw8697_Summer_RTP_37_175Hz.bin" },
	{ "aw8697_Toys_RTP_38_175Hz.bin" },
	{ "aw8697_Travel_RTP_39.bin" },
	{ "aw8697_Vision_RTP_40.bin" },

	{ "aw8697_waltz_channel_RTP_41_175Hz.bin" },
	{ "aw8697_cut_channel_RTP_42_175Hz.bin" },
	{ "aw8697_clock_channel_RTP_43_175Hz.bin" },
	{ "aw8697_long_sound_channel_RTP_44_175Hz.bin" },
	{ "aw8697_short_channel_RTP_45_175Hz.bin" },
	{ "aw8697_two_error_remaind_RTP_46_175Hz.bin" },

	{ "aw8697_kill_program_RTP_47_175Hz.bin" },
	{ "aw8697_Simple_channel_RTP_48.bin" },
	{ "aw8697_Pure_RTP_49_175Hz.bin" },
	{ "aw8697_reserved_sound_channel_RTP_50.bin" },

	{ "aw8697_high_temp_high_humidity_channel_RTP_51.bin" },

	{ "aw8697_old_steady_test_RTP_52.bin" },
	{ "aw8697_listen_pop_53.bin" },
	{ "aw8697_desk_7_RTP_54_175Hz.bin" },
	{ "aw8697_nfc_10_RTP_55_175Hz.bin" },
	{ "aw8697_vibrator_remain_12_RTP_56_175Hz.bin" },
	{ "aw8697_notice_13_RTP_57.bin" },
	{ "aw8697_third_ring_14_RTP_58.bin" },
	{ "aw8697_emergency_warning_RTP_59_175Hz.bin" },

	{ "aw8697_honor_fisrt_kill_RTP_60_175Hz.bin" },
	{ "aw8697_honor_two_kill_RTP_61_175Hz.bin" },
	{ "aw8697_honor_three_kill_RTP_62_175Hz.bin" },
	{ "aw8697_honor_four_kill_RTP_63_175Hz.bin" },
	{ "aw8697_honor_five_kill_RTP_64_175Hz.bin" },
	{ "aw8697_honor_three_continu_kill_RTP_65_175Hz.bin" },
	{ "aw8697_honor_four_continu_kill_RTP_66_175Hz.bin" },
	{ "aw8697_honor_unstoppable_RTP_67_175Hz.bin" },
	{ "aw8697_honor_thousands_kill_RTP_68_175Hz.bin" },
	{ "aw8697_honor_lengendary_RTP_69_175Hz.bin" },

	{ "aw8697_Freshmorning_RTP_70_175Hz.bin" },
	{ "aw8697_Peaceful_RTP_71_175Hz.bin" },
	{ "aw8697_Cicada_RTP_72_175Hz.bin" },
	{ "aw8697_Electronica_RTP_73_175Hz.bin" },
	{ "aw8697_Holiday_RTP_74_175Hz.bin" },
	{ "aw8697_Funk_RTP_75_175Hz.bin" },
	{ "aw8697_House_RTP_76_175Hz.bin" },
	{ "aw8697_Temple_RTP_77_175Hz.bin" },
	{ "aw8697_Dreamyjazz_RTP_78_175Hz.bin" },
	{ "aw8697_Modern_RTP_79_175Hz.bin" },

	{ "aw8697_Round_RTP_80_175Hz.bin" },
	{ "aw8697_Rising_RTP_81_175Hz.bin" },
	{ "aw8697_Wood_RTP_82_175Hz.bin" },
	{ "aw8697_Heys_RTP_83_175Hz.bin" },
	{ "aw8697_Mbira_RTP_84_175Hz.bin" },
	{ "aw8697_News_RTP_85_175Hz.bin" },
	{ "aw8697_Peak_RTP_86_175Hz.bin" },
	{ "aw8697_Crisp_RTP_87_175Hz.bin" },
	{ "aw8697_Singingbowls_RTP_88_175Hz.bin" },
	{ "aw8697_Bounce_RTP_89_175Hz.bin" },

	{ "aw8697_reserved_90.bin" },
	{ "aw8697_reserved_91.bin" },
	{ "aw8697_reserved_92.bin" },
	{ "aw8697_reserved_93.bin" },
	{ "aw8697_ALCloudscape_94_175HZ.bin" },
	{ "aw8697_ALGoodenergy_95_175HZ.bin" },
	{ "aw8697_NTblink_96_175HZ.bin" },
	{ "aw8697_NTwhoop_97_175HZ.bin" },
	{ "aw8697_Newfeeling_98_175HZ.bin" },
	{ "aw8697_nature_99_175HZ.bin" },

	{ "aw8697_soldier_first_kill_RTP_100_175Hz.bin" },
	{ "aw8697_soldier_second_kill_RTP_101_175Hz.bin" },
	{ "aw8697_soldier_third_kill_RTP_102_175Hz.bin" },
	{ "aw8697_soldier_fourth_kill_RTP_103_175Hz.bin" },
	{ "aw8697_soldier_fifth_kill_RTP_104_175Hz.bin" },
	{ "aw8697_stepable_regulate_RTP_105_175Hz.bin" },
	{ "aw8697_voice_level_bar_edge_RTP_106_175Hz.bin" },
	{ "aw8697_strength_level_bar_edge_RTP_107_175Hz.bin" },
	{ "aw8697_charging_simulation_RTP_108_175Hz.bin" },
	{ "aw8697_fingerprint_success_RTP_109_175Hz.bin" },

	{ "aw8697_fingerprint_effect1_RTP_110_175Hz.bin" },
	{ "aw8697_fingerprint_effect2_RTP_111_175Hz.bin" },
	{ "aw8697_fingerprint_effect3_RTP_112_175Hz.bin" },
	{ "aw8697_fingerprint_effect4_RTP_113_175Hz.bin" },
	{ "aw8697_fingerprint_effect5_RTP_114_175Hz.bin" },
	{ "aw8697_fingerprint_effect6_RTP_115_175Hz.bin" },
	{ "aw8697_fingerprint_effect7_RTP_116_175Hz.bin" },
	{ "aw8697_fingerprint_effect8_RTP_117_175Hz.bin" },
	{ "aw8697_breath_simulation_RTP_118_175Hz.bin" },
	{ "aw8697_reserved_119.bin" },

	{ "aw8697_Miss_RTP_120.bin" },
	{ "aw8697_Scenic_RTP_121_175Hz.bin" },
	{ "aw8697_voice_assistant_RTP_122_175Hz.bin" },
	/* used for 7 */
	{ "aw8697_Appear_channel_RTP_123_175Hz.bin" },
	{ "aw8697_Miss_RTP_124_175Hz.bin" },
	{ "aw8697_Music_channel_RTP_125_175Hz.bin" },
	{ "aw8697_Percussion_channel_RTP_126_175Hz.bin" },
	{ "aw8697_Ripple_channel_RTP_127_175Hz.bin" },
	{ "aw8697_Bright_channel_RTP_128_175Hz.bin" },
	{ "aw8697_Fun_channel_RTP_129_175Hz.bin" },
	{ "aw8697_Glittering_channel_RTP_130_175Hz.bin" },
	{ "aw8697_Harp_channel_RTP_131_175Hz.bin" },
	{ "aw8697_Overtone_channel_RTP_132_175Hz.bin" },
	{ "aw8697_Simple_channel_RTP_133_175Hz.bin" },

	{ "aw8697_Seine_past_RTP_134_175Hz.bin" },
	{ "aw8697_Classical_ring_RTP_135_175Hz.bin" },
	{ "aw8697_Long_for_RTP_136_175Hz.bin" },
	{ "aw8697_Romantic_RTP_137_175Hz.bin" },
	{ "aw8697_Bliss_RTP_138_175Hz.bin" },
	{ "aw8697_Dream_RTP_139_175Hz.bin" },
	{ "aw8697_Relax_RTP_140_175Hz.bin" },
	{ "aw8697_Joy_channel_RTP_141_175Hz.bin" },
	{ "aw8697_weather_wind_RTP_142_175Hz.bin" },
	{ "aw8697_weather_cloudy_RTP_143_175Hz.bin" },
	{ "aw8697_weather_thunderstorm_RTP_144_175Hz.bin" },
	{ "aw8697_weather_default_RTP_145_175Hz.bin" },
	{ "aw8697_weather_sunny_RTP_146_175Hz.bin" },
	{ "aw8697_weather_smog_RTP_147_175Hz.bin" },
	{ "aw8697_weather_snow_RTP_148_175Hz.bin" },
	{ "aw8697_weather_rain_RTP_149_175Hz.bin" },
	/* used for 7 end*/
	{ "aw8697_rtp_lighthouse.bin" },
	{ "aw8697_rtp_silk.bin" },
	{ "aw8697_reserved_152.bin" },
	{ "aw8697_reserved_153.bin" },
	{ "aw8697_reserved_154.bin" },
	{ "aw8697_reserved_155.bin" },
	{ "aw8697_reserved_156.bin" },
	{ "aw8697_reserved_157.bin" },
	{ "aw8697_reserved_158.bin" },
	{ "aw8697_reserved_159.bin" },
	{ "aw8697_reserved_160.bin" },
	{ "aw8697_reserved_161.bin" },
	{ "aw8697_reserved_162.bin" },
	{ "aw8697_reserved_163.bin" },
	{ "aw8697_reserved_164.bin" },
	{ "aw8697_reserved_165.bin" },
	{ "aw8697_reserved_166.bin" },
	{ "aw8697_reserved_167.bin" },
	{ "aw8697_reserved_168.bin" },
	{ "aw8697_reserved_169.bin" },
	{ "aw8697_reserved_170.bin" },
	{ "aw8697_reserved_171.bin" },
	{ "aw8697_reserved_172.bin" },
	{ "aw8697_reserved_173.bin" },
	{ "aw8697_reserved_174.bin" },
	{ "aw8697_reserved_175.bin" },
	{ "aw8697_reserved_176.bin" },
	{ "aw8697_reserved_177.bin" },
	{ "aw8697_reserved_178.bin" },
	{ "aw8697_reserved_179.bin" },
	{ "aw8697_reserved_180.bin" },
	{ "aw8697_reserved_181.bin" },
	{ "aw8697_reserved_182.bin" },
	{ "aw8697_reserved_183.bin" },
	{ "aw8697_reserved_184.bin" },
	{ "aw8697_reserved_185.bin" },
	{ "aw8697_reserved_186.bin" },
	{ "aw8697_reserved_187.bin" },
	{ "aw8697_reserved_188.bin" },
	{ "aw8697_reserved_189.bin" },
	{ "aw8697_reserved_190.bin" },
	{ "aw8697_reserved_191.bin" },
	{ "aw8697_reserved_192.bin" },
	{ "aw8697_reserved_193.bin" },
	{ "aw8697_reserved_194.bin" },
	{ "aw8697_reserved_195.bin" },
	{ "aw8697_reserved_196.bin" },
	{ "aw8697_reserved_197.bin" },
	{ "alarm_Pacman_RTP_198_175Hz.bin" },
	{ "notif_Pacman_RTP_199_175Hz.bin" },
	{ "ringtone_Pacman_RTP_200_175Hz.bin" },
	{ "ringtone_Alacrity_RTP_201_175Hz.bin" },
	{ "ring_Amenity_RTP_202_175Hz.bin" },
	{ "ringtone_Blues_RTP_203_175Hz.bin" },
	{ "ring_Bounce_RTP_204_175Hz.bin" },
	{ "ring_Calm_RTP_205_175Hz.bin" },
	{ "ringtone_Cloud_RTP_206_175Hz.bin" },
	{ "ringtone_Cyclotron_RTP_207_175Hz.bin" },
	{ "ringtone_Distinct_RTP_208_175Hz.bin" },
	{ "ringtone_Dynamic_RTP_209_175Hz.bin" },
	{ "ringtone_Echo_RTP_210_175Hz.bin" },
	{ "ringtone_Expect_RTP_211_175Hz.bin" },
	{ "ringtone_Fanatical_RTP_212_175Hz.bin" },
	{ "ringtone_Funky_RTP_213_175Hz.bin" },
	{ "ringtone_Guitar_RTP_214_175Hz.bin" },
	{ "ringtone_Harping_RTP_215_175Hz.bin" },
	{ "ringtone_Highlight_RTP_216_175Hz.bin" },
	{ "ringtone_Idyl_RTP_217_175Hz.bin" },
	{ "ringtone_Innocence_RTP_218_175Hz.bin" },
	{ "ringtone_Journey_RTP_219_175Hz.bin" },
	{ "ringtone_Joyous_RTP_220_175Hz.bin" },
	{ "ring_Lazy_RTP_221_175Hz.bin" },
	{ "ringtone_Marimba_RTP_222_175Hz.bin" },
	{ "ring_Mystical_RTP_223_175Hz.bin" },
	{ "ringtone_Old_telephone_RTP_224_175Hz.bin" },
	{ "ringtone_Oneplus_tune_RTP_225_175Hz.bin" },
	{ "ringtone_Rhythm_RTP_226_175Hz.bin" },
	{ "ringtone_Optimistic_RTP_227_175Hz.bin" },
	{ "ringtone_Piano_RTP_228_175Hz.bin" },
	{ "ring_Whirl_RTP_229_175Hz.bin" },
	{ "VZW_Alrwave_RTP_230_175Hz.bin" },
	{ "t-jingle_RTP_231_175Hz.bin" },
	{ "ringtone_Eager_232_175Hz.bin" },
	{ "ringtone_Ebullition_233_175Hz.bin" },
	{ "ringtone_Friendship_234_175Hz.bin" },
	{ "ringtone_Jazz_life_RTP_235_175Hz.bin" },
	{ "ringtone_Sun_glittering_RTP_236_175Hz.bin" },
	{ "notif_Allay_RTP_237_175Hz.bin" },
	{ "notif_Allusion_RTP_238_175Hz.bin" },
	{ "notif_Amiable_RTP_239_175Hz.bin" },
	{ "notif_Blare_RTP_240_175Hz.bin" },
	{ "notif_Blissful_RTP_241_175Hz.bin" },
	{ "notif_Brisk_RTP_242_175Hz.bin" },
	{ "notif_Bubble_RTP_243_175Hz.bin" },
	{ "notif_Cheerful_RTP_244_175Hz.bin" },
	{ "notif_Clear_RTP_245_175Hz.bin" },
	{ "notif_Comely_RTP_246_175Hz.bin" },
	{ "notif_Cozy_RTP_247_175Hz.bin" },
	{ "notif_Ding_RTP_248_175Hz.bin" },
	{ "notif_Effervesce_RTP_249_175Hz.bin" },
	{ "notif_Elegant_RTP_250_175Hz.bin" },
	{ "notif_Free_RTP_251_175Hz.bin" },
	{ "notif_Hallucination_RTP_252_175Hz.bin" },
	{ "notif_Inbound_RTP_253_175Hz.bin" },
	{ "notif_Light_RTP_254_175Hz.bin" },
	{ "notif_Meet_RTP_255_175Hz.bin" },
	{ "notif_Naivety_RTP_256_175Hz.bin" },
	{ "notif_Quickly_RTP_257_175Hz.bin" },
	{ "notif_Rhythm_RTP_258_175Hz.bin" },
	{ "notif_Surprise_RTP_259_175Hz.bin" },
	{ "notif_Twinkle_RTP_260_175Hz.bin" },
	{ "Version_Alert_RTP_261_175Hz.bin" },
	{ "alarm_Alarm_clock_RTP_262_175Hz.bin" },
	{ "alarm_Beep_RTP_263_175Hz.bin" },
	{ "alarm_Breeze_RTP_264_175Hz.bin" },
	{ "alarm_Dawn_RTP_265_175Hz.bin" },
	{ "alarm_Dream_RTP_266_175Hz.bin" },
	{ "alarm_Fluttering_RTP_267_175Hz.bin" },
	{ "alarm_Flyer_RTP_268_175Hz.bin" },
	{ "alarm_Interesting_RTP_269_175Hz.bin" },
	{ "alarm_Leisurely_RTP_270_175Hz.bin" },
	{ "alarm_Memory_RTP_271_175Hz.bin" },
	{ "alarm_Relieved_RTP_272_175Hz.bin" },
	{ "alarm_Ripple_RTP_273_175Hz.bin" },
	{ "alarm_Slowly_RTP_274_175Hz.bin" },
	{ "alarm_spring_RTP_275_175Hz.bin" },
	{ "alarm_Stars_RTP_276_175Hz.bin" },
	{ "alarm_Surging_RTP_277_175Hz.bin" },
	{ "alarm_tactfully_RTP_278_175Hz.bin" },
	{ "alarm_The_wind_RTP_279_175Hz.bin" },
	{ "alarm_Walking_in_the_rain_RTP_280_175Hz.bin" },
	{ "BoHaoPanAnJian_281_175Hz.bin" },
	{ "BoHaoPanAnNiu_282_175Hz.bin" },
	{ "BoHaoPanKuaiJie_283_175Hz.bin" },
	{ "DianHuaGuaDuan_284_175Hz.bin" },
	{ "DianJinMoShiQieHuan_285_175Hz.bin" },
	{ "HuaDongTiaoZhenDong_286_175Hz.bin" },
	{ "LeiShen_287_175Hz.bin" },
	{ "XuanZhongYouXi_288_175Hz.bin" },
	{ "YeJianMoShiDaZi_289_175Hz.bin" },
	{ "YouXiSheZhiKuang_290_175Hz.bin" },
	{ "ZhuanYeMoShi_291_175Hz.bin" },
	{ "Climber_RTP_292_175Hz.bin" },
	{ "Chase_RTP_293_175Hz.bin" },
	{ "shuntai24k_rtp_294_175Hz.bin" },
	{ "wentai24k_rtp_295_175Hz.bin" },
	{ "20ms_RTP_296_175Hz.bin" },
	{ "40ms_RTP_297_175Hz.bin" },
	{ "60ms_RTP_298_175Hz.bin" },
	{ "80ms_RTP_299_175Hz.bin" },
	{ "100ms_RTP_300_175Hz.bin" },
	{ "120ms_RTP_301_175Hz.bin" },
	{ "140ms_RTP_302_175Hz.bin" },
	{ "160ms_RTP_303_175Hz.bin" },
	{ "180ms_RTP_304_175Hz.bin" },
	{ "200ms_RTP_305_175Hz.bin" },
	{ "220ms_RTP_306_175Hz.bin" },
	{ "240ms_RTP_307_175Hz.bin" },
	{ "260ms_RTP_308_175Hz.bin" },
	{ "280ms_RTP_309_175Hz.bin" },
	{ "300ms_RTP_310_175Hz.bin" },
	{ "320ms_RTP_311_175Hz.bin" },
	{ "340ms_RTP_312_175Hz.bin" },
	{ "360ms_RTP_313_175Hz.bin" },
	{ "380ms_RTP_314_175Hz.bin" },
	{ "400ms_RTP_315_175Hz.bin" },
	{ "420ms_RTP_316_175Hz.bin" },
	{ "440ms_RTP_317_175Hz.bin" },
	{ "460ms_RTP_318_175Hz.bin" },
	{ "480ms_RTP_319_175Hz.bin" },
	{ "500ms_RTP_320_175Hz.bin" },
	{ "AT500ms_RTP_321_175Hz.bin" },
};
#endif /* OPLUS_FEATURE_CHG_BASIC */

static char aw86907_ram_name_19065[5][30] = {
	{ "aw8697_haptic_235.bin" }, { "aw8697_haptic_235.bin" },
	{ "aw8697_haptic_235.bin" }, { "aw8697_haptic_235.bin" },
	{ "aw8697_haptic_235.bin" },
};

static char aw86907_ram_name_19161[5][30] = {
	{ "aw8697_haptic_235_19161.bin" }, { "aw8697_haptic_235_19161.bin" },
	{ "aw8697_haptic_235_19161.bin" }, { "aw8697_haptic_235_19161.bin" },
	{ "aw8697_haptic_235_19161.bin" },
};

#ifdef OPLUS_FEATURE_CHG_BASIC
static char aw86907_rtp_name_19065_226Hz[][AW86907_RTP_NAME_MAX] = {
	{ "aw8697_rtp.bin" },
	{ "aw8697_Hearty_channel_RTP_1.bin" },
	{ "aw8697_Instant_channel_RTP_2_226Hz.bin" },
	{ "aw8697_Music_channel_RTP_3.bin" },
	{ "aw8697_Percussion_channel_RTP_4.bin" },
	{ "aw8697_Ripple_channel_RTP_5.bin" },
	{ "aw8697_Bright_channel_RTP_6.bin" },
	{ "aw8697_Fun_channel_RTP_7.bin" },
	{ "aw8697_Glittering_channel_RTP_8.bin" },
	{ "aw8697_Granules_channel_RTP_9_226Hz.bin" },
	{ "aw8697_Harp_channel_RTP_10.bin" },
	{ "aw8697_Impression_channel_RTP_11.bin" },
	{ "aw8697_Ingenious_channel_RTP_12_226Hz.bin" },
	{ "aw8697_Joy_channel_RTP_13_226Hz.bin" },
	{ "aw8697_Overtone_channel_RTP_14.bin" },
	{ "aw8697_Receive_channel_RTP_15_226Hz.bin" },
	{ "aw8697_Splash_channel_RTP_16_226Hz.bin" },

	{ "aw8697_About_School_RTP_17_226Hz.bin" },
	{ "aw8697_Bliss_RTP_18.bin" },
	{ "aw8697_Childhood_RTP_19_226Hz.bin" },
	{ "aw8697_Commuting_RTP_20_226Hz.bin" },
	{ "aw8697_Dream_RTP_21.bin" },
	{ "aw8697_Firefly_RTP_22_226Hz.bin" },
	{ "aw8697_Gathering_RTP_23.bin" },
	{ "aw8697_Gaze_RTP_24_226Hz.bin" },
	{ "aw8697_Lakeside_RTP_25_226Hz.bin" },
	{ "aw8697_Lifestyle_RTP_26.bin" },
	{ "aw8697_Memories_RTP_27_226Hz.bin" },
	{ "aw8697_Messy_RTP_28_226Hz.bin" },
	{ "aw8697_Night_RTP_29_226Hz.bin" },
	{ "aw8697_Passionate_Dance_RTP_30_226Hz.bin" },
	{ "aw8697_Playground_RTP_31_226Hz.bin" },
	{ "aw8697_Relax_RTP_32_226Hz.bin" },
	{ "aw8697_Reminiscence_RTP_33.bin" },
	{ "aw8697_Silence_From_Afar_RTP_34_226Hz.bin" },
	{ "aw8697_Silence_RTP_35_226Hz.bin" },
	{ "aw8697_Stars_RTP_36_226Hz.bin" },
	{ "aw8697_Summer_RTP_37_226Hz.bin" },
	{ "aw8697_Toys_RTP_38_226Hz.bin" },
	{ "aw8697_Travel_RTP_39.bin" },
	{ "aw8697_Vision_RTP_40.bin" },

	{ "aw8697_waltz_channel_RTP_41_226Hz.bin" },
	{ "aw8697_cut_channel_RTP_42_226Hz.bin" },
	{ "aw8697_clock_channel_RTP_43_226Hz.bin" },
	{ "aw8697_long_sound_channel_RTP_44_226Hz.bin" },
	{ "aw8697_short_channel_RTP_45_226Hz.bin" },
	{ "aw8697_two_error_remaind_RTP_46_226Hz.bin" },

	{ "aw8697_kill_program_RTP_47_226Hz.bin" },
	{ "aw8697_Simple_channel_RTP_48.bin" },
	{ "aw8697_Pure_RTP_49_226Hz.bin" },
	{ "aw8697_reserved_sound_channel_RTP_50.bin" },

	{ "aw8697_high_temp_high_humidity_channel_RTP_51.bin" },

	{ "aw8697_old_steady_test_RTP_52.bin" },
	{ "aw8697_listen_pop_53_235Hz.bin" },
	{ "aw8697_desk_7_RTP_54_226Hz.bin" },
	{ "aw8697_nfc_10_RTP_55_226Hz.bin" },
	{ "aw8697_vibrator_remain_12_RTP_56_226Hz.bin" },
	{ "aw8697_notice_13_RTP_57.bin" },
	{ "aw8697_third_ring_14_RTP_58.bin" },
	{ "aw8697_emergency_warning_RTP_59_226Hz.bin" },

	{ "aw8697_honor_fisrt_kill_RTP_60_226Hz.bin" },
	{ "aw8697_honor_two_kill_RTP_61_226Hz.bin" },
	{ "aw8697_honor_three_kill_RTP_62_226Hz.bin" },
	{ "aw8697_honor_four_kill_RTP_63_226Hz.bin" },
	{ "aw8697_honor_five_kill_RTP_64_226Hz.bin" },
	{ "aw8697_honor_three_continu_kill_RTP_65_226Hz.bin" },
	{ "aw8697_honor_four_continu_kill_RTP_66_226Hz.bin" },
	{ "aw8697_honor_unstoppable_RTP_67_226Hz.bin" },
	{ "aw8697_honor_thousands_kill_RTP_68_226Hz.bin" },
	{ "aw8697_honor_lengendary_RTP_69_226Hz.bin" },
	{ "aw8697_Airy_morning_RTP_70_226Hz.bin" },
	{ "aw8697_Temple_morning_RTP_71_226Hz.bin" },
	{ "aw8697_Water_cicidas_RTP_72_226Hz.bin" },
	{ "aw8697_Electro_club_RTP_73_226Hz.bin" },
	{ "aw8697_Vacation_RTP_74_226Hz.bin" },
	{ "aw8697_Jazz_funk_RTP_75_226Hz.bin" },
	{ "aw8697_House_club_RTP_76_226Hz.bin" },
	{ "aw8697_temple_tone_RTP_77_226Hz.bin" },
	{ "aw8697_Jazz_dreamy_RTP_78_226Hz.bin" },
	{ "aw8697_Jazz_modern_RTP_79_226Hz.bin" },
	{ "aw8697_Tone_round_RTP_80_226Hz.bin" },
	{ "aw8697_Digi_rise_RTP_81_226Hz.bin" },
	{ "aw8697_Wood_phone_RTP_82_226Hz.bin" },
	{ "aw8697_Hey_RTP_83_226Hz.bin" },
	{ "aw8697_Zanza_RTP_84_226Hz.bin" },
	{ "aw8697_Info_RTP_85_226Hz.bin" },
	{ "aw8697_Tip_top_RTP_86_226Hz.bin" },
	{ "aw8697_Opop_short_RTP_87_226Hz.bin" },
	{ "aw8697_bowl_bells_RTP_88_226Hz.bin" },
	{ "aw8697_jumpy_RTP_89_226Hz.bin" },

	{ "aw8697_reserved_90.bin" },
	{ "aw8697_reserved_91.bin" },
	{ "aw8697_reserved_92.bin" },
	{ "aw8697_reserved_93.bin" },
	{ "aw8697_reserved_94.bin" },
	{ "aw8697_reserved_95.bin" },
	{ "aw8697_reserved_96.bin" },
	{ "aw8697_reserved_97.bin" },
	{ "aw8697_reserved_98.bin" },
	{ "aw8697_reserved_99.bin" },

	{ "aw8697_soldier_first_kill_RTP_100_226Hz.bin" },
	{ "aw8697_soldier_second_kill_RTP_101_226Hz.bin" },
	{ "aw8697_soldier_third_kill_RTP_102_226Hz.bin" },
	{ "aw8697_soldier_fourth_kill_RTP_103_226Hz.bin" },
	{ "aw8697_soldier_fifth_kill_RTP_104_226Hz.bin" },
	{ "aw8697_reserved_105.bin" },
	{ "aw8697_reserved_106.bin" },
	{ "aw8697_reserved_107.bin" },
	{ "aw8697_reserved_108.bin" },
	{ "aw8697_reserved_109.bin" },

	{ "aw8697_reserved_110.bin" },
	{ "aw8697_reserved_111.bin" },
	{ "aw8697_reserved_112.bin" },
	{ "aw8697_reserved_113.bin" },
	{ "aw8697_reserved_114.bin" },
	{ "aw8697_reserved_115.bin" },
	{ "aw8697_reserved_116.bin" },
	{ "aw8697_reserved_117.bin" },
	{ "aw8697_reserved_118.bin" },
	{ "aw8697_reserved_119.bin" },

	{ "aw8697_Miss_RTP_120.bin" },
	{ "aw8697_Scenic_RTP_121_226Hz.bin" },
	{ "aw8697_reserved_122.bin" },
	/* used for 7 */
	{ "aw8697_Appear_channel_RTP_123_226Hz.bin" },
	{ "aw8697_Miss_RTP_124_226Hz.bin" },
	{ "aw8697_Music_channel_RTP_125_226Hz.bin" },
	{ "aw8697_Percussion_channel_RTP_126_226Hz.bin" },
	{ "aw8697_Ripple_channel_RTP_127_226Hz.bin" },
	{ "aw8697_Bright_channel_RTP_128_226Hz.bin" },
	{ "aw8697_Fun_channel_RTP_129_226Hz.bin" },
	{ "aw8697_Glittering_channel_RTP_130_226Hz.bin" },
	{ "aw8697_Harp_channel_RTP_131_226Hz.bin" },
	{ "aw8697_Overtone_channel_RTP_132_226Hz.bin" },
	{ "aw8697_Simple_channel_RTP_133_226Hz.bin" },

	{ "aw8697_Seine_past_RTP_134_226Hz.bin" },
	{ "aw8697_Classical_ring_RTP_135_226Hz.bin" },
	{ "aw8697_Long_for_RTP_136_226Hz.bin" },
	{ "aw8697_Romantic_RTP_137_226Hz.bin" },
	{ "aw8697_Bliss_RTP_138_226Hz.bin" },
	{ "aw8697_Dream_RTP_139_226Hz.bin" },
	{ "aw8697_Relax_RTP_140_226Hz.bin" },
	{ "aw8697_Joy_channel_RTP_141_226Hz.bin" },
	{ "aw8697_weather_wind_RTP_142_226Hz.bin" },
	{ "aw8697_weather_cloudy_RTP_143_226Hz.bin" },
	{ "aw8697_weather_thunderstorm_RTP_144_226Hz.bin" },
	{ "aw8697_weather_default_RTP_145_226Hz.bin" },
	{ "aw8697_weather_sunny_RTP_146_226Hz.bin" },
	{ "aw8697_weather_smog_RTP_147_226Hz.bin" },
	{ "aw8697_weather_snow_RTP_148_226Hz.bin" },
	{ "aw8697_weather_rain_RTP_149_226Hz.bin" },
	/* used for 7 end*/
	{ "aw8697_rtp_lighthouse.bin" },
	{ "aw8697_rtp_silk_19081.bin" },
};
#endif /* OPLUS_FEATURE_CHG_BASIC */

#ifdef OPLUS_FEATURE_CHG_BASIC
static char aw86907_rtp_name_19065_230Hz[][AW86907_RTP_NAME_MAX] = {
	{ "aw8697_rtp.bin" },
	{ "aw8697_Hearty_channel_RTP_1.bin" },
	{ "aw8697_Instant_channel_RTP_2_230Hz.bin" },
	{ "aw8697_Music_channel_RTP_3.bin" },
	{ "aw8697_Percussion_channel_RTP_4.bin" },
	{ "aw8697_Ripple_channel_RTP_5.bin" },
	{ "aw8697_Bright_channel_RTP_6.bin" },
	{ "aw8697_Fun_channel_RTP_7.bin" },
	{ "aw8697_Glittering_channel_RTP_8.bin" },
	{ "aw8697_Granules_channel_RTP_9_230Hz.bin" },
	{ "aw8697_Harp_channel_RTP_10.bin" },
	{ "aw8697_Impression_channel_RTP_11.bin" },
	{ "aw8697_Ingenious_channel_RTP_12_230Hz.bin" },
	{ "aw8697_Joy_channel_RTP_13_230Hz.bin" },
	{ "aw8697_Overtone_channel_RTP_14.bin" },
	{ "aw8697_Receive_channel_RTP_15_230Hz.bin" },
	{ "aw8697_Splash_channel_RTP_16_230Hz.bin" },

	{ "aw8697_About_School_RTP_17_230Hz.bin" },
	{ "aw8697_Bliss_RTP_18.bin" },
	{ "aw8697_Childhood_RTP_19_230Hz.bin" },
	{ "aw8697_Commuting_RTP_20_230Hz.bin" },
	{ "aw8697_Dream_RTP_21.bin" },
	{ "aw8697_Firefly_RTP_22_230Hz.bin" },
	{ "aw8697_Gathering_RTP_23.bin" },
	{ "aw8697_Gaze_RTP_24_230Hz.bin" },
	{ "aw8697_Lakeside_RTP_25_230Hz.bin" },
	{ "aw8697_Lifestyle_RTP_26.bin" },
	{ "aw8697_Memories_RTP_27_230Hz.bin" },
	{ "aw8697_Messy_RTP_28_230Hz.bin" },
	{ "aw8697_Night_RTP_29_230Hz.bin" },
	{ "aw8697_Passionate_Dance_RTP_30_230Hz.bin" },
	{ "aw8697_Playground_RTP_31_230Hz.bin" },
	{ "aw8697_Relax_RTP_32_230Hz.bin" },
	{ "aw8697_Reminiscence_RTP_33.bin" },
	{ "aw8697_Silence_From_Afar_RTP_34_230Hz.bin" },
	{ "aw8697_Silence_RTP_35_230Hz.bin" },
	{ "aw8697_Stars_RTP_36_230Hz.bin" },
	{ "aw8697_Summer_RTP_37_230Hz.bin" },
	{ "aw8697_Toys_RTP_38_230Hz.bin" },
	{ "aw8697_Travel_RTP_39.bin" },
	{ "aw8697_Vision_RTP_40.bin" },

	{ "aw8697_waltz_channel_RTP_41_230Hz.bin" },
	{ "aw8697_cut_channel_RTP_42_230Hz.bin" },
	{ "aw8697_clock_channel_RTP_43_230Hz.bin" },
	{ "aw8697_long_sound_channel_RTP_44_230Hz.bin" },
	{ "aw8697_short_channel_RTP_45_230Hz.bin" },
	{ "aw8697_two_error_remaind_RTP_46_230Hz.bin" },

	{ "aw8697_kill_program_RTP_47_230Hz.bin" },
	{ "aw8697_Simple_channel_RTP_48.bin" },
	{ "aw8697_Pure_RTP_49_230Hz.bin" },
	{ "aw8697_reserved_sound_channel_RTP_50.bin" },

	{ "aw8697_high_temp_high_humidity_channel_RTP_51.bin" },

	{ "aw8697_old_steady_test_RTP_52.bin" },
	{ "aw8697_listen_pop_53_235Hz.bin" },
	{ "aw8697_desk_7_RTP_54_230Hz.bin" },
	{ "aw8697_nfc_10_RTP_55_230Hz.bin" },
	{ "aw8697_vibrator_remain_12_RTP_56_230Hz.bin" },
	{ "aw8697_notice_13_RTP_57.bin" },
	{ "aw8697_third_ring_14_RTP_58.bin" },
	{ "aw8697_emergency_warning_RTP_59_230Hz.bin" },

	{ "aw8697_honor_fisrt_kill_RTP_60_230Hz.bin" },
	{ "aw8697_honor_two_kill_RTP_61_230Hz.bin" },
	{ "aw8697_honor_three_kill_RTP_62_230Hz.bin" },
	{ "aw8697_honor_four_kill_RTP_63_230Hz.bin" },
	{ "aw8697_honor_five_kill_RTP_64_230Hz.bin" },
	{ "aw8697_honor_three_continu_kill_RTP_65_230Hz.bin" },
	{ "aw8697_honor_four_continu_kill_RTP_66_230Hz.bin" },
	{ "aw8697_honor_unstoppable_RTP_67_230Hz.bin" },
	{ "aw8697_honor_thousands_kill_RTP_68_230Hz.bin" },
	{ "aw8697_honor_lengendary_RTP_69_230Hz.bin" },
	{ "aw8697_Airy_morning_RTP_70_230Hz.bin" },
	{ "aw8697_Temple_morning_RTP_71_230Hz.bin" },
	{ "aw8697_Water_cicidas_RTP_72_230Hz.bin" },
	{ "aw8697_Electro_club_RTP_73_230Hz.bin" },
	{ "aw8697_Vacation_RTP_74_230Hz.bin" },
	{ "aw8697_Jazz_funk_RTP_75_230Hz.bin" },
	{ "aw8697_House_club_RTP_76_230Hz.bin" },
	{ "aw8697_temple_tone_RTP_77_230Hz.bin" },
	{ "aw8697_Jazz_dreamy_RTP_78_230Hz.bin" },
	{ "aw8697_Jazz_modern_RTP_79_230Hz.bin" },
	{ "aw8697_Tone_round_RTP_80_230Hz.bin" },
	{ "aw8697_Digi_rise_RTP_81_230Hz.bin" },
	{ "aw8697_Wood_phone_RTP_82_230Hz.bin" },
	{ "aw8697_Hey_RTP_83_230Hz.bin" },
	{ "aw8697_Zanza_RTP_84_230Hz.bin" },
	{ "aw8697_Info_RTP_85_230Hz.bin" },
	{ "aw8697_Tip_top_RTP_86_230Hz.bin" },
	{ "aw8697_Opop_short_RTP_87_230Hz.bin" },
	{ "aw8697_bowl_bells_RTP_88_230Hz.bin" },
	{ "aw8697_jumpy_RTP_89_230Hz.bin" },

	{ "aw8697_reserved_90.bin" },
	{ "aw8697_reserved_91.bin" },
	{ "aw8697_reserved_92.bin" },
	{ "aw8697_reserved_93.bin" },
	{ "aw8697_reserved_94.bin" },
	{ "aw8697_reserved_95.bin" },
	{ "aw8697_reserved_96.bin" },
	{ "aw8697_reserved_97.bin" },
	{ "aw8697_reserved_98.bin" },
	{ "aw8697_reserved_99.bin" },

	{ "aw8697_soldier_first_kill_RTP_100_230Hz.bin" },
	{ "aw8697_soldier_second_kill_RTP_101_230Hz.bin" },
	{ "aw8697_soldier_third_kill_RTP_102_230Hz.bin" },
	{ "aw8697_soldier_fourth_kill_RTP_103_230Hz.bin" },
	{ "aw8697_soldier_fifth_kill_RTP_104_230Hz.bin" },
	{ "aw8697_reserved_105.bin" },
	{ "aw8697_reserved_106.bin" },
	{ "aw8697_reserved_107.bin" },
	{ "aw8697_reserved_108.bin" },
	{ "aw8697_reserved_109.bin" },

	{ "aw8697_reserved_110.bin" },
	{ "aw8697_reserved_111.bin" },
	{ "aw8697_reserved_112.bin" },
	{ "aw8697_reserved_113.bin" },
	{ "aw8697_reserved_114.bin" },
	{ "aw8697_reserved_115.bin" },
	{ "aw8697_reserved_116.bin" },
	{ "aw8697_reserved_117.bin" },
	{ "aw8697_reserved_118.bin" },
	{ "aw8697_reserved_119.bin" },

	{ "aw8697_Miss_RTP_120.bin" },
	{ "aw8697_Scenic_RTP_121_230Hz.bin" },
	{ "aw8697_reserved_122.bin" },
	/* used for 7 */
	{ "aw8697_Appear_channel_RTP_123_230Hz.bin" },
	{ "aw8697_Miss_RTP_124_230Hz.bin" },
	{ "aw8697_Music_channel_RTP_125_230Hz.bin" },
	{ "aw8697_Percussion_channel_RTP_126_230Hz.bin" },
	{ "aw8697_Ripple_channel_RTP_127_230Hz.bin" },
	{ "aw8697_Bright_channel_RTP_128_230Hz.bin" },
	{ "aw8697_Fun_channel_RTP_129_230Hz.bin" },
	{ "aw8697_Glittering_channel_RTP_130_230Hz.bin" },
	{ "aw8697_Harp_channel_RTP_131_230Hz.bin" },
	{ "aw8697_Overtone_channel_RTP_132_230Hz.bin" },
	{ "aw8697_Simple_channel_RTP_133_230Hz.bin" },

	{ "aw8697_Seine_past_RTP_134_230Hz.bin" },
	{ "aw8697_Classical_ring_RTP_135_230Hz.bin" },
	{ "aw8697_Long_for_RTP_136_230Hz.bin" },
	{ "aw8697_Romantic_RTP_137_230Hz.bin" },
	{ "aw8697_Bliss_RTP_138_230Hz.bin" },
	{ "aw8697_Dream_RTP_139_230Hz.bin" },
	{ "aw8697_Relax_RTP_140_230Hz.bin" },
	{ "aw8697_Joy_channel_RTP_141_230Hz.bin" },
	{ "aw8697_weather_wind_RTP_142_230Hz.bin" },
	{ "aw8697_weather_cloudy_RTP_143_230Hz.bin" },
	{ "aw8697_weather_thunderstorm_RTP_144_230Hz.bin" },
	{ "aw8697_weather_default_RTP_145_230Hz.bin" },
	{ "aw8697_weather_sunny_RTP_146_230Hz.bin" },
	{ "aw8697_weather_smog_RTP_147_230Hz.bin" },
	{ "aw8697_weather_snow_RTP_148_230Hz.bin" },
	{ "aw8697_weather_rain_RTP_149_230Hz.bin" },
	/* used for 7 end*/
	{ "aw8697_rtp_lighthouse.bin" },
	{ "aw8697_rtp_silk_19081.bin" },
};
#endif /* OPLUS_FEATURE_CHG_BASIC */

static char aw86907_rtp_name_19065_234Hz[][AW86907_RTP_NAME_MAX] = {
	{ "aw8697_rtp.bin" },
#ifdef OPLUS_FEATURE_CHG_BASIC
	{ "aw8697_Hearty_channel_RTP_1.bin" },
	{ "aw8697_Instant_channel_RTP_2_234Hz.bin" },
	{ "aw8697_Music_channel_RTP_3.bin" },
	{ "aw8697_Percussion_channel_RTP_4.bin" },
	{ "aw8697_Ripple_channel_RTP_5.bin" },
	{ "aw8697_Bright_channel_RTP_6.bin" },
	{ "aw8697_Fun_channel_RTP_7.bin" },
	{ "aw8697_Glittering_channel_RTP_8.bin" },
	{ "aw8697_Granules_channel_RTP_9_234Hz.bin" },
	{ "aw8697_Harp_channel_RTP_10.bin" },
	{ "aw8697_Impression_channel_RTP_11.bin" },
	{ "aw8697_Ingenious_channel_RTP_12_234Hz.bin" },
	{ "aw8697_Joy_channel_RTP_13_234Hz.bin" },
	{ "aw8697_Overtone_channel_RTP_14.bin" },
	{ "aw8697_Receive_channel_RTP_15_234Hz.bin" },
	{ "aw8697_Splash_channel_RTP_16_234Hz.bin" },

	{ "aw8697_About_School_RTP_17_234Hz.bin" },
	{ "aw8697_Bliss_RTP_18.bin" },
	{ "aw8697_Childhood_RTP_19_234Hz.bin" },
	{ "aw8697_Commuting_RTP_20_234Hz.bin" },
	{ "aw8697_Dream_RTP_21.bin" },
	{ "aw8697_Firefly_RTP_22_234Hz.bin" },
	{ "aw8697_Gathering_RTP_23.bin" },
	{ "aw8697_Gaze_RTP_24_234Hz.bin" },
	{ "aw8697_Lakeside_RTP_25_234Hz.bin" },
	{ "aw8697_Lifestyle_RTP_26.bin" },
	{ "aw8697_Memories_RTP_27_234Hz.bin" },
	{ "aw8697_Messy_RTP_28_234Hz.bin" },
	{ "aw8697_Night_RTP_29_234Hz.bin" },
	{ "aw8697_Passionate_Dance_RTP_30_234Hz.bin" },
	{ "aw8697_Playground_RTP_31_234Hz.bin" },
	{ "aw8697_Relax_RTP_32_234Hz.bin" },
	{ "aw8697_Reminiscence_RTP_33.bin" },
	{ "aw8697_Silence_From_Afar_RTP_34_234Hz.bin" },
	{ "aw8697_Silence_RTP_35_234Hz.bin" },
	{ "aw8697_Stars_RTP_36_234Hz.bin" },
	{ "aw8697_Summer_RTP_37_234Hz.bin" },
	{ "aw8697_Toys_RTP_38_234Hz.bin" },
	{ "aw8697_Travel_RTP_39.bin" },
	{ "aw8697_Vision_RTP_40.bin" },

	{ "aw8697_waltz_channel_RTP_41_234Hz.bin" },
	{ "aw8697_cut_channel_RTP_42_234Hz.bin" },
	{ "aw8697_clock_channel_RTP_43_234Hz.bin" },
	{ "aw8697_long_sound_channel_RTP_44_234Hz.bin" },
	{ "aw8697_short_channel_RTP_45_234Hz.bin" },
	{ "aw8697_two_error_remaind_RTP_46_234Hz.bin" },

	{ "aw8697_kill_program_RTP_47_234Hz.bin" },
	{ "aw8697_Simple_channel_RTP_48.bin" },
	{ "aw8697_Pure_RTP_49_234Hz.bin" },
	{ "aw8697_reserved_sound_channel_RTP_50.bin" },

	{ "aw8697_high_temp_high_humidity_channel_RTP_51.bin" },

	{ "aw8697_old_steady_test_RTP_52.bin" },
	{ "aw8697_listen_pop_53_235Hz.bin" },
	{ "aw8697_desk_7_RTP_54_234Hz.bin" },
	{ "aw8697_nfc_10_RTP_55_234Hz.bin" },
	{ "aw8697_vibrator_remain_12_RTP_56_234Hz.bin" },
	{ "aw8697_notice_13_RTP_57.bin" },
	{ "aw8697_third_ring_14_RTP_58.bin" },
	{ "aw8697_emergency_warning_RTP_59_234Hz.bin" },

	{ "aw8697_honor_fisrt_kill_RTP_60_234Hz.bin" },
	{ "aw8697_honor_two_kill_RTP_61_234Hz.bin" },
	{ "aw8697_honor_three_kill_RTP_62_234Hz.bin" },
	{ "aw8697_honor_four_kill_RTP_63_234Hz.bin" },
	{ "aw8697_honor_five_kill_RTP_64_234Hz.bin" },
	{ "aw8697_honor_three_continu_kill_RTP_65_234Hz.bin" },
	{ "aw8697_honor_four_continu_kill_RTP_66_234Hz.bin" },
	{ "aw8697_honor_unstoppable_RTP_67_234Hz.bin" },
	{ "aw8697_honor_thousands_kill_RTP_68_234Hz.bin" },
	{ "aw8697_honor_lengendary_RTP_69_234Hz.bin" },
	{ "aw8697_Airy_morning_RTP_70_234Hz.bin" },
	{ "aw8697_Temple_morning_RTP_71_234Hz.bin" },
	{ "aw8697_Water_cicidas_RTP_72_234Hz.bin" },
	{ "aw8697_Electro_club_RTP_73_234Hz.bin" },
	{ "aw8697_Vacation_RTP_74_234Hz.bin" },
	{ "aw8697_Jazz_funk_RTP_75_234Hz.bin" },
	{ "aw8697_House_club_RTP_76_234Hz.bin" },
	{ "aw8697_temple_tone_RTP_77_234Hz.bin" },
	{ "aw8697_Jazz_dreamy_RTP_78_234Hz.bin" },
	{ "aw8697_Jazz_modern_RTP_79_234Hz.bin" },
	{ "aw8697_Tone_round_RTP_80_234Hz.bin" },
	{ "aw8697_Digi_rise_RTP_81_234Hz.bin" },
	{ "aw8697_Wood_phone_RTP_82_234Hz.bin" },
	{ "aw8697_Hey_RTP_83_234Hz.bin" },
	{ "aw8697_Zanza_RTP_84_234Hz.bin" },
	{ "aw8697_Info_RTP_85_234Hz.bin" },
	{ "aw8697_Tip_top_RTP_86_234Hz.bin" },
	{ "aw8697_Opop_short_RTP_87_234Hz.bin" },
	{ "aw8697_bowl_bells_RTP_88_234Hz.bin" },
	{ "aw8697_jumpy_RTP_89_234Hz.bin" },

	{ "aw8697_reserved_90.bin" },
	{ "aw8697_reserved_91.bin" },
	{ "aw8697_reserved_92.bin" },
	{ "aw8697_reserved_93.bin" },
	{ "aw8697_reserved_94.bin" },
	{ "aw8697_reserved_95.bin" },
	{ "aw8697_reserved_96.bin" },
	{ "aw8697_reserved_97.bin" },
	{ "aw8697_reserved_98.bin" },
	{ "aw8697_reserved_99.bin" },

	{ "aw8697_soldier_first_kill_RTP_100_234Hz.bin" },
	{ "aw8697_soldier_second_kill_RTP_101_234Hz.bin" },
	{ "aw8697_soldier_third_kill_RTP_102_234Hz.bin" },
	{ "aw8697_soldier_fourth_kill_RTP_103_234Hz.bin" },
	{ "aw8697_soldier_fifth_kill_RTP_104_234Hz.bin" },
	{ "aw8697_reserved_105.bin" },
	{ "aw8697_reserved_106.bin" },
	{ "aw8697_reserved_107.bin" },
	{ "aw8697_reserved_108.bin" },
	{ "aw8697_reserved_109.bin" },

	{ "aw8697_reserved_110.bin" },
	{ "aw8697_reserved_111.bin" },
	{ "aw8697_reserved_112.bin" },
	{ "aw8697_reserved_113.bin" },
	{ "aw8697_reserved_114.bin" },
	{ "aw8697_reserved_115.bin" },
	{ "aw8697_reserved_116.bin" },
	{ "aw8697_reserved_117.bin" },
	{ "aw8697_reserved_118.bin" },
	{ "aw8697_reserved_119.bin" },

	{ "aw8697_Miss_RTP_120.bin" },
	{ "aw8697_Scenic_RTP_121_234Hz.bin" },
	{ "aw8697_reserved_122.bin" },
	/* used for 7 */
	{ "aw8697_Appear_channel_RTP_123_234Hz.bin" },
	{ "aw8697_Miss_RTP_124_234Hz.bin" },
	{ "aw8697_Music_channel_RTP_125_234Hz.bin" },
	{ "aw8697_Percussion_channel_RTP_126_234Hz.bin" },
	{ "aw8697_Ripple_channel_RTP_127_234Hz.bin" },
	{ "aw8697_Bright_channel_RTP_128_234Hz.bin" },
	{ "aw8697_Fun_channel_RTP_129_234Hz.bin" },
	{ "aw8697_Glittering_channel_RTP_130_234Hz.bin" },
	{ "aw8697_Harp_channel_RTP_131_234Hz.bin" },
	{ "aw8697_Overtone_channel_RTP_132_234Hz.bin" },
	{ "aw8697_Simple_channel_RTP_133_234Hz.bin" },

	{ "aw8697_Seine_past_RTP_134_234Hz.bin" },
	{ "aw8697_Classical_ring_RTP_135_234Hz.bin" },
	{ "aw8697_Long_for_RTP_136_234Hz.bin" },
	{ "aw8697_Romantic_RTP_137_234Hz.bin" },
	{ "aw8697_Bliss_RTP_138_234Hz.bin" },
	{ "aw8697_Dream_RTP_139_234Hz.bin" },
	{ "aw8697_Relax_RTP_140_234Hz.bin" },
	{ "aw8697_Joy_channel_RTP_141_234Hz.bin" },
	{ "aw8697_weather_wind_RTP_142_234Hz.bin" },
	{ "aw8697_weather_cloudy_RTP_143_234Hz.bin" },
	{ "aw8697_weather_thunderstorm_RTP_144_234Hz.bin" },
	{ "aw8697_weather_default_RTP_145_234Hz.bin" },
	{ "aw8697_weather_sunny_RTP_146_234Hz.bin" },
	{ "aw8697_weather_smog_RTP_147_234Hz.bin" },
	{ "aw8697_weather_snow_RTP_148_234Hz.bin" },
	{ "aw8697_weather_rain_RTP_149_234Hz.bin" },

#endif
	{ "aw8697_rtp_lighthouse.bin" },
	{ "aw8697_rtp_silk_19081.bin" },
};

static char aw86907_rtp_name_19065_237Hz[][AW86907_RTP_NAME_MAX] = {
	{ "aw8697_rtp.bin" },
#ifdef OPLUS_FEATURE_CHG_BASIC
	{ "aw8697_Hearty_channel_RTP_1.bin" },
	{ "aw8697_Instant_channel_RTP_2_237Hz.bin" },
	{ "aw8697_Music_channel_RTP_3.bin" },
	{ "aw8697_Percussion_channel_RTP_4.bin" },
	{ "aw8697_Ripple_channel_RTP_5.bin" },
	{ "aw8697_Bright_channel_RTP_6.bin" },
	{ "aw8697_Fun_channel_RTP_7.bin" },
	{ "aw8697_Glittering_channel_RTP_8.bin" },
	{ "aw8697_Granules_channel_RTP_9_237Hz.bin" },
	{ "aw8697_Harp_channel_RTP_10.bin" },
	{ "aw8697_Impression_channel_RTP_11.bin" },
	{ "aw8697_Ingenious_channel_RTP_12_237Hz.bin" },
	{ "aw8697_Joy_channel_RTP_13_237Hz.bin" },
	{ "aw8697_Overtone_channel_RTP_14.bin" },
	{ "aw8697_Receive_channel_RTP_15_237Hz.bin" },
	{ "aw8697_Splash_channel_RTP_16_237Hz.bin" },

	{ "aw8697_About_School_RTP_17_237Hz.bin" },
	{ "aw8697_Bliss_RTP_18.bin" },
	{ "aw8697_Childhood_RTP_19_237Hz.bin" },
	{ "aw8697_Commuting_RTP_20_237Hz.bin" },
	{ "aw8697_Dream_RTP_21.bin" },
	{ "aw8697_Firefly_RTP_22_237Hz.bin" },
	{ "aw8697_Gathering_RTP_23.bin" },
	{ "aw8697_Gaze_RTP_24_237Hz.bin" },
	{ "aw8697_Lakeside_RTP_25_237Hz.bin" },
	{ "aw8697_Lifestyle_RTP_26.bin" },
	{ "aw8697_Memories_RTP_27_237Hz.bin" },
	{ "aw8697_Messy_RTP_28_237Hz.bin" },
	{ "aw8697_Night_RTP_29_237Hz.bin" },
	{ "aw8697_Passionate_Dance_RTP_30_237Hz.bin" },
	{ "aw8697_Playground_RTP_31_237Hz.bin" },
	{ "aw8697_Relax_RTP_32_237Hz.bin" },
	{ "aw8697_Reminiscence_RTP_33.bin" },
	{ "aw8697_Silence_From_Afar_RTP_34_237Hz.bin" },
	{ "aw8697_Silence_RTP_35_237Hz.bin" },
	{ "aw8697_Stars_RTP_36_237Hz.bin" },
	{ "aw8697_Summer_RTP_37_237Hz.bin" },
	{ "aw8697_Toys_RTP_38_237Hz.bin" },
	{ "aw8697_Travel_RTP_39.bin" },
	{ "aw8697_Vision_RTP_40.bin" },

	{ "aw8697_waltz_channel_RTP_41_237Hz.bin" },
	{ "aw8697_cut_channel_RTP_42_237Hz.bin" },
	{ "aw8697_clock_channel_RTP_43_237Hz.bin" },
	{ "aw8697_long_sound_channel_RTP_44_237Hz.bin" },
	{ "aw8697_short_channel_RTP_45_237Hz.bin" },
	{ "aw8697_two_error_remaind_RTP_46_237Hz.bin" },

	{ "aw8697_kill_program_RTP_47_237Hz.bin" },
	{ "aw8697_Simple_channel_RTP_48.bin" },
	{ "aw8697_Pure_RTP_49_237Hz.bin" },
	{ "aw8697_reserved_sound_channel_RTP_50.bin" },

	{ "aw8697_high_temp_high_humidity_channel_RTP_51.bin" },

	{ "aw8697_old_steady_test_RTP_52.bin" },
	{ "aw8697_listen_pop_53_235Hz.bin" },
	{ "aw8697_desk_7_RTP_54_237Hz.bin" },
	{ "aw8697_nfc_10_RTP_55_237Hz.bin" },
	{ "aw8697_vibrator_remain_12_RTP_56_237Hz.bin" },
	{ "aw8697_notice_13_RTP_57.bin" },
	{ "aw8697_third_ring_14_RTP_58.bin" },
	{ "aw8697_emergency_warning_RTP_59_234Hz.bin" },

	{ "aw8697_honor_fisrt_kill_RTP_60_237Hz.bin" },
	{ "aw8697_honor_two_kill_RTP_61_237Hz.bin" },
	{ "aw8697_honor_three_kill_RTP_62_237Hz.bin" },
	{ "aw8697_honor_four_kill_RTP_63_237Hz.bin" },
	{ "aw8697_honor_five_kill_RTP_64_237Hz.bin" },
	{ "aw8697_honor_three_continu_kill_RTP_65_237Hz.bin" },
	{ "aw8697_honor_four_continu_kill_RTP_66_237Hz.bin" },
	{ "aw8697_honor_unstoppable_RTP_67_237Hz.bin" },
	{ "aw8697_honor_thousands_kill_RTP_68_237Hz.bin" },
	{ "aw8697_honor_lengendary_RTP_69_237Hz.bin" },
	{ "aw8697_Airy_morning_RTP_70_237Hz.bin" },
	{ "aw8697_Temple_morning_RTP_71_237Hz.bin" },
	{ "aw8697_Water_cicidas_RTP_72_237Hz.bin" },
	{ "aw8697_Electro_club_RTP_73_237Hz.bin" },
	{ "aw8697_Vacation_RTP_74_237Hz.bin" },
	{ "aw8697_Jazz_funk_RTP_75_237Hz.bin" },
	{ "aw8697_House_club_RTP_76_237Hz.bin" },
	{ "aw8697_temple_tone_RTP_77_237Hz.bin" },
	{ "aw8697_Jazz_dreamy_RTP_78_237Hz.bin" },
	{ "aw8697_Jazz_modern_RTP_79_237Hz.bin" },
	{ "aw8697_Tone_round_RTP_80_237Hz.bin" },
	{ "aw8697_Digi_rise_RTP_81_237Hz.bin" },
	{ "aw8697_Wood_phone_RTP_82_237Hz.bin" },
	{ "aw8697_Hey_RTP_83_237Hz.bin" },
	{ "aw8697_Zanza_RTP_84_237Hz.bin" },
	{ "aw8697_Info_RTP_85_237Hz.bin" },
	{ "aw8697_Tip_top_RTP_86_237Hz.bin" },
	{ "aw8697_Opop_short_RTP_87_237Hz.bin" },
	{ "aw8697_bowl_bells_RTP_88_237Hz.bin" },
	{ "aw8697_jumpy_RTP_89_237Hz.bin" },

	{ "aw8697_reserved_90.bin" },
	{ "aw8697_reserved_91.bin" },
	{ "aw8697_reserved_92.bin" },
	{ "aw8697_reserved_93.bin" },
	{ "aw8697_reserved_94.bin" },
	{ "aw8697_reserved_95.bin" },
	{ "aw8697_reserved_96.bin" },
	{ "aw8697_reserved_97.bin" },
	{ "aw8697_reserved_98.bin" },
	{ "aw8697_reserved_99.bin" },

	{ "aw8697_soldier_first_kill_RTP_100_237Hz.bin" },
	{ "aw8697_soldier_second_kill_RTP_101_237Hz.bin" },
	{ "aw8697_soldier_third_kill_RTP_102_237Hz.bin" },
	{ "aw8697_soldier_fourth_kill_RTP_103_237Hz.bin" },
	{ "aw8697_soldier_fifth_kill_RTP_104_237Hz.bin" },
	{ "aw8697_reserved_105.bin" },
	{ "aw8697_reserved_106.bin" },
	{ "aw8697_reserved_107.bin" },
	{ "aw8697_reserved_108.bin" },
	{ "aw8697_reserved_109.bin" },

	{ "aw8697_reserved_110.bin" },
	{ "aw8697_reserved_111.bin" },
	{ "aw8697_reserved_112.bin" },
	{ "aw8697_reserved_113.bin" },
	{ "aw8697_reserved_114.bin" },
	{ "aw8697_reserved_115.bin" },
	{ "aw8697_reserved_116.bin" },
	{ "aw8697_reserved_117.bin" },
	{ "aw8697_reserved_118.bin" },
	{ "aw8697_reserved_119.bin" },

	{ "aw8697_Miss_RTP_120.bin" },
	{ "aw8697_Scenic_RTP_121_237Hz.bin" },
	{ "aw8697_reserved_122.bin" },
	/* used for 7 */
	{ "aw8697_Appear_channel_RTP_123_237Hz.bin" },
	{ "aw8697_Miss_RTP_124_237Hz.bin" },
	{ "aw8697_Music_channel_RTP_125_237Hz.bin" },
	{ "aw8697_Percussion_channel_RTP_126_237Hz.bin" },
	{ "aw8697_Ripple_channel_RTP_127_237Hz.bin" },
	{ "aw8697_Bright_channel_RTP_128_237Hz.bin" },
	{ "aw8697_Fun_channel_RTP_129_237Hz.bin" },
	{ "aw8697_Glittering_channel_RTP_130_237Hz.bin" },
	{ "aw8697_Harp_channel_RTP_131_237Hz.bin" },
	{ "aw8697_Overtone_channel_RTP_132_237Hz.bin" },
	{ "aw8697_Simple_channel_RTP_133_237Hz.bin" },

	{ "aw8697_Seine_past_RTP_134_237Hz.bin" },
	{ "aw8697_Classical_ring_RTP_135_237Hz.bin" },
	{ "aw8697_Long_for_RTP_136_237Hz.bin" },
	{ "aw8697_Romantic_RTP_137_237Hz.bin" },
	{ "aw8697_Bliss_RTP_138_237Hz.bin" },
	{ "aw8697_Dream_RTP_139_237Hz.bin" },
	{ "aw8697_Relax_RTP_140_237Hz.bin" },
	{ "aw8697_Joy_channel_RTP_141_237Hz.bin" },
	{ "aw8697_weather_wind_RTP_142_237Hz.bin" },
	{ "aw8697_weather_cloudy_RTP_143_237Hz.bin" },
	{ "aw8697_weather_thunderstorm_RTP_144_237Hz.bin" },
	{ "aw8697_weather_default_RTP_145_237Hz.bin" },
	{ "aw8697_weather_sunny_RTP_146_237Hz.bin" },
	{ "aw8697_weather_smog_RTP_147_237Hz.bin" },
	{ "aw8697_weather_snow_RTP_148_237Hz.bin" },
	{ "aw8697_weather_rain_RTP_149_237Hz.bin" },

#endif
	{ "aw8697_rtp_lighthouse.bin" },
	{ "aw8697_rtp_silk_19081.bin" },
};

struct aw86907_container *aw86907_rtp = NULL;
struct aw86907 *g_aw86907;
#define AW86907_CONTAINER_DEFAULT_SIZE (2 * 1024 * 1024) //  2M
int aw86907_container_size = AW86907_CONTAINER_DEFAULT_SIZE;

static int aw86907_container_init(int size)
{
	if (!aw86907_rtp || size > aw86907_container_size) {
		if (aw86907_rtp) {
			vfree(aw86907_rtp);
		}
		aw86907_rtp = vmalloc(size);
		if (!aw86907_rtp) {
			pr_err("%s: error allocating memory\n", __func__);
			return -1;
		}
		aw86907_container_size = size;
	}

	memset(aw86907_rtp, 0, size);

	return 0;
}

/******************************************************
 *
 * functions
 *
 ******************************************************/
static void aw86907_interrupt_clear(struct aw86907 *aw86907);
static int aw86907_haptic_juge_RTP_is_going_on(struct aw86907 *aw86907);
static int aw86907_haptic_stop(struct aw86907 *aw86907);
static void aw86907_haptic_reset_init(struct aw86907 *aw86907);

/******************************************************
 *
 * aw86907 i2c write/read
 *
 ******************************************************/
static int aw86907_i2c_write(struct aw86907 *aw86907, unsigned char reg_addr,
			     unsigned char reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	if (reg_addr == 0x04 || reg_addr == 0x05) {
		pr_debug("%s: reg_addr_0x%02x = 0x%02x\n", __func__, reg_addr,
			 reg_data);
	}
	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_write_byte_data(aw86907->i2c, reg_addr,
						reg_data);
		if (ret < 0) {
			pr_err("%s: i2c_write cnt=%d error=%d\n", __func__, cnt,
			       ret);
		} else {
			break;
		}
		cnt++;
		msleep(AW_I2C_RETRY_DELAY);
	}

	return ret;
}

static int aw86907_i2c_read(struct aw86907 *aw86907, unsigned char reg_addr,
			    unsigned char *reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_read_byte_data(aw86907->i2c, reg_addr);
		if (ret < 0) {
			pr_err("%s: i2c_read cnt=%d error=%d\n", __func__, cnt,
			       ret);
		} else {
			*reg_data = ret;
			break;
		}
		cnt++;
		msleep(AW_I2C_RETRY_DELAY);
	}

	return ret;
}

static int aw86907_i2c_write_bits(struct aw86907 *aw86907,
				  unsigned char reg_addr, unsigned int mask,
				  unsigned char reg_data)
{
	unsigned char reg_val = 0;

	aw86907_i2c_read(aw86907, reg_addr, &reg_val);
	reg_val &= mask;
	reg_val |= reg_data;
	aw86907_i2c_write(aw86907, reg_addr, reg_val);

	return 0;
}

static int aw86907_i2c_writes(struct aw86907 *aw86907, unsigned char reg_addr,
			      unsigned char *buf, unsigned int len)
{
	int ret = -1;
	unsigned char *data;

	data = kmalloc(len + 1, GFP_KERNEL);
	if (data == NULL) {
		pr_err("%s: can not allocate memory\n", __func__);
		return -ENOMEM;
	}

	data[0] = reg_addr;
	memcpy(&data[1], buf, len);

	ret = i2c_master_send(aw86907->i2c, data, len + 1);
	if (ret < 0) {
		pr_err("%s: i2c master send error\n", __func__);
	}

	kfree(data);

	return ret;
}

static int aw86907_i2c_reads(struct aw86907 *aw86907, unsigned char reg_addr,
			     unsigned char *buf, unsigned int len)
{
	int ret = -1;

	ret = i2c_smbus_write_byte(aw86907->i2c, reg_addr);
	if (ret) {
		pr_err("%s: couldn't send request, ret=%d\n", __func__, ret);
		return ret;
	}
	ret = i2c_master_recv(aw86907->i2c, buf, len);
	if (ret != len) {
		pr_err("%s: couldn't read registers, return %d bytes\n",
		       __func__, ret);
		return ret;
	}

	return ret;
}

/*****************************************************
 *
 * ram update
 *
 *****************************************************/
static void aw86907_rtp_loaded(const struct firmware *cont, void *context)
{
	struct aw86907 *aw86907 = context;
	int ret = 0;
	pr_info("%s enter\n", __func__);

	if (!cont) {
		pr_err("%s: failed to read %s\n", __func__,
		       aw86907_rtp_name[aw86907->rtp_file_num]);
		release_firmware(cont);
		return;
	}

	pr_info("%s: loaded %s - size: %zu\n", __func__,
		aw86907_rtp_name[aw86907->rtp_file_num], cont ? cont->size : 0);

	/* aw86907 rtp update */
	mutex_lock(&aw86907->rtp_lock);
#ifndef OPLUS_FEATURE_CHG_BASIC
	aw86907_rtp = kzalloc(cont->size + sizeof(int), GFP_KERNEL);
	if (!aw86907_rtp) {
		release_firmware(cont);
		mutex_unlock(&aw86907->rtp_lock);
		pr_err("%s: Error allocating memory\n", __func__);
		return;
	}
#else
	ret = aw86907_container_init(cont->size + sizeof(int));
	if (ret < 0) {
		release_firmware(cont);
		mutex_unlock(&aw86907->rtp_lock);
		pr_err("%s: Error allocating memory\n", __func__);
		return;
	}
#endif
	aw86907_rtp->len = cont->size;
	pr_info("%s: rtp size = %d\n", __func__, aw86907_rtp->len);
	memcpy(aw86907_rtp->data, cont->data, cont->size);
	release_firmware(cont);
	mutex_unlock(&aw86907->rtp_lock);

	aw86907->rtp_init = 1;
	pr_info("%s: rtp update complete\n", __func__);
}

static int aw86907_rtp_update(struct aw86907 *aw86907)
{
	pr_info("%s enter\n", __func__);

	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				       aw86907_rtp_name[aw86907->rtp_file_num],
				       aw86907->dev, GFP_KERNEL, aw86907,
				       aw86907_rtp_loaded);
}

static void aw86907_container_update(struct aw86907 *aw86907,
				     struct aw86907_container *aw86907_cont)
{
	int i = 0;
	unsigned int shift = 0;
	unsigned char reg_val = 0;
	unsigned int temp = 0;
	pr_info("%s enter\n", __func__);

	mutex_lock(&aw86907->lock);

	aw86907->ram.baseaddr_shift = 2;
	aw86907->ram.ram_shift = 4;

	/* RAMINIT Enable */
	aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSCTRL1,
			       AW86907_BIT_SYSCTRL1_RAMINIT_MASK,
			       AW86907_BIT_SYSCTRL1_RAMINIT_ON);

	/* base addr */
	shift = aw86907->ram.baseaddr_shift;
	aw86907->ram.base_addr =
		(unsigned int)((aw86907_cont->data[0 + shift] << 8) |
			       (aw86907_cont->data[1 + shift]));
	pr_info("%s: base_addr=0x%4x\n", __func__, aw86907->ram.base_addr);

	aw86907_i2c_write(aw86907, AW86907_REG_RTPCFG1,
			  aw86907_cont->data[0 + shift]); /*ADDRH*/
	aw86907_i2c_write(aw86907, AW86907_REG_RTPCFG2,
			  aw86907_cont->data[1 + shift]); /*ADDRL*/

	/* FIFO_AEH */
	aw86907_i2c_write_bits(
		aw86907, AW86907_REG_RTPCFG3, AW86907_BIT_RTPCFG3_FIFO_AEH_MASK,
		(unsigned char)(((aw86907->ram.base_addr >> 1) >> 4) & 0xF0));
	/* FIFO AEL */
	aw86907_i2c_write(
		aw86907, AW86907_REG_RTPCFG4,
		(unsigned char)(((aw86907->ram.base_addr >> 1) & 0x00FF)));
	/* FIFO_AFH */
	aw86907_i2c_write_bits(
		aw86907, AW86907_REG_RTPCFG3, AW86907_BIT_RTPCFG3_FIFO_AFH_MASK,
		(unsigned char)(((aw86907->ram.base_addr -
				  (aw86907->ram.base_addr >> 2)) >>
				 8) &
				0x0F));
	/* FIFO_AFL */
	aw86907_i2c_write(aw86907, AW86907_REG_RTPCFG5,
			  (unsigned char)(((aw86907->ram.base_addr -
					    (aw86907->ram.base_addr >> 2)) &
					   0x00FF)));

	/*
*	unsigned int temp
*	HIGH<byte4 byte3 byte2 byte1>LOW
*	|_ _ _ _AF-12BIT_ _ _ _AE-12BIT|
*/
	aw86907_i2c_read(aw86907, AW86907_REG_RTPCFG3, &reg_val);
	temp = ((reg_val & 0x0f) << 24) | ((reg_val & 0xf0) << 4);
	aw86907_i2c_read(aw86907, AW86907_REG_RTPCFG4, &reg_val);
	temp = temp | reg_val;
	pr_info("%s: almost_empty_threshold = %d\n", __func__,
		(unsigned short)temp);

	aw86907_i2c_read(aw86907, AW86907_REG_RTPCFG5, &reg_val);
	temp = temp | (reg_val << 16);
	pr_info("%s: almost_full_threshold = %d\n", __func__, temp >> 16);

	/* ram */
	shift = aw86907->ram.baseaddr_shift;
	aw86907_i2c_write_bits(aw86907, AW86907_REG_RAMADDRH,
			       AW86907_BIT_RAMADDRH_MASK,
			       aw86907_cont->data[0 + shift]);
	aw86907_i2c_write(aw86907, AW86907_REG_RAMADDRL,
			  aw86907_cont->data[1 + shift]);
	shift = aw86907->ram.ram_shift;
	for (i = shift; i < aw86907_cont->len; i++) {
		aw86907_i2c_write(aw86907, AW86907_REG_RAMDATA,
				  aw86907_cont->data[i]);
	}

#if 0
    /* ram check */
    shift = aw86907->ram.baseaddr_shift;
    aw86907_i2c_write(aw86907, AW86907_REG_RAMADDRH, aw86907_cont->data[0+shift]);
    aw86907_i2c_write(aw86907, AW86907_REG_RAMADDRL, aw86907_cont->data[1+shift]);
    shift = aw86907->ram.ram_shift;
    for(i=shift; i<aw86907_cont->len; i++) {
        aw86907_i2c_read(aw86907, AW86907_REG_RAMDATA, &reg_val);
        if(reg_val != aw86907_cont->data[i]) {
            pr_err("%s: ram check error addr=0x%04x, file_data=0x%02x, ram_data=0x%02x\n",
                    __func__, i, aw86907_cont->data[i], reg_val);
            return;
        }
    }
#endif

	/* RAMINIT Disable */
	aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSCTRL1,
			       AW86907_BIT_SYSCTRL1_RAMINIT_MASK,
			       AW86907_BIT_SYSCTRL1_RAMINIT_OFF);

	mutex_unlock(&aw86907->lock);

	pr_info("%s exit\n", __func__);
}

static void aw86907_ram_loaded(const struct firmware *cont, void *context)
{
	struct aw86907 *aw86907 = context;
	struct aw86907_container *aw86907_fw;
	int i = 0;
	unsigned short check_sum = 0;

	pr_info("%s enter\n", __func__);

	if (!cont) {
		pr_err("%s: failed to fw \n", __func__);
		release_firmware(cont);
		return;
	}

	/*
    for(i=0; i<cont->size; i++) {
        pr_info("%s: addr:0x%04x, data:0x%02x\n", __func__, i, *(cont->data+i));
    }
*/

	/* check sum */
	for (i = 2; i < cont->size; i++) {
		check_sum += cont->data[i];
	}
	if (check_sum !=
	    (unsigned short)((cont->data[0] << 8) | (cont->data[1]))) {
		pr_err("%s: check sum err: check_sum=0x%04x\n", __func__,
		       check_sum);
		return;
	} else {
		pr_info("%s: check sum pass : 0x%04x\n", __func__, check_sum);
		aw86907->ram.check_sum = check_sum;
	}

	/* aw86907 ram update */
	aw86907_fw = vmalloc(cont->size + sizeof(int));
	if (!aw86907_fw) {
		release_firmware(cont);
		pr_err("%s: Error allocating memory\n", __func__);
		return;
	}
	memset(aw86907_fw, 0, cont->size + sizeof(int));
	aw86907_fw->len = cont->size;
	memcpy(aw86907_fw->data, cont->data, cont->size);
	release_firmware(cont);

	aw86907_container_update(aw86907, aw86907_fw);

	aw86907->ram.len = aw86907_fw->len - aw86907->ram.ram_shift;

	vfree(aw86907_fw);

	aw86907->ram_init = 1;
	pr_info("%s: fw update complete\n", __func__);

	aw86907_rtp_update(aw86907);
}

static int aw86907_ram_update(struct aw86907 *aw86907)
{
	unsigned char index = 0;
	aw86907->ram_init = 0;
	aw86907->rtp_init = 0;

	//f0 165 166,166
	//167 168, 168
	//169 170 ,170
	//171 172, 172
	//173 174,174
	//>175 ,174
	if (aw86907->device_id == 815) {
		if (aw86907->f0 < F0_VAL_MIN_0815 ||
		    aw86907->f0 > F0_VAL_MAX_0815) {
			aw86907->f0 = 1700;
		}
	} else if (aw86907->device_id == 832) {
		if (aw86907->f0 < F0_VAL_MIN_0832 ||
		    aw86907->f0 > F0_VAL_MAX_0832) {
			aw86907->f0 = 2350;
		}
	} else {
		if (aw86907->f0 < F0_VAL_MIN_0833 ||
		    aw86907->f0 > F0_VAL_MAX_0833) {
			aw86907->f0 = 2350;
		}
	}
	aw86907->haptic_real_f0 = (aw86907->f0 / 10); // get f0 from nvram
	pr_err("%s: aw86907->haptic_real_f0 [%d]\n", __func__,
	       aw86907->haptic_real_f0);
	/*
    if (aw86907->haptic_real_f0 <167)
    {
        index = 0;
    }
    else if(aw86907->haptic_real_f0 <169)
    {
        index = 1;
    }
    else if(aw86907->haptic_real_f0 <171)
    {   
        index = 2;
    }
    else if(aw86907->haptic_real_f0 <173)
    {   
        index = 3;
    }
    else
    {
        index = 4;
    }
*/
	if (aw86907->device_id == 832) {
		pr_info("%s:19065 haptic bin name  %s \n", __func__,
			aw86907_ram_name_19065[index]);
		return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
					       aw86907_ram_name_19065[index],
					       aw86907->dev, GFP_KERNEL,
					       aw86907, aw86907_ram_loaded);
	} else if (aw86907->device_id == 833) {
		pr_info("%s:19065 haptic bin name  %s \n", __func__,
			aw86907_ram_name_19161[index]);
		return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
					       aw86907_ram_name_19161[index],
					       aw86907->dev, GFP_KERNEL,
					       aw86907, aw86907_ram_loaded);
	} else {
		pr_info("%s:haptic bin name  %s \n", __func__,
			aw86907_ram_name[index]);
		return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
					       aw86907_ram_name[index],
					       aw86907->dev, GFP_KERNEL,
					       aw86907, aw86907_ram_loaded);
	}
}

#ifdef AWINIC_RAM_UPDATE_DELAY
static void aw86907_ram_work_routine(struct work_struct *work)
{
	struct aw86907 *aw86907 =
		container_of(work, struct aw86907, ram_work.work);

	pr_info("%s enter\n", __func__);

	aw86907_ram_update(aw86907);
}
#endif

static int aw86907_ram_init(struct aw86907 *aw86907)
{
#ifdef AWINIC_RAM_UPDATE_DELAY
	int ram_timer_val = 5000;
	INIT_DELAYED_WORK(&aw86907->ram_work, aw86907_ram_work_routine);
	schedule_delayed_work(&aw86907->ram_work,
			      msecs_to_jiffies(ram_timer_val));
#else
	aw86907_ram_update(aw86907);
#endif
	return 0;
}

/*****************************************************
 *
 * haptic control
 *
 *****************************************************/
static int aw86907_haptic_softreset(struct aw86907 *aw86907)
{
	pr_info("%s enter\n", __func__);

	aw86907_i2c_write(aw86907, AW86907_REG_ID, 0xAA);
	usleep_range(3000, 3500);
	return 0;
}

static int aw86907_haptic_play_mode(struct aw86907 *aw86907,
				    unsigned char play_mode)
{
	pr_debug("%s enter\n", __func__);

	switch (play_mode) {
	case AW86907_HAPTIC_STANDBY_MODE:
		pr_info("%s: enter standby mode\n", __func__);
		aw86907->play_mode = AW86907_HAPTIC_STANDBY_MODE;
		aw86907_haptic_stop(aw86907);
		break;
	case AW86907_HAPTIC_RAM_MODE:
		pr_info("%s: enter ram mode\n", __func__);
		aw86907->play_mode = AW86907_HAPTIC_RAM_MODE;
		aw86907_i2c_write_bits(aw86907, AW86907_REG_PLAYCFG3,
				       AW86907_BIT_PLAYCFG3_PLAY_MODE_MASK,
				       AW86907_BIT_PLAYCFG3_PLAY_MODE_RAM);
		/* bst mode */
		aw86907_i2c_write_bits(aw86907, AW86907_REG_PLAYCFG1,
				       AW86907_BIT_PLAYCFG1_BST_MODE_MASK,
				       AW86907_BIT_PLAYCFG1_BST_MODE_BOOST);
		mdelay(2);
		break;
	case AW86907_HAPTIC_RAM_LOOP_MODE:
		pr_info("%s: enter ram loop mode\n", __func__);
		aw86907->play_mode = AW86907_HAPTIC_RAM_LOOP_MODE;
		aw86907_i2c_write_bits(aw86907, AW86907_REG_PLAYCFG3,
				       AW86907_BIT_PLAYCFG3_PLAY_MODE_MASK,
				       AW86907_BIT_PLAYCFG3_PLAY_MODE_RAM);
		/* bst mode */
		aw86907_i2c_write_bits(aw86907, AW86907_REG_PLAYCFG1,
				       AW86907_BIT_PLAYCFG1_BST_MODE_MASK,
				       AW86907_BIT_PLAYCFG1_BST_MODE_BYPASS);
		aw86907_i2c_write_bits(aw86907, AW86907_REG_PLAYCFG1,
				       AW86907_BIT_PLAYCFG1_BST_MODE_MASK,
				       AW86907_BIT_PLAYCFG1_BST_MODE_BOOST);
		mdelay(2);
		break;
	case AW86907_HAPTIC_RTP_MODE:
		pr_info("%s: enter rtp mode\n", __func__);
		aw86907->play_mode = AW86907_HAPTIC_RTP_MODE;
		aw86907_i2c_write_bits(aw86907, AW86907_REG_PLAYCFG3,
				       AW86907_BIT_PLAYCFG3_PLAY_MODE_MASK,
				       AW86907_BIT_PLAYCFG3_PLAY_MODE_RTP);
		/* bst mode */
		aw86907_i2c_write_bits(aw86907, AW86907_REG_PLAYCFG1,
				       AW86907_BIT_PLAYCFG1_BST_MODE_MASK,
				       AW86907_BIT_PLAYCFG1_BST_MODE_BOOST);
		mdelay(2);
		break;
	case AW86907_HAPTIC_TRIG_MODE:
		pr_info("%s: enter trig mode\n", __func__);
		aw86907->play_mode = AW86907_HAPTIC_TRIG_MODE;
		aw86907_i2c_write_bits(aw86907, AW86907_REG_PLAYCFG3,
				       AW86907_BIT_PLAYCFG3_PLAY_MODE_MASK,
				       AW86907_BIT_PLAYCFG3_PLAY_MODE_RAM);
		mdelay(2);
		break;
	case AW86907_HAPTIC_CONT_MODE:
		pr_info("%s: enter cont mode\n", __func__);
		aw86907->play_mode = AW86907_HAPTIC_CONT_MODE;
		aw86907_i2c_write_bits(aw86907, AW86907_REG_PLAYCFG3,
				       AW86907_BIT_PLAYCFG3_PLAY_MODE_MASK,
				       AW86907_BIT_PLAYCFG3_PLAY_MODE_CONT);
		/* bst mode */
		aw86907_i2c_write_bits(aw86907, AW86907_REG_PLAYCFG1,
				       AW86907_BIT_PLAYCFG1_BST_MODE_MASK,
				       AW86907_BIT_PLAYCFG1_BST_MODE_BYPASS);
		break;
	default:
		dev_err(aw86907->dev, "%s: play mode %d err", __func__,
			play_mode);
		break;
	}
	return 0;
}

static int aw86907_haptic_play_go(struct aw86907 *aw86907, bool flag)
{
	pr_debug("%s enter\n", __func__);

	if (flag == true) {
		aw86907_i2c_write(aw86907, AW86907_REG_PLAYCFG4,
				  AW86907_BIT_PLAYCFG4_GO_ON);
		mdelay(2);
	} else {
		aw86907_haptic_stop(aw86907);
	}

	return 0;
}

static int aw86907_set_clock(struct aw86907 *aw86907, int clock_type)
{
	if (clock_type == AW86907_HAPTIC_CLOCK_CALI_F0) {
		aw86907_i2c_write_bits(aw86907, AW86907_REG_TRIMCFG3,
				       AW86907_BIT_TRIMCFG3_TRIM_LRA_MASK,
				       aw86907->clock_system_f0_cali_lra);
	} else if (clock_type == AW86907_HAPTIC_CLOCK_CALI_OSC_STANDARD) {
		aw86907_i2c_write_bits(
			aw86907, AW86907_REG_TRIMCFG3,
			AW86907_BIT_TRIMCFG3_TRIM_LRA_MASK,
			aw86907->clock_standard_osc_lra_rtim_code);
	}
	return 0;
}

static int aw86907_haptic_stop(struct aw86907 *aw86907)
{
	unsigned char cnt = 40;
	unsigned char reg_val = 0;
	bool force_flag = true;

	pr_info("%s enter\n", __func__);
	aw86907->play_mode = AW86907_HAPTIC_STANDBY_MODE;
	aw86907_i2c_read(aw86907, AW86907_REG_GLBRD5, &reg_val);
	if ((reg_val & 0x0f) == AW86907_BIT_GLBRD5_STATE_STANDBY ||
	    (reg_val & 0x0f) == AW86907_BIT_GLBRD5_STATE_I2S_GO) {
		force_flag = false;
		pr_info("%s already in standby mode! glb_state=0x%02X\n",
			__func__, reg_val);
	} else {
		aw86907_i2c_write(aw86907, AW86907_REG_PLAYCFG4,
				  AW86907_BIT_PLAYCFG4_STOP_ON);
		while (cnt) {
			aw86907_i2c_read(aw86907, AW86907_REG_GLBRD5, &reg_val);
			if ((reg_val & 0x0f) ==
				    AW86907_BIT_GLBRD5_STATE_STANDBY ||
			    (reg_val & 0x0f) ==
				    AW86907_BIT_GLBRD5_STATE_I2S_GO) {
				cnt = 0;
				force_flag = false;
				pr_info("%s entered standby! glb_state=0x%02X\n",
					__func__, reg_val);
			} else {
				cnt--;
				pr_info("%s wait for standby, glb_state=0x%02X\n",
					__func__, reg_val);
			}
			usleep_range(2000, 2500);
		}
	}

	if (force_flag) {
		pr_err("%s force to enter standby mode!\n", __func__);
		aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSCTRL2,
				       AW86907_BIT_SYSCTRL2_STANDBY_MASK,
				       AW86907_BIT_SYSCTRL2_STANDBY_ON);

		aw86907_haptic_softreset(aw86907);
		aw86907_haptic_reset_init(aw86907);
	}
	return 0;
}

static int aw86907_haptic_start(struct aw86907 *aw86907)
{
	pr_debug("%s enter\n", __func__);

	aw86907_haptic_play_go(aw86907, true);

	return 0;
}

static int aw86907_haptic_set_wav_seq(struct aw86907 *aw86907,
				      unsigned char wav, unsigned char seq)
{
	aw86907_i2c_write(aw86907, AW86907_REG_WAVCFG1 + wav, seq);
	return 0;
}

static int aw86907_haptic_set_wav_loop(struct aw86907 *aw86907,
				       unsigned char wav, unsigned char loop)
{
	unsigned char tmp = 0;

	if (wav % 2) {
		tmp = loop << 0;
		aw86907_i2c_write_bits(aw86907, AW86907_REG_WAVCFG9 + (wav / 2),
				       AW86907_BIT_WAVLOOP_SEQ_EVEN_MASK, tmp);
	} else {
		tmp = loop << 4;
		aw86907_i2c_write_bits(aw86907, AW86907_REG_WAVCFG9 + (wav / 2),
				       AW86907_BIT_WAVLOOP_SEQ_ODD_MASK, tmp);
	}

	return 0;
}
/*
static int aw86907_haptic_set_main_loop(struct aw86907 *aw86907,
        unsigned char loop)
{
    aw86907_i2c_write_bits(aw86907, AW86907_REG_WAVCFG13, AW86907_BIT_WAVCFG13_MAINLOOP_MASK, loop);
    return 0;
}
*/

static int aw86907_haptic_set_repeat_wav_seq(struct aw86907 *aw86907,
					     unsigned char seq)
{
	aw86907_haptic_set_wav_seq(aw86907, 0x00, seq);
	aw86907_haptic_set_wav_loop(aw86907, 0x00,
				    AW86907_BIT_WAVLOOP_INIFINITELY);

	return 0;
}

static int aw86907_haptic_set_bst_vol(struct aw86907 *aw86907,
				      unsigned char bst_vol)
{
	if (bst_vol & 0xc0) {
		bst_vol = 0x3f;
	}
	aw86907_i2c_write_bits(aw86907, AW86907_REG_PLAYCFG1,
			       AW86907_BIT_PLAYCFG1_BST_VOUT_RDA_MASK, bst_vol);
	return 0;
}

static int aw86907_haptic_set_bst_peak_cur(struct aw86907 *aw86907,
					   unsigned char peak_cur)
{
	aw86907_i2c_write_bits(aw86907, AW86907_REG_BSTCFG1,
			       AW86907_BIT_BSTCFG1_BST_PC_MASK, peak_cur);
	return 0;
}

static int aw86907_haptic_set_gain(struct aw86907 *aw86907, unsigned char gain)
{
	aw86907_i2c_write(aw86907, AW86907_REG_PLAYCFG2, gain);
	return 0;
}

static int aw86907_haptic_set_pwm(struct aw86907 *aw86907, unsigned char mode)
{
	switch (mode) {
	case AW86907_PWM_48K:
		aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSCTRL2,
				       AW86907_BIT_SYSCTRL2_WAVDAT_MODE_MASK,
				       AW86907_BIT_SYSCTRL2_RATE_48K);
		break;
	case AW86907_PWM_24K:
		aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSCTRL2,
				       AW86907_BIT_SYSCTRL2_WAVDAT_MODE_MASK,
				       AW86907_BIT_SYSCTRL2_RATE_24K);
		break;
	case AW86907_PWM_12K:
		aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSCTRL2,
				       AW86907_BIT_SYSCTRL2_WAVDAT_MODE_MASK,
				       AW86907_BIT_SYSCTRL2_RATE_12K);
		break;
	default:
		break;
	}
	return 0;
}

static int aw86907_haptic_play_wav_seq(struct aw86907 *aw86907,
				       unsigned char flag)
{
	pr_debug("%s enter\n", __func__);

	if (flag) {
		aw86907_haptic_play_mode(aw86907, AW86907_HAPTIC_RAM_MODE);
		aw86907_haptic_start(aw86907);
	}
	return 0;
}

static int aw86907_haptic_play_repeat_seq(struct aw86907 *aw86907,
					  unsigned char flag)
{
	pr_debug("%s enter\n", __func__);

	if (flag) {
		aw86907_haptic_play_mode(aw86907, AW86907_HAPTIC_RAM_LOOP_MODE);
		aw86907_haptic_start(aw86907);
	}

	return 0;
}

/*****************************************************
 *
 * motor protect
 *
 *****************************************************/
static int aw86907_haptic_swicth_motorprotect_config(struct aw86907 *aw86907,
						     unsigned char addr,
						     unsigned char val)
{
	pr_debug("%s enter\n", __func__);
	if (addr == 1) {
		aw86907_i2c_write_bits(aw86907, AW86907_REG_DETCFG1,
				       AW86907_BIT_DETCFG1_PRCT_MODE_MASK,
				       AW86907_BIT_DETCFG1_PRCT_MODE_VALID);
		aw86907_i2c_write_bits(aw86907, AW86907_REG_PWMCFG1,
				       AW86907_BIT_PWMCFG1_PRC_EN_MASK,
				       AW86907_BIT_PWMCFG1_PRC_ENABLE);
		aw86907_i2c_write_bits(aw86907, AW86907_REG_PWMCFG3,
				       AW86907_BIT_PWMCFG3_PR_EN_MASK,
				       AW86907_BIT_PWMCFG3_PR_ENABLE);
	} else if (addr == 0) {
		aw86907_i2c_write_bits(aw86907, AW86907_REG_DETCFG1,
				       AW86907_BIT_DETCFG1_PRCT_MODE_MASK,
				       AW86907_BIT_DETCFG1_PRCT_MODE_INVALID);
		aw86907_i2c_write_bits(aw86907, AW86907_REG_PWMCFG1,
				       AW86907_BIT_PWMCFG1_PRC_EN_MASK,
				       AW86907_BIT_PWMCFG1_PRC_DISABLE);
		aw86907_i2c_write_bits(aw86907, AW86907_REG_PWMCFG3,
				       AW86907_BIT_PWMCFG3_PR_EN_MASK,
				       AW86907_BIT_PWMCFG3_PR_DISABLE);
	} else if (addr == 0x2d) {
		aw86907_i2c_write_bits(aw86907, AW86907_REG_PWMCFG1,
				       AW86907_BIT_PWMCFG1_PRCTIME_MASK, val);
	} else if (addr == 0x3e) {
		aw86907_i2c_write_bits(aw86907, AW86907_REG_PWMCFG3,
				       AW86907_BIT_PWMCFG3_PRLVL_MASK, val);
	} else if (addr == 0x3f) {
		aw86907_i2c_write(aw86907, AW86907_REG_PWMCFG4, val);
	} else {
		/*nothing to do;*/
	}
	return 0;
}

/*****************************************************
 *
 * os calibration
 *
 *****************************************************/
static int aw86907_haptic_os_calibration(struct aw86907 *aw86907)
{
	unsigned int cont = 2000;
	unsigned char reg_val = 0;
	pr_debug("%s enter\n", __func__);
	aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSCTRL1,
			       AW86907_BIT_SYSCTRL1_RAMINIT_MASK,
			       AW86907_BIT_SYSCTRL1_RAMINIT_ON);
	aw86907_i2c_write_bits(aw86907, AW86907_REG_DETCFG2,
			       AW86907_BIT_DETCFG2_DIAG_GO_MASK,
			       AW86907_BIT_DETCFG2_DIAG_GO_ON);

	while (1) {
		aw86907_i2c_read(aw86907, AW86907_REG_DETCFG2, &reg_val);
		if ((reg_val & 0x01) == 0 || cont == 0)
			break;
		cont--;
	}

	if (cont == 0)
		pr_err("%s calibration offset failed!\n", __func__);

	aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSCTRL1,
			       AW86907_BIT_SYSCTRL1_RAMINIT_MASK,
			       AW86907_BIT_SYSCTRL1_RAMINIT_OFF);
	return 0;
}

static int aw86907_haptic_auto_boost_config(struct aw86907 *aw86907,
					    unsigned char flag)
{
	aw86907->auto_boost = flag;
	if (flag) {
		aw86907_i2c_write_bits(aw86907, AW86907_REG_PLAYCFG3,
				       AW86907_BIT_PLAYCFG3_AUTO_BST_MASK,
				       AW86907_BIT_PLAYCFG3_AUTO_BST_ENABLE);
	} else {
		aw86907_i2c_write_bits(aw86907, AW86907_REG_PLAYCFG3,
				       AW86907_BIT_PLAYCFG3_AUTO_BST_MASK,
				       AW86907_BIT_PLAYCFG3_AUTO_BST_DISABLE);
	}
	return 0;
}

/*****************************************************
 *
 * vbat mode
 *
 *****************************************************/
static int aw86907_haptic_cont_vbat_mode(struct aw86907 *aw86907,
					 unsigned char flag)
{
	if (flag == AW86907_HAPTIC_CONT_VBAT_HW_COMP_MODE) {
		aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSCTRL1,
				       AW86907_BIT_SYSCTRL1_VBAT_MODE_MASK,
				       AW86907_BIT_SYSCTRL1_VBAT_MODE_HW);
	} else {
		aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSCTRL1,
				       AW86907_BIT_SYSCTRL1_VBAT_MODE_MASK,
				       AW86907_BIT_SYSCTRL1_VBAT_MODE_SW);
	}
	return 0;
}

static int aw86907_haptic_get_vbat(struct aw86907 *aw86907)
{
	unsigned char reg_val = 0;
	unsigned int vbat_code = 0;
	unsigned int cont = 2000;

	aw86907_haptic_stop(aw86907);
	aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSCTRL1,
			       AW86907_BIT_SYSCTRL1_RAMINIT_MASK,
			       AW86907_BIT_SYSCTRL1_RAMINIT_ON);
	aw86907_i2c_write_bits(aw86907, AW86907_REG_DETCFG2,
			       AW86907_BIT_DETCFG2_VBAT_GO_MASK,
			       AW86907_BIT_DETCFG2_VABT_GO_ON);

	while (1) {
		aw86907_i2c_read(aw86907, AW86907_REG_DETCFG2, &reg_val);
		if ((reg_val & 0x02) == 0 || cont == 0)
			break;
		cont--;
	}

	aw86907_i2c_read(aw86907, AW86907_REG_DET_VBAT, &reg_val);
	vbat_code = (vbat_code | reg_val) << 2;
	aw86907_i2c_read(aw86907, AW86907_REG_DET_LO, &reg_val);
	vbat_code = vbat_code | ((reg_val & 0x30) >> 4);
	aw86907->vbat = 6100 * vbat_code / 1024;
	if (aw86907->vbat > AW86907_VBAT_MAX) {
		aw86907->vbat = AW86907_VBAT_MAX;
		pr_debug("%s vbat max limit = %dmV\n", __func__, aw86907->vbat);
	}
	if (aw86907->vbat < AW86907_VBAT_MIN) {
		aw86907->vbat = AW86907_VBAT_MIN;
		pr_debug("%s vbat min limit = %dmV\n", __func__, aw86907->vbat);
	}

	aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSCTRL1,
			       AW86907_BIT_SYSCTRL1_RAMINIT_MASK,
			       AW86907_BIT_SYSCTRL1_RAMINIT_OFF);

	return 0;
}

static int aw86907_haptic_ram_vbat_comp(struct aw86907 *aw86907, bool flag)
{
	int temp_gain = 0;

	if (flag) {
		if (aw86907->ram_vbat_comp ==
		    AW86907_HAPTIC_RAM_VBAT_COMP_ENABLE) {
			aw86907_haptic_get_vbat(aw86907);
			temp_gain = aw86907->gain * AW86907_VBAT_REFER /
				    aw86907->vbat;
			if (temp_gain >
			    (128 * AW86907_VBAT_REFER / AW86907_VBAT_MIN)) {
				temp_gain = 128 * AW86907_VBAT_REFER /
					    AW86907_VBAT_MIN;
				pr_debug("%s gain limit=%d\n", __func__,
					 temp_gain);
			}
			aw86907_haptic_set_gain(aw86907, temp_gain);
		} else {
			aw86907_haptic_set_gain(aw86907, aw86907->gain);
		}
	} else {
		aw86907_haptic_set_gain(aw86907, aw86907->gain);
	}
	pr_err("%s: aw86907->gain = 0x%x, temp_gain = 0x%x, aw86907->ram_vbat_comp=%d.\n",
	       __func__, aw86907->gain, temp_gain, aw86907->ram_vbat_comp);

	return 0;
}

/*****************************************************
 *
 * f0
 *
 *****************************************************/
static int aw86907_haptic_read_lra_f0(struct aw86907 *aw86907)
{
	int ret = 0;
	unsigned char reg_val = 0;
	unsigned int f0_reg = 0;
	unsigned long f0_tmp = 0;

	pr_info("%s enter\n", __func__);
	/* F_LRA_F0_H */
	ret = aw86907_i2c_read(aw86907, AW86907_REG_CONTRD14, &reg_val);
	f0_reg = (f0_reg | reg_val) << 8;
	/* F_LRA_F0_L */
	ret = aw86907_i2c_read(aw86907, AW86907_REG_CONTRD15, &reg_val);
	f0_reg |= (reg_val << 0);
	if (!f0_reg) {
		pr_err("%s didn't get lra f0 because f0_reg value is 0!\n",
		       __func__);
		aw86907->f0 = aw86907->f0_pre;
		return ret;
	} else {
		f0_tmp = 384000 * 10 / f0_reg;
		aw86907->f0 = (unsigned int)f0_tmp;
		pr_info("%s lra_f0=%d\n", __func__, aw86907->f0);
	}

	return ret;
}

static int aw86907_haptic_read_cont_f0(struct aw86907 *aw86907)
{
	int ret = 0;
	unsigned char reg_val = 0;
	unsigned int f0_reg = 0;
	unsigned long f0_tmp = 0;

	pr_info("%s enter\n", __func__);
	ret = aw86907_i2c_read(aw86907, AW86907_REG_CONTRD16, &reg_val);
	f0_reg = (f0_reg | reg_val) << 8;
	ret = aw86907_i2c_read(aw86907, AW86907_REG_CONTRD17, &reg_val);
	f0_reg |= (reg_val << 0);
	if (!f0_reg) {
		pr_err("%s didn't get cont f0 because f0_reg value is 0!\n",
		       __func__);
		aw86907->cont_f0 = aw86907->f0_pre;
		return ret;
	} else {
		f0_tmp = 384000 * 10 / f0_reg;
		aw86907->cont_f0 = (unsigned int)f0_tmp;
		pr_info("%s cont_f0=%d\n", __func__, aw86907->cont_f0);
	}

	return ret;
}

/*****************************************************
 *
 * rtp
 *
 *****************************************************/
static void aw86907_haptic_set_rtp_aei(struct aw86907 *aw86907, bool flag)
{
	if (flag) {
		aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSINTM,
				       AW86907_BIT_SYSINTM_FF_AEM_MASK,
				       AW86907_BIT_SYSINTM_FF_AEM_ON);
	} else {
		aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSINTM,
				       AW86907_BIT_SYSINTM_FF_AEM_MASK,
				       AW86907_BIT_SYSINTM_FF_AEM_OFF);
	}
}

static unsigned char aw86907_haptic_rtp_get_fifo_aei(struct aw86907 *aw86907)
{
	unsigned char ret;
	unsigned char reg_val;

	if (aw86907->osc_cali_flag == 1) {
		aw86907_i2c_read(aw86907, AW86907_REG_SYSST, &reg_val);
		reg_val &= AW86907_BIT_SYSST_FF_AES;
		ret = reg_val >> 4;
	} else {
		aw86907_i2c_read(aw86907, AW86907_REG_SYSINT, &reg_val);
		reg_val &= AW86907_BIT_SYSINT_FF_AEI;
		ret = reg_val >> 4;
	}

	return ret;
}

static unsigned char aw86907_haptic_rtp_get_fifo_afs(struct aw86907 *aw86907)
{
	unsigned char ret = 0;
	unsigned char reg_val = 0;

	aw86907_i2c_read(aw86907, AW86907_REG_SYSST, &reg_val);
	reg_val &= AW86907_BIT_SYSST_FF_AFS;
	ret = reg_val >> 3;

	return ret;
}

/*****************************************************
 *
 * rtp
 *
 *****************************************************/
static void aw86907_dump_rtp_regs(struct aw86907 *aw86907)
{
#if 0
    unsigned char reg_val = 0;

    aw86907_i2c_read(aw86907, 0x01, &reg_val);
    pr_info("%s reg_0x02 = 0x%02x\n", __func__, reg_val);

    aw86907_i2c_read(aw86907, 0x02, &reg_val);
    pr_info("%s reg_0x02 = 0x%02x\n", __func__, reg_val);

    aw86907_i2c_read(aw86907, 0x03, &reg_val);
    pr_info("%s reg_0x03 = 0x%02x\n", __func__, reg_val);

    aw86907_i2c_read(aw86907, 0x04, &reg_val);
    pr_info("%s reg_0x04 = 0x%02x\n", __func__, reg_val);

    aw86907_i2c_read(aw86907, 0x05, &reg_val);
    pr_info("%s reg_0x05 = 0x%02x\n", __func__, reg_val);

    aw86907_i2c_read(aw86907, 0x09, &reg_val);
    pr_info("%s reg_0x05 = 0x%02x\n", __func__, reg_val);

    aw86907_i2c_read(aw86907, 0x3F, &reg_val);
    pr_info("%s reg_0x3F = 0x%02x\n", __func__, reg_val);
#endif
}

static void aw86907_pm_qos_enable(struct aw86907 *aw86907, bool enabled)
{
	if (enabled) {
		if (atomic_read(&aw86907->qos_cnt) == 0) {
			pm_qos_add_request(&pm_qos_req_vb_aw86907,
					   PM_QOS_CPU_DMA_LATENCY,
					   PM_QOS_VALUE_VB);
		}
		atomic_inc(&aw86907->qos_cnt);
	} else {
		if (atomic_dec_and_test(&aw86907->qos_cnt)) {
			pm_qos_remove_request(&pm_qos_req_vb_aw86907);
		}
	}
}

static int aw86907_haptic_rtp_init(struct aw86907 *aw86907)
{
	unsigned int buf_len = 0;
	bool rtp_start = true;
	unsigned int cnt = 200;
	unsigned char reg_val = 0;
	bool rtp_work_flag = false;

	pr_info("%s enter\n", __func__);
	aw86907_pm_qos_enable(aw86907, true);
	if (aw86907->ram.base_addr == 0) {
		aw86907_ram_update(aw86907);
	}
	aw86907->rtp_cnt = 0;
	mutex_lock(&aw86907->rtp_lock);
	aw86907_dump_rtp_regs(aw86907);

	/* rtp mode config */
	aw86907_haptic_play_mode(aw86907, AW86907_HAPTIC_RTP_MODE);
	aw86907_haptic_play_go(aw86907, true);
	usleep_range(2000, 2500);
	while (cnt) {
		aw86907_i2c_read(aw86907, AW86907_REG_GLBRD5, &reg_val);
		if ((reg_val & 0x0f) == 0x08) {
			cnt = 0;
			rtp_work_flag = true;
			pr_info("%s RTP_GO! glb_state=0x08\n", __func__);
		} else {
			cnt--;
			pr_info("%s wait for RTP_GO, glb_state=0x%02X\n",
				__func__, reg_val);
		}
		usleep_range(2000, 2500);
	}

	if (!rtp_work_flag) {
		/* enter standby mode */
		aw86907_haptic_stop(aw86907);
		pr_err("%s failed to enter RTP_GO status!\n", __func__);
		mutex_unlock(&aw86907->rtp_lock);
		return 0;
	}

	while ((!aw86907_haptic_rtp_get_fifo_afs(aw86907)) &&
	       (aw86907->play_mode == AW86907_HAPTIC_RTP_MODE)) {
		///pr_info("%s rtp cnt = %d\n", __func__, aw86907->rtp_cnt);
		if (rtp_start) {
			if ((aw86907_rtp->len - aw86907->rtp_cnt) <
			    (aw86907->ram.base_addr)) {
				buf_len = aw86907_rtp->len - aw86907->rtp_cnt;
			} else {
				buf_len = (aw86907->ram.base_addr);
			}
			aw86907_i2c_writes(aw86907, AW86907_REG_RTPDATA,
					   &aw86907_rtp->data[aw86907->rtp_cnt],
					   buf_len);
			//pr_info("%s 111 rtp cnt = %d\n", __func__, aw86907->rtp_cnt);
			rtp_start = false;
		} else {
			if ((aw86907_rtp->len - aw86907->rtp_cnt) <
			    (aw86907->ram.base_addr >> 2)) {
				buf_len = aw86907_rtp->len - aw86907->rtp_cnt;
			} else {
				buf_len = (aw86907->ram.base_addr >> 2);
			}
			aw86907_i2c_writes(aw86907, AW86907_REG_RTPDATA,
					   &aw86907_rtp->data[aw86907->rtp_cnt],
					   buf_len);
			//pr_info("%s 222 rtp cnt = %d\n", __func__, aw86907->rtp_cnt);
		}
		aw86907->rtp_cnt += buf_len;
		if (aw86907->rtp_cnt >= aw86907_rtp->len ||
		    aw86907->ram.base_addr == 0) {
			pr_info("%s: rtp update complete, aw86907->ram.base_addr[%d]\n",
				__func__, aw86907->ram.base_addr);
			aw86907->rtp_cnt = 0;
			aw86907_dump_rtp_regs(aw86907);
			mutex_unlock(&aw86907->rtp_lock);
			aw86907_pm_qos_enable(aw86907, false);
			return 0;
		}
	}
	mutex_unlock(&aw86907->rtp_lock);

	if (aw86907->play_mode == AW86907_HAPTIC_RTP_MODE) {
		aw86907_haptic_set_rtp_aei(aw86907, true);
	}
	aw86907_pm_qos_enable(aw86907, false);

	pr_info("%s exit\n", __func__);

	return 0;
}

static int aw86907_clock_OSC_trim_calibration(unsigned long int theory_time,
					      unsigned long int real_time)
{
	unsigned int real_code = 0;
	unsigned int LRA_TRIM_CODE = 0;
	unsigned int DFT_LRA_TRIM_CODE = 0;
	unsigned int Not_need_cali_threshold = 10;

	///unsigned char real_code;

	if (theory_time == real_time) {
		printk("aw_osctheory_time == real_time:%ld  theory_time = %ld not need to cali\n",
		       real_time, theory_time);
		return 0;
	}
	if (theory_time < real_time) {
		if ((real_time - theory_time) > (theory_time / 50)) {
			printk("aw_osc(real_time - theory_time) > (theory_time/50 ) not to cali\n");
			return DFT_LRA_TRIM_CODE;
		}

		if ((real_time - theory_time) <
		    (Not_need_cali_threshold * theory_time / 10000)) {
			printk("aw_oscmicrosecond:%ld  theory_time = %ld not need to cali\n",
			       real_time, theory_time);
			return DFT_LRA_TRIM_CODE;
		}

		real_code = ((real_time - theory_time) * 4000) / theory_time;
		real_code = ((real_code % 10 < 5) ? 0 : 1) + real_code / 10;
		real_code = 32 + real_code;
	}
	if (theory_time > real_time) {
		if ((theory_time - real_time) > (theory_time / 50)) {
			printk("aw_osc(( theory_time - real_time) > (theory_time/50 ))  not to cali \n");
			return DFT_LRA_TRIM_CODE;
		}
		if ((theory_time - real_time) <
		    (Not_need_cali_threshold * theory_time / 10000)) {
			printk("aw_oscmicrosecond:%ld  theory_time = %ld not need to cali\n",
			       real_time, theory_time);
			return DFT_LRA_TRIM_CODE;
		}
		real_code = ((theory_time - real_time) * 4000) / theory_time;
		real_code = ((real_code % 10 < 5) ? 0 : 1) + real_code / 10;
		real_code = 32 - real_code;
	}

	if (real_code > 31) {
		LRA_TRIM_CODE = real_code - 32;
	} else {
		LRA_TRIM_CODE = real_code + 32;
	}
	printk("aw_oscmicrosecond:%ld  theory_time = %ld real_code =0X%02X LRA_TRIM_CODE 0X%02X\n",
	       real_time, theory_time, real_code, LRA_TRIM_CODE);

	return LRA_TRIM_CODE;
}

static int aw86907_rtp_trim_lra_calibration(struct aw86907 *aw86907)
{
	unsigned char reg_val = 0;
	unsigned int fre_val = 0;
	unsigned int theory_time = 0;
	///unsigned int real_code = 0;
	unsigned int lra_rtim_code = 0;

	aw86907_i2c_read(aw86907, AW86907_REG_SYSCTRL2, &reg_val);
	fre_val = (reg_val & 0x03) >> 0;

	if (fre_val == 2 || fre_val == 3)
		theory_time = (aw86907->rtp_len / 12000) * 1000000; /*12K */
	if (fre_val == 0)
		theory_time = (aw86907->rtp_len / 24000) * 1000000; /*24K */
	if (fre_val == 1)
		theory_time = (aw86907->rtp_len / 48000) * 1000000; /*48K */

	printk("microsecond:%ld  theory_time = %d\n", aw86907->microsecond,
	       theory_time);

	lra_rtim_code = aw86907_clock_OSC_trim_calibration(
		theory_time, aw86907->microsecond);

	if (lra_rtim_code > 0)
		aw86907_i2c_write_bits(aw86907, AW86907_REG_TRIMCFG3,
				       AW86907_BIT_TRIMCFG3_TRIM_LRA_MASK,
				       (char)lra_rtim_code);
	aw86907->clock_standard_osc_lra_rtim_code = lra_rtim_code;
	return 0;
}

static unsigned char aw86907_haptic_osc_read_status(struct aw86907 *aw86907)
{
	unsigned char reg_val = 0;
	aw86907_i2c_read(aw86907, AW86907_REG_SYSST2, &reg_val);
	return reg_val;
}

static int aw86907_rtp_osc_calibration(struct aw86907 *aw86907)
{
	const struct firmware *rtp_file;
	int ret = -1;
	unsigned int buf_len = 0;
	///unsigned char reg_val = 0;
	unsigned char osc_int_state = 0;
	///unsigned int  pass_cont=1;
	///unsigned int  cyc_cont=150;
	aw86907->rtp_cnt = 0;
	aw86907->timeval_flags = 1;
	aw86907->osc_cali_flag = 1;

	pr_info("%s enter\n", __func__);

	aw86907_haptic_stop(aw86907);
	aw86907_set_clock(aw86907, AW86907_HAPTIC_CLOCK_CALI_OSC_STANDARD);

	aw86907->f0_cali_flag = AW86907_HAPTIC_CALI_F0;

	if (aw86907->f0_cali_flag == AW86907_HAPTIC_CALI_F0) {
		aw86907_i2c_write_bits(aw86907, AW86907_REG_TRIMCFG3,
				       AW86907_BIT_TRIMCFG3_LRA_TRIM_SRC_MASK,
				       AW86907_BIT_TRIMCFG3_LRA_TRIM_SRC_REG);
	} else {
		aw86907_i2c_write_bits(aw86907, AW86907_REG_TRIMCFG3,
				       AW86907_BIT_TRIMCFG3_LRA_TRIM_SRC_MASK,
				       AW86907_BIT_TRIMCFG3_LRA_TRIM_SRC_EFUSE);
	}
	ret = request_firmware(&rtp_file,
			       aw86907_rtp_name[/*aw86907->rtp_file_num*/ 0],
			       aw86907->dev);
	if (ret < 0) {
		pr_err("%s: failed to read %s\n", __func__,
		       aw86907_rtp_name[/*aw86907->rtp_file_num*/ 0]);
		return ret;
	}
	aw86907->rtp_init = 0;
	mutex_lock(&aw86907->rtp_lock);
#ifndef OPLUS_FEATURE_CHG_BASIC
	kfree(aw86907_rtp);
	aw86907_rtp = kzalloc(rtp_file->size + sizeof(int), GFP_KERNEL);
	if (!aw86907_rtp) {
		release_firmware(rtp_file);
		mutex_unlock(&aw86907->rtp_lock);
		pr_err("%s: error allocating memory\n", __func__);
		return -1;
	}
#else
	ret = aw86907_container_init(rtp_file->size + sizeof(int));
	if (ret < 0) {
		release_firmware(rtp_file);
		mutex_unlock(&aw86907->rtp_lock);
		pr_err("%s: error allocating memory\n", __func__);
		return -1;
	}
#endif
	aw86907_rtp->len = rtp_file->size;
	aw86907->rtp_len = rtp_file->size;
	mutex_unlock(&aw86907->rtp_lock);

	pr_info("%s: rtp file [%s] size = %d\n", __func__,
		aw86907_rtp_name[/*aw86907->rtp_file_num*/ 0],
		aw86907_rtp->len);
	memcpy(aw86907_rtp->data, rtp_file->data, rtp_file->size);
	release_firmware(rtp_file);

	//aw86907->rtp_init = 1; //Don't enter aw86907_irq,because osc calibration use while(1) function

	/* gain */
	aw86907_haptic_ram_vbat_comp(aw86907, false);

	/* rtp mode config */
	aw86907_haptic_play_mode(aw86907, AW86907_HAPTIC_RTP_MODE);
	/* bst mode */
	aw86907_i2c_write_bits(aw86907, AW86907_REG_PLAYCFG1,
			       AW86907_BIT_PLAYCFG1_BST_MODE_MASK,
			       AW86907_BIT_PLAYCFG1_BST_MODE_BYPASS);

	disable_irq(gpio_to_irq(aw86907->irq_gpio));
	/* haptic start */
	aw86907_haptic_start(aw86907);

	aw86907_pm_qos_enable(aw86907, true);

	//while(aw86907->rtp_file_num > 0 && (aw86907->play_mode == AW86907_HAPTIC_RTP_MODE)){
	while (1) {
		if (!aw86907_haptic_rtp_get_fifo_afs(aw86907)) //not almost full
		{
			pr_info("%s !aw86907_haptic_rtp_get_fifo_afs done aw86907->rtp_cnt= %d \n",
				__func__, aw86907->rtp_cnt);
			mutex_lock(&aw86907->rtp_lock);
			if ((aw86907_rtp->len - aw86907->rtp_cnt) <
			    (aw86907->ram.base_addr >> 2)) {
				buf_len = aw86907_rtp->len - aw86907->rtp_cnt;
			} else {
				buf_len = (aw86907->ram.base_addr >> 2);
			}

			if (aw86907->rtp_cnt != aw86907_rtp->len) {
				if (aw86907->timeval_flags == 1) {
					do_gettimeofday(&aw86907->start);
					aw86907->timeval_flags = 0;
				}

				aw86907_i2c_writes(
					aw86907, AW86907_REG_RTPDATA,
					&aw86907_rtp->data[aw86907->rtp_cnt],
					buf_len);
				aw86907->rtp_cnt += buf_len;
			}
			mutex_unlock(&aw86907->rtp_lock);
		}

		osc_int_state = aw86907_haptic_osc_read_status(aw86907);
		if (osc_int_state & AW86907_BIT_SYSST2_FF_EMPTY) {
			do_gettimeofday(&aw86907->end);
			pr_info("%s vincent playback done aw86907->rtp_cnt= %d \n",
				__func__, aw86907->rtp_cnt);
			break;
		}

		do_gettimeofday(&aw86907->end);
		aw86907->microsecond =
			(aw86907->end.tv_sec - aw86907->start.tv_sec) *
				1000000 +
			(aw86907->end.tv_usec - aw86907->start.tv_usec);
		if (aw86907->microsecond > OSC_CALI_MAX_LENGTH) {
			pr_info("%s vincent time out aw86907->rtp_cnt %d osc_int_state %02x\n",
				__func__, aw86907->rtp_cnt, osc_int_state);
			break;
		}
	}
	//ENABLE IRQ
	aw86907_pm_qos_enable(aw86907, false);
	enable_irq(gpio_to_irq(aw86907->irq_gpio));

	aw86907->osc_cali_flag = 0;
	aw86907->microsecond =
		(aw86907->end.tv_sec - aw86907->start.tv_sec) * 1000000 +
		(aw86907->end.tv_usec - aw86907->start.tv_usec);
	/*calibration osc*/
	printk("%s 2018_microsecond:%ld \n", __func__, aw86907->microsecond);

	pr_info("%s exit\n", __func__);
	return 0;
}

static void aw86907_op_clean_status(struct aw86907 *aw86907)
{
	aw86907->audio_ready = false;
	aw86907->haptic_ready = false;
	aw86907->pre_haptic_number = 0;
	aw86907->rtp_routine_on = 0;
	pr_info("%s enter\n", __FUNCTION__);
}

const struct firmware *
aw86907_old_work_file_load_accord_f0(struct aw86907 *aw86907)
{
	const struct firmware *rtp_file;
	unsigned int f0_file_num = 1024;
	int ret = -1;

	if (aw86907->rtp_file_num == AW86907_WAVEFORM_INDEX_OLD_STEADY ||
	    aw86907->rtp_file_num == AW86907_WAVEFORM_INDEX_HIGH_TEMP) {
		if (aw86907->device_id == 815) {
			if (aw86907->f0 <= 1610)
				f0_file_num = 0;
			else if (aw86907->f0 <= 1630)
				f0_file_num = 1;
			else if (aw86907->f0 <= 1650)
				f0_file_num = 2;
			else if (aw86907->f0 <= 1670)
				f0_file_num = 3;
			else if (aw86907->f0 <= 1690)
				f0_file_num = 4;
			else if (aw86907->f0 <= 1710)
				f0_file_num = 5;
			else if (aw86907->f0 <= 1730)
				f0_file_num = 6;
			else if (aw86907->f0 <= 1750)
				f0_file_num = 7;
			else if (aw86907->f0 <= 1770)
				f0_file_num = 8;
			else if (aw86907->f0 <= 1790)
				f0_file_num = 9;
			else
				f0_file_num = 10;
		} else if (aw86907->device_id == 832 ||
			   aw86907->device_id == 833) {
			if (aw86907->f0 <= 2255)
				f0_file_num = 0;
			else if (aw86907->f0 <= 2265)
				f0_file_num = 1;
			else if (aw86907->f0 <= 2275)
				f0_file_num = 2;
			else if (aw86907->f0 <= 2285)
				f0_file_num = 3;
			else if (aw86907->f0 <= 2295)
				f0_file_num = 4;
			else if (aw86907->f0 <= 2305)
				f0_file_num = 5;
			else if (aw86907->f0 <= 2315)
				f0_file_num = 6;
			else if (aw86907->f0 <= 2325)
				f0_file_num = 7;
			else if (aw86907->f0 <= 2335)
				f0_file_num = 8;
			else if (aw86907->f0 <= 2345)
				f0_file_num = 9;
			else
				f0_file_num = 10;
		}
		if (aw86907->rtp_file_num ==
		    AW86907_WAVEFORM_INDEX_OLD_STEADY) {
			if (aw86907->device_id == 815) {
				ret = request_firmware(
					&rtp_file,
					aw86907_old_steady_test_rtp_name_0815
						[f0_file_num],
					aw86907->dev);
			} else if (aw86907->device_id == 832 ||
				   aw86907->device_id == 833) {
				ret = request_firmware(
					&rtp_file,
					aw86907_old_steady_test_rtp_name_0832
						[f0_file_num],
					aw86907->dev);
			}
		} else {
			if (aw86907->device_id == 815) {
				ret = request_firmware(
					&rtp_file,
					aw86907_high_temp_high_humidity_0815
						[f0_file_num],
					aw86907->dev);
			} else if (aw86907->device_id == 832 ||
				   aw86907->device_id == 833) {
				ret = request_firmware(
					&rtp_file,
					aw86907_high_temp_high_humidity_0832
						[f0_file_num],
					aw86907->dev);
			}
		}
		if (ret < 0) {
			pr_err("%s: failed to read id[%d],index[%d]\n",
			       __func__, aw86907->device_id, f0_file_num);
			aw86907->rtp_routine_on = 0;
			return NULL;
		}
		return rtp_file;
	}
	return NULL;
}

const struct firmware *aw86907_rtp_load_file_accord_f0(struct aw86907 *aw86907)
{
	const struct firmware *rtp_file;
	unsigned int f0_file_num = 1024;
	int ret = -1;

	if (aw86907->rtp_file_num == AW86907_WAVEFORM_INDEX_OLD_STEADY ||
	    aw86907->rtp_file_num == AW86907_WAVEFORM_INDEX_HIGH_TEMP) {
		return aw86907_old_work_file_load_accord_f0(aw86907);
	}

	return NULL;

	/*if ((aw86907->rtp_file_num >= RINGTONES_START_INDEX &&
	     aw86907->rtp_file_num <= RINGTONES_END_INDEX) ||
	    (aw86907->rtp_file_num >= NEW_RING_START &&
	     aw86907->rtp_file_num <= NEW_RING_END) ||
	    aw86907->rtp_file_num == RINGTONES_SIMPLE_INDEX ||
	    aw86907->rtp_file_num == RINGTONES_PURE_INDEX) {
		if (aw86907->f0 <= 1670) {
			f0_file_num = aw86907->rtp_file_num;
			pr_info("%s  ringtone f0_file_num[%d]\n", __func__,
				f0_file_num);
			ret = request_firmware(
				&rtp_file,
				aw86907_ringtone_rtp_f0_170_name[f0_file_num],
				aw86907->dev);
			if (ret < 0) {
				pr_err("%s: failed to read %s\n", __func__,
				       aw86907_ringtone_rtp_f0_170_name
					       [f0_file_num]);
				aw86907->rtp_routine_on = 0;
				return NULL;
			}
			return rtp_file;
		}
		return NULL;
	} else if (aw86907->rtp_file_num == AW86907_RTP_LONG_SOUND_INDEX ||
		   aw86907->rtp_file_num == AW86907_WAVEFORM_INDEX_OLD_STEADY) {
		if (aw86907->f0 <= 1650)
			f0_file_num = 0;
		else if (aw86907->f0 <= 1670)
			f0_file_num = 1;
		else if (aw86907->f0 <= 1700)
			f0_file_num = 2;
		else
			f0_file_num = 3;
		pr_info("%s long sound or old steady test f0_file_num[%d], aw86907->rtp_file_num[%d]\n",
			__func__, f0_file_num, aw86907->rtp_file_num);

		if (aw86907->rtp_file_num == AW86907_RTP_LONG_SOUND_INDEX) {
			ret = request_firmware(
				&rtp_file,
				aw86907_long_sound_rtp_name[f0_file_num],
				aw86907->dev);
		} else if (aw86907->rtp_file_num ==
			   AW86907_WAVEFORM_INDEX_OLD_STEADY) {
			ret = request_firmware(
				&rtp_file,
				aw86907_old_steady_test_rtp_name_0815
					[f0_file_num],
				aw86907->dev);
		}
		if (ret < 0) {
			pr_err("%s: failed to read %s\n", __func__,
			       aw86907_long_sound_rtp_name[f0_file_num]);
			aw86907->rtp_routine_on = 0;
			return NULL;
		}
		return rtp_file;
	}
	return NULL; */
}

static void aw86907_rtp_work_routine(struct work_struct *work)
{
	const struct firmware *rtp_file = NULL;
	int ret = -1;
	struct aw86907 *aw86907 = container_of(work, struct aw86907, rtp_work);

	pr_info("%s enter\n", __func__);
	aw86907->rtp_routine_on = 1;
	/* fw loaded */

	rtp_file = aw86907_rtp_load_file_accord_f0(aw86907);
	if (!rtp_file) {
		pr_info("%s  aw86907->rtp_file_num[%d]\n", __func__,
			aw86907->rtp_file_num);
		aw86907->rtp_routine_on = 1;
		if (aw86907->device_id == 815) {
			if (aw86907->f0 <= 1670) {
				ret = request_firmware(
					&rtp_file,
					aw86907_rtp_name_165Hz
						[aw86907->rtp_file_num],
					aw86907->dev);
			} else if (aw86907->f0 <= 1725) {
				ret = request_firmware(
					&rtp_file,
					aw86907_rtp_name[aw86907->rtp_file_num],
					aw86907->dev);
			} else {
				ret = request_firmware(
					&rtp_file,
					aw86907_rtp_name_175Hz
						[aw86907->rtp_file_num],
					aw86907->dev);
			}
		} else if (aw86907->device_id == 832) {
			if (aw86907->f0 <= 2280) {
				ret = request_firmware(
					&rtp_file,
					aw86907_rtp_name_19065_226Hz
						[aw86907->rtp_file_num],
					aw86907->dev);
			} else if (aw86907->f0 <= 2320) {
				ret = request_firmware(
					&rtp_file,
					aw86907_rtp_name_19065_230Hz
						[aw86907->rtp_file_num],
					aw86907->dev);
			} else {
				ret = request_firmware(
					&rtp_file,
					aw86907_rtp_name_19065_234Hz
						[aw86907->rtp_file_num],
					aw86907->dev);
			}
		} else {
			if (aw86907->f0 <= 2280) {
				ret = request_firmware(
					&rtp_file,
					aw86907_rtp_name_19065_226Hz
						[aw86907->rtp_file_num],
					aw86907->dev);
			} else if (aw86907->f0 <= 2320) {
				ret = request_firmware(
					&rtp_file,
					aw86907_rtp_name_19065_230Hz
						[aw86907->rtp_file_num],
					aw86907->dev);
			} else if (aw86907->f0 <= 2350) {
				ret = request_firmware(
					&rtp_file,
					aw86907_rtp_name_19065_234Hz
						[aw86907->rtp_file_num],
					aw86907->dev);
			} else {
				ret = request_firmware(
					&rtp_file,
					aw86907_rtp_name_19065_237Hz
						[aw86907->rtp_file_num],
					aw86907->dev);
			}
		}
		if (ret < 0) {
			pr_err("%s: failed to read %s, aw86907->f0=%d\n",
			       __func__,
			       aw86907_rtp_name[aw86907->rtp_file_num],
			       aw86907->f0);
			aw86907->rtp_routine_on = 0;
			return;
		}
	}
	aw86907->rtp_init = 0;

	mutex_lock(&aw86907->rtp_lock);
#ifndef OPLUS_FEATURE_CHG_BASIC
	kfree(aw86907_rtp);
	aw86907_rtp = kzalloc(rtp_file->size + sizeof(int), GFP_KERNEL);
	if (!aw86907_rtp) {
		release_firmware(rtp_file);
		mutex_unlock(&aw86907->rtp_lock);
		pr_err("%s: error allocating memory\n", __func__);
		return;
	}
#else
	ret = aw86907_container_init(rtp_file->size + sizeof(int));
	if (ret < 0) {
		release_firmware(rtp_file);
		mutex_unlock(&aw86907->rtp_lock);
		pr_err("%s: error allocating memory\n", __func__);

		aw86907_op_clean_status(aw86907);
		aw86907->rtp_routine_on = 0;
		return;
	}
#endif
	aw86907_rtp->len = rtp_file->size;

	memcpy(aw86907_rtp->data, rtp_file->data, rtp_file->size);
	mutex_unlock(&aw86907->rtp_lock);
	release_firmware(rtp_file);

	if (aw86907->device_id == 815) {
		pr_info("%s: rtp file [%s] size = %d\n", __func__,
			aw86907_rtp_name[aw86907->rtp_file_num],
			aw86907_rtp->len);
	} else {
		pr_info("%s: rtp file [%s] size = %d\n", __func__,
			aw86907_rtp_name_19065_230Hz[aw86907->rtp_file_num],
			aw86907_rtp->len);
	}

	aw86907->rtp_init = 1;
	mutex_lock(&aw86907->lock);

	/* set clock to stand */
	aw86907_set_clock(aw86907, AW86907_HAPTIC_CLOCK_CALI_OSC_STANDARD);

	/* gain */
	aw86907_haptic_ram_vbat_comp(aw86907, false);

	aw86907_haptic_set_bst_vol(aw86907, aw86907->vmax);

	mutex_unlock(&aw86907->lock);
	aw86907_haptic_rtp_init(aw86907);

	aw86907_op_clean_status(aw86907);
	aw86907->rtp_routine_on = 0;
}

static void aw86907_rtp_single_cycle_routine(struct work_struct *work)
{
	struct aw86907 *aw86907 =
		container_of(work, struct aw86907, rtp_single_cycle_work);
	const struct firmware *rtp_file;
	int ret = -1;
	unsigned int buf_len = 0;
	unsigned char reg_val = 0;
	//  unsigned int  pass_cont=1;
	unsigned int cyc_cont = 150;
	aw86907->rtp_cnt = 0;
	aw86907->osc_cali_flag = 1;

	pr_info("%s enter\n", __func__);
	printk("%s---%d\n", __func__, __LINE__);
	/* fw loaded */
	if (aw86907->rtp_loop == 0xFF) {
		ret = request_firmware(&rtp_file,
				       aw86907_rtp_name[aw86907->rtp_serial[1]],
				       aw86907->dev);
	} else {
		printk("%s A single cycle : err value\n", __func__);
	}
	if (ret < 0) {
		pr_err("%s: failed to read %s\n", __func__,
		       aw86907_rtp_name[aw86907->rtp_serial[1]]);
		return;
	}
	aw86907->rtp_init = 0;
#ifndef OPLUS_FEATURE_CHG_BASIC
	kfree(aw86907_rtp);
	printk("%s---%d\n", __func__, __LINE__);
	aw86907_rtp = kzalloc(rtp_file->size + sizeof(int), GFP_KERNEL);
	if (!aw86907_rtp) {
		release_firmware(rtp_file);
		pr_err("%s: error allocating memory\n", __func__);
		return;
	}
#else
	ret = aw86907_container_init(rtp_file->size + sizeof(int));
	if (ret < 0) {
		release_firmware(rtp_file);
		pr_err("%s: error allocating memory\n", __func__);
		return;
	}
#endif
	aw86907_rtp->len = rtp_file->size;
	pr_info("%s: rtp file [%s] size = %d\n", __func__,
		aw86907_rtp_name[aw86907->rtp_serial[1]], aw86907_rtp->len);
	memcpy(aw86907_rtp->data, rtp_file->data, rtp_file->size);
	printk("%s---%d\n", __func__, __LINE__);
	release_firmware(rtp_file);

	//aw86907->rtp_init = 1; //Don't enter aw86907_irq,because osc calibration use while(1) function
	/* gain */
	aw86907_haptic_ram_vbat_comp(aw86907, false);
	printk("%s---%d\n", __func__, __LINE__);
	/* rtp mode config */
	aw86907_haptic_play_mode(aw86907, AW86907_HAPTIC_RTP_MODE);

	aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSCTRL7,
			       AW86907_BIT_SYSCTRL7_INT_EDGE_MODE_MASK,
			       AW86907_BIT_SYSCTRL7_INT_MODE_EDGE);
	/* haptic start */
	aw86907_haptic_start(aw86907);

	while (aw86907->rtp_cycle_flag == 1) {
		if (!aw86907_haptic_rtp_get_fifo_afs(aw86907)) {
			if ((aw86907_rtp->len - aw86907->rtp_cnt) <
			    (aw86907->ram.base_addr >> 2)) {
				buf_len = aw86907_rtp->len - aw86907->rtp_cnt;
			} else {
				buf_len = (aw86907->ram.base_addr >> 2);
			}

			aw86907_i2c_writes(aw86907, AW86907_REG_RTPDATA,
					   &aw86907_rtp->data[aw86907->rtp_cnt],
					   buf_len);
			aw86907->rtp_cnt += buf_len;

			if (aw86907->rtp_cnt == aw86907_rtp->len) {
				//  pr_info("%s: rtp update complete,enter again\n", __func__);
				aw86907->rtp_cnt = 0;
			}
		} else {
			aw86907_i2c_read(aw86907, AW86907_REG_SYSINT, &reg_val);
			if (reg_val & AW86907_BIT_SYSST_DONES) {
				pr_info("%s chip playback done\n", __func__);
				break;
			}
			while (1) {
				if (aw86907_haptic_rtp_get_fifo_aei(aw86907)) {
					printk("-----%s---%d----while(1)--\n",
					       __func__, __LINE__);
					break;
				}
				cyc_cont--;
				if (cyc_cont == 0) {
					cyc_cont = 150;
					break;
				}
			}
		} /*else*/
	} /*while*/
	pr_info("%s exit\n", __func__);
}

static void aw86907_rtp_regroup_routine(struct work_struct *work)
{
	struct aw86907 *aw86907 =
		container_of(work, struct aw86907, rtp_regroup_work);
	const struct firmware *rtp_file;
	unsigned int buf_len = 0;
	unsigned char reg_val = 0;
	unsigned int cyc_cont = 150;
	int rtp_len_tmp = 0;
	int aw86907_rtp_len = 0;
	int i, ret = 0;
	unsigned char *p = NULL;
	aw86907->rtp_cnt = 0;
	aw86907->osc_cali_flag = 1;
	pr_info("%s enter\n", __func__);

	for (i = 1; i <= aw86907->rtp_serial[0]; i++) {
		if ((request_firmware(&rtp_file,
				      aw86907_rtp_name[aw86907->rtp_serial[i]],
				      aw86907->dev)) < 0) {
			pr_err("%s: failed to read %s\n", __func__,
			       aw86907_rtp_name[aw86907->rtp_serial[i]]);
		}
		aw86907_rtp_len = rtp_len_tmp + rtp_file->size;
		rtp_len_tmp = aw86907_rtp_len;
		pr_info("%s:111 rtp file [%s] size = %d\n", __func__,
			aw86907_rtp_name[aw86907->rtp_serial[i]],
			aw86907_rtp_len);
		release_firmware(rtp_file);
	}

	rtp_len_tmp = 0;
	aw86907->rtp_init = 0;
#ifndef OPLUS_FEATURE_CHG_BASIC
	kfree(aw86907_rtp);
	aw86907_rtp = kzalloc(aw86907_rtp_len + sizeof(int), GFP_KERNEL);
	if (!aw86907_rtp) {
		pr_err("%s: error allocating memory\n", __func__);
		return;
	}
#else
	ret = aw86907_container_init(aw86907_rtp_len + sizeof(int));
	if (ret < 0) {
		pr_err("%s: error allocating memory\n", __func__);
		return;
	}
#endif
	aw86907_rtp->len = aw86907_rtp_len;
	for (i = 1; i <= aw86907->rtp_serial[0]; i++) {
		if ((request_firmware(&rtp_file,
				      aw86907_rtp_name[aw86907->rtp_serial[i]],
				      aw86907->dev)) < 0) {
			pr_err("%s: failed to read %s\n", __func__,
			       aw86907_rtp_name[aw86907->rtp_serial[i]]);
		}
		p = &(aw86907_rtp->data[0]) + rtp_len_tmp;
		memcpy(p, rtp_file->data, rtp_file->size);
		rtp_len_tmp += rtp_file->size;
		release_firmware(rtp_file);
		pr_info("%s: rtp file [%s]\n", __func__,
			aw86907_rtp_name[aw86907->rtp_serial[i]]);
	}

	//for(j=0; j<aw86907_rtp_len; j++) {
	//    printk("%s: addr:%d, data:%d\n", __func__, j, aw86907_rtp->data[j]);
	//   }
	//aw86907->rtp_init = 1; //Don't enter aw86907_irq,because osc calibration use while(1) function
	/* gain */
	aw86907_haptic_ram_vbat_comp(aw86907, false);
	/* rtp mode config */
	aw86907_haptic_play_mode(aw86907, AW86907_HAPTIC_RTP_MODE);
	aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSCTRL7,
			       AW86907_BIT_SYSCTRL7_INT_EDGE_MODE_MASK,
			       AW86907_BIT_SYSCTRL7_INT_MODE_EDGE);
	/* haptic start */
	aw86907_haptic_start(aw86907);

	while (aw86907->rtp_cycle_flag == 1) {
		if (!aw86907_haptic_rtp_get_fifo_afs(aw86907)) {
			if ((aw86907_rtp->len - aw86907->rtp_cnt) <
			    (aw86907->ram.base_addr >> 2)) {
				buf_len = aw86907_rtp->len - aw86907->rtp_cnt;
			} else {
				buf_len = (aw86907->ram.base_addr >> 2);
			}
			aw86907_i2c_writes(aw86907, AW86907_REG_RTPDATA,
					   &aw86907_rtp->data[aw86907->rtp_cnt],
					   buf_len);
			aw86907->rtp_cnt += buf_len;
			if (aw86907->rtp_cnt == aw86907_rtp->len) {
				pr_info("%s: rtp update complete\n", __func__);
				aw86907->rtp_cnt = 0;
				aw86907->rtp_loop--;
				if (aw86907->rtp_loop == 0)
					return;
			}
		} else {
			aw86907_i2c_read(aw86907, AW86907_REG_SYSINT, &reg_val);
			if (reg_val & AW86907_BIT_SYSST_DONES) {
				pr_info("%s chip playback done\n", __func__);
				break;
			}
			while (1) {
				if (aw86907_haptic_rtp_get_fifo_aei(aw86907)) {
					printk("-----%s---%d----while(1)--\n",
					       __func__, __LINE__);
					break;
				}
				cyc_cont--;
				if (cyc_cont == 0) {
					cyc_cont = 150;
					break;
				}
			}
		} /*else*/
	} /*while*/
	pr_info("%s exit\n", __func__);
}

static int aw86907_rtp_regroup_work(struct aw86907 *aw86907)
{
	aw86907_haptic_stop(aw86907);
	if (aw86907->rtp_serial[0] > 0) {
		printk("%s---%d\n", __func__, __LINE__);
		aw86907_haptic_set_rtp_aei(aw86907, false);
		aw86907_interrupt_clear(aw86907);
		if (aw86907->rtp_serial[0] <=
		    (sizeof(aw86907_rtp_name) / AW86907_RTP_NAME_MAX)) {
			if (aw86907->rtp_loop ==
			    0xFF) { //if aw86907->rtp_loop = 0xff then  single cycle ;
				aw86907->rtp_cycle_flag = 1;
				schedule_work(&aw86907->rtp_single_cycle_work);
			} else if ((aw86907->rtp_loop > 0) &&
				   (aw86907->rtp_loop < 0xff)) {
				aw86907->rtp_cycle_flag = 1;
				schedule_work(&aw86907->rtp_regroup_work);
			} else {
				printk("%s---%d\n", __func__, __LINE__);
			}
		}
	} else {
		aw86907->rtp_cycle_flag = 0;
	}
	return 0;
}

/*****************************************************
 *
 * haptic - audio
 *
 *****************************************************/
static int aw86907_haptic_audio_uevent_report_scan(struct aw86907 *aw86907,
						   bool flag)
{
#ifdef AISCAN_CTRL
	char *envp[2];

	pr_info("%s: flag=%d\n", __func__, flag);

	if (flag) {
		envp[0] = "AI_SCAN=1";
	} else {
		envp[0] = "AI_SCAN=0";
	}

	envp[1] = NULL;

	kobject_uevent_env(&aw86907->dev->kobj, KOBJ_CHANGE, envp);
#endif
	return 0;
}

static int
aw86907_haptic_audio_tp_list_match(struct shake_point *pt_info,
				   struct haptic_audio_trust_zone *p_tmp)
{
	if ((pt_info->x + AW86907_HAPTIC_AI_X_JITTER * p_tmp->w /
				  AW86907_HAPTIC_AI_X_DFT_W >
	     p_tmp->x) &&
	    (pt_info->x < p_tmp->x + p_tmp->w +
				  AW86907_HAPTIC_AI_X_JITTER * p_tmp->w /
					  AW86907_HAPTIC_AI_X_DFT_W) &&
	    (pt_info->y + AW86907_HAPTIC_AI_Y_JITTER * p_tmp->h /
				  AW86907_HAPTIC_AI_Y_DFT_H >
	     p_tmp->y) &&
	    (pt_info->y < p_tmp->y + p_tmp->h +
				  AW86907_HAPTIC_AI_Y_JITTER * p_tmp->h /
					  AW86907_HAPTIC_AI_Y_DFT_H)) {
		return 1;
	} else {
		return 0;
	}
}

static int
aw86907_haptic_audio_tz_list_match(struct haptic_audio_trust_zone *p_tmp,
				   struct trust_zone_info *p_tmp_new)
{
	int match = 0;
	int deta_x = 0;
	int deta_y = 0;
	int deta_x_w = 0;
	int deta_y_h = 0;
	int h = (p_tmp->h > p_tmp_new->h) ? p_tmp_new->h : p_tmp->h;
	int w = (p_tmp->w > p_tmp_new->w) ? p_tmp_new->w : p_tmp->w;

	deta_x = p_tmp->x - p_tmp_new->x;
	deta_y = p_tmp->y - p_tmp_new->y;
	deta_x_w = (p_tmp->x + p_tmp->w) - (p_tmp_new->x + p_tmp_new->w);
	deta_y_h = (p_tmp->y + p_tmp->h) - (p_tmp_new->y + p_tmp_new->h);

	if ((ABS(deta_x) <
	     AW86907_HAPTIC_AI_X_JITTER * w / AW86907_HAPTIC_AI_X_DFT_W) &&
	    (ABS(deta_y) <
	     AW86907_HAPTIC_AI_X_JITTER * h / AW86907_HAPTIC_AI_Y_DFT_H) &&
	    (ABS(deta_y_h) <
	     AW86907_HAPTIC_AI_X_JITTER * h / AW86907_HAPTIC_AI_Y_DFT_H) &&
	    (ABS(deta_x_w) <
	     AW86907_HAPTIC_AI_X_JITTER * w / AW86907_HAPTIC_AI_X_DFT_W)) {
		match = 1;
	} else {
		match = 0;
	}
	return match;
}

static int aw86907_haptic_audio_tz_list_show(struct haptic_audio *haptic_audio)
{
	struct haptic_audio_trust_zone *p_tmp = NULL;
	unsigned int i = 0;

	list_for_each_entry (p_tmp, &haptic_audio->list, list) {
		pr_info("%s: tz[%02d]: [%01d, %04d, %04d, %04d, %04d]\n",
			__func__, i, p_tmp->level, p_tmp->x, p_tmp->y, p_tmp->w,
			p_tmp->h);
		i++;
	}
	return 0;
}

static int
aw86907_haptic_audio_tz_list_insert(struct haptic_audio *haptic_audio,
				    struct trust_zone_info *tz_info)
{
	struct haptic_audio_trust_zone *p_new = NULL;

	pr_info("%s: enter\n", __func__);

	p_new = (struct haptic_audio_trust_zone *)kzalloc(
		sizeof(struct haptic_audio_trust_zone), GFP_KERNEL);
	if (p_new == NULL) {
		pr_err("%s: kzalloc memory fail\n", __func__);
		return -1;
	}

	/* update new list info */
	p_new->level = tz_info->level;
	p_new->cnt = haptic_audio->tz_cnt_thr * 2;
	p_new->x = tz_info->x;
	p_new->y = tz_info->y;
	p_new->w = tz_info->w;
	p_new->h = tz_info->h;
	p_new->dirty = 0;

	INIT_LIST_HEAD(&(p_new->list));
	//list_add(&(p_new->list), &(haptic_audio->list));
	if (s_tz_init) {
		list_add(&(p_new->list), &(haptic_audio->score_list));
		haptic_audio->tz_num = haptic_audio->tz_num + 1;
	} else
		list_add(&(p_new->list), &(haptic_audio->list));

	return 0;
}

static int aw86907_haptic_audio_tz_list_clear(struct haptic_audio *haptic_audio)
{
	struct haptic_audio_trust_zone *p_tmp = NULL;
	struct haptic_audio_trust_zone *p_tmp_bak = NULL;

	list_for_each_entry_safe (p_tmp, p_tmp_bak, &(haptic_audio->list),
				  list) {
		list_del(&p_tmp->list);
		kfree(p_tmp);
	}

	list_for_each_entry_safe (p_tmp, p_tmp_bak, &(haptic_audio->score_list),
				  list) {
		list_del(&p_tmp->list);
		kfree(p_tmp);
	}

	return 0;
}

/*
static int aw86907_haptic_audio_ctr_list_show(struct haptic_audio *haptic_audio)
{
    struct haptic_ctr *p_tmp = NULL;
    unsigned int i = 0;

    list_for_each_entry(p_tmp, &haptic_audio->ctr_list, list) {
        pr_info("%s: ctr[%02d]: [%03d, %03d, %03d, %03d, %03d]\n",
            __func__, i, p_tmp->cmd, p_tmp->play,
            p_tmp->wavseq, p_tmp->loop, p_tmp->gain);
        i++;

    }
    return 0;
}
*/

static int
aw86907_haptic_audio_ctr_list_insert(struct haptic_audio *haptic_audio,
				     struct haptic_ctr *haptic_ctr)
{
	struct haptic_ctr *p_new = NULL;

	p_new = (struct haptic_ctr *)kzalloc(sizeof(struct haptic_ctr),
					     GFP_KERNEL);
	if (p_new == NULL) {
		pr_err("%s: kzalloc memory fail\n", __func__);
		return -1;
	}
	/* update new list info */
	p_new->cnt = haptic_ctr->cnt;
	p_new->cmd = haptic_ctr->cmd;
	p_new->play = haptic_ctr->play;
	p_new->wavseq = haptic_ctr->wavseq;
	p_new->loop = haptic_ctr->loop;
	p_new->gain = haptic_ctr->gain;

	INIT_LIST_HEAD(&(p_new->list));
	list_add(&(p_new->list), &(haptic_audio->ctr_list));

	return 0;
}

static int
aw86907_haptic_audio_ctr_list_clear(struct haptic_audio *haptic_audio)
{
	struct haptic_ctr *p_ctr = NULL;
	struct haptic_ctr *p_ctr_bak = NULL;

	list_for_each_entry_safe_reverse (p_ctr, p_ctr_bak,
					  &(haptic_audio->ctr_list), list) {
		list_del(&p_ctr->list);
		kfree(p_ctr);
	}

	return 0;
}
/*
static int aw86907_fb_notifier_callback_tp(struct notifier_block *self, unsigned long event, void *data)
{
    struct aw86907 *aw86907 = container_of(self, struct aw86907, fb_notif);
    struct tp *tp = &(aw86907->haptic_audio.tp);
    int i = 0;
    int *blank;
    struct fb_event *evdata = data;
    struct haptic_audio *haptic_audio = NULL;
    struct haptic_audio_trust_zone *p_tmp = NULL;

    haptic_audio = &(aw86907->haptic_audio);

    blank = evdata->data;
    pr_debug("%s: tp event = %ld, blank = %d\n", __func__, event, *blank);
    i= *blank;
#if 0
    if (event == 11) {
        pr_debug("%s: tp down\n", __func__);
            pr_debug("%s: tp record_point_down[%d].status = %d\n", __func__, i, record_point[i].status);
            if((AW86907_HAPTIC_TP_ST_PRESS == record_point[i].status) &&
                (record_point[i].status != tp->id[i].pt_info.status)) {
                tp->id[i].pt_info.status = record_point[i].status;
                tp->id[i].tp_flag = AW86907_HAPTIC_TP_PRESS;
                tp->id[i].press_flag = AW86907_HAPTIC_TP_PRESS;
                tp->id[i].release_flag = AW86907_HAPTIC_TP_NULL;
                tp->id[i].no_play_cnt = 0;
                do_gettimeofday(&tp->id[i].t_press);
                pr_info("%s: tp_press_release: status=%d, flag=%d", __func__, tp->id[i].pt_info.status, tp->id[i].tp_flag);
            }
    }
    if (event == 10) {
        pr_debug("%s: tp up\n", __func__);
            pr_debug("%s: tp record_point_up[%d].status = %d\n", __func__, i, record_point[i].status);
            if((AW86907_HAPTIC_TP_ST_RELEASE == record_point[i].status) &&
                (record_point[i].status != tp->id[i].pt_info.status)) {
                tp->id[i].pt_info.status = record_point[i].status;
                tp->id[i].tp_flag = AW86907_HAPTIC_TP_RELEASE;
                tp->id[i].release_flag = AW86907_HAPTIC_TP_RELEASE;
                do_gettimeofday(&tp->id[i].t_release);
                pr_info("%s: tp_press_release: status=%d, flag=%d", __func__, tp->id[i].pt_info.status, tp->id[i].tp_flag);
            }
    }
#endif
    if (event == 0) {
        list_for_each_entry(p_tmp, &(haptic_audio->list), list) {
            if ((tp->id[i].pt_info.x+AW86907_HAPTIC_AI_X_JITTER*p_tmp->w/AW86907_HAPTIC_AI_X_DFT_W > p_tmp->x) &&
                (tp->id[i].pt_info.x < p_tmp->x+p_tmp->w+AW86907_HAPTIC_AI_X_JITTER*p_tmp->w/AW86907_HAPTIC_AI_X_DFT_W) &&
                (tp->id[i].pt_info.y+AW86907_HAPTIC_AI_Y_JITTER*p_tmp->h/AW86907_HAPTIC_AI_Y_DFT_H > p_tmp->y) &&
                (tp->id[i].pt_info.y < p_tmp->y+p_tmp->h+AW86907_HAPTIC_AI_Y_JITTER*p_tmp->h/AW86907_HAPTIC_AI_Y_DFT_H)) {
                pr_debug("%s: tp input point[%d, %04d, %04d] up is in the ai trust zone[%d, %04d, %04d, %04d, %04d]\n",
                    __func__, tp->id[i].pt_info.id, tp->id[i].pt_info.x, tp->id[i].pt_info.y,
                    p_tmp->level, p_tmp->x, p_tmp->y, p_tmp->w, p_tmp->h);
                if(tp->virtual_id == i) {
                    tp->id[AW86907_HAPTIC_TP_ID_MAX].pt_info.id = tp->id[i].pt_info.id;
                    tp->id[AW86907_HAPTIC_TP_ID_MAX].pt_info.x = tp->id[i].pt_info.x;
                    tp->id[AW86907_HAPTIC_TP_ID_MAX].pt_info.y = tp->id[i].pt_info.y;
                    tp->id[AW86907_HAPTIC_TP_ID_MAX].pt_info.status = AW86907_HAPTIC_TP_ST_RELEASE;
                    tp->id[AW86907_HAPTIC_TP_ID_MAX].tp_flag = AW86907_HAPTIC_TP_RELEASE;
                    tp->id[AW86907_HAPTIC_TP_ID_MAX].release_flag = AW86907_HAPTIC_TP_RELEASE;
                    do_gettimeofday(&tp->id[AW86907_HAPTIC_TP_ID_MAX].t_release);
                }
                break;
            }
        }
        tp->id[i].pt_info.status = AW86907_HAPTIC_TP_ST_RELEASE;
        tp->id[i].tp_flag = AW86907_HAPTIC_TP_RELEASE;
        tp->id[i].release_flag = AW86907_HAPTIC_TP_RELEASE;
        do_gettimeofday(&tp->id[i].t_release);

        pr_debug("%s: tp input point[%d, %04d, %04d] up to [%d, %04d, %04d] \n",
            __func__, i, tp->id[i].pt_info.x, tp->id[i].pt_info.y,
            i, tp->id[i].pt_info.x, tp->id[i].pt_info.y);
    }

    return 0;
}
*/
#ifdef OP_AW_DEBUG
static int aw86907_haptic_audio_init(struct aw86907 *aw86907)
{
	unsigned int i = 0;

	pr_debug("%s enter\n", __func__);

	aw86907_haptic_set_wav_seq(aw86907, 0x01, 0x00);

	for (i = 0; i < AW86907_HAPTIC_TP_ID_MAX + 1; i++) {
		aw86907->haptic_audio.tp.id[i].tp_flag = AW86907_HAPTIC_TP_NULL;
		aw86907->haptic_audio.tp.id[i].press_flag =
			AW86907_HAPTIC_TP_NULL;
		aw86907->haptic_audio.tp.id[i].release_flag =
			AW86907_HAPTIC_TP_NULL;
		aw86907->haptic_audio.tp.id[i].play_flag =
			AW86907_HAPTIC_TP_PLAY_NULL;
		aw86907->haptic_audio.tp.id[i].tp_ai_match_flag = 1;
		aw86907->haptic_audio.tp.id[i].no_play_cnt = 0;
		aw86907->haptic_audio.tp.id[i].pt_info.status = 0;
	}
	aw86907->haptic_audio.tp.hap_match_without_tz_cnt = 0;
	aw86907->haptic_audio.uevent_report_flag = 0;
	aw86907->haptic_audio.hap_cnt_outside_tz = 0;
	return 0;
}

static int aw86907_haptic_audio_off(struct aw86907 *aw86907)
{
	pr_debug("%s enter\n", __func__);
	mutex_lock(&aw86907->lock);
	aw86907_haptic_set_gain(aw86907, 0x80);
	aw86907_haptic_stop(aw86907);
	mutex_unlock(&aw86907->lock);

	//mutex_lock(&aw86907->haptic_audio.lock);
	s_tz_init = 0;
	aw86907->gun_type = 0xff;
	aw86907->bullet_nr = 0;
	aw86907_haptic_audio_ctr_list_clear(&aw86907->haptic_audio);
	aw86907_haptic_audio_tz_list_clear(&aw86907->haptic_audio);
	//mutex_unlock(&aw86907->haptic_audio.lock);

	return 0;
}

static int aw86907_haptic_audio_stop(struct aw86907 *aw86907)
{
	pr_debug("%s enter\n", __func__);
#ifdef OP_AW_DEBUG
	do_gettimeofday(&aw86907->t_stop);
#endif

	aw86907_haptic_stop(aw86907);
	return 0;
}

static int aw86907_haptic_audio_play_cfg(struct aw86907 *aw86907)
{
	pr_debug("%s enter\n", __func__);

	aw86907->play_mode = AW86907_HAPTIC_RAM_MODE;
	aw86907_i2c_write_bits(aw86907, AW86907_REG_PLAYCFG3,
			       AW86907_BIT_PLAYCFG3_PLAY_MODE_MASK,
			       AW86907_BIT_PLAYCFG3_PLAY_MODE_RAM);

	aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSINTM,
			       AW86907_BIT_SYSINTM_UVLM_MASK,
			       AW86907_BIT_SYSINTM_UVLM_OFF);

	aw86907_i2c_write_bits(aw86907, AW86907_REG_PLAYCFG1,
			       AW86907_BIT_PLAYCFG1_BST_MODE_MASK,
			       AW86907_BIT_PLAYCFG1_BST_MODE_BYPASS);
	aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSCTRL2,
			       AW86907_BIT_SYSCTRL2_STANDBY_MASK,
			       AW86907_BIT_SYSCTRL2_STANDBY_OFF);
	aw86907_i2c_write_bits(aw86907, AW86907_REG_PLAYCFG1,
			       AW86907_BIT_PLAYCFG1_BST_MODE_MASK,
			       AW86907_BIT_PLAYCFG1_BST_MODE_BOOST);

	return 0;
}
#endif

/*****************************************************
 *
 * haptic - audio
 *
 *****************************************************/
static enum hrtimer_restart
aw86907_haptic_audio_timer_func(struct hrtimer *timer)
{
	struct aw86907 *aw86907 =
		container_of(timer, struct aw86907, haptic_audio.timer);

	pr_debug("%s enter\n", __func__);
	schedule_work(&aw86907->haptic_audio.work);

	hrtimer_start(
		&aw86907->haptic_audio.timer,
		ktime_set(aw86907->haptic_audio.timer_val / 1000000,
			  (aw86907->haptic_audio.timer_val % 1000000) * 1000),
		HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static void aw86907_haptic_audio_work_routine(struct work_struct *work)
{
	struct aw86907 *aw86907 =
		container_of(work, struct aw86907, haptic_audio.work);
	struct tp *tp = &(aw86907->haptic_audio.tp);
	struct haptic_audio *haptic_audio = NULL;
	struct haptic_audio_trust_zone *p_tmp = NULL;
	struct haptic_audio_trust_zone *p_tmp_bak = NULL;
	struct haptic_audio_trust_zone *p_tmp_r = NULL;
	struct haptic_audio_trust_zone *p_tmp_l = NULL;
	struct haptic_ctr *p_ctr = NULL;
	struct haptic_ctr *p_ctr_bak = NULL;
	struct timeval tmp_time;
	unsigned int press_delay = 0;
	unsigned int release_delay = 0;
	unsigned int tp_touch_play_flag = 0;
	unsigned int i = 0, j = 0;
	unsigned int touch_vibrator_id = 0;
	//  unsigned int score_touch_vibrator_id = 0;
	//  unsigned int touch_vibrator_flag = 0;
	//  unsigned int tz_num = 0;
	unsigned int uevent_report_flag = 0;
	unsigned int ctr_list_flag = 0;
	unsigned int ctr_list_input_cnt = 0;
	unsigned int ctr_list_output_cnt = 0;
	unsigned int ctr_list_diff_cnt = 0;
	unsigned int ctr_list_del_cnt = 0;
	unsigned int tp_ai_match_id = 0;
	unsigned int x_thr = aw86907->haptic_audio.tp_size.x >> 1;

	/*OP add for juge rtp on begin*/
	//int rtp_is_going_on = 0;
	//int match_re = 0, dirty = 0;

	pr_debug("%s enter\n", __func__);
	/*OP add for juge rtp on end*/

	haptic_audio = &(aw86907->haptic_audio);

	mutex_lock(&aw86907->haptic_audio.lock);
	memset(&aw86907->haptic_audio.ctr, 0, sizeof(struct haptic_ctr));
	ctr_list_flag = 0;
	list_for_each_entry_safe_reverse (p_ctr, p_ctr_bak,
					  &(haptic_audio->ctr_list), list) {
		ctr_list_flag = 1;
		break;
	}
	if (ctr_list_flag == 0) {
		pr_info("%s: ctr list empty\n", __func__);
	}
	if (ctr_list_flag == 1) {
		list_for_each_entry_safe (p_ctr, p_ctr_bak,
					  &(haptic_audio->ctr_list), list) {
			ctr_list_input_cnt = p_ctr->cnt;
			break;
		}
		list_for_each_entry_safe_reverse (
			p_ctr, p_ctr_bak, &(haptic_audio->ctr_list), list) {
			ctr_list_output_cnt = p_ctr->cnt;
			break;
		}
		if (ctr_list_input_cnt > ctr_list_output_cnt) {
			ctr_list_diff_cnt =
				ctr_list_input_cnt - ctr_list_output_cnt;
		}
		if (ctr_list_input_cnt < ctr_list_output_cnt) {
			ctr_list_diff_cnt =
				32 + ctr_list_input_cnt - ctr_list_output_cnt;
		}
		if (ctr_list_diff_cnt > 2) {
			list_for_each_entry_safe_reverse (
				p_ctr, p_ctr_bak, &(haptic_audio->ctr_list),
				list) {
				if ((p_ctr->play == 0) &&
				    (AW86907_HAPTIC_CMD_ENABLE ==
				     (AW86907_HAPTIC_CMD_HAPTIC & p_ctr->cmd))) {
					list_del(&p_ctr->list);
					kfree(p_ctr);
					ctr_list_del_cnt++;
				}
				if (ctr_list_del_cnt == ctr_list_diff_cnt) {
					break;
				}
			}
		}
	}

	/* get the last data from list */
	list_for_each_entry_safe_reverse (p_ctr, p_ctr_bak,
					  &(haptic_audio->ctr_list), list) {
		aw86907->haptic_audio.ctr.cnt = p_ctr->cnt;
		aw86907->haptic_audio.ctr.cmd = p_ctr->cmd;
		aw86907->haptic_audio.ctr.play = p_ctr->play;
		aw86907->haptic_audio.ctr.wavseq = p_ctr->wavseq;
		aw86907->haptic_audio.ctr.loop = p_ctr->loop;
		aw86907->haptic_audio.ctr.gain = p_ctr->gain;
		list_del(&p_ctr->list);
		kfree(p_ctr);
		break;
	}
	if (aw86907->haptic_audio.ctr.play) {
		pr_debug(
			"%s: cnt=%d, cmd=%d, play=%d, wavseq=%d, loop=%d, gain=%d\n",
			__func__, aw86907->haptic_audio.ctr.cnt,
			aw86907->haptic_audio.ctr.cmd,
			aw86907->haptic_audio.ctr.play,
			aw86907->haptic_audio.ctr.wavseq,
			aw86907->haptic_audio.ctr.loop,
			aw86907->haptic_audio.ctr.gain);
	}

	/* rtp mode jump */
	//rtp_is_going_on = aw86907_haptic_juge_RTP_is_going_on(aw86907);
	//if (rtp_is_going_on) {
	if (aw86907->rtp_routine_on) {
		mutex_unlock(&aw86907->haptic_audio.lock);
		return;
	}
	/* haptic play with tp adjust info */
	if (AW86907_HAPTIC_CMD_TP ==
	    (aw86907->haptic_audio.ctr.cmd & AW86907_HAPTIC_CMD_SYS)) {
		do_gettimeofday(&tmp_time);
		for (i = 0; i < AW86907_HAPTIC_TP_ID_MAX + 1; i++) {
			/* get tp press delay and tp release delay */
			press_delay = 0;
			release_delay = 0;
			if (AW86907_HAPTIC_TP_PRESS == tp->id[i].press_flag) {
				press_delay = (tmp_time.tv_sec -
					       tp->id[i].t_press.tv_sec) *
						      1000000 +
					      (tmp_time.tv_usec -
					       tp->id[i].t_press.tv_usec);
			}
			if (AW86907_HAPTIC_TP_RELEASE ==
			    tp->id[i].release_flag) {
				release_delay = (tmp_time.tv_sec -
						 tp->id[i].t_release.tv_sec) *
							1000000 +
						(tmp_time.tv_usec -
						 tp->id[i].t_release.tv_usec);
			}
			if (tp->id[i].press_flag || tp->id[i].release_flag) {
				pr_debug(
					"%s: id[%d]: press_flag=%d, press_delay=%dus, release_flag=%d, release_delaly=%dus\n",
					__func__, i, tp->id[i].press_flag,
					press_delay, tp->id[i].release_flag,
					release_delay);
			}
			/* adjust tp play with tp press delay and tp release delay */
			if (AW86907_HAPTIC_PLAY_ENABLE ==
			    aw86907->haptic_audio.ctr.play) {
				if (AW86907_HAPTIC_TP_PRESS ==
				    tp->id[i].press_flag) {
					if (AW86907_HAPTIC_TP_PLAY_NULL ==
					    tp->id[i].play_flag) {
						if (press_delay >
						    tp->press_delay_max) {
							/* no play time > tp-play delay max time */
							tp->id[i].play_flag =
								AW86907_HAPTIC_TP_PLAY_NOMORE;
							tp->id[i].press_flag =
								AW86907_HAPTIC_TP_NULL;
							tp->id[i].release_flag =
								AW86907_HAPTIC_TP_NULL;
							if (i ==
							    AW86907_HAPTIC_TP_ID_MAX) {
								tp->id[i]
									.pt_info
									.status =
									AW86907_HAPTIC_TP_ST_RELEASE;
								tp->id[i]
									.pt_info
									.touch_flag =
									AW86907_HAPTIC_TP_TOUCH_INVAIL;
								tp->id[i]
									.tp_flag =
									AW86907_HAPTIC_TP_NULL;
								tp->id[i]
									.press_flag =
									AW86907_HAPTIC_TP_NULL;
								tp->id[i]
									.release_flag =
									AW86907_HAPTIC_TP_NULL;
								tp->id[i]
									.no_play_cnt =
									0;
								//tp->id[i].press_no_vibrate_flag = 0;
							}
							if (tp->id[i].press_no_vibrate_flag ==
							    0)
								tp->id[i]
									.press_no_vibrate_flag =
									2;
						} else {
							if (press_delay >
							    tp->press_delay_min) {
								/* tp-play delay min time < no play time < tp-play delay max time */
								tp->id[i]
									.play_flag =
									AW86907_HAPTIC_TP_PLAY_ENABLE;
								tp->id[i]
									.press_no_vibrate_flag =
									1;
							} else {
								/* no play time < tp-play delay min time */
								tp->id[i]
									.play_flag =
									AW86907_HAPTIC_TP_PLAY_NOMORE;
								if (tp->id[i].press_no_vibrate_flag ==
								    0)
									tp->id[i]
										.press_no_vibrate_flag =
										2;
							}
						}
					} else if (AW86907_HAPTIC_TP_PLAY_ENABLE ==
						   tp->id[i].play_flag) {
					}
					pr_debug(
						"%s: %d: id[%d]:play_flag=%d, press_flag=%d, release_flag=%d, press_no_vibrate_flag=%d, no_play_cnt=%d\n",
						__func__, __LINE__, i,
						tp->id[i].play_flag,
						tp->id[i].press_flag,
						tp->id[i].release_flag,
						tp->id[i].press_no_vibrate_flag,
						tp->id[i].no_play_cnt);
				}
				if (AW86907_HAPTIC_TP_RELEASE ==
				    tp->id[i].release_flag) {
					if (release_delay >
					    tp->release_delay_max) {
						/* tp release time > 3-continue-time play time*/
						if (AW86907_HAPTIC_TP_PLAY_ENABLE ==
						    tp->id[i].play_flag) {
							tp->id[i].play_flag =
								AW86907_HAPTIC_TP_PLAY_NOMORE;
							tp->id[i].press_flag =
								AW86907_HAPTIC_TP_NULL;
							tp->id[i].release_flag =
								AW86907_HAPTIC_TP_NULL;
							if (i ==
							    AW86907_HAPTIC_TP_ID_MAX) {
								tp->id[i]
									.pt_info
									.status =
									AW86907_HAPTIC_TP_ST_RELEASE;
								tp->id[i]
									.pt_info
									.touch_flag =
									AW86907_HAPTIC_TP_TOUCH_INVAIL;
								tp->id[i]
									.tp_flag =
									AW86907_HAPTIC_TP_NULL;
								tp->id[i]
									.press_flag =
									AW86907_HAPTIC_TP_NULL;
								tp->id[i]
									.release_flag =
									AW86907_HAPTIC_TP_NULL;
								tp->id[i]
									.no_play_cnt =
									0;
								//tp->id[i].press_no_vibrate_flag = 0;
							}
							if (tp->id[i].press_no_vibrate_flag ==
							    0)
								tp->id[i]
									.press_no_vibrate_flag =
									2;
						} else {
						}
					}
					pr_debug(
						"%s: %d: id[%d]:play_flag=%d, press_flag=%d, release_flag=%d, press_no_vibrate_flag=%d, no_play_cnt=%d\n",
						__func__, __LINE__, i,
						tp->id[i].play_flag,
						tp->id[i].press_flag,
						tp->id[i].release_flag,
						tp->id[i].press_no_vibrate_flag,
						tp->id[i].no_play_cnt);
				}
				tp->id[i].no_play_cnt = 0;
			} else if (AW86907_HAPTIC_PLAY_NULL ==
				   aw86907->haptic_audio.ctr.play) {
				if (AW86907_HAPTIC_TP_PRESS ==
				    tp->id[i].press_flag) {
					if (AW86907_HAPTIC_TP_PLAY_ENABLE ==
					    tp->id[i].play_flag) {
						tp->id[i].no_play_cnt++;
						if (tp->id[i].no_play_cnt >
						    tp->no_play_cnt_max) {
							/* no play cnt > play interal time */
							tp->id[i].no_play_cnt =
								tp->no_play_cnt_max;
							tp->id[i].play_flag =
								AW86907_HAPTIC_TP_PLAY_NULL;
							tp->id[i].press_flag =
								AW86907_HAPTIC_TP_NULL;
							tp->id[i].release_flag =
								AW86907_HAPTIC_TP_NULL;
							if (i ==
							    AW86907_HAPTIC_TP_ID_MAX) {
								tp->id[i]
									.pt_info
									.status =
									AW86907_HAPTIC_TP_ST_RELEASE;
								tp->id[i]
									.pt_info
									.touch_flag =
									AW86907_HAPTIC_TP_TOUCH_INVAIL;
								tp->id[i]
									.tp_flag =
									AW86907_HAPTIC_TP_NULL;
								tp->id[i]
									.press_flag =
									AW86907_HAPTIC_TP_NULL;
								tp->id[i]
									.release_flag =
									AW86907_HAPTIC_TP_NULL;
								//tp->id[i].press_no_vibrate_flag = 0;
							}
							if (tp->id[i].press_no_vibrate_flag ==
							    0)
								tp->id[i]
									.press_no_vibrate_flag =
									2;
						} else {
						}
					} else {
						tp->id[i].play_flag =
							AW86907_HAPTIC_TP_PLAY_NULL;
					}
					pr_debug(
						"%s: %d: id[%d]:play_flag=%d, press_flag=%d, release_flag=%d, press_no_vibrate_flag=%d, no_play_cnt=%d\n",
						__func__, __LINE__, i,
						tp->id[i].play_flag,
						tp->id[i].press_flag,
						tp->id[i].release_flag,
						tp->id[i].press_no_vibrate_flag,
						tp->id[i].no_play_cnt);
				}
				if (AW86907_HAPTIC_TP_RELEASE ==
				    tp->id[i].release_flag) {
					/* tp release time > 3-continue-time play time*/
					if (release_delay >
					    tp->release_delay_max) {
						tp->id[i].play_flag =
							AW86907_HAPTIC_TP_PLAY_NULL;
						tp->id[i].press_flag =
							AW86907_HAPTIC_TP_NULL;
						tp->id[i].release_flag =
							AW86907_HAPTIC_TP_NULL;
						tp->id[i].tp_ai_match_flag = 1;
						if (tp->id[i].press_no_vibrate_flag ==
						    0)
							tp->id[i]
								.press_no_vibrate_flag =
								2;
						tp->tp_ai_check_flag = 0;
					} else {
					}
					pr_debug(
						"%s: %d: id[%d]:play_flag=%d, press_flag=%d, release_flag=%d, press_no_vibrate_flag=%d, no_play_cnt=%d\n",
						__func__, __LINE__, i,
						tp->id[i].play_flag,
						tp->id[i].press_flag,
						tp->id[i].release_flag,
						tp->id[i].press_no_vibrate_flag,
						tp->id[i].no_play_cnt);
				}
			} else {
			}
		}

		/* adjust tp play enable */
		tp->play_flag = AW86907_HAPTIC_TP_PLAY_NULL;
		for (i = 0; i < AW86907_HAPTIC_TP_ID_MAX + 1; i++) {
			if (AW86907_HAPTIC_TP_PLAY_ENABLE ==
			    tp->id[i].play_flag) {
				tp_touch_play_flag =
					AW86907_HAPTIC_TP_PLAY_ENABLE;
				list_for_each_entry (
					p_tmp, &(haptic_audio->list), list) {
					//match_re = aw86907_haptic_audio_tp_list_match(&(tp->id[i].pt_info), p_tmp);
					if (aw86907_haptic_audio_tp_list_match(
						    &(tp->id[i].pt_info),
						    p_tmp)) {
						tp->play_flag =
							AW86907_HAPTIC_TP_PLAY_ENABLE;
						touch_vibrator_id = i;
						pr_debug(
							"%s: tp input point[%d, %04d, %04d] is in the ai trust zone[%d, %04d, %04d, %04d, %04d]\n",
							__func__, i,
							tp->id[i].pt_info.x,
							tp->id[i].pt_info.y,
							p_tmp->level, p_tmp->x,
							p_tmp->y, p_tmp->w,
							p_tmp->h);
						break;
					}
				}
			}
			if (tp->play_flag == AW86907_HAPTIC_TP_PLAY_ENABLE) {
				break;
			}
		}
		if (tp->play_flag) {
			pr_debug(
				"%s: tp.play_flag with tp =%d, touch_vibrator_id=%d\n",
				__func__, tp->play_flag, touch_vibrator_id);
		}

		/* haptic play cnt outside trust zone over limit, restart ai scan */
		tp->tp_ai_match_flag = 1;
		for (i = 0; i < AW86907_HAPTIC_TP_ID_MAX; i++) {
			pr_debug("%s: [%d] tp_ai_match_flag=%d\n", __func__, i,
				 tp->id[i].tp_ai_match_flag);
			if (tp->id[i].tp_ai_match_flag == 0) {
				tp->tp_ai_match_flag = 0;
				tp_ai_match_id = i;
				break;
			}
		}
		pr_debug(
			"%s: tp_ai_match_flag=%d, tp_ai_check_flag=%d, tz_high_num=%d, tp_touch_play_flag=%d, tp->last_play_flag=%d, hap_cnt_outside_tz=%d\n",
			__func__, tp->tp_ai_match_flag, tp->tp_ai_check_flag,
			haptic_audio->tz_high_num, tp_touch_play_flag,
			tp->last_play_flag, haptic_audio->hap_cnt_outside_tz);

		/* restart ai scan when tz_high_num=2 */
		if (/*(haptic_audio->tz_high_num == 2) &&*/
		    (tp->tp_ai_match_flag == 0) &&
		    (tp->tp_ai_check_flag == 1)) {
			if ((AW86907_HAPTIC_TP_PLAY_ENABLE ==
			     tp_touch_play_flag) &&
			    (AW86907_HAPTIC_TP_PLAY_NULL == tp->play_flag)) {
				//(AW86907_HAPTIC_TP_PLAY_NULL == tp->last_play_flag)) {
				haptic_audio->hap_cnt_outside_tz++;
				if (haptic_audio->hap_cnt_outside_tz >=
				    haptic_audio->hap_cnt_max_outside_tz) {
					uevent_report_flag = 1;
					if (haptic_audio->uevent_report_flag !=
					    uevent_report_flag) {
						pr_info("%s: haptic play cnt outside trust zone over limit, restart ai scan\n",
							__func__);
					}
					haptic_audio->hap_cnt_outside_tz =
						haptic_audio
							->hap_cnt_max_outside_tz;
				}
#ifdef FIRST_SCOREMODE
				//add by Jerry.Won
				i = 0;
				p_tmp_l = NULL;
				p_tmp_r = NULL;
				list_for_each_entry (
					p_tmp, &(haptic_audio->list), list) {
					if ((p_tmp->level >= TRUST_LEVEL) &&
					    (p_tmp_l == NULL)) {
						p_tmp_l = p_tmp;
					} else if ((p_tmp->level >=
						    TRUST_LEVEL) &&
						   (p_tmp_r == NULL)) {
						p_tmp_r = p_tmp;
					}
					i++;
				}

				if (p_tmp_l != NULL && p_tmp_r != NULL) {
					if (p_tmp_l->x > p_tmp_r->x) {
						p_tmp = p_tmp_r;
						p_tmp_r = p_tmp_l;
						p_tmp_l = p_tmp;
					}
				}

				pr_info("%s: haptic_audio->list : %d\n",
					__func__, i);

				for (j = 0; j < AW86907_HAPTIC_TP_ID_MAX + 1;
				     j++) {
					if (tp->id[j].play_flag) {
						list_for_each_entry (
							p_tmp,
							&(haptic_audio
								  ->score_list),
							list) {
							if (aw86907_haptic_audio_tp_list_match(
								    &(tp->id[j]
									      .pt_info),
								    p_tmp)) {
								//p_tmp->level = (p_tmp->level < TRUST_LEVEL) ? (++p_tmp->level) : TRUST_LEVEL;
								p_tmp->level++;
								if ((p_tmp->level) >
								    TRUST_LEVEL) {
									p_tmp->level =
										TRUST_LEVEL;
								}
								break;
							}
						}
					}
				}

				j = 0;
				list_for_each_entry_safe (
					p_tmp, p_tmp_bak,
					&(haptic_audio->score_list), list) {
					if (p_tmp->level >= TRUST_LEVEL) {
						if (i <= 1) {
							//notthing to do
						} else if (p_tmp_r != NULL &&
							   p_tmp->x >= x_thr) {
							list_del(
								&p_tmp_r->list);
							kfree(p_tmp_r);
							p_tmp_r = NULL;
						} else if (p_tmp_l != NULL) {
							list_del(
								&p_tmp_l->list);
							kfree(p_tmp_l);
							p_tmp_l = NULL;
						}
						list_del(&p_tmp->list);
						list_add(&p_tmp->list,
							 &(haptic_audio->list));
						//break;
					} else {
						j++;
					}
				}

				pr_debug("%s: haptic_audio->score_list : %d\n",
					 __func__, j);
				if (j > 2) {
					list_for_each_entry_safe_reverse (
						p_tmp, p_tmp_bak,
						&(haptic_audio->score_list),
						list) {
						list_del(&p_tmp->list);
						if (--j <= 2) {
							break;
						}
					}
				}
#endif
				//add by Jerry.Won end

				tp->tp_ai_check_flag = 0;
				tp->id[tp_ai_match_id].tp_ai_match_flag = 1;
			} else {
				if (tp->id[touch_vibrator_id]
					    .press_no_vibrate_flag == 2) {
					if (haptic_audio->hap_cnt_outside_tz) {
						haptic_audio
							->hap_cnt_outside_tz--;
					}
					tp->tp_ai_check_flag = 0;
				}
			}
		}

		/* ai scan uevent report */
		if (haptic_audio->uevent_report_flag != uevent_report_flag) {
			if (uevent_report_flag == 1) {
				aw86907_haptic_audio_uevent_report_scan(aw86907,
									true);
				haptic_audio->uevent_report_flag =
					uevent_report_flag;
			}
		}

		/* haptic play with tz num */
		/* //del by Jerry.Won
    if (AW86907_HAPTIC_TP_PLAY_ENABLE == tp_touch_play_flag) {
        touch_vibrator_flag = 0;
        list_for_each_entry(p_tmp, &(haptic_audio->list), list) {
            if (aw86907_haptic_audio_tp_list_match(&(tp->id[touch_vibrator_id].pt_info), p_tmp)) {
                if(AW86907_HAPTIC_TP_PLAY_ENABLE == tp->play_flag) {
                    touch_vibrator_flag = 1;
                    break;
                }
                break;
            } else {}
        }

        if(touch_vibrator_flag) {
            tp->play_flag = AW86907_HAPTIC_TP_PLAY_ENABLE;
        } else {
            tp->play_flag = AW86907_HAPTIC_TP_PLAY_NULL;
        }
        pr_debug("%s: tz_num=%d, touch_vibrator_flag=%d\n", __func__, tz_num, touch_vibrator_flag);
    }

    if(tp->play_flag) {
        pr_debug("%s: tp.play_flag with ai tz =%d\n", __func__, tp->play_flag);
    }
//del by Jerry.Won end*/

		if (AW86907_HAPTIC_PLAY_ENABLE ==
		    aw86907->haptic_audio.ctr.play) {
			tp->last_play_flag = tp->play_flag;
		}
		/* clear all id tp flag */
		for (i = 0; i < AW86907_HAPTIC_TP_ID_MAX + 1; i++) {
			tp->id[i].tp_flag = AW86907_HAPTIC_TP_NULL;
		}
	} else {
		for (i = 0; i < AW86907_HAPTIC_TP_ID_MAX + 1; i++) {
			tp->id[i].tp_flag = AW86907_HAPTIC_TP_NULL;
		}
	}
	mutex_unlock(&aw86907->haptic_audio.lock);

	/* aw86907 haptic play control */
	if (AW86907_HAPTIC_CMD_ENABLE ==
	    (AW86907_HAPTIC_CMD_HAPTIC & aw86907->haptic_audio.ctr.cmd)) {
		if (AW86907_HAPTIC_PLAY_ENABLE ==
		    aw86907->haptic_audio.ctr.play) {
			pr_info("%s: haptic_audio_play_start\n", __func__);
			if (AW86907_HAPTIC_CMD_TP ==
				    (AW86907_HAPTIC_CMD_SYS &
				     aw86907->haptic_audio.ctr.cmd) &&
			    (AW86907_HAPTIC_TP_PLAY_ENABLE != tp->play_flag)) {
				pr_info("%s: cancel haptic with tp event\n",
					__func__);
			} else {
				pr_info("%s: normal haptic with tp event\n",
					__func__);
				mutex_lock(&aw86907->lock);
				aw86907_haptic_audio_stop(aw86907);
				aw86907_haptic_audio_play_cfg(aw86907);

				aw86907_haptic_set_wav_seq(
					aw86907, 0x00,
					aw86907->haptic_audio.ctr.wavseq);

				aw86907_haptic_set_wav_loop(
					aw86907, 0x00,
					aw86907->haptic_audio.ctr.loop);

				aw86907_haptic_set_gain(
					aw86907,
					aw86907->haptic_audio.ctr.gain);

				aw86907_haptic_start(aw86907);
				mutex_unlock(&aw86907->lock);
			}
		} else if (AW86907_HAPTIC_PLAY_STOP ==
			   aw86907->haptic_audio.ctr.play) {
			mutex_lock(&aw86907->lock);
			aw86907_haptic_audio_stop(aw86907);
			mutex_unlock(&aw86907->lock);
		} else if (AW86907_HAPTIC_PLAY_GAIN ==
			   aw86907->haptic_audio.ctr.play) {
			mutex_lock(&aw86907->lock);
			aw86907_haptic_set_gain(aw86907,
						aw86907->haptic_audio.ctr.gain);
			mutex_unlock(&aw86907->lock);
		}
	}
}

//hch 20190917
static ssize_t aw86907_gun_type_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	return snprintf(buf, PAGE_SIZE, "0x%02x\n", aw86907->gun_type);
}

static ssize_t aw86907_gun_type_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	pr_debug("%s: value=%d\n", __FUNCTION__, val);

	mutex_lock(&aw86907->lock);
	aw86907->gun_type = val;
	mutex_unlock(&aw86907->lock);
	return count;
}

//hch 20190917
static ssize_t aw86907_bullet_nr_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	return snprintf(buf, PAGE_SIZE, "0x%02x\n", aw86907->bullet_nr);
}

static ssize_t aw86907_bullet_nr_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	pr_debug("%s: value=%d\n", __FUNCTION__, val);

	mutex_lock(&aw86907->lock);
	aw86907->bullet_nr = val;
	mutex_unlock(&aw86907->lock);
	return count;
}

/*****************************************************
 *
 * haptic cont
 *
 *****************************************************/
static int aw86907_haptic_cont(struct aw86907 *aw86907)
{
	pr_info("%s enter\n", __func__);

	/* work mode */
	aw86907_haptic_play_mode(aw86907, AW86907_HAPTIC_CONT_MODE);

	/* cont config */
	/* aw86907_i2c_write_bits(aw86907, AW86907_REG_CONTCFG1,
     **                     AW86907_BIT_CONTCFG1_EN_F0_DET_MASK,
     **                     AW86907_BIT_CONTCFG1_F0_DET_ENABLE);
     */
	aw86907_i2c_write_bits(aw86907, AW86907_REG_CONTCFG6,
			       AW86907_BIT_CONTCFG6_TRACK_EN_MASK,
			       AW86907_BIT_CONTCFG6_TRACK_ENABLE);
	/* f0 driver level */
	aw86907_i2c_write_bits(aw86907, AW86907_REG_CONTCFG6,
			       AW86907_BIT_CONTCFG6_DRV1_LVL_MASK,
			       aw86907->cont_drv1_lvl);
	aw86907_i2c_write(aw86907, AW86907_REG_CONTCFG7,
			  aw86907->cont_drv2_lvl);
	/* DRV1_TIME */
	/* aw86907_i2c_write(aw86907, AW86907_REG_CONTCFG8, 0xFF); */
	/* DRV2_TIME */
	aw86907_i2c_write(aw86907, AW86907_REG_CONTCFG9, 0xFF);

	/* cont play go */
	aw86907_haptic_play_go(aw86907, true);

	return 0;
}

static int aw86907_dump_f0_registers(struct aw86907 *aw86907)
{
#if 0
    unsigned char reg_val = 0;

    aw86907_i2c_read(aw86907, 0x04, &reg_val);
    pr_info("%s reg_0x04 = 0x%02x\n", __func__, reg_val);

    aw86907_i2c_read(aw86907, 0x46, &reg_val);
    pr_info("%s reg_0x46 = 0x%02x\n", __func__, reg_val);

    aw86907_i2c_read(aw86907, 0x48, &reg_val);
    pr_info("%s reg_0x48 = 0x%02x\n", __func__, reg_val);

    aw86907_i2c_read(aw86907, 0x49, &reg_val);
    pr_info("%s reg_0x49 = 0x%02x\n", __func__, reg_val);

    aw86907_i2c_read(aw86907, 0x4A, &reg_val);
    pr_info("%s reg_0x4A = 0x%02x\n", __func__, reg_val);

    aw86907_i2c_read(aw86907, 0x68, &reg_val);
    pr_info("%s reg_0x68 = 0x%02x\n", __func__, reg_val);

    aw86907_i2c_read(aw86907, 0x69, &reg_val);
    pr_info("%s reg_0x69 = 0x%02x\n", __func__, reg_val);

    aw86907_i2c_read(aw86907, 0x6a, &reg_val);
    pr_info("%s reg_0x6a = 0x%02x\n", __func__, reg_val);

    aw86907_i2c_read(aw86907, 0x6b, &reg_val);
    pr_info("%s reg_0x6b = 0x%02x\n", __func__, reg_val);

    aw86907_i2c_read(aw86907, 0x6D, &reg_val);
    pr_info("%s reg_0x6D = 0x%02x\n", __func__, reg_val);

    aw86907_i2c_read(aw86907, 0x6f, &reg_val);
    pr_info("%s reg_0x6f = 0x%02x\n", __func__, reg_val);

    aw86907_i2c_read(aw86907, 0x70, &reg_val);
    pr_info("%s reg_0x70 = 0x%02x\n", __func__, reg_val);

    aw86907_i2c_read(aw86907, 0x71, &reg_val);
    pr_info("%s reg_0x71 = 0x%02x\n", __func__, reg_val);

    aw86907_i2c_read(aw86907, 0x7D, &reg_val);
    pr_info("%s reg_0x7D = 0x%02x\n", __func__, reg_val);

    aw86907_i2c_read(aw86907, 0x7E, &reg_val);
    pr_info("%s reg_0x7E = 0x%02x\n", __func__, reg_val);

    aw86907_i2c_read(aw86907, 0x7F, &reg_val);
    pr_info("%s reg_0x7F = 0x%02x\n", __func__, reg_val);

    aw86907_i2c_read(aw86907, 0x36, &reg_val);
    pr_info("%s reg_0x36 = 0x%02x\n", __func__, reg_val);

    aw86907_i2c_read(aw86907, 0x5b, &reg_val);
    pr_info("%s reg_0x5b = 0x%02x\n", __func__, reg_val);
#endif
	return 0;
}

/*****************************************************
 *
 * haptic f0 cali
 *
 *****************************************************/
static int aw86907_haptic_get_f0(struct aw86907 *aw86907)
{
	int ret = 0;
	//unsigned char i = 0;
	unsigned char reg_val = 0;
	unsigned char d2s_gain_temp = 0;
	unsigned int cnt = 200;
	//unsigned char f0_pre_num = 0;
	//unsigned char f0_wait_num = 0;
	//unsigned char f0_repeat_num = 0;
	//unsigned char f0_trace_num = 0;
	//unsigned int t_f0_ms = 0;
	//unsigned int t_f0_trace_ms = 0;
	//unsigned int f0_cali_cnt = 50;
	bool get_f0_flag = false;
	unsigned char brk_en_temp = 0;

	pr_info("%s enter\n", __func__);

	aw86907->f0 = aw86907->f0_pre;
	/* enter standby mode */
	aw86907_haptic_stop(aw86907);
	/* config max d2s_gain */
	aw86907_i2c_read(aw86907, AW86907_REG_SYSCTRL7, &reg_val);
	d2s_gain_temp = 0x07 & reg_val;
	aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSCTRL7,
			       AW86907_BIT_SYSCTRL7_D2S_GAIN_MASK,
			       AW86907_BIT_SYSCTRL7_D2S_GAIN_26);

	/* f0 calibrate work mode */
	aw86907_haptic_play_mode(aw86907, AW86907_HAPTIC_CONT_MODE);

	/* enable f0 detect */
	aw86907_i2c_write_bits(aw86907, AW86907_REG_CONTCFG1,
			       AW86907_BIT_CONTCFG1_EN_F0_DET_MASK,
			       AW86907_BIT_CONTCFG1_F0_DET_ENABLE);

	/* cont config */
	aw86907_i2c_write_bits(aw86907, AW86907_REG_CONTCFG6,
			       AW86907_BIT_CONTCFG6_TRACK_EN_MASK,
			       AW86907_BIT_CONTCFG6_TRACK_ENABLE);

	/* enable auto break */
	aw86907_i2c_read(aw86907, AW86907_REG_PLAYCFG3, &reg_val);
	brk_en_temp = 0x04 & reg_val;
	aw86907_i2c_write_bits(aw86907, AW86907_REG_PLAYCFG3,
			       AW86907_BIT_PLAYCFG3_BRK_EN_MASK,
			       AW86907_BIT_PLAYCFG3_BRK_ENABLE);

	/* LRA OSC Source */
	if (aw86907->f0_cali_flag == AW86907_HAPTIC_CALI_F0) {
		aw86907_i2c_write_bits(aw86907, AW86907_REG_TRIMCFG3,
				       AW86907_BIT_TRIMCFG3_LRA_TRIM_SRC_MASK,
				       AW86907_BIT_TRIMCFG3_LRA_TRIM_SRC_REG);
	} else {
		aw86907_i2c_write_bits(aw86907, AW86907_REG_TRIMCFG3,
				       AW86907_BIT_TRIMCFG3_LRA_TRIM_SRC_MASK,
				       AW86907_BIT_TRIMCFG3_LRA_TRIM_SRC_EFUSE);
	}

	/* f0 driver level */
	aw86907_i2c_write_bits(aw86907, AW86907_REG_CONTCFG6,
			       AW86907_BIT_CONTCFG6_DRV1_LVL_MASK,
			       aw86907->cont_drv1_lvl);
	aw86907_i2c_write(aw86907, AW86907_REG_CONTCFG7,
			  aw86907->cont_drv2_lvl);
	/* DRV1_TIME */
	aw86907_i2c_write(aw86907, AW86907_REG_CONTCFG8,
			  aw86907->cont_drv1_time);
	/* DRV2_TIME */
	aw86907_i2c_write(aw86907, AW86907_REG_CONTCFG9,
			  aw86907->cont_drv2_time);
	/* TRACK_MARGIN */
	if (!aw86907->cont_track_margin) {
		pr_info("%s aw86907->cont_track_margin = 0!\n", __func__);
	} else {
		aw86907_i2c_write(aw86907, AW86907_REG_CONTCFG11,
				  (unsigned char)aw86907->cont_track_margin);
	}
	/* DRV_WIDTH */
	/*
     * aw86907_i2c_write(aw86907, AW86907_REG_CONTCFG3,
     *                aw86907->cont_drv_width);
     */
	aw86907_dump_f0_registers(aw86907);

	/* play go and start f0 calibration */
	aw86907_haptic_play_go(aw86907, true);

	/* 300ms */
	while (cnt) {
		aw86907_i2c_read(aw86907, AW86907_REG_GLBRD5, &reg_val);
		if ((reg_val & 0x0f) == 0x00) {
			cnt = 0;
			get_f0_flag = true;
			pr_info("%s entered standby mode! glb_state=0x%02X\n",
				__func__, reg_val);
		} else {
			cnt--;
			pr_info("%s waitting for standby, glb_state=0x%02X\n",
				__func__, reg_val);
		}
		usleep_range(10000, 10500);
	}
	if (get_f0_flag) {
		aw86907_haptic_read_lra_f0(aw86907);
		aw86907_haptic_read_cont_f0(aw86907);
	} else {
		pr_err("%s enter standby mode failed, stop reading f0!\n",
		       __func__);
	}

	aw86907_dump_f0_registers(aw86907);

	/* restore d2s_gain config */
	aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSCTRL7,
			       AW86907_BIT_SYSCTRL7_D2S_GAIN_MASK,
			       d2s_gain_temp);
	/* restore default config */
	aw86907_i2c_write_bits(aw86907, AW86907_REG_CONTCFG1,
			       AW86907_BIT_CONTCFG1_EN_F0_DET_MASK,
			       AW86907_BIT_CONTCFG1_F0_DET_DISABLE);
	/* recover auto break config */
	aw86907_i2c_write_bits(aw86907, AW86907_REG_PLAYCFG3,
			       AW86907_BIT_PLAYCFG3_BRK_EN_MASK, brk_en_temp);

	return ret;
}

static int aw86907_haptic_f0_calibration(struct aw86907 *aw86907)
{
	int ret = 0;
	unsigned char reg_val = 0;
	unsigned int f0_limit = 0;
	char f0_cali_lra = 0;
	int f0_cali_step = 0;
	//int f0_dft_step = 0;

	pr_info("%s enter\n", __func__);
	aw86907->f0_cali_flag = AW86907_HAPTIC_CALI_F0;
	aw86907_i2c_write_bits(aw86907, AW86907_REG_TRIMCFG3,
			       AW86907_BIT_TRIMCFG3_TRIM_LRA_MASK, 0);

	if (aw86907_haptic_get_f0(aw86907)) {
		pr_err("%s get f0 error, user defafult f0\n", __func__);
	} else {
		/* max and min limit */
		f0_limit = aw86907->f0;
		pr_info("%s f0_ref = %d, f0 = %d\n", __func__, aw86907->f0_pre,
			aw86907->f0);

#ifdef OPLUS_FEATURE_CHG_BASIC
		if (aw86907->device_id == 832 || aw86907->device_id == 833) {
			if (aw86907->f0 * 100 <
			    AW86907_0832_HAPTIC_F0_PRE *
				    (100 -
				     AW86907_0832_HAPTIC_F0_CALI_PERCEN)) {
				f0_limit =
					AW86907_0832_HAPTIC_F0_PRE *
					(100 -
					 AW86907_0832_HAPTIC_F0_CALI_PERCEN) /
					100;
			}
			if (aw86907->f0 * 100 >
			    AW86907_0832_HAPTIC_F0_PRE *
				    (100 +
				     AW86907_0832_HAPTIC_F0_CALI_PERCEN)) {
				f0_limit =
					AW86907_0832_HAPTIC_F0_PRE *
					(100 +
					 AW86907_0832_HAPTIC_F0_CALI_PERCEN) /
					100;
			}
		} else {
			if (aw86907->f0 * 100 <
			    AW86907_0815_HAPTIC_F0_PRE *
				    (100 -
				     AW86907_0815_HAPTIC_F0_CALI_PERCEN)) {
				f0_limit =
					AW86907_0815_HAPTIC_F0_PRE *
					(100 -
					 AW86907_0815_HAPTIC_F0_CALI_PERCEN) /
					100;
			}
			if (aw86907->f0 * 100 >
			    AW86907_0815_HAPTIC_F0_PRE *
				    (100 +
				     AW86907_0815_HAPTIC_F0_CALI_PERCEN)) {
				f0_limit =
					AW86907_0815_HAPTIC_F0_PRE *
					(100 +
					 AW86907_0815_HAPTIC_F0_CALI_PERCEN) /
					100;
			}
		}
#else
		if (aw86907->f0 * 100 <
		    aw86907->f0_pre * (100 - aw86907->f0_cali_percent)) {
			f0_limit = aw86907->f0_pre *
				   (100 - aw86907->f0_cali_percent) / 100;
		}
		if (aw86907->f0 * 100 >
		    aw86907->f0_pre * (100 + f0_cali_percent)) {
			f0_limit = aw86907->f0_pre *
				   (100 + aw86907->f0_cali_percent) / 100;
		}
#endif

		/* calculate cali step */
		f0_cali_step = 100000 * ((int)f0_limit - (int)aw86907->f0_pre) /
			       ((int)f0_limit * 24);
		pr_debug("%s f0_cali_step=%d\n", __func__, f0_cali_step);

		if (f0_cali_step >= 0) { /*f0_cali_step >= 0 */
			if (f0_cali_step % 10 >= 5)
				f0_cali_step = 32 + (f0_cali_step / 10 + 1);
			else
				f0_cali_step = 32 + f0_cali_step / 10;
		} else { /* f0_cali_step < 0 */
			if (f0_cali_step % 10 <= -5)
				f0_cali_step = 32 + (f0_cali_step / 10 - 1);
			else
				f0_cali_step = 32 + f0_cali_step / 10;
		}

		if (f0_cali_step > 31)
			f0_cali_lra = (char)f0_cali_step - 32;
		else
			f0_cali_lra = (char)f0_cali_step + 32;

		/* update cali step */
		aw86907_i2c_read(aw86907, AW86907_REG_TRIMCFG3, &reg_val);
		aw86907->clock_system_f0_cali_lra =
			((int)f0_cali_lra + (int)(reg_val & 0x3f)) & 0x3f;
		pr_info("%s origin trim_lra = 0x%02X, f0_cali_lra = 0x%02X, final f0_cali_data = 0x%02X\n",
			__func__, (reg_val & 0x3f), f0_cali_lra,
			aw86907->clock_system_f0_cali_lra);

		/* update cali step */
		aw86907_set_clock(aw86907, AW86907_HAPTIC_CLOCK_CALI_F0);
	}

	if (aw86907_haptic_get_f0(aw86907)) {
		pr_err("%s get f0 error, user defafult f0\n", __func__);
	}

	/* restore default work mode */
	aw86907_haptic_play_mode(aw86907, AW86907_HAPTIC_STANDBY_MODE);
	aw86907->play_mode = AW86907_HAPTIC_RAM_MODE;
	aw86907_i2c_write_bits(aw86907, AW86907_REG_PLAYCFG3,
			       AW86907_BIT_PLAYCFG3_PLAY_MODE_MASK,
			       AW86907_BIT_PLAYCFG3_PLAY_MODE_RAM);
	aw86907_haptic_stop(aw86907);

	return ret;
}

#ifdef AAC_RICHTAP
static void haptic_clean_buf(struct aw86907 *aw86907, int status)
{
	struct mmap_buf_format *opbuf = aw86907->start_buf;
	int i = 0;

	for (i = 0; i < RICHTAP_MMAP_BUF_SUM; i++) {
		opbuf->status = status;
		opbuf = opbuf->kernel_next;
	}
}

static inline unsigned int aw86907_get_sys_msecs()
{
	struct timespec64 ts64 = current_kernel_time64();
	return jiffies_to_msecs(timespec64_to_jiffies(&ts64));
}

static void rtp_work_proc(struct work_struct *work)
{
	struct aw86907 *aw86907 =
		container_of(work, struct aw86907, haptic_rtp_work);
	struct mmap_buf_format *opbuf = aw86907->start_buf;
	uint32_t count = 100;
	uint8_t reg_val = 0x10;
	unsigned int write_start;

	opbuf = aw86907->start_buf;
	count = 100;
	while (true && count--) {
		if (opbuf->status == MMAP_BUF_DATA_VALID) {
			mutex_lock(&aw86907->lock);
			aw86907_haptic_play_mode(aw86907,
						 AW86907_HAPTIC_RTP_MODE);
			aw86907_haptic_set_rtp_aei(aw86907, true);
			aw86907_interrupt_clear(aw86907);
			aw86907_haptic_start(aw86907);
			mutex_unlock(&aw86907->lock);
			break;
		} else {
			msleep(1);
		}
	}
	write_start = aw86907_get_sys_msecs();
	reg_val = 0x10;
	while (true) {
		if (aw86907_get_sys_msecs() > (write_start + 800)) {
			pr_info("Failed ! %s endless loop\n", __func__);
			break;
		}
		if (reg_val & 0x01 || (aw86907->done_flag == true) ||
		    (opbuf->status == MMAP_BUF_DATA_FINISHED) ||
		    (opbuf->status == MMAP_BUF_DATA_INVALID)) {
			break;
		} else if (opbuf->status == MMAP_BUF_DATA_VALID &&
			   (reg_val & 0x01 << 4)) {
			aw86907_i2c_writes(aw86907, AW86907_REG_PLAYCFG1,
					   opbuf->data, opbuf->length);
			memset(opbuf->data, 0, opbuf->length);
			opbuf->status = MMAP_BUF_DATA_INVALID;
			opbuf = opbuf->kernel_next;
			write_start = aw86907_get_sys_msecs();
		} else {
			msleep(1);
		}
		aw86907_i2c_read(aw86907, AW86907_REG_SYSST, &reg_val);
	}
	aw86907_haptic_set_rtp_aei(aw86907, false);
	aw86907->haptic_rtp_mode = false;
}
#endif

/*****************************************************
 *
 * haptic fops
 *
 *****************************************************/
static int aw86907_file_open(struct inode *inode, struct file *file)
{
	if (!try_module_get(THIS_MODULE))
		return -ENODEV;

	file->private_data = (void *)g_aw86907;

	return 0;
}

static int aw86907_file_release(struct inode *inode, struct file *file)
{
	file->private_data = (void *)NULL;

	module_put(THIS_MODULE);

	return 0;
}

static long aw86907_file_unlocked_ioctl(struct file *file, unsigned int cmd,
					unsigned long arg)
{
	struct aw86907 *aw86907 = (struct aw86907 *)file->private_data;
#ifdef AAC_RICHTAP
	uint32_t tmp;
#endif
	int ret = 0;

	dev_info(aw86907->dev, "%s: cmd=0x%x, arg=0x%lx\n", __func__, cmd, arg);

	mutex_lock(&aw86907->lock);

#ifdef AAC_RICHTAP
	switch (cmd) {
	case RICHTAP_GET_HWINFO:
		tmp = RICHTAP_AW_8697;
		if (copy_to_user((void __user *)arg, &tmp, sizeof(uint32_t)))
			ret = -EFAULT;
		break;
	case RICHTAP_RTP_MODE:
		aw86907_haptic_stop(aw86907);
		if (RICHTAP_MMAP_BUF_SIZE * RICHTAP_MMAP_BUF_SUM > sizeof(aw86907->rtp_ptr)) {
			return -EFAULT;
		}
		if (aw86907->rtp_ptr == NULL || (void __user *)arg == NULL) {
			return -EFAULT;
		}
		if (copy_from_user(aw86907->rtp_ptr, (void __user *)arg,
				   RICHTAP_MMAP_BUF_SIZE *
					   RICHTAP_MMAP_BUF_SUM)) {
			ret = -EFAULT;
			break;
		}
		tmp = *((uint32_t *)aw86907->rtp_ptr);
		if (tmp > (RICHTAP_MMAP_BUF_SIZE * RICHTAP_MMAP_BUF_SUM - 4)) {
			dev_err(aw86907->dev, "rtp mode date len error %d\n",
				tmp);
			ret = -EINVAL;
			break;
		}
		aw86907_haptic_set_bst_vol(aw86907, 0x11);
		aw86907_haptic_play_mode(aw86907, AW86907_HAPTIC_RTP_MODE);
		aw86907_haptic_start(aw86907);
		usleep_range(2000, 2500);
		aw86907_i2c_writes(aw86907, AW86907_REG_PLAYCFG1,
				   &aw86907->rtp_ptr[4], tmp);
		break;
	case RICHTAP_OFF_MODE:
		break;
	case RICHTAP_GET_F0:
		tmp = aw86907->f0;
		if (copy_to_user((void __user *)arg, &tmp, sizeof(uint32_t)))
			ret = -EFAULT;
		break;
	case RICHTAP_SETTING_GAIN:
		if (arg > 0x80)
			arg = 0x80;
		aw86907_i2c_write(aw86907, AW86907_REG_TRGCFG7, arg);
		break;
	case RICHTAP_STREAM_MODE:
		haptic_clean_buf(aw86907, MMAP_BUF_DATA_INVALID);
		aw86907_haptic_stop(aw86907);
		aw86907->done_flag = false;
		aw86907->haptic_rtp_mode = true;
		aw86907_haptic_set_bst_vol(aw86907, 0x11);
		schedule_work(&aw86907->haptic_rtp_work);
		break;
	case RICHTAP_STOP_MODE:
		dev_warn(aw86907->dev, "%s,RICHTAP_STOP_MODE  stop enter\n",
			 __func__);
		aw86907->done_flag = true;
		aw86907_op_clean_status(aw86907);
		usleep_range(2000, 2000);
		aw86907_haptic_set_rtp_aei(aw86907, false);
		aw86907_haptic_stop(aw86907);
		aw86907->haptic_rtp_mode = false;
		dev_warn(aw86907->dev, "%s,RICHTAP_STOP_MODE  stop end\n",
			 __func__);
		break;
	default:
		dev_err(aw86907->dev, "%s, unknown cmd = %d\n", __func__, cmd);
		break;
	}
#else
	if (_IOC_TYPE(cmd) != AW86907_HAPTIC_IOCTL_MAGIC) {
		dev_err(aw86907->dev, "%s: cmd magic err\n", __func__);
		mutex_unlock(&aw86907->lock);
		return -EINVAL;
	}

	switch (cmd) {
	default:
		dev_err(aw86907->dev, "%s, unknown cmd\n", __func__);
		break;
	}
#endif
	mutex_unlock(&aw86907->lock);

	return ret;
}

static ssize_t aw86907_file_read(struct file *filp, char *buff, size_t len,
				 loff_t *offset)
{
	struct aw86907 *aw86907 = (struct aw86907 *)filp->private_data;
	int ret = 0;
	int i = 0;
	unsigned char reg_val = 0;
	unsigned char *pbuff = NULL;

	mutex_lock(&aw86907->lock);

	dev_info(aw86907->dev, "%s: len=%zu\n", __func__, len);

	switch (aw86907->fileops.cmd) {
	case AW86907_HAPTIC_CMD_READ_REG:
		pbuff = (unsigned char *)kzalloc(len, GFP_KERNEL);
		if (pbuff != NULL) {
			for (i = 0; i < len; i++) {
				aw86907_i2c_read(aw86907,
						 aw86907->fileops.reg + i,
						 &reg_val);
				pbuff[i] = reg_val;
			}
			for (i = 0; i < len; i++) {
				dev_info(aw86907->dev, "%s: pbuff[%d]=0x%02x\n",
					 __func__, i, pbuff[i]);
			}
			ret = copy_to_user(buff, pbuff, len);
			if (ret) {
				dev_err(aw86907->dev, "%s: copy to user fail\n",
					__func__);
			}
			kfree(pbuff);
		} else {
			dev_err(aw86907->dev, "%s: alloc memory fail\n",
				__func__);
		}
		break;
	default:
		dev_err(aw86907->dev, "%s, unknown cmd %d \n", __func__,
			aw86907->fileops.cmd);
		break;
	}

	mutex_unlock(&aw86907->lock);

	return len;
}

static ssize_t aw86907_file_write(struct file *filp, const char *buff,
				  size_t len, loff_t *off)
{
	struct aw86907 *aw86907 = (struct aw86907 *)filp->private_data;
	int i = 0;
	int ret = 0;
	unsigned char *pbuff = NULL;

	pbuff = (unsigned char *)kzalloc(len, GFP_KERNEL);
	if (pbuff == NULL) {
		dev_err(aw86907->dev, "%s: alloc memory fail\n", __func__);
		return len;
	}
	ret = copy_from_user(pbuff, buff, len);
	if (ret) {
#ifdef OPLUS_FEATURE_CHG_BASIC
		if (pbuff != NULL) {
			kfree(pbuff);
		}
#endif
		dev_err(aw86907->dev, "%s: copy from user fail\n", __func__);
		return len;
	}

	for (i = 0; i < len; i++) {
		dev_info(aw86907->dev, "%s: pbuff[%d]=0x%02x\n", __func__, i,
			 pbuff[i]);
	}

	mutex_lock(&aw86907->lock);

	aw86907->fileops.cmd = pbuff[0];

	switch (aw86907->fileops.cmd) {
	case AW86907_HAPTIC_CMD_READ_REG:
		if (len == 2) {
			aw86907->fileops.reg = pbuff[1];
		} else {
			dev_err(aw86907->dev, "%s: read cmd len %zu err\n",
				__func__, len);
		}
		break;
	case AW86907_HAPTIC_CMD_WRITE_REG:
		if (len > 2) {
			for (i = 0; i < len - 2; i++) {
				dev_info(aw86907->dev,
					 "%s: write reg0x%02x=0x%02x\n",
					 __func__, pbuff[1] + i, pbuff[i + 2]);
				aw86907_i2c_write(aw86907, pbuff[1] + i,
						  pbuff[2 + i]);
			}
		} else {
			dev_err(aw86907->dev, "%s: write cmd len %zu err\n",
				__func__, len);
		}
		break;
	default:
		dev_err(aw86907->dev, "%s, unknown cmd %d \n", __func__,
			aw86907->fileops.cmd);
		break;
	}

	mutex_unlock(&aw86907->lock);

	if (pbuff != NULL) {
		kfree(pbuff);
	}
	return len;
}

#ifdef AAC_RICHTAP
static int aw86907_file_mmap(struct file *filp, struct vm_area_struct *vma)
{
	unsigned long phys;
	struct aw86907 *aw86907 = (struct aw86907 *)filp->private_data;
	int ret = 0;
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 7, 0)
	/*only accept PROT_READ, PROT_WRITE and MAP_SHARED from the API of mmap*/
	vm_flags_t vm_flags = calc_vm_prot_bits(PROT_READ | PROT_WRITE, 0) |
			      calc_vm_flag_bits(MAP_SHARED);
	vm_flags |= current->mm->def_flags | VM_MAYREAD | VM_MAYWRITE |
		    VM_MAYEXEC | VM_SHARED | VM_MAYSHARE;
	if (vma && (pgprot_val(vma->vm_page_prot) !=
		    pgprot_val(vm_get_page_prot(vm_flags))))
		return -EPERM;

	if (vma && ((vma->vm_end - vma->vm_start) !=
		    (PAGE_SIZE << RICHTAP_MMAP_PAGE_ORDER)))
		return -ENOMEM;
#endif
	phys = virt_to_phys(aw86907->start_buf);

	ret = remap_pfn_range(vma, vma->vm_start, (phys >> PAGE_SHIFT),
			      (vma->vm_end - vma->vm_start), vma->vm_page_prot);
	if (ret) {
		dev_err(aw86907->dev, "Error mmap failed\n");
		return ret;
	}

	return ret;
}
#endif

static struct file_operations fops = {
	.owner = THIS_MODULE,
	.read = aw86907_file_read,
	.write = aw86907_file_write,
#ifdef AAC_RICHTAP
	.mmap = aw86907_file_mmap,
#endif
	.unlocked_ioctl = aw86907_file_unlocked_ioctl,
	.open = aw86907_file_open,
	.release = aw86907_file_release,
};

static struct miscdevice aw86907_haptic_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = AW86907_HAPTIC_NAME,
	.fops = &fops,
};

static void aw86907_haptic_misc_para_init(struct aw86907 *aw86907)
{
	pr_info("%s enter!\n", __func__);

	aw86907_i2c_write(aw86907, AW86907_REG_BSTCFG1, 0x20);
	aw86907_i2c_write(aw86907, AW86907_REG_BSTCFG2, 0x24);
	aw86907_i2c_write(aw86907, AW86907_REG_BSTCFG3, 0x96);
	aw86907_i2c_write(aw86907, AW86907_REG_BSTCFG4, 0x40);
	aw86907_i2c_write(aw86907, AW86907_REG_BSTCFG5, 0x11);

	aw86907_i2c_write(aw86907, AW86907_REG_SYSCTRL3, 0x05);
	aw86907_i2c_write(aw86907, AW86907_REG_SYSCTRL4, 0xB2);
	aw86907_i2c_write(aw86907, AW86907_REG_SYSCTRL5, 0xFF);
	aw86907_i2c_write(aw86907, AW86907_REG_SYSCTRL6, 0xEF);

	/* d2s_gain */
	aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSCTRL7,
			       AW86907_BIT_SYSCTRL7_D2S_GAIN_MASK,
			       aw86907->d2s_gain);
	/* cont_tset */
	aw86907_i2c_write_bits(aw86907, AW86907_REG_CONTCFG13,
			       AW86907_BIT_CONTCFG13_TSET_MASK,
			       aw86907->cont_test << 4);
	/* cont_bemf_set */
	aw86907_i2c_write_bits(aw86907, AW86907_REG_CONTCFG13,
			       AW86907_BIT_CONTCFG13_BEME_SET_MASK,
			       aw86907->cont_bemf_set);
	/* cont_brk_time */
	aw86907_i2c_write(aw86907, AW86907_REG_CONTCFG10,
			  aw86907->cont_brk_time);
	/* cont_bst_brk_gain */
	aw86907_i2c_write_bits(aw86907, AW86907_REG_CONTCFG5,
			       AW86907_BIT_CONTCFG5_BST_BRK_GAIN_MASK,
			       aw86907->cont_bst_brk_gain);
	/* cont_brk_gain */
	aw86907_i2c_write_bits(aw86907, AW86907_REG_CONTCFG5,
			       AW86907_BIT_CONTCFG5_BRK_GAIN_MASK,
			       aw86907->cont_brk_gain);
	/* i2s disable */
	aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSCTRL2,
			       AW86907_BIT_SYSCTRL2_I2S_PIN_MASK,
			       AW86907_BIT_SYSCTRL2_I2S_PIN_TRIG);
	pr_info("%s out!\n", __func__);
}

static int aw86907_haptic_init(struct aw86907 *aw86907)
{
	int ret = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;
	struct tp *tp = &(aw86907->haptic_audio.tp);

	pr_info("%s enter!\n", __func__);

	ret = misc_register(&aw86907_haptic_misc);
	if (ret) {
		dev_err(aw86907->dev, "%s: misc fail: %d\n", __func__, ret);
		return ret;
	}

	/* haptic audio */
	aw86907->haptic_audio.delay_val = 1;
	aw86907->haptic_audio.timer_val = 21318;
	INIT_LIST_HEAD(&(aw86907->haptic_audio.ctr_list));

	hrtimer_init(&aw86907->haptic_audio.timer, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);
	aw86907->haptic_audio.timer.function = aw86907_haptic_audio_timer_func;
	INIT_WORK(&aw86907->haptic_audio.work,
		  aw86907_haptic_audio_work_routine);

	mutex_init(&aw86907->haptic_audio.lock);
	aw86907->gun_type = 0xff;
	aw86907->bullet_nr = 0x00;
	aw86907->haptic_audio.tp.press_delay_min = 1;
	aw86907->haptic_audio.tp.press_delay_max = 5300000;
	aw86907->haptic_audio.tp.release_delay_max = 300000;
	aw86907->haptic_audio.tp.no_play_cnt_max = 230;
	for (i = 0; i < AW86907_HAPTIC_TP_ID_MAX + 1; i++) {
		tp->id[i].pt_info.status = 0;
	}

	INIT_LIST_HEAD(&(aw86907->haptic_audio.list));
	INIT_LIST_HEAD(&(aw86907->haptic_audio.score_list));
	aw86907->haptic_audio.tp_size.x = 2400;
	aw86907->haptic_audio.tp_size.y = 1080;
	aw86907->haptic_audio.tz_cnt_thr = 3;
	aw86907->haptic_audio.tz_cnt_max = 10;
	aw86907->haptic_audio.hap_cnt_max_outside_tz = 5;
	aw86907_op_clean_status(aw86907);
	mutex_init(&aw86907->rtp_lock);

	/* haptic init */
	mutex_lock(&aw86907->lock);
#ifdef OPLUS_FEATURE_CHG_BASIC
	if (aw86907->device_id == 832 ||
	    aw86907->device_id == 833) { //19065 and 19161
		aw86907->f0_pre = AW86907_0832_HAPTIC_F0_PRE;
		aw86907->f0_cali_percent = AW86907_0832_HAPTIC_F0_CALI_PERCEN;
		aw86907->cont_drv1_lvl = AW86907_0832_HAPTIC_CONT_DRV1_LVL;
		aw86907->cont_drv2_lvl = AW86907_0832_HAPTIC_CONT_DRV2_LVL;
		aw86907->cont_drv1_time = AW86907_0832_HAPTIC_CONT_DRV1_TIME;
		aw86907->cont_drv2_time = AW86907_0832_HAPTIC_CONT_DRV2_TIME;
		aw86907->cont_wait_num = AW86907_0832_HAPTIC_CONT_WAIT_NUM;
		aw86907->cont_brk_time = AW86907_0832_HAPTIC_CONT_BRK_TIME;
		aw86907->cont_track_margin =
			AW86907_0832_HAPTIC_CONT_TRACK_MARGIN;
		aw86907->cont_test = AW86907_0832_HAPTIC_CONT_TEST;
		aw86907->cont_drv_width = AW86907_0832_HAPTIC_CONT_DRV_WIDTH;
		aw86907->cont_bemf_set = AW86907_0832_HAPTIC_CONT_BEMF_SET;
		aw86907->cont_brk_gain = AW86907_0832_HAPTIC_CONT_BRK_GAIN;
		aw86907->cont_bst_brk_gain =
			AW86907_0832_HAPTIC_CONT_BST_BRK_GAIN;

		aw86907->d2s_gain = AW86907_0832_HAPTIC_D2S_GAIN;
	} else {
		aw86907->f0_pre = AW86907_0815_HAPTIC_F0_PRE;
		aw86907->f0_cali_percent = AW86907_0815_HAPTIC_F0_CALI_PERCEN;
		aw86907->cont_drv1_lvl = AW86907_0815_HAPTIC_CONT_DRV1_LVL;
		aw86907->cont_drv2_lvl = AW86907_0815_HAPTIC_CONT_DRV2_LVL;
		aw86907->cont_drv1_time = AW86907_0815_HAPTIC_CONT_DRV1_TIME;
		aw86907->cont_drv2_time = AW86907_0815_HAPTIC_CONT_DRV2_TIME;
		aw86907->cont_wait_num = AW86907_0815_HAPTIC_CONT_WAIT_NUM;
		aw86907->cont_brk_time = AW86907_0815_HAPTIC_CONT_BRK_TIME;
		aw86907->cont_track_margin =
			AW86907_0815_HAPTIC_CONT_TRACK_MARGIN;
		aw86907->cont_test = AW86907_0815_HAPTIC_CONT_TEST;
		aw86907->cont_drv_width = AW86907_0815_HAPTIC_CONT_DRV_WIDTH;
		aw86907->cont_bemf_set = AW86907_0815_HAPTIC_CONT_BEMF_SET;
		aw86907->cont_brk_gain = AW86907_0815_HAPTIC_CONT_BRK_GAIN;
		aw86907->cont_bst_brk_gain =
			AW86907_0815_HAPTIC_CONT_BST_BRK_GAIN;

		aw86907->d2s_gain = AW86907_0815_HAPTIC_D2S_GAIN;
	}
	pr_err("%s get f0_pre=%d\n", __func__, aw86907->f0_pre);
	/* misc value init */

#endif

	aw86907->activate_mode = AW86907_HAPTIC_ACTIVATE_CONT_MODE;

	ret = aw86907_i2c_read(aw86907, AW86907_REG_WAVCFG1, &reg_val);
	aw86907->index = reg_val & 0x7F;
	ret = aw86907_i2c_read(aw86907, AW86907_REG_PLAYCFG2, &reg_val);
	aw86907->gain = reg_val & 0xFF;
	pr_info("%s aw86907->gain =0x%02X\n", __func__, aw86907->gain);

	ret = aw86907_i2c_read(aw86907, AW86907_REG_PLAYCFG1, &reg_val);
#ifdef OPLUS_FEATURE_CHG_BASIC
	aw86907->vmax = AW86907_HAPTIC_HIGH_LEVEL_REG_VAL;
#else
	aw86907->vmax = reg_val & 0x3F;
#endif
	for (i = 0; i < AW86907_SEQUENCER_SIZE; i++) {
		ret = aw86907_i2c_read(aw86907, AW86907_REG_WAVCFG1 + i,
				       &reg_val);
		aw86907->seq[i] = reg_val;
	}

	aw86907_haptic_play_mode(aw86907, AW86907_HAPTIC_STANDBY_MODE);

	aw86907_haptic_set_pwm(aw86907, AW86907_PWM_24K);

	aw86907_haptic_misc_para_init(aw86907);

	/* set BST_ADJ */
	aw86907_i2c_write_bits(aw86907, AW86907_REG_BSTCFG5,
			       AW86907_BIT_BSTCFG5_BST_ADJ_MASK,
			       AW86907_BIT_BSTCFG5_BST_ADJ_LOW);

	aw86907_haptic_set_bst_peak_cur(aw86907, AW86907_DEFAULT_PEAKCUR);

	aw86907_haptic_swicth_motorprotect_config(aw86907, 0x00, 0x00);

	aw86907_haptic_auto_boost_config(aw86907, false);

	aw86907_haptic_os_calibration(aw86907);

	aw86907_haptic_cont_vbat_mode(aw86907,
				      AW86907_HAPTIC_CONT_VBAT_HW_COMP_MODE);
	aw86907->ram_vbat_comp = AW86907_HAPTIC_RAM_VBAT_COMP_ENABLE;

	mutex_unlock(&aw86907->lock);
#ifdef AW_F0_COARSE_CALI /* Only Test for F0 calibration offset */
	aw86907_i2c_write_bits(aw86907, AW86907_REG_TRIMCFG3,
			       AW86907_BIT_TRIMCFG3_OSC_TRIM_SRC_MASK,
			       AW86907_BIT_TRIMCFG3_OSC_TRIM_SRC_REG);
	/* aw86907_i2c_write(aw86907, AW86907_REG_TRIMCFG3,0xCF); */
	/* 170-0xEC 235-0x06 260-0xfc */
	aw86907_i2c_write(aw86907, AW86907_REG_TRIMCFG4, 0xEC);
#endif

	mutex_lock(&aw86907->lock);
#ifndef OPLUS_FEATURE_CHG_BASIC
	/* f0 calibration */
	pr_info("%s powerup f0 calibration is enable!\n", __func__);
	aw86907_i2c_write_bits(aw86907, AW86907_REG_TRIMCFG3,
			       AW86907_BIT_TRIMCFG3_TRIM_LRA_MASK, 0);
	aw86907_haptic_f0_calibration(aw86907);
#endif
	mutex_unlock(&aw86907->lock);

	pr_info("%s out!\n", __func__);
	return ret;
}

/*****************************************************
 *
 * vibrator
 *
 *****************************************************/
#ifdef TIMED_OUTPUT
static int aw86907_vibrator_get_time(struct timed_output_dev *dev)
{
	struct aw86907 *aw86907 = container_of(dev, struct aw86907, to_dev);

	if (hrtimer_active(&aw86907->timer)) {
		ktime_t r = hrtimer_get_remaining(&aw86907->timer);
		return ktime_to_ms(r);
	}

	return 0;
}

static void aw86907_vibrator_enable(struct timed_output_dev *dev, int value)
{
	struct aw86907 *aw86907 = container_of(dev, struct aw86907, to_dev);
	unsigned char reg_val = 0;

	mutex_lock(&aw86907->lock);

	pr_debug("%s enter\n", __func__);
	/*RTP mode do not allow other vibrate begign*/
	if (aw86907->play_mode == AW86907_HAPTIC_RTP_MODE) {
		aw86907_i2c_read(aw86907, AW86907_REG_GLBRD5, &reg_val);
		if ((reg_val & 0x0f) != 0x00) {
			pr_info("%s RTP on && not stop,return\n", __func__);
			mutex_unlock(&aw86907->lock);
			return;
		}
	}
	/*RTP mode do not allow other vibrate end*/

	aw86907_haptic_stop(aw86907);

	if (value > 0) {
		aw86907_haptic_ram_vbat_comp(aw86907, false);
		aw86907_haptic_play_wav_seq(aw86907, value);
	}

	mutex_unlock(&aw86907->lock);

	pr_debug("%s exit\n", __func__);
}

#else
static enum led_brightness
aw86907_haptic_brightness_get(struct led_classdev *cdev)
{
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);

	return aw86907->amplitude;
}

static void aw86907_haptic_brightness_set(struct led_classdev *cdev,
					  enum led_brightness level)
{
	int rtp_is_going_on = 0;
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);

	rtp_is_going_on = aw86907_haptic_juge_RTP_is_going_on(aw86907);
	if (rtp_is_going_on) {
		return;
	}

	aw86907->amplitude = level;

	mutex_lock(&aw86907->lock);

	aw86907_haptic_stop(aw86907);
	if (aw86907->amplitude > 0) {
		aw86907_haptic_ram_vbat_comp(aw86907, false);
		aw86907_haptic_play_wav_seq(aw86907, aw86907->amplitude);
	}

	mutex_unlock(&aw86907->lock);
}
#endif

static ssize_t aw86907_f0_save_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"clock_system_f0_cali_lra = 0x%02X\n",
			aw86907->clock_system_f0_cali_lra);

	return len;
}

static ssize_t aw86907_f0_save_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	aw86907->clock_system_f0_cali_lra = val;
	return count;
}

static ssize_t aw86907_state_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif

	return snprintf(buf, PAGE_SIZE, "%d\n", aw86907->state);
}

static ssize_t aw86907_state_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	return count;
}

static ssize_t aw86907_duration_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	ktime_t time_rem;
	s64 time_ms = 0;

	if (hrtimer_active(&aw86907->timer)) {
		time_rem = hrtimer_get_remaining(&aw86907->timer);
		time_ms = ktime_to_ms(time_rem);
	}

	return snprintf(buf, PAGE_SIZE, "%lld\n", time_ms);
}

static ssize_t aw86907_duration_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

#ifdef OPLUS_FEATURE_CHG_BASIC
	pr_err("%s: value=%d\n", __FUNCTION__, val);
#endif
	/* setting 0 on duration is NOP for now */
	if (val <= 0)
		return count;

	aw86907->duration = val;

	return count;
}

static ssize_t aw86907_activate_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif

	/* For now nothing to show */
	return snprintf(buf, PAGE_SIZE, "%d\n", aw86907->state);
}

static int aw86907_haptic_juge_RTP_is_going_on(struct aw86907 *aw86907)
{
	unsigned char reg_val1 = 0;
	unsigned char reg_val2 = 0;
	unsigned char rtp_state = 0;

	aw86907_i2c_read(aw86907, AW86907_REG_PLAYCFG3, &reg_val1);
	aw86907_i2c_read(aw86907, AW86907_REG_GLBRD5, &reg_val2);

	if ((reg_val1 & AW86907_BIT_PLAYCFG3_PLAY_MODE_RTP) &&
	    ((reg_val2 & 0x0f) != AW86907_BIT_GLBRD5_STATE_STANDBY)) {
		rtp_state = 1; // is going on
		pr_info("%s:rtp & wakeup mode !\n", __func__);
	}
	if (aw86907->rtp_routine_on) {
		pr_info("%s:rtp_routine_on\n", __func__);
		rtp_state = 1; /*is going on*/
	}

	return rtp_state;
}

static ssize_t aw86907_activate_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;
	int rtp_is_going_on = 0;

	rtp_is_going_on = aw86907_haptic_juge_RTP_is_going_on(aw86907);
	if (rtp_is_going_on) {
		return count;
	}

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	if (val != 0 && val != 1)
		return count;

	pr_err("%s: value=%d\n", __FUNCTION__, val);

	if (0 == val) {
		mdelay(10);
	}

	hrtimer_cancel(&aw86907->timer);

	aw86907->state = val;
#ifdef OPLUS_FEATURE_CHG_BASIC
	if (aw86907->state) {
		pr_err("%s: aw86907->gain=0x%02x\n", __FUNCTION__,
		       aw86907->gain);
		if (aw86907->gain >= AW86907_HAPTIC_RAM_VBAT_COMP_GAIN) {
			aw86907->gain = AW86907_HAPTIC_RAM_VBAT_COMP_GAIN;
		}
		mutex_lock(&aw86907->lock);

		if (aw86907->device_id == 815)
			aw86907_haptic_set_gain(aw86907, aw86907->gain);
		aw86907_haptic_set_repeat_wav_seq(
			aw86907, AW86907_WAVEFORM_INDEX_SINE_CYCLE);

		mutex_unlock(&aw86907->lock);
		cancel_work_sync(&aw86907->vibrator_work);
		//schedule_work(&aw86907->vibrator_work);
		queue_work(system_highpri_wq, &aw86907->vibrator_work);
	} else {
		mutex_lock(&aw86907->lock);
		aw86907_haptic_stop(aw86907);
		mutex_unlock(&aw86907->lock);
	}
#endif

	return count;
}

static ssize_t aw86907_activate_mode_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif

	return snprintf(buf, PAGE_SIZE, "activate_mode=%c\n",
			aw86907->activate_mode);
}

static ssize_t aw86907_activate_mode_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&aw86907->lock);
	aw86907->activate_mode = val;
	mutex_unlock(&aw86907->lock);
	return count;
}

static ssize_t aw86907_index_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	unsigned char reg_val = 0;
	aw86907_i2c_read(aw86907, AW86907_REG_WAVCFG1, &reg_val);
	aw86907->index = reg_val;

	return snprintf(buf, PAGE_SIZE, "%d\n", aw86907->index);
}

static ssize_t aw86907_index_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	pr_debug("%s: value=%d\n", __FUNCTION__, val);

	mutex_lock(&aw86907->lock);
	aw86907->index = val;
	aw86907_haptic_set_repeat_wav_seq(aw86907, aw86907->index);
	mutex_unlock(&aw86907->lock);
	return count;
}

static ssize_t aw86907_vmax_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", aw86907->vmax);
}

struct aw86907_vmax_map {
	int level;
	int vmax;
	int gain;
};
static struct aw86907_vmax_map vmax_map[] = {
	{ 800, 0x00, 0x40 },  { 900, 0x00, 0x49 },  { 1000, 0x00, 0x51 },
	{ 1100, 0x00, 0x5A }, { 1200, 0x00, 0x62 }, { 1300, 0x00, 0x6B },
	{ 1400, 0x00, 0x73 }, { 1500, 0x00, 0x7C }, { 1600, 0x02, 0x80 },
	{ 1700, 0x08, 0x80 }, { 1800, 0x0D, 0x80 }, { 1900, 0x12, 0x80 },
	{ 2000, 0x17, 0x80 }, { 2100, 0x1C, 0x80 }, { 2200, 0x21, 0x80 },
	{ 2300, 0x26, 0x80 }, { 2400, 0x2B, 0x80 },
};

int aw86907_convert_level_to_vmax(struct aw86907 *aw86907, int val)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(vmax_map); i++) {
		if (val == vmax_map[i].level) {
			aw86907->vmax = vmax_map[i].vmax;
			aw86907->gain = vmax_map[i].gain;
			break;
		}
	}
	if (i == ARRAY_SIZE(vmax_map)) {
		aw86907->vmax = vmax_map[i - 1].vmax;
		aw86907->gain = vmax_map[i - 1].gain;
	}

	return i;
}

static ssize_t aw86907_vmax_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	pr_err("%s: value=%d\n", __FUNCTION__, val);

	if (!aw86907)
		return -1;
	mutex_lock(&aw86907->lock);
#ifdef OPLUS_FEATURE_CHG_BASIC
	if (val <= 255) {
		aw86907->gain = (val * AW86907_HAPTIC_RAM_VBAT_COMP_GAIN) / 255;
	} else if (aw86907 && aw86907->support_promise_vib && val <= 2400) {
		aw86907_convert_level_to_vmax(aw86907, val);
	} else if (val <= AW86907_HAPTIC_LOW_LEVEL_VOL) {
		aw86907->vmax = AW86907_HAPTIC_LOW_LEVEL_REG_VAL;
		aw86907->gain = 0x16;
	} else if (val <= 1200) {
		aw86907->vmax = AW86907_HAPTIC_MEDIUM_LEVEL_REG_VAL;
		aw86907->gain = 0x40;
	} else if (val <= AW86907_HAPTIC_MEDIUM_LEVEL_VOL) {
		aw86907->vmax = 0x1c;
		aw86907->gain = 0x40;
	} else if (val <= 2000) {
		aw86907->vmax = 0x0E;
		aw86907->gain = 0x80;
	} else {
		aw86907->vmax = AW86907_HAPTIC_HIGH_LEVEL_REG_VAL;
		aw86907->gain = 0x80;
	}

	if (val == 2550) { // for old test only
		aw86907->gain = AW86907_HAPTIC_RAM_VBAT_COMP_GAIN;
	}
	//aw86907->vmax = AW86907_HAPTIC_HIGH_LEVEL_REG_VAL;
	//aw86907->gain = 0x80;
	aw86907_haptic_set_gain(aw86907, aw86907->gain);
	aw86907_haptic_set_bst_vol(aw86907, aw86907->vmax);
#else
	aw86907->vmax = val;
	aw86907_haptic_set_bst_vol(aw86907, aw86907->vmax);
#endif
	mutex_unlock(&aw86907->lock);
	pr_err("%s:aw86907->gain[0x%x], aw86907->vmax[0x%x] end\n",
	       __FUNCTION__, aw86907->gain, aw86907->vmax);
	return count;
}

static ssize_t aw86907_gain_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", aw86907->gain);
}

static ssize_t aw86907_gain_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	pr_err("%s: value=%d\n", __FUNCTION__, val);

	mutex_lock(&aw86907->lock);
	aw86907->gain = val;
	aw86907_haptic_set_gain(aw86907, aw86907->gain);
	mutex_unlock(&aw86907->lock);
	return count;
}

static ssize_t aw86907_seq_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	size_t count = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;

	for (i = 0; i < AW86907_SEQUENCER_SIZE; i++) {
		aw86907_i2c_read(aw86907, AW86907_REG_WAVCFG1 + i, &reg_val);
		count += snprintf(buf + count, PAGE_SIZE - count,
				  "seq%d: 0x%02x\n", i + 1, reg_val);
		aw86907->seq[i] |= reg_val;
	}
	return count;
}

static ssize_t aw86907_seq_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	unsigned int databuf[2] = { 0, 0 };

	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
		pr_debug("%s: seq%d=0x%x\n", __FUNCTION__, databuf[0],
			 databuf[1]);
		mutex_lock(&aw86907->lock);
		aw86907->seq[databuf[0]] = (unsigned char)databuf[1];
		aw86907_haptic_set_wav_seq(aw86907, (unsigned char)databuf[0],
					   aw86907->seq[databuf[0]]);
		mutex_unlock(&aw86907->lock);
	}
	return count;
}

static ssize_t aw86907_loop_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	size_t count = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;

	for (i = 0; i < AW86907_SEQUENCER_LOOP_SIZE; i++) {
		aw86907_i2c_read(aw86907, AW86907_REG_WAVCFG9 + i, &reg_val);
		aw86907->loop[i * 2 + 0] = (reg_val >> 4) & 0x0F;
		aw86907->loop[i * 2 + 1] = (reg_val >> 0) & 0x0F;

		count += snprintf(buf + count, PAGE_SIZE - count,
				  "seq%d loop: 0x%02x\n", i * 2 + 1,
				  aw86907->loop[i * 2 + 0]);
		count += snprintf(buf + count, PAGE_SIZE - count,
				  "seq%d loop: 0x%02x\n", i * 2 + 2,
				  aw86907->loop[i * 2 + 1]);
	}
	count += snprintf(buf + count, PAGE_SIZE - count, "loop: 0x%02x\n",
			  aw86907->rtp_loop);
	return count;
}

static ssize_t aw86907_loop_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	unsigned int databuf[2] = { 0, 0 };
	unsigned int val = 0;
	int rc = 0;
	aw86907->rtp_loop = 0;
	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
		pr_debug("%s: seq%d loop=0x%x\n", __FUNCTION__, databuf[0],
			 databuf[1]);
		mutex_lock(&aw86907->lock);
		aw86907->loop[databuf[0]] = (unsigned char)databuf[1];
		aw86907_haptic_set_wav_loop(aw86907, (unsigned char)databuf[0],
					    aw86907->loop[databuf[0]]);
		mutex_unlock(&aw86907->lock);

	} else {
		rc = kstrtouint(buf, 0, &val);
		if (rc < 0)
			return rc;
		aw86907->rtp_loop = val;
		printk("%s  aw86907->rtp_loop = 0x%x", __func__,
		       aw86907->rtp_loop);
	}

	return count;
}

static ssize_t aw86907_rtp_num_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	ssize_t len = 0;
	unsigned char i = 0;
	//  unsigned char reg_val = 0;
	for (i = 0; i < AW86907_RTP_NUM; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len,
				"num: %u, serial:%u \n", i,
				aw86907->rtp_serial[i]);
	}
	return len;
}

static ssize_t aw86907_rtp_num_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	/*datebuf[0] is connect number */
	/*databuf[1]- databuf[x] is sequence and number*/
	unsigned int databuf[AW86907_RTP_NUM] = {
		0, 0, 0, 0, 0, 0
	}; /*custom modify it,if you want*/
	unsigned int val = 0;
	int rc = 0;

	if (AW86907_RTP_NUM == sscanf(buf, "%x %x %x %x %x %x", &databuf[0],
				      &databuf[1], &databuf[2], &databuf[3],
				      &databuf[4], &databuf[5])) {
		for (val = 0; val < AW86907_RTP_NUM; val++) {
			printk("%s: databuf = %d \n", __FUNCTION__,
			       databuf[val]);
			aw86907->rtp_serial[val] = (unsigned char)databuf[val];
		}
	} else {
		rc = kstrtouint(buf, 0, &val);
		if (rc < 0)
			return rc;
		if (val == 0)
			aw86907->rtp_serial[0] = 0;
	}
	aw86907_rtp_regroup_work(aw86907);
	return count;
}

static ssize_t aw86907_reg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	ssize_t len = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;
	for (i = 0; i < AW86907_REG_MAX; i++) {
		if (!(aw86907_reg_access[i] & REG_RD_ACCESS))
			continue;
		aw86907_i2c_read(aw86907, i, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len,
				"reg:0x%02x=0x%02x \n", i, reg_val);
	}
	return len;
}

static ssize_t aw86907_reg_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	unsigned int databuf[2] = { 0, 0 };

	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
		aw86907_i2c_write(aw86907, (unsigned char)databuf[0],
				  (unsigned char)databuf[1]);
	}

	return count;
}

static ssize_t aw86907_rtp_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	ssize_t len = 0;
	len += snprintf(buf + len, PAGE_SIZE - len, "rtp play: %u\n",
			aw86907->rtp_cnt);

	return len;
}

static ssize_t aw86907_rtp_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;
	int rtp_is_going_on = 0;
	static bool mute = false;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	pr_err("%s: val [%d] \n", __func__, val);

	if (val == 1025) {
		mute = true;
		return count;
	} else if (val == 1026) {
		mute = false;
		return count;
	}
	if (val == 0)
		flush_work(&aw86907->rtp_work);

	mutex_lock(&aw86907->lock);
	/*OP add for juge rtp on begin*/
	rtp_is_going_on = aw86907_haptic_juge_RTP_is_going_on(aw86907);
	if (rtp_is_going_on && (val == AUDIO_READY_STATUS)) {
		pr_info("%s: seem audio status rtp[%d]\n", __FUNCTION__, val);
		mutex_unlock(&aw86907->lock);
		return count;
	}
	/*OP add for juge rtp on end*/
	if (((val >= RINGTONES_START_INDEX && val <= RINGTONES_END_INDEX) ||
	     (val >= NEW_RING_START && val <= NEW_RING_END) ||
	     (val >= OPLUS_RING_START && val <= OPLUS_RING_END) ||
	     (val >= OPLUS_NEW_RING_1_START && val <= OPLUS_NEW_RING_1_END) ||
	     (val >= OPLUS_NEW_RING_2_START && val <= OPLUS_NEW_RING_2_END) ||
	     val == RINGTONES_SIMPLE_INDEX || val == RINGTONES_PURE_INDEX ||
	     val == AUDIO_READY_STATUS)) {
		if (val == AUDIO_READY_STATUS)
			aw86907->audio_ready = true;
		else
			aw86907->haptic_ready = true;
		pr_info("%s:audio[%d]and haptic[%d] ready\n", __FUNCTION__,
			aw86907->audio_ready, aw86907->haptic_ready);
		if (aw86907->haptic_ready && !aw86907->audio_ready) {
			aw86907->pre_haptic_number = val;
		}
		if (!aw86907->audio_ready || !aw86907->haptic_ready) {
			mutex_unlock(&aw86907->lock);
			return count;
		}
	}
	if (val == AUDIO_READY_STATUS && aw86907->pre_haptic_number) {
		pr_info("pre_haptic_number:%d\n", aw86907->pre_haptic_number);
		val = aw86907->pre_haptic_number;
	}
	if (!val) {
		aw86907_op_clean_status(aw86907);
	}
	aw86907_haptic_stop(aw86907);

	aw86907_haptic_set_rtp_aei(aw86907, false);
	aw86907_interrupt_clear(aw86907);
	mutex_unlock(&aw86907->lock);
	if (val < (sizeof(aw86907_rtp_name) / AW86907_RTP_NAME_MAX)) {
		aw86907->rtp_file_num = val;
		if (val) {
			//schedule_work(&aw86907->rtp_work);
			queue_work(system_unbound_wq, &aw86907->rtp_work);
		}
	} else {
		pr_err("%s: rtp_file_num 0x%02x over max value \n", __func__,
		       aw86907->rtp_file_num);
	}

	return count;
}

static ssize_t aw86907_ram_update_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	//struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	//struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	//struct led_classdev *cdev = dev_get_drvdata(dev);
	//struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	ssize_t len = 0;
	len += snprintf(buf + len, PAGE_SIZE - len, "sram update mode\n");
	return len;
}

static ssize_t aw86907_ram_update_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	if (val) {
		aw86907_ram_update(aw86907);
	}
	return count;
}

static ssize_t aw86907_f0_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	ssize_t len = 0;

	mutex_lock(&aw86907->lock);
	aw86907->f0_cali_flag = AW86907_HAPTIC_CALI_F0; //AW8697_HAPTIC_LRA_F0;
	aw86907_haptic_get_f0(aw86907);
	mutex_unlock(&aw86907->lock);

#ifdef OPLUS_FEATURE_CHG_BASIC
	len += snprintf(buf + len, PAGE_SIZE - len, "%u\n", aw86907->f0);
#else
	len += snprintf(buf + len, PAGE_SIZE - len,
			"aw86907 lra f0 = %u cont_f0 = %u\n", aw86907->f0,
			aw86907->cont_f0);
#endif
	return len;
}

static ssize_t aw86907_f0_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	pr_err("%s:  f0 = %d\n", __FUNCTION__, val);

	if (aw86907->device_id == 815) {
		aw86907->f0 = val;
		if (aw86907->f0 < F0_VAL_MIN_0815 ||
		    aw86907->f0 > F0_VAL_MAX_0815) {
			aw86907->f0 = 1700;
		}
	} else if (aw86907->device_id == 832) {
		aw86907->f0 = val;
		if (aw86907->f0 < F0_VAL_MIN_0832 ||
		    aw86907->f0 > F0_VAL_MAX_0832) {
			aw86907->f0 = 2300;
		}
	} else if (aw86907->device_id == 833) {
		aw86907->f0 = val;
		if (aw86907->f0 < F0_VAL_MIN_0833 ||
		    aw86907->f0 > F0_VAL_MAX_0833) {
			aw86907->f0 = 2330;
		}
	}
	aw86907_ram_update(aw86907);

	return count;
}

static ssize_t aw86907_cali_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	ssize_t len = 0;
	mutex_lock(&aw86907->lock);
	aw86907->f0_cali_flag = AW86907_HAPTIC_CALI_F0;
	aw86907_set_clock(aw86907, AW86907_HAPTIC_CLOCK_CALI_F0);
	aw86907_haptic_get_f0(aw86907);
	mutex_unlock(&aw86907->lock);
	len += snprintf(buf + len, PAGE_SIZE - len,
			"aw86907 lra_f0 = %u cont_f0 = %u\n", aw86907->f0,
			aw86907->cont_f0);
	return len;
}

static ssize_t aw86907_cali_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	if (val) {
		mutex_lock(&aw86907->lock);
		aw86907->clock_system_f0_cali_lra = 0;
		aw86907_haptic_f0_calibration(aw86907);
		mutex_unlock(&aw86907->lock);
	}
	return count;
}

static ssize_t aw86907_cont_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	ssize_t len = 0;
	aw86907_haptic_read_cont_f0(aw86907);
	len += snprintf(buf + len, PAGE_SIZE - len, "aw86907 cont_f0 = %u\n",
			aw86907->cont_f0);
	return len;
}

static ssize_t aw86907_cont_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	mutex_lock(&aw86907->lock);
	aw86907_haptic_stop(aw86907);
	if (val) {
		aw86907_haptic_cont(aw86907);
	}
	mutex_unlock(&aw86907->lock);
	return count;
}

static ssize_t aw86907_cont_wait_num_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "cont_wait_num = 0x%02X\n",
			aw86907->cont_wait_num);
	return len;
}

static ssize_t aw86907_cont_wait_num_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	unsigned int databuf[1] = { 0 };

	if (sscanf(buf, "%x", &databuf[0]) == 1) {
		aw86907->cont_wait_num = databuf[0];
		aw86907_i2c_write(aw86907, AW86907_REG_CONTCFG4, databuf[0]);
	}
	return count;
}

static ssize_t aw86907_cont_drv_lvl_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"cont_drv1_lvl = 0x%02X, cont_drv2_lvl = 0x%02X\n",
			aw86907->cont_drv1_lvl, aw86907->cont_drv2_lvl);
	return len;
}

static ssize_t aw86907_cont_drv_lvl_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	unsigned int databuf[2] = { 0, 0 };

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		aw86907->cont_drv1_lvl = databuf[0];
		aw86907->cont_drv2_lvl = databuf[1];
		aw86907_i2c_write_bits(aw86907, AW86907_REG_CONTCFG6,
				       AW86907_BIT_CONTCFG6_DRV1_LVL_MASK,
				       aw86907->cont_drv1_lvl);
		aw86907_i2c_write(aw86907, AW86907_REG_CONTCFG7,
				  aw86907->cont_drv2_lvl);
	}
	return count;
}

static ssize_t aw86907_cont_drv_time_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"cont_drv1_time = 0x%02X, cont_drv2_time = 0x%02X\n",
			aw86907->cont_drv1_time, aw86907->cont_drv2_time);
	return len;
}

static ssize_t aw86907_cont_drv_time_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	unsigned int databuf[2] = { 0, 0 };

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		aw86907->cont_drv1_time = databuf[0];
		aw86907->cont_drv2_time = databuf[1];
		aw86907_i2c_write(aw86907, AW86907_REG_CONTCFG8,
				  aw86907->cont_drv1_time);
		aw86907_i2c_write(aw86907, AW86907_REG_CONTCFG9,
				  aw86907->cont_drv2_time);
	}
	return count;
}

static ssize_t aw86907_cont_brk_time_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "cont_brk_time = 0x%02X\n",
			aw86907->cont_brk_time);
	return len;
}

static ssize_t aw86907_cont_brk_time_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	unsigned int databuf[1] = { 0 };

	if (sscanf(buf, "%x", &databuf[0]) == 1) {
		aw86907->cont_brk_time = databuf[0];
		aw86907_i2c_write(aw86907, AW86907_REG_CONTCFG10,
				  aw86907->cont_brk_time);
	}
	return count;
}

static ssize_t aw86907_vbat_monitor_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	ssize_t len = 0;

	mutex_lock(&aw86907->lock);
	aw86907_haptic_stop(aw86907);
	aw86907_haptic_get_vbat(aw86907);
	len += snprintf(buf + len, PAGE_SIZE - len, "vbat=%umV\n",
			aw86907->vbat);
	mutex_unlock(&aw86907->lock);

	return len;
}

static ssize_t aw86907_vbat_monitor_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	return count;
}

static ssize_t aw86907_lra_resistance_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	ssize_t len = 0;
	unsigned char d2s_gain_temp = 0;
	unsigned char reg_val = 0;
	unsigned int lra_code = 0;

	mutex_lock(&aw86907->lock);
	aw86907_haptic_stop(aw86907);

	aw86907_i2c_read(aw86907, AW86907_REG_SYSCTRL7, &reg_val);
	d2s_gain_temp = 0x07 & reg_val;
	aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSCTRL7,
			       AW86907_BIT_SYSCTRL7_D2S_GAIN_MASK,
			       aw86907->d2s_gain);
	aw86907_i2c_read(aw86907, AW86907_REG_SYSCTRL7, &reg_val);
	pr_info("%s: d2s_gain=%d\n", __func__, 0x07 & reg_val);

	aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSCTRL1,
			       AW86907_BIT_SYSCTRL1_RAMINIT_MASK,
			       AW86907_BIT_SYSCTRL1_RAMINIT_ON);

	/* enter standby mode */
	aw86907_haptic_stop(aw86907);
	usleep_range(2000, 2500);
	aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSCTRL2,
			       AW86907_BIT_SYSCTRL2_STANDBY_MASK,
			       AW86907_BIT_SYSCTRL2_STANDBY_OFF);
	aw86907_i2c_write_bits(aw86907, AW86907_REG_DETCFG1,
			       AW86907_BIT_DETCFG1_RL_OS_MASK,
			       AW86907_BIT_DETCFG1_RL);
	aw86907_i2c_write_bits(aw86907, AW86907_REG_DETCFG2,
			       AW86907_BIT_DETCFG2_DIAG_GO_MASK,
			       AW86907_BIT_DETCFG2_DIAG_GO_ON);
	usleep_range(30000, 35000);
	aw86907_i2c_read(aw86907, AW86907_REG_DET_RL, &reg_val);
	lra_code = (lra_code | reg_val) << 2;
	aw86907_i2c_read(aw86907, AW86907_REG_DET_LO, &reg_val);
	lra_code = lra_code | (reg_val & 0x03);
	/* 2num */
	aw86907->lra = (lra_code * 678 * 1000) / (1024 * 10);

#ifdef OPLUS_FEATURE_CHG_BASIC
	len += snprintf(buf + len, PAGE_SIZE - len, "%u\n", aw86907->lra);
#else
	len += snprintf(buf + len, PAGE_SIZE - len, "r_lra=%dmohm\n",
			aw86907->lra);
#endif
	aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSCTRL1,
			       AW86907_BIT_SYSCTRL1_RAMINIT_MASK,
			       AW86907_BIT_SYSCTRL1_RAMINIT_OFF);

	aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSCTRL7,
			       AW86907_BIT_SYSCTRL7_D2S_GAIN_MASK,
			       d2s_gain_temp);
	mutex_unlock(&aw86907->lock);

	return len;
}

static ssize_t aw86907_lra_resistance_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	return count;
}

static ssize_t aw86907_auto_boost_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "auto_boost=%c\n",
			aw86907->auto_boost);

	return len;
}

static ssize_t aw86907_auto_boost_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&aw86907->lock);
	aw86907_haptic_stop(aw86907);
	aw86907_haptic_auto_boost_config(aw86907, val);
	mutex_unlock(&aw86907->lock);

	return count;
}

static ssize_t aw86907_prctmode_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	ssize_t len = 0;
	unsigned char reg_val = 0;

	aw86907_i2c_read(aw86907, AW86907_REG_DETCFG1, &reg_val);

	len += snprintf(buf + len, PAGE_SIZE - len, "prctmode=%d\n",
			reg_val & 0x08);
	return len;
}

static ssize_t aw86907_prctmode_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	unsigned int databuf[2] = { 0, 0 };
	unsigned int addr = 0;
	unsigned int val = 0;
	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
		addr = databuf[0];
		val = databuf[1];
		mutex_lock(&aw86907->lock);
		aw86907_haptic_swicth_motorprotect_config(aw86907, addr, val);
		mutex_unlock(&aw86907->lock);
	}
	return count;
}

static ssize_t aw86907_ram_vbat_comp_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "ram_vbat_comp=%c\n",
			aw86907->ram_vbat_comp);

	return len;
}

static ssize_t aw86907_ram_vbat_comp_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&aw86907->lock);
	if (val) {
		aw86907->ram_vbat_comp = AW86907_HAPTIC_RAM_VBAT_COMP_ENABLE;
	} else {
		aw86907->ram_vbat_comp = AW86907_HAPTIC_RAM_VBAT_COMP_DISABLE;
	}
	mutex_unlock(&aw86907->lock);

	return count;
}

#ifdef OPLUS_FEATURE_CHG_BASIC
static ssize_t aw86907_f0_data_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "%u\n",
			aw86907->clock_system_f0_cali_lra);
	return len;
}

static ssize_t aw86907_f0_data_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	pr_err("%s:  f0 = %d\n", __FUNCTION__, val);

	aw86907->clock_system_f0_cali_lra = val;

	mutex_lock(&aw86907->lock);
	aw86907_i2c_write_bits(aw86907, AW86907_REG_TRIMCFG3,
			       AW86907_BIT_TRIMCFG3_TRIM_LRA_MASK,
			       aw86907->clock_system_f0_cali_lra);
	mutex_unlock(&aw86907->lock);

	return count;
}

static ssize_t aw86907_rtp_going_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	ssize_t len = 0;
	int val = -1;

	val = aw86907_haptic_juge_RTP_is_going_on(aw86907);
	len += snprintf(buf + len, PAGE_SIZE - len, "%d\n", val);
	return len;
}

static ssize_t aw86907_rtp_going_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	return count;
}

static ssize_t aw86907_osc_cali_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	ssize_t len = 0;

	printk("aw86907_osc_cali_show: 2018_microsecond:%ld \n",
	       aw86907->microsecond);
	len += snprintf(buf + len, PAGE_SIZE - len, "%lu\n",
			aw86907->microsecond);

	return len;
}
#endif

static ssize_t aw86907_osc_cali_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	unsigned int val = 0;

	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&aw86907->lock);
	if (val == 3) {
		aw86907_i2c_write_bits(aw86907, AW86907_REG_D2SCFG1,
				       AW86907_BIT_D2SCFG1_CLK_TRIM_MODE_MASK,
				       AW86907_BIT_D2SCFG1_CLK_TRIM_MODE_24K);
		aw86907_i2c_write_bits(aw86907, AW86907_REG_TRIMCFG3,
				       AW86907_BIT_TRIMCFG3_TRIM_LRA_MASK, 0);
		aw86907->clock_standard_osc_lra_rtim_code = 0;
		aw86907_rtp_osc_calibration(aw86907);
		aw86907_rtp_trim_lra_calibration(aw86907);
	}
	if (val == 1)
		aw86907_rtp_osc_calibration(aw86907);

	mutex_unlock(&aw86907->lock);

	return count;
}

static ssize_t aw86907_haptic_audio_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	ssize_t len = 0;
	len += snprintf(buf + len, PAGE_SIZE - len, "%c\n",
			aw86907->haptic_audio.ctr.cnt);
	return len;
}

static ssize_t aw86907_haptic_audio_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	unsigned int databuf[6] = { 0 };
	struct haptic_ctr *hap_ctr = NULL;

	if (aw86907->rtp_on)
		return count;
	if (!aw86907->ram_init)
		return count;

	if (!aw86907->ram_init)
		return count;
	if (6 == sscanf(buf, "%d %d %d %d %d %d", &databuf[0], &databuf[1],
			&databuf[2], &databuf[3], &databuf[4], &databuf[5])) {
		if (databuf[2]) {
			pr_debug(
				"%s: cnt=%d, cmd=%d, play=%d, wavseq=%d, loop=%d, gain=%d\n",
				__func__, databuf[0], databuf[1], databuf[2],
				databuf[3], databuf[4], databuf[5]);
		}

		hap_ctr = (struct haptic_ctr *)kzalloc(
			sizeof(struct haptic_ctr), GFP_KERNEL);
		if (hap_ctr == NULL) {
			pr_err("%s: kzalloc memory fail\n", __func__);
			return count;
		}
		mutex_lock(&aw86907->haptic_audio.lock);
		hap_ctr->cnt = (unsigned char)databuf[0];
		hap_ctr->cmd = (unsigned char)databuf[1];
		hap_ctr->play = (unsigned char)databuf[2];
		hap_ctr->wavseq = (unsigned char)databuf[3];
		hap_ctr->loop = (unsigned char)databuf[4];
		hap_ctr->gain = (unsigned char)databuf[5];
		aw86907_haptic_audio_ctr_list_insert(&aw86907->haptic_audio,
						     hap_ctr);

		if (hap_ctr->cmd == 0xff) {
			pr_debug("%s: haptic_audio stop\n", __func__);
			if (hrtimer_active(&aw86907->haptic_audio.timer)) {
				pr_debug("%s: cancel haptic_audio_timer\n",
					 __func__);
				hrtimer_cancel(&aw86907->haptic_audio.timer);
				aw86907->haptic_audio.ctr.cnt = 0;
				aw86907_haptic_audio_off(aw86907);
			}
		} else {
			if (hrtimer_active(&aw86907->haptic_audio.timer)) {
			} else {
				pr_debug("%s: start haptic_audio_timer\n",
					 __func__);
				aw86907_haptic_audio_init(aw86907);
				hrtimer_start(
					&aw86907->haptic_audio.timer,
					ktime_set(
						aw86907->haptic_audio.delay_val /
							1000000,
						(aw86907->haptic_audio.delay_val %
						 1000000) *
							1000),
					HRTIMER_MODE_REL);
			}
		}
		mutex_unlock(&aw86907->haptic_audio.lock);
		kfree(hap_ctr);
	}
	return count;
}

static ssize_t aw86907_haptic_audio_time_show(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	ssize_t len = 0;
	len += snprintf(buf + len, PAGE_SIZE - len,
			"haptic_audio.delay_val=%dus\n",
			aw86907->haptic_audio.delay_val);
	len += snprintf(buf + len, PAGE_SIZE - len,
			"haptic_audio.timer_val=%dus\n",
			aw86907->haptic_audio.timer_val);
	return len;
}

static ssize_t aw86907_haptic_audio_time_store(struct device *dev,
					       struct device_attribute *attr,
					       const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	unsigned int databuf[2] = { 0 };

	if (aw86907->rtp_on)
		return count;
	if (!aw86907->ram_init)
		return count;

	if (2 == sscanf(buf, "%d %d", &databuf[0], &databuf[1])) {
		aw86907->haptic_audio.delay_val = databuf[0];
		aw86907->haptic_audio.timer_val = databuf[1];
	}
	return count;
}

static ssize_t aw86907_haptic_audio_tp_time_show(struct device *dev,
						 struct device_attribute *attr,
						 char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	struct tp *tp = &(aw86907->haptic_audio.tp);
	ssize_t len = 0;
	len += snprintf(buf + len, PAGE_SIZE - len,
			"tp->press_delay_min=%uus\n", tp->press_delay_min);
	len += snprintf(buf + len, PAGE_SIZE - len,
			"tp->press_delay_max=%uus\n", tp->press_delay_max);
	len += snprintf(buf + len, PAGE_SIZE - len,
			"tp->release_delay_max=%uus\n", tp->release_delay_max);
	len += snprintf(buf + len, PAGE_SIZE - len, "tp->no_play_cnt_max=%u\n",
			tp->no_play_cnt_max);
	return len;
}

static ssize_t aw86907_haptic_audio_tp_time_store(struct device *dev,
						  struct device_attribute *attr,
						  const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	unsigned int databuf[4] = { 0 };
	struct tp *tp = &(aw86907->haptic_audio.tp);

	if (aw86907->rtp_on)
		return count;
	if (!aw86907->ram_init)
		return count;

	if (4 == sscanf(buf, "%d %d %d %d", &databuf[0], &databuf[1],
			&databuf[2], &databuf[3])) {
		tp->press_delay_min = databuf[0];
		tp->press_delay_max = databuf[1];
		tp->release_delay_max = databuf[2];
		tp->no_play_cnt_max = databuf[3];
		pr_info("tp->press_delay_min=%dus\n", tp->press_delay_min);
		pr_info("tp->press_delay_max=%dus\n", tp->press_delay_max);
		pr_info("tp->release_delay_max=%dus\n", tp->release_delay_max);
		pr_info("tp->no_play_cnt_max=%dus\n", tp->no_play_cnt_max);
	}
	return count;
}

static ssize_t aw86907_haptic_audio_tp_input_show(struct device *dev,
						  struct device_attribute *attr,
						  char *buf)
{
#ifdef TIMED_OUTPUT
	//struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	//struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	//struct led_classdev *cdev = dev_get_drvdata(dev);
	//struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	ssize_t len = 0;
	return len;
}

static ssize_t
aw86907_haptic_audio_tp_input_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	struct tp_input_info tp_input;
	struct tp *tp = &(aw86907->haptic_audio.tp);
	struct haptic_audio *haptic_audio = NULL;
	struct haptic_audio_trust_zone *p_tmp = NULL;

	if (aw86907->rtp_on)
		return count;
	if (!aw86907->ram_init)
		return count;

	haptic_audio = &(aw86907->haptic_audio);

	if (count != sizeof(struct tp_input_info)) {
		pr_err("%s: unmatch buf len, node_len=%d, struct len=%d\n",
		       __func__, count, sizeof(struct tp_input_info));
		return count;
	}

	memcpy(&tp_input, buf, sizeof(struct tp_input_info));
	pr_debug("%s: id[%d]: status=%d, x=%d, y=%d\n", __func__, tp_input.id,
		 tp_input.status, tp_input.x, tp_input.y);

	mutex_lock(&aw86907->haptic_audio.lock);
	if ((AW86907_HAPTIC_TP_ST_PRESS == tp_input.status) &&
	    (tp_input.status != tp->id[tp_input.id].pt_info.status)) {
		tp->tp_ai_check_flag = 1;
		tp->id[tp_input.id].tp_ai_match_flag = 0;
		list_for_each_entry (p_tmp, &(haptic_audio->list), list) {
			if ((tp_input.x + AW86907_HAPTIC_AI_X_JITTER *
						  p_tmp->w /
						  AW86907_HAPTIC_AI_X_DFT_W >
			     p_tmp->x) &&
			    (tp_input.x <
			     p_tmp->x + p_tmp->w +
				     AW86907_HAPTIC_AI_X_JITTER * p_tmp->w /
					     AW86907_HAPTIC_AI_X_DFT_W) &&
			    (tp_input.y + AW86907_HAPTIC_AI_Y_JITTER *
						  p_tmp->h /
						  AW86907_HAPTIC_AI_Y_DFT_H >
			     p_tmp->y) &&
			    (tp_input.y <
			     p_tmp->y + p_tmp->h +
				     AW86907_HAPTIC_AI_Y_JITTER * p_tmp->h /
					     AW86907_HAPTIC_AI_Y_DFT_H)) {
				pr_debug(
					"%s: tp input point[%d, %04d, %04d] is in the ai trust zone[%d, %04d, %04d, %04d, %04d]\n",
					__func__, tp_input.id, tp_input.x,
					tp_input.y, p_tmp->level, p_tmp->x,
					p_tmp->y, p_tmp->w, p_tmp->h);
				tp->id[tp_input.id].tp_ai_match_flag = 1;
				do_gettimeofday(
					&tp->id[AW86907_HAPTIC_TP_ID_MAX]
						 .t_press);
				tp->virtual_id = tp_input.id;
				tp->id[AW86907_HAPTIC_TP_ID_MAX].pt_info.id =
					tp_input.id;
				tp->id[AW86907_HAPTIC_TP_ID_MAX].pt_info.x =
					tp_input.x;
				tp->id[AW86907_HAPTIC_TP_ID_MAX].pt_info.y =
					tp_input.y;
				tp->id[AW86907_HAPTIC_TP_ID_MAX].pt_info.status =
					tp_input.status;
				tp->id[AW86907_HAPTIC_TP_ID_MAX]
					.pt_info.touch_flag =
					AW86907_HAPTIC_TP_TOUCH_VAIL;
				tp->id[AW86907_HAPTIC_TP_ID_MAX]
					.pt_info.touch_outside_tz_flag =
					AW86907_HAPTIC_TP_TOUCH_VAIL;
				tp->id[AW86907_HAPTIC_TP_ID_MAX].tp_flag =
					AW86907_HAPTIC_TP_PRESS;
				tp->id[AW86907_HAPTIC_TP_ID_MAX].press_flag =
					AW86907_HAPTIC_TP_PRESS;
				tp->id[AW86907_HAPTIC_TP_ID_MAX].release_flag =
					AW86907_HAPTIC_TP_NULL;
				tp->id[AW86907_HAPTIC_TP_ID_MAX].no_play_cnt =
					0;
				tp->id[AW86907_HAPTIC_TP_ID_MAX]
					.press_no_vibrate_flag = 0;
				break;
			}
		}
		if (tp->id[tp_input.id].tp_ai_match_flag == 0) {
			pr_debug(
				"%s: tp input point[%d, %04d, %04d] is out the trust zone\n",
				__func__, tp_input.id, tp_input.x, tp_input.y);
		}
		tp->id[tp_input.id].pt_info.id = tp_input.id;
		tp->id[tp_input.id].pt_info.x = tp_input.x;
		tp->id[tp_input.id].pt_info.y = tp_input.y;
		tp->id[tp_input.id].pt_info.status = tp_input.status;
		tp->id[tp_input.id].pt_info.touch_flag =
			AW86907_HAPTIC_TP_TOUCH_VAIL;
		tp->id[tp_input.id].pt_info.touch_outside_tz_flag =
			AW86907_HAPTIC_TP_TOUCH_VAIL;
		tp->id[tp_input.id].tp_flag = AW86907_HAPTIC_TP_PRESS;
		tp->id[tp_input.id].press_flag = AW86907_HAPTIC_TP_PRESS;
		tp->id[tp_input.id].release_flag = AW86907_HAPTIC_TP_NULL;
		tp->id[tp_input.id].no_play_cnt = 0;
		tp->id[tp_input.id].press_no_vibrate_flag = 0;
		do_gettimeofday(&tp->id[tp_input.id].t_press);
		pr_debug("%s: tp_press: status=%d, flag=%d", __func__,
			 tp->id[tp_input.id].pt_info.status,
			 tp->id[tp_input.id].tp_flag);
	} else if ((AW86907_HAPTIC_TP_ST_RELEASE == tp_input.status) &&
		   (tp_input.status != tp->id[tp_input.id].pt_info.status)) {
		list_for_each_entry (p_tmp, &(haptic_audio->list), list) {
			if ((tp->id[tp_input.id].pt_info.x +
				     AW86907_HAPTIC_AI_X_JITTER * p_tmp->w /
					     AW86907_HAPTIC_AI_X_DFT_W >
			     p_tmp->x) &&
			    (tp->id[tp_input.id].pt_info.x <
			     p_tmp->x + p_tmp->w +
				     AW86907_HAPTIC_AI_X_JITTER * p_tmp->w /
					     AW86907_HAPTIC_AI_X_DFT_W) &&
			    (tp->id[tp_input.id].pt_info.y +
				     AW86907_HAPTIC_AI_Y_JITTER * p_tmp->h /
					     AW86907_HAPTIC_AI_Y_DFT_H >
			     p_tmp->y) &&
			    (tp->id[tp_input.id].pt_info.y <
			     p_tmp->y + p_tmp->h +
				     AW86907_HAPTIC_AI_Y_JITTER * p_tmp->h /
					     AW86907_HAPTIC_AI_Y_DFT_H)) {
				pr_debug(
					"%s: tp input point[%d, %04d, %04d] up is in the ai trust zone[%d, %04d, %04d, %04d, %04d]\n",
					__func__,
					tp->id[tp_input.id].pt_info.id,
					tp->id[tp_input.id].pt_info.x,
					tp->id[tp_input.id].pt_info.y,
					p_tmp->level, p_tmp->x, p_tmp->y,
					p_tmp->w, p_tmp->h);
				if (tp->virtual_id == tp_input.id) {
					tp->id[AW86907_HAPTIC_TP_ID_MAX]
						.pt_info.id =
						tp->id[tp_input.id].pt_info.id;
					tp->id[AW86907_HAPTIC_TP_ID_MAX]
						.pt_info.x =
						tp->id[tp_input.id].pt_info.x;
					tp->id[AW86907_HAPTIC_TP_ID_MAX]
						.pt_info.y =
						tp->id[tp_input.id].pt_info.y;
					tp->id[AW86907_HAPTIC_TP_ID_MAX]
						.pt_info.status =
						tp_input.status;
					tp->id[AW86907_HAPTIC_TP_ID_MAX]
						.tp_flag =
						AW86907_HAPTIC_TP_RELEASE;
					tp->id[AW86907_HAPTIC_TP_ID_MAX]
						.release_flag =
						AW86907_HAPTIC_TP_RELEASE;
					do_gettimeofday(
						&tp->id[AW86907_HAPTIC_TP_ID_MAX]
							 .t_release);
				}
				break;
			}
		}
		tp->id[tp_input.id].pt_info.status = tp_input.status;
		tp->id[tp_input.id].tp_flag = AW86907_HAPTIC_TP_RELEASE;
		tp->id[tp_input.id].release_flag = AW86907_HAPTIC_TP_RELEASE;
		do_gettimeofday(&tp->id[tp_input.id].t_release);
		pr_debug(
			"%s: tp input point[%d, %04d, %04d] up to [%d, %04d, %04d] \n",
			__func__, tp_input.id, tp->id[tp_input.id].pt_info.x,
			tp->id[tp_input.id].pt_info.y, tp_input.id, tp_input.x,
			tp_input.y);
		pr_debug("%s: tp_release: status=%d, flag=%d", __func__,
			 tp->id[tp_input.id].pt_info.status,
			 tp->id[tp_input.id].tp_flag);
	}
	mutex_unlock(&aw86907->haptic_audio.lock);

	return count;
}

static ssize_t aw86907_haptic_audio_ai_input_show(struct device *dev,
						  struct device_attribute *attr,
						  char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	struct haptic_audio *haptic_audio = NULL;
	struct haptic_audio_trust_zone *p_tmp = NULL;
	char *p_ai_data = NULL;
	ssize_t len = 0;
	int i = 0;
	uint8_t ai_tz_num = 0;

	mutex_lock(&aw86907->haptic_audio.lock);
	haptic_audio = &(aw86907->haptic_audio);
	ai_tz_num = 0;

	list_for_each_entry (p_tmp, &(haptic_audio->list), list) {
		ai_tz_num++;
	}
	pr_info("%s: ai_tz_num=%d\n", __func__, ai_tz_num);

	p_ai_data = (char *)kzalloc(
		sizeof(uint8_t) + ai_tz_num * sizeof(struct ai_trust_zone),
		GFP_KERNEL);
	if (!p_ai_data) {
		pr_err("%s: p_ai_data error allocating memory\n", __func__);
		mutex_unlock(&aw86907->haptic_audio.lock);
		return len;
	}

	memcpy(&p_ai_data[len], &ai_tz_num, sizeof(ai_tz_num));
	len += sizeof(ai_tz_num);

	list_for_each_entry (p_tmp, &(haptic_audio->list), list) {
		haptic_audio->output_tz_info[i].level = p_tmp->level;
		haptic_audio->output_tz_info[i].x = p_tmp->x;
		haptic_audio->output_tz_info[i].y = p_tmp->y;
		haptic_audio->output_tz_info[i].w = p_tmp->w;
		haptic_audio->output_tz_info[i].h = p_tmp->h;
		memcpy(&p_ai_data[len], &haptic_audio->output_tz_info[i],
		       sizeof(struct trust_zone_info));
		len += sizeof(struct trust_zone_info);
		pr_info("%s: trust zone [%d]: level=%d, x=%d, y=%d, w=%d, h=%d\n",
			__func__, i, haptic_audio->output_tz_info[i].level,
			haptic_audio->output_tz_info[i].x,
			haptic_audio->output_tz_info[i].y,
			haptic_audio->output_tz_info[i].w,
			haptic_audio->output_tz_info[i].h);
		i++;
	}
	memcpy(buf, p_ai_data, len);

	kfree(p_ai_data);
	mutex_unlock(&aw86907->haptic_audio.lock);

	return len;
}

static ssize_t
aw86907_haptic_audio_ai_input_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct haptic_audio_trust_zone *p_tmp = NULL;
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	int ret = -1;
	unsigned int i = 0;
	int match = 0;
	struct ai_trust_zone *ai_tz = NULL;
	struct trust_zone_info *ai_tz_info = NULL;
	struct haptic_audio *haptic_audio = NULL;

	if (aw86907->rtp_on)
		return count;
	if (!aw86907->ram_init)
		return count;

	haptic_audio = &(aw86907->haptic_audio);

	if (count != sizeof(struct ai_trust_zone)) {
		pr_err("%s: unmatch buf len, node_len=%d, struct len=%d\n",
		       __func__, count, sizeof(struct ai_trust_zone));
		return count;
	}

	mutex_lock(&aw86907->haptic_audio.lock);
	ai_tz = kzalloc(sizeof(struct ai_trust_zone), GFP_KERNEL);
	if (!ai_tz) {
		pr_err("%s: ai_tz error allocating memory\n", __func__);
		mutex_unlock(&aw86907->haptic_audio.lock);
		return count;
	}
	memcpy(ai_tz, buf, sizeof(struct ai_trust_zone));
	pr_info("%s: ai_tz num=%d\n", __func__, ai_tz->num);

	ai_tz_info = kzalloc(ai_tz->num * sizeof(struct trust_zone_info),
			     GFP_KERNEL);
	if (!ai_tz_info) {
		pr_err("%s: ai_tz_info error allocating memory\n", __func__);
		mutex_unlock(&aw86907->haptic_audio.lock);
		kfree(ai_tz);
		return count;
	}
	ret = copy_from_user(ai_tz_info, ai_tz->tz_info,
			     ai_tz->num * sizeof(struct trust_zone_info));
	if (ret) {
		kfree(ai_tz_info);
		kfree(ai_tz);
		dev_err(aw86907->dev, "%s: copy from user fail\n", __func__);
		mutex_unlock(&aw86907->haptic_audio.lock);
		return count;
	}

	aw86907->haptic_audio.tz_high_num = 0;
	aw86907->haptic_audio.hap_cnt_outside_tz = 0;
	if (ai_tz->num == 2) {
		aw86907->haptic_audio.tz_high_num = 2;
		if (haptic_audio->uevent_report_flag) {
			aw86907_haptic_audio_uevent_report_scan(aw86907, false);
			haptic_audio->uevent_report_flag = 0;
		}
	}

#ifdef SCORE_MODE

#else
	/* clear unused list link */
	aw86907_haptic_audio_tz_list_clear(&aw86907->haptic_audio);
#endif

	for (i = 0; i < ai_tz->num; i++) {
		pr_info("%s: trust zone [%d]: level=%d, x=%d, y=%d, w=%d, h=%d\n",
			__func__, i, ai_tz_info[i].level, ai_tz_info[i].x,
			ai_tz_info[i].y, ai_tz_info[i].w, ai_tz_info[i].h);

		if (s_tz_init)
			ai_tz_info[i].level = 0;
		else
			ai_tz_info[i].level = TRUST_LEVEL;

		if (s_tz_init) {
			list_for_each_entry (p_tmp, &haptic_audio->score_list,
					     list) {
				if (aw86907_haptic_audio_tz_list_match(
					    p_tmp, &ai_tz_info[i])) {
					match = 1;
					break;
				}
			}
		}

		if (match != 1)
			aw86907_haptic_audio_tz_list_insert(haptic_audio,
							    &ai_tz_info[i]);

		match = 0;
	}
	s_tz_init = 1;
	aw86907_haptic_audio_tz_list_show(haptic_audio);

	kfree(ai_tz_info);
	kfree(ai_tz);

	mutex_unlock(&aw86907->haptic_audio.lock);
	return count;
}

static ssize_t aw86907_haptic_audio_tp_size_show(struct device *dev,
						 struct device_attribute *attr,
						 char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	struct haptic_audio_tp_size *tp_size = NULL;
	ssize_t len = 0;

	tp_size = &(aw86907->haptic_audio.tp_size);

	len += snprintf(buf + len, PAGE_SIZE - len, "tp_size: x=%04hu, y=%04hu\n",
			tp_size->x, tp_size->y);

	return len;
}

static ssize_t aw86907_haptic_audio_tp_size_store(struct device *dev,
						  struct device_attribute *attr,
						  const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	struct haptic_audio_tp_size *tp_size = NULL;

	if (aw86907->rtp_on)
		return count;
	if (!aw86907->ram_init)
		return count;

	tp_size = &(aw86907->haptic_audio.tp_size);

	if (count != sizeof(struct haptic_audio_tp_size)) {
		pr_err("%s: unmatch buf len, node_len=%d, struct len=%d\n",
		       __func__, count, sizeof(struct haptic_audio_tp_size));
		return count;
	}

	memcpy(tp_size, buf, sizeof(struct haptic_audio_tp_size));
	pr_info("%s: tp_size: x=%d, y=%d\n", __func__, tp_size->x, tp_size->y);

	return count;
}

static ssize_t aw86907_haptic_audio_tz_cnt_show(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	ssize_t len = 0;
	len += snprintf(buf + len, PAGE_SIZE - len, "tz_cnt_thr=%d\n",
			aw86907->haptic_audio.tz_cnt_thr);
	len += snprintf(buf + len, PAGE_SIZE - len, "tz_cnt_max=%d\n",
			aw86907->haptic_audio.tz_cnt_max);
	return len;
}

static ssize_t aw86907_haptic_audio_tz_cnt_store(struct device *dev,
						 struct device_attribute *attr,
						 const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	unsigned int databuf[2] = { 0 };

	if (aw86907->rtp_on)
		return count;
	if (!aw86907->ram_init)
		return count;

	if (2 == sscanf(buf, "%d %d", &databuf[0], &databuf[1])) {
		aw86907->haptic_audio.tz_cnt_thr = databuf[0];
		aw86907->haptic_audio.tz_cnt_max = databuf[1];
	}
	return count;
}

static ssize_t aw86907_haptic_audio_hap_cnt_max_outside_tz_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	ssize_t len = 0;
	len += snprintf(buf + len, PAGE_SIZE - len,
			"hap_cnt_max_outside_tz=%u\n",
			aw86907->haptic_audio.hap_cnt_max_outside_tz);
	return len;
}

static ssize_t
aw86907_haptic_audio_hap_cnt_max_outside_tz_store(struct device *dev,
						  struct device_attribute *attr,
						  const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	unsigned int databuf[1] = { 0 };

	if (aw86907->rtp_on)
		return count;
	if (!aw86907->ram_init)
		return count;

	if (1 == sscanf(buf, "%d", &databuf[0])) {
		aw86907->haptic_audio.hap_cnt_max_outside_tz = databuf[0];
	}
	return count;
}

#ifdef OPLUS_FEATURE_CHG_BASIC
static void oplus_motor_old_test_work(struct work_struct *work)
{
	struct aw86907 *aw86907 =
		container_of(work, struct aw86907, motor_old_test_work);

	pr_err("%s: motor_old_test_mode = %d. aw86907->gain[0x%02x]\n",
	       __func__, aw86907->motor_old_test_mode, aw86907->gain);

	if (aw86907->motor_old_test_mode == MOTOR_OLD_TEST_TRANSIENT) {
		mutex_lock(&aw86907->lock);

		aw86907_haptic_stop(aw86907);
		aw86907->gain = 0x80;
		aw86907_haptic_set_gain(aw86907, aw86907->gain);
		aw86907_haptic_set_bst_vol(aw86907,
					   AW86907_HAPTIC_HIGH_LEVEL_REG_VAL);
		aw86907_haptic_set_wav_seq(aw86907, 0,
					   AW86907_WAVEFORM_INDEX_TRANSIENT);
		aw86907_haptic_set_wav_loop(aw86907, 0, 0);
		aw86907_haptic_play_wav_seq(aw86907, 1);

		mutex_unlock(&aw86907->lock);
	} else if (aw86907->motor_old_test_mode == MOTOR_OLD_TEST_STEADY) {
		mutex_lock(&aw86907->lock);
		aw86907_haptic_stop(aw86907);
		aw86907->gain = 0x80;
		aw86907_haptic_set_gain(aw86907, aw86907->gain);
		aw86907_haptic_set_bst_vol(aw86907,
					   AW86907_HAPTIC_HIGH_LEVEL_REG_VAL);
		aw86907_haptic_set_rtp_aei(aw86907, false);
		aw86907_interrupt_clear(aw86907);
		mutex_unlock(&aw86907->lock);
		if (AW86907_WAVEFORM_INDEX_OLD_STEADY <
		    (sizeof(aw86907_rtp_name) / AW86907_RTP_NAME_MAX)) {
			aw86907->rtp_file_num =
				AW86907_WAVEFORM_INDEX_OLD_STEADY;
			if (AW86907_WAVEFORM_INDEX_OLD_STEADY) {
				//schedule_work(&aw86907->rtp_work);
				queue_work(system_unbound_wq,
					   &aw86907->rtp_work);
			}
		} else {
			pr_err("%s: rtp_file_num 0x%02x over max value \n",
			       __func__, aw86907->rtp_file_num);
		}
	} else if (aw86907->motor_old_test_mode ==
		   MOTOR_OLD_TEST_HIGH_TEMP_HUMIDITY) {
		mutex_lock(&aw86907->lock);
		aw86907_haptic_stop(aw86907);
		aw86907->gain = 0x80;
		aw86907_haptic_set_gain(aw86907, aw86907->gain);
		aw86907_haptic_set_bst_vol(aw86907,
					   AW86907_HAPTIC_HIGH_LEVEL_REG_VAL);
		aw86907_haptic_set_rtp_aei(aw86907, false);
		aw86907_interrupt_clear(aw86907);
		mutex_unlock(&aw86907->lock);
		if (AW86907_WAVEFORM_INDEX_HIGH_TEMP <
		    (sizeof(aw86907_rtp_name) / AW86907_RTP_NAME_MAX)) {
			aw86907->rtp_file_num =
				AW86907_WAVEFORM_INDEX_HIGH_TEMP;
			if (AW86907_WAVEFORM_INDEX_HIGH_TEMP) {
				//schedule_work(&aw86907->rtp_work);
				queue_work(system_unbound_wq,
					   &aw86907->rtp_work);
			}
		} else {
			pr_err("%s: rtp_file_num 0x%02x over max value \n",
			       __func__, aw86907->rtp_file_num);
		}
	} else if (aw86907->motor_old_test_mode == MOTOR_OLD_TEST_LISTEN_POP) {
		mutex_lock(&aw86907->lock);
		aw86907_haptic_stop(aw86907);
		aw86907->gain = 0x80;
		aw86907_haptic_set_gain(aw86907, aw86907->gain);
		aw86907_haptic_set_bst_vol(aw86907,
					   AW86907_HAPTIC_HIGH_LEVEL_REG_VAL);
		aw86907_haptic_set_rtp_aei(aw86907, false);
		aw86907_interrupt_clear(aw86907);
		mutex_unlock(&aw86907->lock);
		if (AW86907_WAVEFORM_INDEX_LISTEN_POP <
		    (sizeof(aw86907_rtp_name) / AW86907_RTP_NAME_MAX)) {
			aw86907->rtp_file_num =
				AW86907_WAVEFORM_INDEX_LISTEN_POP;
			if (AW86907_WAVEFORM_INDEX_LISTEN_POP) {
				//schedule_work(&aw86907->rtp_work);
				queue_work(system_unbound_wq,
					   &aw86907->rtp_work);
			}
		} else {
			pr_err("%s: rtp_file_num 0x%02x over max value \n",
			       __func__, aw86907->rtp_file_num);
		}
	} else {
		aw86907->motor_old_test_mode = 0;
		mutex_lock(&aw86907->lock);
		aw86907_haptic_stop(aw86907);
		//aw86907_haptic_android_stop(aw86907);
		mutex_unlock(&aw86907->lock);
	}
}

static ssize_t aw86907_motor_old_test_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	return 0;
}

static ssize_t aw86907_motor_old_test_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif

	unsigned int databuf[1] = { 0 };

	if (1 == sscanf(buf, "%x", &databuf[0])) {
		if (databuf[0] == 0) {
			cancel_work_sync(&aw86907->motor_old_test_work);
			mutex_lock(&aw86907->lock);
			aw86907_haptic_stop(aw86907);
			mutex_unlock(&aw86907->lock);
		} else if (databuf[0] <= MOTOR_OLD_TEST_ALL_NUM) {
			cancel_work_sync(&aw86907->motor_old_test_work);
			aw86907->motor_old_test_mode = databuf[0];
			pr_err("%s: motor_old_test_mode = %d.\n", __func__,
			       aw86907->motor_old_test_mode);
			schedule_work(
				&aw86907->motor_old_test_work); //oplus_motor_old_test_work
		}
	}

	return count;
}

static ssize_t aw86907_waveform_index_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	return 0;
}

static ssize_t aw86907_waveform_index_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	unsigned int databuf[1] = { 0 };

	//    aw86907->vmax = AW86907_HAPTIC_HIGH_LEVEL_REG_VAL;
	//    aw86907->gain = 0x80;
	//    aw86907_haptic_set_gain(aw86907, aw86907->gain);
	//    aw86907_haptic_set_bst_vol(aw86907, aw86907->vmax);

	if (1 == sscanf(buf, "%x", &databuf[0])) {
		pr_err("%s: waveform_index = %d\n", __FUNCTION__, databuf[0]);
		mutex_lock(&aw86907->lock);
		aw86907->seq[0] = (unsigned char)databuf[0];
		aw86907_haptic_set_wav_seq(aw86907, 0, aw86907->seq[0]);
		aw86907_haptic_set_wav_seq(aw86907, 1, 0);
		aw86907_haptic_set_wav_loop(aw86907, 0, 0);
		mutex_unlock(&aw86907->lock);
	}
	return count;
}

static ssize_t aw86907_osc_data_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	unsigned char reg_val = 0;
	unsigned char lra_rtim_code = 0;
	ssize_t len = 0;

	mutex_lock(&aw86907->lock);
	aw86907_i2c_read(aw86907, AW86907_REG_TRIMCFG3, &reg_val);
	lra_rtim_code = reg_val & 0x3f;
	mutex_unlock(&aw86907->lock);

	pr_err("%s: lra_rtim_code = %d, code=%d\n", __FUNCTION__, lra_rtim_code,
	       aw86907->clock_standard_osc_lra_rtim_code);

	len += snprintf(buf + len, PAGE_SIZE - len, "%u\n",
			aw86907->clock_standard_osc_lra_rtim_code);
	return len;
}

static ssize_t aw86907_osc_data_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	unsigned int databuf[1] = { 0 };
	//unsigned int lra_rtim_code = 0;

	if (1 == sscanf(buf, "%d", &databuf[0])) {
		pr_err("%s: lra_rtim_code = %d\n", __FUNCTION__, databuf[0]);
		aw86907->clock_standard_osc_lra_rtim_code = databuf[0];
		mutex_lock(&aw86907->lock);
		if (databuf[0] > 0)
			aw86907_i2c_write_bits(
				aw86907, AW86907_REG_TRIMCFG3,
				AW86907_BIT_TRIMCFG3_TRIM_LRA_MASK,
				(char)databuf[0]);
		mutex_unlock(&aw86907->lock);
	}
	return count;
}

static ssize_t aw86907_haptic_ram_test_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	ssize_t len = 0;
	unsigned int ram_test_result = 0;
	//len += snprintf(buf+len, PAGE_SIZE-len, "aw86907->ram_test_flag_0=%d  0:pass,!0= failed\n", aw86907->ram_test_flag_0 );
	//len += snprintf(buf+len, PAGE_SIZE-len, "aw86907->ram_test_flag_1=%d  0:pass,!0= failed\n", aw86907->ram_test_flag_1 );
	if (aw86907->ram_test_flag_0 != 0 || aw86907->ram_test_flag_1 != 0) {
		ram_test_result = 1; // failed
		len += snprintf(buf + len, PAGE_SIZE - len, "%u\n",
				ram_test_result);
	} else {
		ram_test_result = 0; // pass
		len += snprintf(buf + len, PAGE_SIZE - len, "%u\n",
				ram_test_result);
	}
	return len;
}

static ssize_t aw86907_haptic_ram_test_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif
	struct aw86907_container *aw86907_ramtest;
	int i, j = 0;
	unsigned int val = 0;
	int rc = 0;
	//   unsigned int databuf[2] = {0};
	unsigned int start_addr;
	//   unsigned int end_addr;
	unsigned int tmp_len, retries;
	//unsigned char reg_val = 0;
	char *pbuf = NULL;
	pr_info("%s enter\n", __func__);

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;
	start_addr = 0;
	aw86907->ram_test_flag_0 = 0;
	aw86907->ram_test_flag_1 = 0;
	tmp_len = 1024; // /*1K*/
	retries = 8; /*tmp_len * retries  =8*1024*/
	aw86907_ramtest =
		kzalloc(tmp_len * sizeof(char) + sizeof(int), GFP_KERNEL);
	if (!aw86907_ramtest) {
		pr_err("%s: error allocating memory\n", __func__);
		return count;
	}
	pbuf = kzalloc(tmp_len * sizeof(char), GFP_KERNEL);
	if (!pbuf) {
		pr_err("%s: Error allocating memory\n", __func__);
		kfree(aw86907_ramtest);
		return count;
	}
	aw86907_ramtest->len = tmp_len;
	//printk("%s  --%d   aw86907_ramtest->len =%d \n",__func__,__LINE__,aw86907_ramtest->len);

	if (val == 1) {
		/* RAMINIT Enable */
		aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSCTRL1,
				       AW86907_BIT_SYSCTRL1_RAMINIT_MASK,
				       AW86907_BIT_SYSCTRL1_RAMINIT_ON);
		for (j = 0; j < retries; j++) {
			/*test 1   start*/
			memset(aw86907_ramtest->data, 0xff,
			       aw86907_ramtest->len);
			memset(pbuf, 0x00, aw86907_ramtest->len);
			/* write ram 1 test */
			aw86907_i2c_write(aw86907, AW86907_REG_RAMADDRH,
					  start_addr >> 8);
			aw86907_i2c_write(aw86907, AW86907_REG_RAMADDRL,
					  start_addr & 0x00FF);

			aw86907_i2c_writes(aw86907, AW86907_REG_RAMDATA,
					   aw86907_ramtest->data,
					   aw86907_ramtest->len);

			/* read ram 1 test */
			aw86907_i2c_write(aw86907, AW86907_REG_RAMADDRH,
					  start_addr >> 8);
			aw86907_i2c_write(aw86907, AW86907_REG_RAMADDRL,
					  start_addr & 0x00FF);

			aw86907_i2c_reads(aw86907, AW86907_REG_RAMDATA, pbuf,
					  aw86907_ramtest->len);

			for (i = 0; i < aw86907_ramtest->len; i++) {
				if (pbuf[i] != 0xff) {
					//   printk("%s---%d---pbuf[%d]= 0x%x\n",__func__,__LINE__,i,pbuf[i]);
					aw86907->ram_test_flag_1++;
				}
			}
			/*test 1  end*/
			/*test 0 start*/
			memset(aw86907_ramtest->data, 0x00,
			       aw86907_ramtest->len);
			memset(pbuf, 0xff, aw86907_ramtest->len);

			/* write ram 0 test */
			aw86907_i2c_write(aw86907, AW86907_REG_RAMADDRH,
					  start_addr >> 8);
			aw86907_i2c_write(aw86907, AW86907_REG_RAMADDRL,
					  start_addr & 0x00FF);

			aw86907_i2c_writes(aw86907, AW86907_REG_RAMDATA,
					   aw86907_ramtest->data,
					   aw86907_ramtest->len);

			/* read ram 0 test */
			aw86907_i2c_write(aw86907, AW86907_REG_RAMADDRH,
					  start_addr >> 8);
			aw86907_i2c_write(aw86907, AW86907_REG_RAMADDRL,
					  start_addr & 0x00FF);

			aw86907_i2c_reads(aw86907, AW86907_REG_RAMDATA, pbuf,
					  aw86907_ramtest->len);

			for (i = 0; i < aw86907_ramtest->len; i++) {
				if (pbuf[i] != 0) {
					//  printk("%s---%d---pbuf[%d]= 0x%x\n",__func__,__LINE__,i,pbuf[i]);
					aw86907->ram_test_flag_0++;
				}
			}

			/*test 0 end*/
			start_addr += tmp_len;
		}
		/* RAMINIT Disable */
		aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSCTRL1,
				       AW86907_BIT_SYSCTRL1_RAMINIT_MASK,
				       AW86907_BIT_SYSCTRL1_RAMINIT_OFF);
	}
	kfree(aw86907_ramtest);
	kfree(pbuf);
	pbuf = NULL;
	pr_info("%s exit\n", __func__);
	return count;
}

#endif

#ifdef OPLUS_FEATURE_CHG_BASIC
static ssize_t aw86907_device_id_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(to_dev, struct aw86907, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw86907 *aw86907 = container_of(cdev, struct aw86907, cdev);
#endif

	return snprintf(buf, PAGE_SIZE, "%d\n", aw86907->device_id);
}

static ssize_t aw86907_device_id_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	return count;
}
#endif

static DEVICE_ATTR(state, S_IWUSR | S_IRUGO, aw86907_state_show,
		   aw86907_state_store);
static DEVICE_ATTR(duration, S_IWUSR | S_IRUGO, aw86907_duration_show,
		   aw86907_duration_store);
static DEVICE_ATTR(activate, S_IWUSR | S_IRUGO, aw86907_activate_show,
		   aw86907_activate_store);
static DEVICE_ATTR(activate_mode, S_IWUSR | S_IRUGO, aw86907_activate_mode_show,
		   aw86907_activate_mode_store);
static DEVICE_ATTR(index, S_IWUSR | S_IRUGO, aw86907_index_show,
		   aw86907_index_store);
static DEVICE_ATTR(vmax, S_IWUSR | S_IRUGO, aw86907_vmax_show,
		   aw86907_vmax_store);
static DEVICE_ATTR(gain, S_IWUSR | S_IRUGO, aw86907_gain_show,
		   aw86907_gain_store);
static DEVICE_ATTR(seq, S_IWUSR | S_IRUGO, aw86907_seq_show, aw86907_seq_store);
static DEVICE_ATTR(loop, S_IWUSR | S_IRUGO, aw86907_loop_show,
		   aw86907_loop_store);
static DEVICE_ATTR(register, S_IWUSR | S_IRUGO, aw86907_reg_show,
		   aw86907_reg_store);
static DEVICE_ATTR(rtp, S_IWUSR | S_IRUGO, aw86907_rtp_show, aw86907_rtp_store);
static DEVICE_ATTR(ram_update, S_IWUSR | S_IRUGO, aw86907_ram_update_show,
		   aw86907_ram_update_store);
static DEVICE_ATTR(f0, S_IWUSR | S_IRUGO, aw86907_f0_show, aw86907_f0_store);
static DEVICE_ATTR(cali, S_IWUSR | S_IRUGO, aw86907_cali_show,
		   aw86907_cali_store);
static DEVICE_ATTR(cont, S_IWUSR | S_IRUGO, aw86907_cont_show,
		   aw86907_cont_store);

static DEVICE_ATTR(f0_save, S_IWUSR | S_IRUGO, aw86907_f0_save_show,
		   aw86907_f0_save_store);
static DEVICE_ATTR(cont_wait_num, S_IWUSR | S_IRUGO, aw86907_cont_wait_num_show,
		   aw86907_cont_wait_num_store);
static DEVICE_ATTR(cont_drv_lvl, S_IWUSR | S_IRUGO, aw86907_cont_drv_lvl_show,
		   aw86907_cont_drv_lvl_store);
static DEVICE_ATTR(cont_drv_time, S_IWUSR | S_IRUGO, aw86907_cont_drv_time_show,
		   aw86907_cont_drv_time_store);
static DEVICE_ATTR(cont_brk_time, S_IWUSR | S_IRUGO, aw86907_cont_brk_time_show,
		   aw86907_cont_brk_time_store);

static DEVICE_ATTR(vbat_monitor, S_IWUSR | S_IRUGO, aw86907_vbat_monitor_show,
		   aw86907_vbat_monitor_store);
static DEVICE_ATTR(lra_resistance, S_IWUSR | S_IRUGO,
		   aw86907_lra_resistance_show, aw86907_lra_resistance_store);
static DEVICE_ATTR(auto_boost, S_IWUSR | S_IRUGO, aw86907_auto_boost_show,
		   aw86907_auto_boost_store);
static DEVICE_ATTR(prctmode, S_IWUSR | S_IRUGO, aw86907_prctmode_show,
		   aw86907_prctmode_store);

static DEVICE_ATTR(ram_vbat_comp, S_IWUSR | S_IRUGO, aw86907_ram_vbat_comp_show,
		   aw86907_ram_vbat_comp_store);
#ifdef OPLUS_FEATURE_CHG_BASIC
static DEVICE_ATTR(osc_cali, S_IWUSR | S_IRUGO, aw86907_osc_cali_show,
		   aw86907_osc_cali_store);
#endif

static DEVICE_ATTR(rtp_num, S_IWUSR | S_IRUGO, aw86907_rtp_num_show,
		   aw86907_rtp_num_store);
static DEVICE_ATTR(haptic_audio, S_IWUSR | S_IRUGO, aw86907_haptic_audio_show,
		   aw86907_haptic_audio_store);
static DEVICE_ATTR(haptic_audio_time, S_IWUSR | S_IRUGO,
		   aw86907_haptic_audio_time_show,
		   aw86907_haptic_audio_time_store);
#ifdef OPLUS_FEATURE_CHG_BASIC
static DEVICE_ATTR(motor_old, S_IWUSR | S_IRUGO, aw86907_motor_old_test_show,
		   aw86907_motor_old_test_store);
static DEVICE_ATTR(waveform_index, S_IWUSR | S_IRUGO,
		   aw86907_waveform_index_show, aw86907_waveform_index_store);
static DEVICE_ATTR(osc_data, S_IWUSR | S_IRUGO, aw86907_osc_data_show,
		   aw86907_osc_data_store);
static DEVICE_ATTR(ram_test, S_IWUSR | S_IRUGO, aw86907_haptic_ram_test_show,
		   aw86907_haptic_ram_test_store);
static DEVICE_ATTR(f0_data, S_IWUSR | S_IRUGO, aw86907_f0_data_show,
		   aw86907_f0_data_store);
static DEVICE_ATTR(rtp_going, S_IWUSR | S_IRUGO, aw86907_rtp_going_show,
		   aw86907_rtp_going_store);

#endif
#ifdef OPLUS_FEATURE_CHG_BASIC
static DEVICE_ATTR(device_id, S_IWUSR | S_IRUGO, aw86907_device_id_show,
		   aw86907_device_id_store);
#endif

static DEVICE_ATTR(gun_type, S_IWUSR | S_IRUGO, aw86907_gun_type_show,
		   aw86907_gun_type_store); //hch 20190917
static DEVICE_ATTR(bullet_nr, S_IWUSR | S_IRUGO, aw86907_bullet_nr_show,
		   aw86907_bullet_nr_store); //hch 20190917

//wanzhong 20191227
static DEVICE_ATTR(haptic_audio_tp_time, S_IWUSR | S_IRUGO,
		   aw86907_haptic_audio_tp_time_show,
		   aw86907_haptic_audio_tp_time_store);
static DEVICE_ATTR(haptic_audio_tp_input, S_IWUSR | S_IRUGO,
		   aw86907_haptic_audio_tp_input_show,
		   aw86907_haptic_audio_tp_input_store);
static DEVICE_ATTR(haptic_audio_ai_input, S_IWUSR | S_IRUGO,
		   aw86907_haptic_audio_ai_input_show,
		   aw86907_haptic_audio_ai_input_store);
static DEVICE_ATTR(haptic_audio_tp_size, S_IWUSR | S_IRUGO,
		   aw86907_haptic_audio_tp_size_show,
		   aw86907_haptic_audio_tp_size_store);
static DEVICE_ATTR(haptic_audio_tz_cnt, S_IWUSR | S_IRUGO,
		   aw86907_haptic_audio_tz_cnt_show,
		   aw86907_haptic_audio_tz_cnt_store);
static DEVICE_ATTR(haptic_audio_hap_cnt_max_outside_tz, S_IWUSR | S_IRUGO,
		   aw86907_haptic_audio_hap_cnt_max_outside_tz_show,
		   aw86907_haptic_audio_hap_cnt_max_outside_tz_store);
//wanzhong 20191227 end
static struct attribute *aw86907_vibrator_attributes[] = {
	&dev_attr_state.attr, &dev_attr_duration.attr, &dev_attr_activate.attr,
	&dev_attr_activate_mode.attr, &dev_attr_index.attr, &dev_attr_vmax.attr,
	&dev_attr_gain.attr, &dev_attr_seq.attr, &dev_attr_loop.attr,
	&dev_attr_register.attr, &dev_attr_rtp.attr, &dev_attr_ram_update.attr,
	&dev_attr_f0.attr, &dev_attr_cali.attr, &dev_attr_cont.attr,

	&dev_attr_f0_save.attr, &dev_attr_cont_wait_num.attr,
	&dev_attr_cont_drv_lvl.attr, &dev_attr_cont_drv_time.attr,
	&dev_attr_cont_brk_time.attr,

	&dev_attr_vbat_monitor.attr, &dev_attr_lra_resistance.attr,
	&dev_attr_auto_boost.attr, &dev_attr_prctmode.attr,
	&dev_attr_ram_vbat_comp.attr, &dev_attr_osc_cali.attr,
	&dev_attr_rtp_num.attr, &dev_attr_haptic_audio.attr,
	&dev_attr_haptic_audio_time.attr,
#ifdef OPLUS_FEATURE_CHG_BASIC

	&dev_attr_motor_old.attr, &dev_attr_waveform_index.attr,
	&dev_attr_osc_data.attr, &dev_attr_ram_test.attr,
	&dev_attr_f0_data.attr, &dev_attr_rtp_going.attr,
#endif
#ifdef OPLUS_FEATURE_CHG_BASIC
	&dev_attr_device_id.attr,
#endif
	&dev_attr_gun_type.attr, //hch 20190917
	&dev_attr_bullet_nr.attr, //hch 20190917
	//wanzhong 20191227
	&dev_attr_haptic_audio_tp_time.attr,
	&dev_attr_haptic_audio_tp_input.attr,
	&dev_attr_haptic_audio_ai_input.attr,
	&dev_attr_haptic_audio_tp_size.attr, &dev_attr_haptic_audio_tz_cnt.attr,
	&dev_attr_haptic_audio_hap_cnt_max_outside_tz.attr,
	//wanzhong 20191227 end
	NULL
};

static struct attribute_group aw86907_vibrator_attribute_group = {
	.attrs = aw86907_vibrator_attributes
};

static enum hrtimer_restart aw86907_vibrator_timer_func(struct hrtimer *timer)
{
	struct aw86907 *aw86907 = container_of(timer, struct aw86907, timer);

	pr_debug("%s enter\n", __func__);
	aw86907->state = 0;
	//schedule_work(&aw86907->vibrator_work);
	queue_work(system_highpri_wq, &aw86907->vibrator_work);

	return HRTIMER_NORESTART;
}

static void aw86907_vibrator_work_routine(struct work_struct *work)
{
	unsigned int val = 0;
	struct aw86907 *aw86907 =
		container_of(work, struct aw86907, vibrator_work);

#ifdef OPLUS_FEATURE_CHG_BASIC
	aw86907->activate_mode = AW86907_HAPTIC_ACTIVATE_RAM_MODE;
	pr_err("%s enter, aw86907->state[%d], aw86907->activate_mode[%d], aw86907->ram_vbat_comp[%d]\n",
	       __func__, aw86907->state, aw86907->activate_mode,
	       aw86907->ram_vbat_comp);
#endif
	mutex_lock(&aw86907->lock);
	aw86907_set_clock(aw86907, AW86907_HAPTIC_CLOCK_CALI_F0);
	aw86907_haptic_stop(aw86907);
	mutex_unlock(&aw86907->lock);
	//aw86907_haptic_android_stop(aw86907);
	if (aw86907->state) {
		mutex_lock(&aw86907->lock);
		if (aw86907->activate_mode ==
		    AW86907_HAPTIC_ACTIVATE_RAM_MODE) {
			if (aw86907->device_id == 832 ||
			    aw86907->device_id == 833)
				aw86907_haptic_ram_vbat_comp(aw86907, false);
			else
				aw86907_haptic_ram_vbat_comp(aw86907, true);

			aw86907_haptic_play_repeat_seq(aw86907, true);
		} else if (aw86907->activate_mode ==
			   AW86907_HAPTIC_ACTIVATE_CONT_MODE) {
			aw86907_haptic_cont(aw86907);
		}
		mutex_unlock(&aw86907->lock);
		val = aw86907->duration;
		/* run ms timer */
		hrtimer_start(&aw86907->timer,
			      ktime_set(val / 1000, (val % 1000) * 1000000),
			      HRTIMER_MODE_REL);
	}
}

static int aw86907_vibrator_init(struct aw86907 *aw86907)
{
	int ret = 0;

	pr_info("%s enter!\n", __func__);

#ifdef TIMED_OUTPUT
	pr_info("%s: TIMED_OUT FRAMEWORK!\n", __func__);
	aw86907->to_dev.name = "vibrator";
	aw86907->to_dev.get_time = aw86907_vibrator_get_time;
	aw86907->to_dev.enable = aw86907_vibrator_enable;

	ret = timed_output_dev_register(&(aw86907->to_dev));
	if (ret < 0) {
		dev_err(aw86907->dev, "%s: fail to create timed output dev\n",
			__func__);
		return ret;
	}
	ret = sysfs_create_group(&aw86907->to_dev.dev->kobj,
				 &aw86907_vibrator_attribute_group);
	if (ret < 0) {
		dev_err(aw86907->dev, "%s error creating sysfs attr files\n",
			__func__);
		return ret;
	}
#else
	pr_info("%s: LED FRAMEWORK!\n", __func__);
	aw86907->cdev.name = "vibrator";
	aw86907->cdev.brightness_get = aw86907_haptic_brightness_get;
	aw86907->cdev.brightness_set = aw86907_haptic_brightness_set;

	ret = devm_led_classdev_register(&aw86907->i2c->dev, &aw86907->cdev);
	if (ret < 0) {
		dev_err(aw86907->dev, "%s: fail to create led dev\n", __func__);
		return ret;
	}
	ret = sysfs_create_group(&aw86907->cdev.dev->kobj,
				 &aw86907_vibrator_attribute_group);
	if (ret < 0) {
		dev_err(aw86907->dev, "%s error creating sysfs attr files\n",
			__func__);
		return ret;
	}
#endif
	hrtimer_init(&aw86907->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw86907->timer.function = aw86907_vibrator_timer_func;
	INIT_WORK(&aw86907->vibrator_work, aw86907_vibrator_work_routine);

	INIT_WORK(&aw86907->rtp_work, aw86907_rtp_work_routine);

	INIT_WORK(&aw86907->rtp_single_cycle_work,
		  aw86907_rtp_single_cycle_routine);
	INIT_WORK(&aw86907->rtp_regroup_work, aw86907_rtp_regroup_routine);
	mutex_init(&aw86907->lock);

	pr_info("%s out!\n", __func__);
	return 0;
}

/******************************************************
 *
 * irq
 *
 ******************************************************/
static void aw86907_interrupt_clear(struct aw86907 *aw86907)
{
	unsigned char reg_val = 0;
	//pr_info("%s enter\n", __func__);
	aw86907_i2c_read(aw86907, AW86907_REG_SYSINT, &reg_val);
	//pr_info("%s: reg SYSINT=0x%x\n", __func__, reg_val);
}

static void aw86907_interrupt_setup(struct aw86907 *aw86907)
{
	unsigned char reg_val = 0;

	pr_info("%s enter\n", __func__);

	aw86907_i2c_read(aw86907, AW86907_REG_SYSINT, &reg_val);
	//pr_info("%s: reg SYSINT=0x%x\n", __func__, reg_val);

	/* edge int mode */
	aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSCTRL7,
			       AW86907_BIT_SYSCTRL7_INT_MODE_MASK,
			       AW86907_BIT_SYSCTRL7_INT_MODE_EDGE);
	aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSCTRL7,
			       AW86907_BIT_SYSCTRL7_INT_EDGE_MODE_MASK,
			       AW86907_BIT_SYSCTRL7_INT_EDGE_MODE_POS);

	/* int enable */
	aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSINTM,
			       AW86907_BIT_SYSINTM_BST_SCPM_MASK,
			       AW86907_BIT_SYSINTM_BST_SCPM_OFF);
	aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSINTM,
			       AW86907_BIT_SYSINTM_BST_OVPM_MASK,
			       AW86907_BIT_SYSINTM_BST_OVPM_ON);
	aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSINTM,
			       AW86907_BIT_SYSINTM_UVLM_MASK,
			       AW86907_BIT_SYSINTM_UVLM_ON);
	aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSINTM,
			       AW86907_BIT_SYSINTM_OCDM_MASK,
			       AW86907_BIT_SYSINTM_OCDM_ON);
	aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSINTM,
			       AW86907_BIT_SYSINTM_OTM_MASK,
			       AW86907_BIT_SYSINTM_OTM_ON);
}

static irqreturn_t aw86907_irq(int irq, void *data)
{
	struct aw86907 *aw86907 = data;
	unsigned char reg_val = 0;
	//unsigned char dbg_val = 0;
	unsigned int buf_len = 0;

	pr_debug("%s enter\n", __func__);

#ifdef AAC_RICHTAP
	if (aw86907->haptic_rtp_mode) {
		return IRQ_HANDLED;
	}
#endif

	aw86907_i2c_read(aw86907, AW86907_REG_SYSINT, &reg_val);
	//pr_info("%s: reg SYSINT=0x%x\n", __func__, reg_val);

	if (reg_val & AW86907_BIT_SYSINT_BST_OVPI) {
		aw86907_op_clean_status(aw86907);
		pr_err("%s chip ov int error\n", __func__);
	}
	if (reg_val & AW86907_BIT_SYSINT_UVLI) {
		aw86907_op_clean_status(aw86907);
		pr_err("%s chip uvlo int error\n", __func__);
	}
	if (reg_val & AW86907_BIT_SYSINT_OCDI) {
		aw86907_op_clean_status(aw86907);
		pr_err("%s chip over current int error\n", __func__);
	}
	if (reg_val & AW86907_BIT_SYSINT_OTI) {
		aw86907_op_clean_status(aw86907);
		pr_err("%s chip over temperature int error\n", __func__);
	}
	if (reg_val & AW86907_BIT_SYSINT_DONEI) {
		aw86907_op_clean_status(aw86907);
		pr_info("%s chip playback done\n", __func__);
	}

	if (reg_val & AW86907_BIT_SYSINT_FF_AEI) {
		//pr_debug("%s: aw86907 rtp fifo almost empty int\n", __func__);
		if (aw86907->rtp_init) {
			while ((!aw86907_haptic_rtp_get_fifo_afs(aw86907)) &&
			       (aw86907->play_mode ==
				AW86907_HAPTIC_RTP_MODE)) {
				pr_debug(
					"%s: aw86907 rtp mode fifo update, cnt=%d\n",
					__func__, aw86907->rtp_cnt);
				if (!aw86907_rtp) {
					pr_info("%s:aw86907_rtp is null break\n",
						__func__);
					break;
				}
				mutex_lock(&aw86907->rtp_lock);
				if ((aw86907_rtp->len - aw86907->rtp_cnt) <
				    (aw86907->ram.base_addr >> 2)) {
					buf_len = aw86907_rtp->len -
						  aw86907->rtp_cnt;
				} else {
					buf_len = (aw86907->ram.base_addr >> 2);
				}
				aw86907_i2c_writes(
					aw86907, AW86907_REG_RTPDATA,
					&aw86907_rtp->data[aw86907->rtp_cnt],
					buf_len);
				aw86907->rtp_cnt += buf_len;

				if (aw86907->rtp_cnt == aw86907_rtp->len) {
					aw86907_op_clean_status(aw86907);
					pr_info("%s: rtp update complete\n",
						__func__);
					aw86907_haptic_set_rtp_aei(aw86907,
								   false);
					aw86907->rtp_cnt = 0;
					aw86907->rtp_init = 0;

					mutex_unlock(&aw86907->rtp_lock);
					break;
				}

				mutex_unlock(&aw86907->rtp_lock);
			}
		} else {
			pr_err("%s: aw86907 rtp init = %d, init error\n",
			       __func__, aw86907->rtp_init);
		}
	}

	if (reg_val & AW86907_BIT_SYSINT_FF_AFI) {
		pr_debug("%s: aw86907 rtp mode fifo full empty\n", __func__);
	}

	if (aw86907->play_mode != AW86907_HAPTIC_RTP_MODE) {
		aw86907_haptic_set_rtp_aei(aw86907, false);
	}

	aw86907_i2c_read(aw86907, AW86907_REG_SYSINT, &reg_val);
	pr_debug("%s: reg SYSINT=0x%x\n", __func__, reg_val);
	aw86907_i2c_read(aw86907, AW86907_REG_SYSST, &reg_val);
	pr_debug("%s: reg SYSST=0x%x\n", __func__, reg_val);

	pr_debug("%s exit\n", __func__);

	return IRQ_HANDLED;
}

/*****************************************************
 *
 * device tree
 *
 *****************************************************/
static int aw86907_parse_dt(struct device *dev, struct aw86907 *aw86907,
			    struct device_node *np)
{
	aw86907->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (aw86907->reset_gpio < 0) {
		dev_err(dev,
			"%s: no reset gpio provided, will not HW reset device\n",
			__func__);
		return -1;
	} else {
		dev_info(dev, "%s: reset gpio provided ok\n", __func__);
	}
	aw86907->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
	if (aw86907->irq_gpio < 0) {
		dev_err(dev, "%s: no irq gpio provided.\n", __func__);
	} else {
		dev_info(dev, "%s: irq gpio provided ok.\n", __func__);
	}
#ifdef OPLUS_FEATURE_CHG_BASIC
	if (of_property_read_u32(np, "qcom,device_id", &aw86907->device_id))
		aw86907->device_id = 815;
	dev_info(dev, "%s: aw86907->device_id=%d\n", __func__,
		 aw86907->device_id);
	aw86907->support_promise_vib =
		of_property_read_bool(np, "support_promise_vib");
#endif
	return 0;
}

static int aw86907_hw_reset(struct aw86907 *aw86907)
{
	pr_info("%s enter\n", __func__);

	if (aw86907 && gpio_is_valid(aw86907->reset_gpio)) {
		gpio_set_value_cansleep(aw86907->reset_gpio, 0);
		usleep_range(1000, 2000);
		gpio_set_value_cansleep(aw86907->reset_gpio, 1);
		usleep_range(3500, 4000);
	} else {
		if (aw86907)
			dev_err(aw86907->dev, "%s:  failed\n", __func__);
	}
	return 0;
}

static void aw86907_haptic_reset_init(struct aw86907 *aw86907)
{
	// int ret = 0;

	unsigned char reg_val = 0;

	pr_info("%s enter !\n", __func__);

	/* haptic init */
	aw86907_haptic_misc_para_init(aw86907);

	aw86907_haptic_set_bst_peak_cur(aw86907, AW86907_DEFAULT_PEAKCUR);
	aw86907_haptic_swicth_motorprotect_config(aw86907, 0x00, 0x00);
	aw86907_haptic_auto_boost_config(aw86907, false);
	aw86907_haptic_os_calibration(aw86907);
	aw86907_haptic_cont_vbat_mode(aw86907,
				      AW86907_HAPTIC_CONT_VBAT_HW_COMP_MODE);
	aw86907->ram_vbat_comp = AW86907_HAPTIC_RAM_VBAT_COMP_ENABLE;

	aw86907_i2c_read(aw86907, AW86907_REG_TRIMCFG3, &reg_val);
	aw86907_i2c_write(aw86907, AW86907_REG_TRIMCFG3, reg_val);
}

/*****************************************************
 *
 * check chip id
 *
 *****************************************************/
static int aw86907_read_chipid(struct aw86907 *aw86907)
{
	int ret = -1;
	unsigned char cnt = 0;
	unsigned char reg = 0;

	while (cnt < AW_READ_CHIPID_RETRIES) {
		/* hardware reset */
		aw86907_hw_reset(aw86907);

		ret = aw86907_i2c_read(aw86907, AW86907_REG_ID, &reg);
		if (ret < 0) {
			dev_err(aw86907->dev,
				"%s: failed to read register AW86907_REG_ID: %d\n",
				__func__, ret);
		}
		switch (reg) {
		case AW86907_CHIPID:
			pr_info("%s aw86907 detected\n", __func__);
			aw86907->chipid = AW86907_CHIPID;
			//aw86907->flags |= AW86907_FLAG_SKIP_INTERRUPTS;
			aw86907_haptic_softreset(aw86907);
			return 0;
		default:
			pr_info("%s unsupported device revision (0x%x)\n",
				__func__, reg);
			break;
		}
		cnt++;

		msleep(AW_READ_CHIPID_RETRY_DELAY);
	}

	return -EINVAL;
}

/******************************************************
 *
 * sys group attribute: reg
 *
 ******************************************************/
static ssize_t aw86907_i2c_reg_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct aw86907 *aw86907 = dev_get_drvdata(dev);

	unsigned int databuf[2] = { 0, 0 };

	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
		aw86907_i2c_write(aw86907, (unsigned char)databuf[0],
				  (unsigned char)databuf[1]);
	}

	return count;
}

static ssize_t aw86907_i2c_reg_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct aw86907 *aw86907 = dev_get_drvdata(dev);
	ssize_t len = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;
	for (i = 0; i < AW86907_REG_MAX; i++) {
		if (!(aw86907_reg_access[i] & REG_RD_ACCESS))
			continue;
		aw86907_i2c_read(aw86907, i, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len,
				"reg:0x%02x=0x%02x \n", i, reg_val);
	}
	return len;
}
static ssize_t aw86907_i2c_ram_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct aw86907 *aw86907 = dev_get_drvdata(dev);

	unsigned int databuf[1] = { 0 };

	if (1 == sscanf(buf, "%x", &databuf[0])) {
		if (1 == databuf[0]) {
			aw86907_ram_update(aw86907);
		}
	}

	return count;
}

static ssize_t aw86907_i2c_ram_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct aw86907 *aw86907 = dev_get_drvdata(dev);
	ssize_t len = 0;
	unsigned int i = 0;
	unsigned char reg_val = 0;

	aw86907_haptic_stop(aw86907);
	/* RAMINIT Enable */
	aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSCTRL1,
			       AW86907_BIT_SYSCTRL1_RAMINIT_MASK,
			       AW86907_BIT_SYSCTRL1_RAMINIT_ON);

	aw86907_i2c_write(aw86907, AW86907_REG_RAMADDRH,
			  (unsigned char)(aw86907->ram.base_addr >> 8));
	aw86907_i2c_write(aw86907, AW86907_REG_RAMADDRL,
			  (unsigned char)(aw86907->ram.base_addr & 0x00ff));
	len += snprintf(buf + len, PAGE_SIZE - len, "aw86907_haptic_ram:\n");
	for (i = 0; i < aw86907->ram.len; i++) {
		aw86907_i2c_read(aw86907, AW86907_REG_RAMDATA, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len, "0x%02x,", reg_val);
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	/* RAMINIT Disable */
	aw86907_i2c_write_bits(aw86907, AW86907_REG_SYSCTRL1,
			       AW86907_BIT_SYSCTRL1_RAMINIT_MASK,
			       AW86907_BIT_SYSCTRL1_RAMINIT_OFF);

	return len;
}

static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, aw86907_i2c_reg_show,
		   aw86907_i2c_reg_store);
static DEVICE_ATTR(ram, S_IWUSR | S_IRUGO, aw86907_i2c_ram_show,
		   aw86907_i2c_ram_store);

static struct attribute *aw86907_attributes[] = { &dev_attr_reg.attr,
						  &dev_attr_ram.attr, NULL };

static struct attribute_group aw86907_attribute_group = {
	.attrs = aw86907_attributes
};

#ifdef OPLUS_FEATURE_CHG_BASIC
static ssize_t proc_aw86907_leakage_read(struct file *filp, char __user *buff,
					 size_t count, loff_t *off)
{
	return count;
}
static ssize_t proc_aw86907_leakage_write(struct file *file,
					  const char __user *buff, size_t len,
					  loff_t *data)
{
	char write_data[32] = { 0 };

	if (copy_from_user(&write_data, buff, len)) {
		pr_err("proc_aw86907_leakage_write error.\n");
		return -EFAULT;
	}
	if (write_data[0] == '1') {
		if (g_aw86907 && gpio_is_valid(g_aw86907->reset_gpio)) {
			gpio_set_value(g_aw86907->reset_gpio, 1);
			pr_err("%s: proc_aw86907_leakage_write success\n",
			       __func__);
		}
	} else {
		if (g_aw86907 && gpio_is_valid(g_aw86907->reset_gpio)) {
			gpio_set_value(g_aw86907->reset_gpio, 0);
			pr_err("%s: proc_aw86907_leakage_write success\n",
			       __func__);
		}
	}
	return len;
}

static const struct file_operations aw86907_leakage_proc_fops = {
	.write = proc_aw86907_leakage_write,
	.read = proc_aw86907_leakage_read,
};

static int aw86907_proc_init(void)
{
	struct proc_dir_entry *p = NULL;
	p = proc_create("aw86907_leakage", 0664, NULL,
			&aw86907_leakage_proc_fops);
	if (!p)
		pr_err("proc_create aw86907_leakage fail!\n");
	return 0;
}
#endif

/******************************************************
 *
 * i2c driver
 *
 ******************************************************/
static int aw86907_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct aw86907 *aw86907;
	struct device_node *np = i2c->dev.of_node;
	unsigned char reg = 0;
	int irq_flags = 0;
	int ret = -1;

	pr_info("%s enter\n", __func__);

	AW86907_HAPTIC_RAM_VBAT_COMP_GAIN = 0x80;

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}

	aw86907 = devm_kzalloc(&i2c->dev, sizeof(struct aw86907), GFP_KERNEL);
	if (aw86907 == NULL)
		return -ENOMEM;

	aw86907->dev = &i2c->dev;
	aw86907->i2c = i2c;

	i2c_set_clientdata(i2c, aw86907);

	/* aw86907 rst & int */
	if (np) {
		ret = aw86907_parse_dt(&i2c->dev, aw86907, np);
		if (ret) {
			dev_err(&i2c->dev,
				"%s: failed to parse device tree node\n",
				__func__);
			goto err_parse_dt;
		}
	} else {
		aw86907->reset_gpio = -1;
		aw86907->irq_gpio = -1;
	}

	if (gpio_is_valid(aw86907->reset_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw86907->reset_gpio,
					    GPIOF_OUT_INIT_LOW, "aw86907_rst");
		if (ret) {
			dev_err(&i2c->dev, "%s: rst request failed\n",
				__func__);
			goto err_reset_gpio_request;
		}
	}

	if (gpio_is_valid(aw86907->irq_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw86907->irq_gpio,
					    GPIOF_DIR_IN, "aw86907_int");
		if (ret) {
			dev_err(&i2c->dev, "%s: int request failed\n",
				__func__);
			goto err_irq_gpio_request;
		}
	}

	/* aw86907 chip id */
	ret = aw86907_read_chipid(aw86907);
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: aw86907_read_chipid failed ret=%d\n",
			__func__, ret);
		goto err_id;
	}

	/* chip qualify */
	ret = aw86907_i2c_read(aw86907, 0x64, &reg);
	if (ret < 0) {
		pr_err("%s: failed to read register 0x64: %d\n", __func__, ret);
	}
	if (!(reg & 0x80)) {
		pr_err("%s:unqualified chip!\n", __func__);
		goto err_qualify;
	}

	ret = aw86907_container_init(aw86907_container_size);
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: aw86907_rtp alloc memory failed \n",
			__func__);
	}

	/* aw86907 irq */
	if (gpio_is_valid(aw86907->irq_gpio) &&
	    !(aw86907->flags & AW86907_FLAG_SKIP_INTERRUPTS)) {
		/* register irq handler */
		aw86907_interrupt_setup(aw86907);
		irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		ret = devm_request_threaded_irq(&i2c->dev,
						gpio_to_irq(aw86907->irq_gpio),
						NULL, aw86907_irq, irq_flags,
						"aw86907", aw86907);
		if (ret != 0) {
			dev_err(&i2c->dev, "%s: failed to request IRQ %d: %d\n",
				__func__, gpio_to_irq(aw86907->irq_gpio), ret);
			goto err_irq;
		}
	} else {
		dev_info(&i2c->dev, "%s skipping IRQ registration\n", __func__);
		/* disable feature support if gpio was invalid */
		aw86907->flags |= AW86907_FLAG_SKIP_INTERRUPTS;
	}

	dev_set_drvdata(&i2c->dev, aw86907);

	ret = sysfs_create_group(&i2c->dev.kobj, &aw86907_attribute_group);
	if (ret < 0) {
		dev_info(&i2c->dev, "%s error creating sysfs attr files\n",
			 __func__);
		goto err_sysfs;
	}

#ifdef AAC_RICHTAP
	aw86907->rtp_ptr = kmalloc(RICHTAP_MMAP_BUF_SIZE * RICHTAP_MMAP_BUF_SUM,
				   GFP_KERNEL);
	if (aw86907->rtp_ptr == NULL) {
		dev_err(&i2c->dev, "malloc rtp memory failed\n");
		return -ENOMEM;
	}

	aw86907->start_buf = (struct mmap_buf_format *)__get_free_pages(
		GFP_KERNEL, RICHTAP_MMAP_PAGE_ORDER);
	if (aw86907->start_buf == NULL) {
		dev_err(&i2c->dev, "Error __get_free_pages failed\n");
		return -ENOMEM;
	}
	SetPageReserved(virt_to_page(aw86907->start_buf));
	{
		struct mmap_buf_format *temp;
		uint32_t i = 0;
		temp = aw86907->start_buf;
		for (i = 1; i < RICHTAP_MMAP_BUF_SUM; i++) {
			temp->kernel_next = (aw86907->start_buf + i);
			temp = temp->kernel_next;
		}
		temp->kernel_next = aw86907->start_buf;
	}

	INIT_WORK(&aw86907->haptic_rtp_work, rtp_work_proc);
	//init_waitqueue_head(&aw86907->doneQ);
	aw86907->done_flag = true;
	aw86907->haptic_rtp_mode = false;
#endif

	g_aw86907 = aw86907;

	aw86907_vibrator_init(aw86907);

	aw86907_haptic_init(aw86907);

	aw86907_ram_init(aw86907);
#ifdef OPLUS_FEATURE_CHG_BASIC
	INIT_WORK(&aw86907->motor_old_test_work, oplus_motor_old_test_work);
	aw86907->motor_old_test_mode = 0;
	atomic_set(&aw86907->qos_cnt, 0);
#endif
#ifdef OPLUS_FEATURE_CHG_BASIC
	aw86907_proc_init();
#endif
	pr_info("%s probe completed successfully!\n", __func__);

	return 0;

err_sysfs:
	devm_free_irq(&i2c->dev, gpio_to_irq(aw86907->irq_gpio), aw86907);
#ifdef AAC_RICHTAP
	kfree(aw86907->rtp_ptr);
	free_pages((unsigned long)aw86907->start_buf, RICHTAP_MMAP_PAGE_ORDER);
#endif
err_irq:
err_qualify:
err_id:
	if (gpio_is_valid(aw86907->irq_gpio))
		devm_gpio_free(&i2c->dev, aw86907->irq_gpio);
err_irq_gpio_request:
	if (gpio_is_valid(aw86907->reset_gpio))
		devm_gpio_free(&i2c->dev, aw86907->reset_gpio);
err_reset_gpio_request:
err_parse_dt:
	devm_kfree(&i2c->dev, aw86907);
	aw86907 = NULL;
	return ret;
}

static int aw86907_i2c_remove(struct i2c_client *i2c)
{
	struct aw86907 *aw86907 = i2c_get_clientdata(i2c);

	pr_info("%s enter\n", __func__);

	sysfs_remove_group(&i2c->dev.kobj, &aw86907_attribute_group);

	devm_free_irq(&i2c->dev, gpio_to_irq(aw86907->irq_gpio), aw86907);

	if (gpio_is_valid(aw86907->irq_gpio))
		devm_gpio_free(&i2c->dev, aw86907->irq_gpio);
	if (gpio_is_valid(aw86907->reset_gpio))
		devm_gpio_free(&i2c->dev, aw86907->reset_gpio);

#ifdef AAC_RICHTAP
	kfree(aw86907->rtp_ptr);
	free_pages((unsigned long)aw86907->start_buf, RICHTAP_MMAP_PAGE_ORDER);
#endif

	devm_kfree(&i2c->dev, aw86907);
	aw86907 = NULL;

	return 0;
}

static int __maybe_unused aw86907_suspend(struct device *dev)
{
	int ret = 0;
	struct aw86907 *aw86907 = dev_get_drvdata(dev);
	mutex_lock(&aw86907->lock);
	aw86907_haptic_stop(aw86907);
	mutex_unlock(&aw86907->lock);

	return ret;
}

static int __maybe_unused aw86907_resume(struct device *dev)
{
	int ret = 0;

	return ret;
}

static SIMPLE_DEV_PM_OPS(aw86907_pm_ops, aw86907_suspend, aw86907_resume);

static const struct i2c_device_id aw86907_i2c_id[] = { { AW86907_I2C_NAME, 0 },
						       {} };
MODULE_DEVICE_TABLE(i2c, aw86907_i2c_id);

static struct of_device_id aw86907_dt_match[] = {
	{ .compatible = "awinic,aw8697_haptic" },
	{},
};

static struct i2c_driver aw86907_i2c_driver = {
    .driver = {
        .name = AW86907_I2C_NAME,
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(aw86907_dt_match),
        .pm = &aw86907_pm_ops,
    },
    .probe = aw86907_i2c_probe,
    .remove = aw86907_i2c_remove,
    .id_table = aw86907_i2c_id,
};

static int __init aw86907_i2c_init(void)
{
	int ret = 0;

	pr_info("aw86907 driver version %s\n", AW86907_VERSION);

	ret = i2c_add_driver(&aw86907_i2c_driver);
	if (ret) {
		pr_err("fail to add aw86907 device into i2c\n");
		return ret;
	}

	return 0;
}

//late_initcall(aw86907_i2c_init);
module_init(aw86907_i2c_init);

static void __exit aw86907_i2c_exit(void)
{
	i2c_del_driver(&aw86907_i2c_driver);
}
module_exit(aw86907_i2c_exit);

MODULE_DESCRIPTION("aw86907 Haptic Driver");
MODULE_LICENSE("GPL v2");
