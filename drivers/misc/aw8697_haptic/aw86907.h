#ifndef _AW86907_H_
#define _AW86907_H_

/*********************************************************
 *
 * kernel version
 *
 ********************************************************/
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 4, 1)
#define TIMED_OUTPUT
#endif

/*********************************************************
 *
 * aw86907.h
 *
 ********************************************************/
#include <linux/regmap.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#ifdef TIMED_OUTPUT
#include <../../../drivers/staging/android/timed_output.h>
#else
#include <linux/leds.h>
#endif

/*********************************************************
 *
 * marco
 *
 ********************************************************/
#define MAX_I2C_BUFFER_SIZE 65536

#define AW86907_REG_MAX 0xff

#define AW86907_SEQUENCER_SIZE 8
#define AW86907_SEQUENCER_LOOP_SIZE 4

#define AW86907_RTP_I2C_SINGLE_MAX_NUM 512

#define HAPTIC_MAX_TIMEOUT 10000

#define AW86907_VBAT_REFER 4400
#define AW86907_VBAT_MIN 3000
#define AW86907_VBAT_MAX 5500
#define OSC_CALI_MAX_LENGTH 5100000

#define AW86907_MAX_BST_VOL 0x3f /* bst_vol-> six bit */

#ifdef OPLUS_FEATURE_CHG_BASIC
/* 0832 */
#define AW86907_0832_HAPTIC_F0_PRE 2350
#define AW86907_0832_HAPTIC_F0_CALI_PERCEN 7
#define AW86907_0832_HAPTIC_CONT_DRV1_LVL 0x7F
#define AW86907_0832_HAPTIC_CONT_DRV2_LVL 0x36
#define AW86907_0832_HAPTIC_CONT_DRV1_TIME 0x04
#define AW86907_0832_HAPTIC_CONT_DRV2_TIME 0x14
#define AW86907_0832_HAPTIC_CONT_WAIT_NUM 0x06
#define AW86907_0832_HAPTIC_CONT_BRK_TIME 0x00
#define AW86907_0832_HAPTIC_CONT_TRACK_MARGIN 0x12
#define AW86907_0832_HAPTIC_CONT_TEST 0x06
#define AW86907_0832_HAPTIC_CONT_DRV_WIDTH 0x6A
#define AW86907_0832_HAPTIC_CONT_BEMF_SET 0x02
#define AW86907_0832_HAPTIC_CONT_BRK_GAIN 0x08
#define AW86907_0832_HAPTIC_CONT_BST_BRK_GAIN 0x05

#define AW86907_0832_HAPTIC_D2S_GAIN 0x04

/* 0815 */
#define AW86907_0815_HAPTIC_F0_PRE 1700
#define AW86907_0815_HAPTIC_F0_CALI_PERCEN 7
#define AW86907_0815_HAPTIC_CONT_DRV1_LVL 0x7F
#define AW86907_0815_HAPTIC_CONT_DRV2_LVL 0x36
#define AW86907_0815_HAPTIC_CONT_DRV1_TIME 0x04
#define AW86907_0815_HAPTIC_CONT_DRV2_TIME 0x14
#define AW86907_0815_HAPTIC_CONT_WAIT_NUM 0x06
#define AW86907_0815_HAPTIC_CONT_BRK_TIME 0x00
#define AW86907_0815_HAPTIC_CONT_TRACK_MARGIN 0x12
#define AW86907_0815_HAPTIC_CONT_TEST 0x06
#define AW86907_0815_HAPTIC_CONT_DRV_WIDTH 0x6A
#define AW86907_0815_HAPTIC_CONT_BEMF_SET 0x02
#define AW86907_0815_HAPTIC_CONT_BRK_GAIN 0x08
#define AW86907_0815_HAPTIC_CONT_BST_BRK_GAIN 0x05

#define AW86907_0815_HAPTIC_D2S_GAIN 0x04

#endif

#define AW86907_RTP_NUM 6

enum aw86907_flags {
	AW86907_FLAG_NONR = 0,
	AW86907_FLAG_SKIP_INTERRUPTS = 1,
};

enum aw86907_haptic_read_write {
	AW86907_HAPTIC_CMD_READ_REG = 0,
	AW86907_HAPTIC_CMD_WRITE_REG = 1,
};

enum aw86907_haptic_work_mode {
	AW86907_HAPTIC_STANDBY_MODE = 0,
	AW86907_HAPTIC_RAM_MODE = 1,
	AW86907_HAPTIC_RTP_MODE = 2,
	AW86907_HAPTIC_TRIG_MODE = 3,
	AW86907_HAPTIC_CONT_MODE = 4,
	AW86907_HAPTIC_RAM_LOOP_MODE = 5,
};

enum aw86907_haptic_bst_mode {
	AW86907_HAPTIC_BYPASS_MODE = 0,
	AW86907_HAPTIC_BOOST_MODE = 1,
};

enum aw86907_haptic_activate_mode {
	AW86907_HAPTIC_ACTIVATE_RAM_MODE = 0,
	AW86907_HAPTIC_ACTIVATE_CONT_MODE = 1,
};

enum aw86907_haptic_cont_vbat_comp_mode {
	AW86907_HAPTIC_CONT_VBAT_SW_COMP_MODE = 0,
	AW86907_HAPTIC_CONT_VBAT_HW_COMP_MODE = 1,
};

enum aw86907_haptic_ram_vbat_comp_mode {
	AW86907_HAPTIC_RAM_VBAT_COMP_DISABLE = 0,
	AW86907_HAPTIC_RAM_VBAT_COMP_ENABLE = 1,
};

enum aw86907_haptic_f0_flag {
	AW86907_HAPTIC_LRA_F0 = 0,
	AW86907_HAPTIC_CALI_F0 = 1,
};

enum aw86907_haptic_pwm_mode {
	AW86907_PWM_48K = 0,
	AW86907_PWM_24K = 1,
	AW86907_PWM_12K = 2,
};

enum aw86907_haptic_clock_type {
	AW86907_HAPTIC_CLOCK_CALI_F0 = 0,
	AW86907_HAPTIC_CLOCK_CALI_OSC_STANDARD = 1,

};
enum aw86907_haptic_play {
	AW86907_HAPTIC_PLAY_NULL = 0,
	AW86907_HAPTIC_PLAY_ENABLE = 1,
	AW86907_HAPTIC_PLAY_STOP = 2,
	AW86907_HAPTIC_PLAY_GAIN = 8,
};

enum aw86907_haptic_cmd {
	AW86907_HAPTIC_CMD_NULL = 0,
	AW86907_HAPTIC_CMD_ENABLE = 1,
	AW86907_HAPTIC_CMD_HAPTIC = 0x0f,
	AW86907_HAPTIC_CMD_TP = 0x10,
	AW86907_HAPTIC_CMD_SYS = 0xf0,
	AW86907_HAPTIC_CMD_STOP = 255,
};

enum aw86907_haptic_tp_flag {
	AW86907_HAPTIC_TP_NULL = 0,
	AW86907_HAPTIC_TP_PRESS = 1,
	AW86907_HAPTIC_TP_PRESS_HOLD = 2,
	AW86907_HAPTIC_TP_RELEASE = 3,
	AW86907_HAPTIC_TP_RELEASE_HOLD = 4,
};

enum aw86907_haptic_tp_staus {
	AW86907_HAPTIC_TP_ST_RELEASE = 0,
	AW86907_HAPTIC_TP_ST_PRESS = 1,
};

enum aw86907_haptic_tp_play_flag {
	AW86907_HAPTIC_TP_PLAY_NULL = 0,
	AW86907_HAPTIC_TP_PLAY_ENABLE = 1,
	AW86907_HAPTIC_TP_PLAY_NOMORE = 2,
};

enum aw86907_haptic_tp_touch_flag {
	AW86907_HAPTIC_TP_TOUCH_INVAIL = 0,
	AW86907_HAPTIC_TP_TOUCH_VAIL = 1,
};

#define AW86907_HAPTIC_TP_ID_MAX 10

#define AW86907_HAPTIC_AI_X_JITTER 28
#define AW86907_HAPTIC_AI_Y_JITTER 28
#define AW86907_HAPTIC_AI_X_DFT_W 200
#define AW86907_HAPTIC_AI_Y_DFT_H 200

enum aw86907_haptic_tz_level {
	AW86907_HAPTIC_TZ_LEVEL_LOW = 0,
	AW86907_HAPTIC_TZ_LEVEL_HIGH = 1,
};
/*********************************************************
 *
 * struct
 *
 ********************************************************/
struct tp_input_info {
	uint8_t id;
	uint8_t status;
	uint16_t x;
	uint16_t y;
};

struct trust_zone_info {
	uint8_t level;
	uint16_t x;
	uint16_t y;
	uint16_t w;
	uint16_t h;
};

struct ai_trust_zone {
	uint8_t num;
	struct trust_zone_info *tz_info;
};

struct haptic_audio_trust_zone {
	uint8_t level; //tz score
	uint8_t cnt;
	uint8_t dirty;
	uint16_t x;
	uint16_t y;
	uint16_t w;
	uint16_t h;
	struct list_head list;
};

struct haptic_audio_tp_size {
	uint16_t x;
	uint16_t y;
};

struct shake_point {
	uint8_t id;
	uint16_t x;
	uint16_t y;
	uint8_t status;
	uint8_t touch_flag;
	uint8_t touch_outside_tz_flag;
};

struct fileops {
	unsigned char cmd;
	unsigned char reg;
	unsigned char ram_addrh;
	unsigned char ram_addrl;
};

struct ram {
	unsigned int len;
	unsigned int check_sum;
	unsigned int base_addr;
	unsigned char version;
	unsigned char ram_shift;
	unsigned char baseaddr_shift;
};

struct haptic_ctr {
	unsigned char cnt;
	unsigned char cmd;
	unsigned char play;
	unsigned char wavseq;
	unsigned char loop;
	unsigned char gain;
	struct list_head list;
};

struct tp_id {
	struct shake_point pt_info;
	unsigned char tp_flag;
	unsigned char press_flag;
	unsigned char release_flag;
	struct timeval t_press;
	struct timeval t_release;
	unsigned char play_flag;
	unsigned int no_play_cnt;
	unsigned char tp_ai_match_flag;
	unsigned char press_no_vibrate_flag; //tp press but no vibrate event
	unsigned char release_no_vibrate_flag;
};

struct tp {
	struct tp_id id[AW86907_HAPTIC_TP_ID_MAX + 1];
	unsigned char id_index;
	unsigned char virtual_id;
	unsigned int press_delay_min;
	unsigned int press_delay_max;
	unsigned int release_delay_max;
	unsigned char play_flag;
	unsigned char last_play_flag;
	unsigned char press_flag;
	unsigned char tp_ai_match_flag;
	unsigned char tp_ai_check_flag;
	unsigned char hap_match_without_tz_cnt;
	unsigned int no_play_cnt_max;
};
struct haptic_audio {
	struct mutex lock;
	struct hrtimer timer;
	struct work_struct work;
	int delay_val;
	int timer_val;
	//unsigned char cnt;
	//struct haptic_ctr data[256];
	struct haptic_ctr ctr;
	struct list_head ctr_list;
	struct tp tp;
	struct list_head list;
	struct list_head score_list;
	struct haptic_audio_tp_size tp_size;
	struct trust_zone_info output_tz_info[10];
	int tz_num;
	int tz_high_num;
	int tz_cnt_thr;
	int tz_cnt_max;
	unsigned int uevent_report_flag;
	unsigned int hap_cnt_outside_tz;
	unsigned int hap_cnt_max_outside_tz;
};

struct trig {
	unsigned char enable;
	unsigned char default_level;
	unsigned char dual_edge;
	unsigned char frist_seq;
	unsigned char second_seq;
};

#define AAC_RICHTAP
#ifdef AAC_RICHTAP

enum {
	RICHTAP_UNKNOWN = -1,
	RICHTAP_AW_8697 = 0x05,
};

enum {
	MMAP_BUF_DATA_VALID = 0x55,
	MMAP_BUF_DATA_FINISHED = 0xAA,
	MMAP_BUF_DATA_INVALID = 0xFF,
};

#define RICHTAP_IOCTL_GROUP 0x52
#define RICHTAP_GET_HWINFO _IO(RICHTAP_IOCTL_GROUP, 0x03)
#define RICHTAP_SET_FREQ _IO(RICHTAP_IOCTL_GROUP, 0x04)
#define RICHTAP_SETTING_GAIN _IO(RICHTAP_IOCTL_GROUP, 0x05)
#define RICHTAP_OFF_MODE _IO(RICHTAP_IOCTL_GROUP, 0x06)
#define RICHTAP_TIMEOUT_MODE _IO(RICHTAP_IOCTL_GROUP, 0x07)
#define RICHTAP_RAM_MODE _IO(RICHTAP_IOCTL_GROUP, 0x08)
#define RICHTAP_RTP_MODE _IO(RICHTAP_IOCTL_GROUP, 0x09)
#define RICHTAP_STREAM_MODE _IO(RICHTAP_IOCTL_GROUP, 0x0A)
#define RICHTAP_UPDATE_RAM _IO(RICHTAP_IOCTL_GROUP, 0x10)
#define RICHTAP_GET_F0 _IO(RICHTAP_IOCTL_GROUP, 0x11)
#define RICHTAP_STOP_MODE _IO(RICHTAP_IOCTL_GROUP, 0x12)

#define RICHTAP_MMAP_BUF_SIZE 1000
#define RICHTAP_MMAP_PAGE_ORDER 2
#define RICHTAP_MMAP_BUF_SUM 16

#pragma pack(4)
struct mmap_buf_format {
	uint8_t status;
	uint8_t bit;
	int16_t length;
	uint32_t reserve;
	struct mmap_buf_format *kernel_next;
	struct mmap_buf_format *user_next;
	uint8_t data[RICHTAP_MMAP_BUF_SIZE];
};
#pragma pack()

#endif

struct aw86907 {
	struct regmap *regmap;
	struct i2c_client *i2c;
	struct device *dev;
	struct input_dev *input;

	struct mutex lock;
	struct mutex rtp_lock;
	struct hrtimer timer;
	struct work_struct vibrator_work;
	struct work_struct rtp_work;
	struct work_struct rtp_single_cycle_work;
	struct work_struct rtp_regroup_work;
	struct delayed_work ram_work;
#ifdef TIMED_OUTPUT
	struct timed_output_dev to_dev;
#else
	struct led_classdev cdev;
#endif
	struct fileops fileops;
	struct ram ram;
	bool haptic_ready;
	bool audio_ready;
	int pre_haptic_number;
	bool rtp_on;
	struct timeval start, end;
	unsigned int timeval_flags;
	unsigned int osc_cali_flag;
	unsigned long int microsecond;
	unsigned int sys_frequency;
	unsigned int rtp_len;

	int reset_gpio;
	int irq_gpio;
	int device_id;

	unsigned char hwen_flag;
	unsigned char flags;
	unsigned char chipid;

	unsigned char play_mode;

	unsigned char activate_mode;

	unsigned char auto_boost;

	int state;
	int duration;
	int amplitude;
	int index;
	int vmax;
	int gain;
	unsigned int gun_type; //hch 20190917
	unsigned int bullet_nr; //hch 20190917

	unsigned char seq[AW86907_SEQUENCER_SIZE];
	unsigned char loop[AW86907_SEQUENCER_SIZE];

	unsigned int rtp_cnt;
	unsigned int rtp_file_num;
	unsigned int rtp_loop;
	unsigned int rtp_cycle_flag;
	unsigned int rtp_serial[AW86907_RTP_NUM];

	unsigned char rtp_init;
	unsigned char ram_init;
	unsigned char rtp_routine_on;

	unsigned int f0;
	unsigned int cont_f0;
	unsigned char f0_cali_flag;

	unsigned int f0_pre;
	unsigned int f0_cali_percent;
	unsigned int cont_drv1_lvl;
	unsigned int cont_drv2_lvl;
	unsigned int cont_drv1_time;
	unsigned int cont_drv2_time;
	unsigned int cont_wait_num;
	unsigned int cont_brk_time;
	unsigned int cont_track_margin;
	unsigned int cont_test;
	unsigned int cont_drv_width;
	unsigned int cont_bemf_set;
	unsigned int cont_brk_gain;
	unsigned int cont_bst_brk_gain;
	unsigned int d2s_gain;

	unsigned char ram_vbat_comp;
	unsigned int vbat;
	unsigned int lra;
	unsigned char haptic_real_f0;

	unsigned int ram_test_flag_0;
	unsigned int ram_test_flag_1;

	struct haptic_audio haptic_audio;
	//struct notifier_block fb_notif;/*register to control tp report*/
	struct timeval t_stop;
	struct timeval t_start;
	unsigned int game_microsecond;
	unsigned int clock_standard_osc_lra_rtim_code;
	unsigned int clock_system_f0_cali_lra;

#ifdef AAC_RICHTAP
	uint8_t *rtp_ptr;
	struct mmap_buf_format *start_buf;
	struct work_struct haptic_rtp_work;
	//wait_queue_head_t doneQ;
	bool done_flag;
	bool haptic_rtp_mode;
#endif

	struct work_struct motor_old_test_work;
	unsigned int motor_old_test_mode;
	atomic_t qos_cnt;
#ifdef OPLUS_FEATURE_CHG_BASIC
	bool support_promise_vib;
#endif
};

struct aw86907_container {
	int len;
	unsigned char data[];
};

/*********************************************************
 *
 * ioctl
 *
 ********************************************************/
struct aw86907_seq_loop {
	unsigned char loop[AW86907_SEQUENCER_SIZE];
};

struct aw86907_que_seq {
	unsigned char index[AW86907_SEQUENCER_SIZE];
};

#define AW86907_HAPTIC_IOCTL_MAGIC 'h'

#define AW86907_HAPTIC_SET_QUE_SEQ                                             \
	_IOWR(AW86907_HAPTIC_IOCTL_MAGIC, 1, struct aw86907_que_seq *)
#define AW86907_HAPTIC_SET_SEQ_LOOP                                            \
	_IOWR(AW86907_HAPTIC_IOCTL_MAGIC, 2, struct aw86907_seq_loop *)
#define AW86907_HAPTIC_PLAY_QUE_SEQ                                            \
	_IOWR(AW86907_HAPTIC_IOCTL_MAGIC, 3, unsigned int)
#define AW86907_HAPTIC_SET_BST_VOL                                             \
	_IOWR(AW86907_HAPTIC_IOCTL_MAGIC, 4, unsigned int)
#define AW86907_HAPTIC_SET_BST_PEAK_CUR                                        \
	_IOWR(AW86907_HAPTIC_IOCTL_MAGIC, 5, unsigned int)
#define AW86907_HAPTIC_SET_GAIN                                                \
	_IOWR(AW86907_HAPTIC_IOCTL_MAGIC, 6, unsigned int)
#define AW86907_HAPTIC_PLAY_REPEAT_SEQ                                         \
	_IOWR(AW86907_HAPTIC_IOCTL_MAGIC, 7, unsigned int)

#ifdef OPLUS_FEATURE_CHG_BASIC
#define F0_VAL_MAX_0815 1800
#define F0_VAL_MIN_0815 1600
#define F0_VAL_MAX_0832 2350
#define F0_VAL_MIN_0832 2250
#define F0_VAL_MAX_0833 2380
#define F0_VAL_MIN_0833 2260

#define AW86907_HAPTIC_BASE_VOLTAGE 6000
#define AW86907_HAPTIC_MAX_VOLTAGE 10000
#define AW86907_HAPTIC_LOW_LEVEL_VOL 800
#define AW86907_HAPTIC_LOW_LEVEL_REG_VAL 0
#define AW86907_HAPTIC_MEDIUM_LEVEL_VOL 1600
#define AW86907_HAPTIC_MEDIUM_LEVEL_REG_VAL 0
#define AW86907_HAPTIC_HIGH_LEVEL_VOL 2500
#define AW86907_HAPTIC_HIGH_LEVEL_REG_VAL 0x18

//#define AW86907_HAPTIC_RAM_VBAT_COMP_GAIN  0x80

#define AW86907_WAVEFORM_INDEX_TRADITIONAL_1 1
#define AW86907_WAVEFORM_INDEX_TRADITIONAL_2 2
#define AW86907_WAVEFORM_INDEX_TRADITIONAL_3 3
#define AW86907_WAVEFORM_INDEX_TRADITIONAL_4 4
#define AW86907_WAVEFORM_INDEX_TRADITIONAL_5 5
#define AW86907_WAVEFORM_INDEX_TRADITIONAL_6 6
#define AW86907_WAVEFORM_INDEX_TRADITIONAL_7 7
#define AW86907_WAVEFORM_INDEX_TRADITIONAL_8 8
#define AW86907_WAVEFORM_INDEX_TRADITIONAL_9 9
#define AW86907_WAVEFORM_INDEX_TRADITIONAL_10 10
#define AW86907_WAVEFORM_INDEX_TRADITIONAL_11 11
#define AW86907_WAVEFORM_INDEX_TRADITIONAL_12 12
#define AW86907_WAVEFORM_INDEX_TRADITIONAL_13 13
#define AW86907_WAVEFORM_INDEX_TRADITIONAL_14 14
#define AW86907_WAVEFORM_INDEX_TRADITIONAL_15 15

#define AW86907_RTP_LONG_SOUND_INDEX 44
#define AUDIO_READY_STATUS 1024
#define RINGTONES_START_INDEX 1
#define RINGTONES_END_INDEX 40
#define RINGTONES_SIMPLE_INDEX 48
#define RINGTONES_PURE_INDEX 49
#define NEW_RING_START 118
#define NEW_RING_END 160
#define OPLUS_RING_START 161
#define OPLUS_RING_END 170
#define OPLUS_NEW_RING_1_START 201
#define OPLUS_NEW_RING_1_END 280
#define OPLUS_NEW_RING_2_START 292
#define OPLUS_NEW_RING_2_END 293

#define AW86907_WAVEFORM_INDEX_CS_PRESS 16
#define AW86907_WAVEFORM_INDEX_TRANSIENT 8
#define AW86907_WAVEFORM_INDEX_SINE_CYCLE 9
#define AW86907_WAVEFORM_INDEX_HIGH_TEMP 51
#define AW86907_WAVEFORM_INDEX_OLD_STEADY 52
#define AW86907_WAVEFORM_INDEX_LISTEN_POP 53

enum aw86907_haptic_custom_level {
	HAPTIC_CUSTOM_LEVEL_WEAK = 0, // 3V
	HAPTIC_CUSTOM_LEVEL_MEDIUM = 1, // 6V
	HAPTIC_CUSTOM_LEVEL_STRONG = 2, // 9V
};

enum aw86907_haptic_custom_vibration_mode {
	VIBRATION_MODE_TRADITIONAL = 0,
	VIBRATION_MODE_RING = 1,
	VIBRATION_MODE_GAME = 2,
};

enum aw86907_haptic_motor_old_test_mode {
	MOTOR_OLD_TEST_TRANSIENT = 1,
	MOTOR_OLD_TEST_STEADY = 2,
	MOTOR_OLD_TEST_HIGH_TEMP_HUMIDITY = 3,
	MOTOR_OLD_TEST_LISTEN_POP = 4,
	MOTOR_OLD_TEST_ALL_NUM,
};

#endif

#endif
