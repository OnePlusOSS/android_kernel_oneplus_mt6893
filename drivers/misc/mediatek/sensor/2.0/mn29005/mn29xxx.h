#ifndef __ELAN_MN_SENSOR__
#define __ELAN_MN_SENSOR__


#define MN_MODE_IDLE        (0x00)
#define MN_MODE_ALS         (0x01)
#define MN_MODE_PS          (0x02)
#define MN_MODE_ALS_PS      (0x03)

#define MN_WAIT_0_MS            (0x0<<4)
#define MN_WAIT_2_MS            (0x1<<4)
#define MN_WAIT_4_MS            (0x2<<4)
#define MN_WAIT_8_MS            (0x3<<4)
#define MN_WAIT_12_MS           (0x4<<4)
#define MN_WAIT_20_MS           (0x5<<4)
#define MN_WAIT_30_MS           (0x6<<4)
#define MN_WAIT_40_MS           (0x7<<4)
#define MN_WAIT_50_MS           (0x8<<4)
#define MN_WAIT_75_MS           (0x9<<4)
#define MN_WAIT_100_MS          (0xA<<4)
#define MN_WAIT_150_MS          (0xB<<4)
#define MN_WAIT_200_MS          (0xC<<4)
#define MN_WAIT_300_MS          (0xD<<4)
#define MN_WAIT_400_MS          (0xE<<4)
#define MN_WAIT_SINGLE          (0xF<<4)
//static int mn_wait_value[] = {0, 2, 4, 8, 12, 20, 30, 40, 50, 75, 100, 150, 200, 300, 400};
//int mn_wait_len = sizeof(mn_wait_value)/sizeof(int);
#define ALS_PIXEL_NOR           (0xF)
#define ALS_DARK_OFF            (0xE)
#define ALS_DARK                (1 << 0)
#define ALS_CLEAR               (1 << 1)
#define ALS_IR                  (1 << 2)
#define WAIT_CLK_1M_DIS         (0 << 3)
#define WAIT_CLK_1M_EN          (1 << 3)
#define MN_INT_CTRL_ALS_OR_PS   (0 << 4)
#define MN_INT_CTRL_ALS         (1 << 4)
#define MN_INT_CTRL_PS          (2 << 4)
#define MN_ADC_FREQ_1M          (0 << 5)
#define MN_ADC_FREQ_500K        (1 << 5)
#define MN_AG_DIS               (0 << 7)
#define MN_AG_EN                (1 << 7)

#define MN_RESETN_RESET         (0 << 1)
#define MN_RESETN_RUN           (1 << 1)
#define MN_POWER_ON                 (0)
#define MN_POWER_OFF                (1)

#define MN_ALS_PRE              (0 << 6)
#define MN_ALS_STD              (1 << 6)
#define MN_ALS_INTT_2           (0<<2)
#define MN_ALS_INTT_4           (1<<2)
#define MN_ALS_INTT_8           (2<<2)
#define MN_ALS_INTT_16          (3<<2)
#define MN_ALS_INTT_32          (4<<2)
#define MN_ALS_INTT_64          (5<<2)
#define MN_ALS_INTT_128         (6<<2)
#define MN_ALS_INTT_256         (7<<2)
#define MN_ALS_INTT_512         (8<<2)
#define MN_ALS_INTT_768         (9<<2)
#define MN_ALS_INTT_1024        (10<<2)
#define MN_ALS_INTT_2048        (11<<2)
#define MN_ALS_INTT_4096        (12<<2)
#define MN_ALS_INTT_6144        (13<<2)
#define MN_ALS_INTT_8192        (14<<2)
#define MN_ALS_INTT_16384       (15<<2)
static int mn_als_intt_value[] = {2, 4, 8, 16, 32, 64, 128, 256, 512, 768, 1024, 2048, 4096, 6144, 8192, 16384};

#define MN_ALS_ENH_MODE_1       (0 << 6)
#define MN_ALS_ENH_MODE_2       (1 << 6)
#define MN_ALS_ENH_MODE_4       (2 << 6)
#define MN_ALS_ENH_MODE_8       (3 << 6)
#define MN_CYCLE_1              (0x00)
#define MN_CYCLE_2              (0x01)
#define MN_CYCLE_4              (0x02)
#define MN_CYCLE_8              (0x03)
#define MN_CYCLE_16             (0x04)
#define MN_CYCLE_32             (0x05)
#define MN_CYCLE_64             (0x06)
#define MN_CYCLE_128            (0x07)
#define MN_CYCLE_256            (0x08)
//static int mn_cycle_value[] = {1, 2, 4, 8, 16, 32, 64, 128, 256};

#define MN_ALS_INT_CHSEL_0      (0 << 4)
#define MN_ALS_INT_CHSEL_1      (1 << 4)

#define MN_IR_ON_CTRL_OFF       (0 << 7)
#define MN_IR_ON_CTRL_ON        (1 << 7)
#define MN_PS_PRE               (0 << 6)
#define MN_PS_STD               (1 << 6)
#define MN_PS_INTT_4            (0<<2)
#define MN_PS_INTT_8            (1<<2)
#define MN_PS_INTT_16           (2<<2)
#define MN_PS_INTT_24           (3<<2)
#define MN_PS_INTT_32           (4<<2)
#define MN_PS_INTT_48           (5<<2)
#define MN_PS_INTT_80           (6<<2)
#define MN_PS_INTT_144          (7<<2)
#define MN_PS_INTT_208          (8<<2)
#define MN_PS_INTT_272          (9<<2)
#define MN_PS_INTT_336          (10<<2)
#define MN_PS_INTT_400          (11<<2)
#define MN_PS_INTT_528          (12<<2)
#define MN_PS_INTT_656          (13<<2)
#define MN_PS_INTT_784          (14<<2)
#define MN_PS_INTT_1040         (15<<2)
//static int mn_ps_intt_value[] = {4, 8, 16, 24, 32, 48, 80, 144, 208, 272, 336, 400, 528, 656, 784, 1040};
#define MN_PS_PULSE(x)          (x)

#define MN_PS_ENH_MODE_1        (0 << 6)
#define MN_PS_ENH_MODE_2        (1 << 6)
#define MN_PS_ENH_MODE_4        (2 << 6)
#define MN_PS_ENH_MODE_8        (3 << 6)
#define MN_IR_DRIVE_15          (0)
#define MN_IR_DRIVE_50          (1)
#define MN_IR_DRIVE_100         (2)
#define MN_IR_DRIVE_200         (3)
#define MN_IR_DRIVE_LOW_B       (0)

#define MN_IR_DRIVE_5_43        (0 << 2)
#define MN_IR_DRIVE_8_10        (1 << 2)
#define MN_IR_DRIVE_10_52       (2 << 2)
#define MN_IR_DRIVE_12_78       (3 << 2)
#define MN_IR_DRIVE_14_92       (4 << 2)
#define MN_IR_DRIVE_16_98       (5 << 2)
#define MN_IR_DRIVE_18_94       (6 << 2)
#define MN_IR_DRIVE_20_0        (7 << 2)
#define MN_IR_DRIVE_NOR         (4 << 2)

#define MN_IR_PIXEL_0           (0<<3)
#define MN_IR_PIXEL_1           (1<<3)
#define MN_IR_PIXEL_2           (2<<3)

#define MN_RS_0                 (0x0<<4)
#define MN_RS_1                 (0x1<<4)
#define MN_RS_2                 (0x2<<4)
#define MN_RS_3                 (0x3<<4)
#define MN_RS_4                 (0x4<<4)
#define MN_RS_5                 (0x5<<4)
#define MN_RS_6                 (0x6<<4)
#define MN_RS_7                 (0x7<<4)
#define MN_RS_8                 (0x8<<4)
#define MN_RS_9                 (0x9<<4)
#define MN_RS_10                (0xA<<4)
#define MN_RS_11                (0xB<<4)
#define MN_RS_12                (0xC<<4)
#define MN_RS_13                (0xD<<4)
#define MN_RS_14                (0xE<<4)
#define MN_RS_15                (0xF<<4)

#define MN_AVG_16               (0x4<<4)
#define MN_AVG_32               (0x5<<4)
#define MN_AVG_64               (0x6<<4)
#define MN_AVG_128              (0x7<<4)
#define MN_AVG_256              (0x8<<4)

#define MN_GAIN_HIGH            (0x00)
#define MN_GAIN_MID             (0x01)
#define MN_GAIN_LOW             (0x03)

#define MN_INTTY_DISABLE        (0x00)
#define MN_INTTY_BINARY         (0x01)
#define MN_INTTY_ACTIVE         (0x02)
#define MN_INTTY_FRAME          (0x03)
#define MN_PERIST_1             (0x00 << 2)
#define MN_PERIST_4             (0x01 << 2)
#define MN_PERIST_8             (0x02 << 2)
#define MN_PERIST_16            (0x03 << 2)


#define MN_AG_H             (0<<4)
#define MN_AG_M             (1<<4)
#define MN_AG_L             (3<<4)

#define MN_INT_TRIGGER      (0x01 << 2)
#define MN_INT_CLEAR        (0x00 << 2)

#define MN_CMP_RESET        (0x00 << 1)
#define MN_CMP_RUN          (0x01 << 1)

#define MN_LOCK         (0x01)
#define MN_UN_LOCK      (0x00)

#define MN_REVNO       (0x12)


struct _ps_data
{
	uint16_t ir_data;
	uint16_t pdata;
};

struct _ges_data
{
	uint16_t ir_data;
	uint16_t data;
};

struct _ps_factory
{
	bool calibration_enable;
	bool calibrated;
	uint16_t cancelation;
	uint16_t high_threshold;
	uint16_t low_threshold;
};

#define ALS_CHANNEL_SIZE    3
struct _als_data
{
	uint16_t channel[ALS_CHANNEL_SIZE];
	uint16_t lux;
};

struct _als_factory
{
	uint16_t lux_per_count;
	uint16_t lux_per_lux;
};

struct _ps_setting
{
	bool polling_mode;
	uint8_t integration_time;
	uint8_t gain;
	uint8_t enh_mode;
	uint8_t pulse;
	uint8_t ps_avg;
	uint8_t ps_rs;
	uint8_t ps_intb_nonlos;
	uint16_t high_threshold;
	uint16_t low_threshold;
	uint8_t ps_std;
	uint8_t ir_on_control;
	uint8_t ir_mode;
	uint8_t ir_drive;
	uint8_t ir_drive_11;
	uint8_t persist;
	uint8_t interrupt_type;
	uint8_t sat_ctia;
	uint8_t sat_1;
	uint8_t sat;
	uint8_t compare_high;
	uint8_t compare_low;
	uint8_t interrupt_flag;
	uint8_t compare_reset;
	uint8_t lock;
	uint16_t cancelation;
	uint8_t ir_pixel_sel;
	uint16_t ps_max_ct;
	uint16_t dynk_max_ir_data;
	uint16_t dynk_low_offset;
	uint16_t dynk_high_offset;
	uint16_t dynk_min_ps_raw_data;
	struct _ps_data data;
};

struct _als_setting
{
	bool als_enable;
	bool als_first_flag;
	bool polling_mode;
	uint8_t report_type;
	uint8_t integration_time;
	uint8_t als_rs;
	uint8_t als_dark_pixel;
	uint8_t gain;
	uint8_t enh_mode;
	uint8_t cycle;
	uint16_t high_threshold;
	uint16_t low_threshold;
	uint8_t als_std;
	uint8_t persist;
	uint8_t interrupt_type;
	uint8_t sat_ctia;
	uint8_t sat_1;
	uint8_t sat;
	uint8_t compare_high;
	uint8_t compare_low;
	uint8_t interrupt_flag;
	uint8_t compare_reset;
	uint8_t lock;
	uint8_t interrupt_channel_select;
	uint16_t dyn_intt_raw;
	uint16_t als_intr_percent;
	uint16_t offset_gain;
	uint16_t scale_gain;
	//auto gain
	bool als_ag_change_flag;
	uint8_t als_ag_en;
	uint8_t als_ag_l;
	uint8_t als_ag_m;
	uint8_t als_ag_h;
	uint8_t als_aintt_l;
	uint8_t als_aintt_m;
	uint8_t als_aintt_h;
	uint8_t als_aenh_l;
	uint8_t als_aenh_m;
	uint8_t als_aenh_h;
	uint8_t als_acycle_l;
	uint8_t als_acycle_m;
	uint8_t als_acycle_h;
	uint16_t als_ag_h2m_thd;
	uint16_t als_ag_m2h_thd;
	uint16_t als_ag_m2l_thd;
	uint16_t als_ag_l2m_thd;
	int32_t  als_factor;
	struct _als_data data;
	struct _als_factory factory;
};

typedef struct _sensor
{
	uint8_t wait;
	uint8_t mode;
	uint8_t als_single_pixel;
	bool enable_factory_calibration;
	uint8_t early_suspend_mode;
	uint8_t osc_sel;
	uint8_t interrupt_control;
	uint8_t reset;
	uint8_t power;
	struct _ps_setting ps;
	struct _als_setting als;
	uint16_t revno;
}mn29_optical_sensor;

typedef enum {
	PS_NEAR,
	PS_FAR,
}ps_report_status_t;

#define L_SENSOR_LTHD           1000
#define L_SENSOR_HTHD           1000
#define L_SENSOR_LTHD_TRIGER    1001


/*REG Table*/
#define    DEVREG_ENABLE        0x00
#define    DEVREG_INT_CTRL      0x01
#define    DEVREG_RESET         0x02
#define    DEVREG_REV_ID        0x03
#define    DEVREG_CHIP_ID       0x04 //REG0x04 not defined in spec
#define    DEVREG_ALS_CONFIG    0x10
#define    DEVREG_ALS_FILT      0x11
#define    DEVREG_ALS_INT       0x12
#define    DEVREG_PS_CONFIG     0x20
#define    DEVREG_PS_PULSE      0x21
#define    DEVREG_PS_INT        0x22
#define    DEVREG_LED_CONFIG    0x23
#define    DEVREG_PS_OFSL       0x25
#define    DEVREG_PS_OFSH       0x26
#define    DEVREG_ALS_ILTL      0x30
#define    DEVREG_ALS_ILTH      0x31
#define    DEVREG_ALS_IHTL      0x32
#define    DEVREG_ALS_IHTH      0x33
#define    DEVREG_PS_ILTL       0x34
#define    DEVREG_PS_ILTH       0x35
#define    DEVREG_PS_IHTL       0x36
#define    DEVREG_PS_IHTH       0x37
#define    DEVREG_ALS_STATUS    0x38
#define    DEVREG_C0DATAL       0x39
#define    DEVREG_C0DATAH       0x3A
#define    DEVREG_C1DATAL       0x3B
#define    DEVREG_C1DATAH       0x3C
#define    DEVREG_C2DATAL       0x3B
#define    DEVREG_C2DATAH       0x3C
#define    DEVREG_PS_STATUS     0x3F
#define    DEVREG_PS_ADATAL     0x40
#define    DEVREG_PS_ADATAH     0x41
#define    DEVREG_PS_RDATAL     0x42
#define    DEVREG_PS_RDATAH     0x43
//AG Config
#define    DEVREG_ALS_AG_CFG_L  0x50
#define    DEVREG_ALS_AG_CFG_M  0x51
#define    DEVREG_ALS_AG_CFG_H  0x52
#define    DEVREG_ALS_AG_H2ML   0x53
#define    DEVREG_ALS_AG_H2MH   0x54
#define    DEVREG_ALS_AG_M2HL   0x55
#define    DEVREG_ALS_AG_M2HH   0x56
#define    DEVREG_ALS_AG_M2LL   0x57
#define    DEVREG_ALS_AG_M2LH   0x58
#define    DEVREG_ALS_AG_L2ML   0x59
#define    DEVREG_ALS_AG_L2MH   0x5A
#define    DEVREG_ALS_AG_INFO   0x5B

/******************************************************************************/

#endif
