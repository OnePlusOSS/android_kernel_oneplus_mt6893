#ifndef _MTK_TS_NTC_CUST_H
#define _MTK_TS_NTC_CUST_H
#include <linux/of.h>

struct mtk_ntc_cust {
	int ntc_cust_support;
	int pull_up_r;
	int over_critical_low;
	int pull_up_voltage;
	int ntc_table;
	int adc_channel;
};

enum {
	NTC_BTS,
	NTC_BTSMDPA,
	NTC_BTSNRPA,
	NTC_BTSCHARGER,
	NTC_NUM_MAX,
};

enum {
	NTC_CUST_SUPPORT,
	PULL_UP_R_INDEX,
	OVER_CRITICAL_LOW_INDEX,
	PULL_UP_VOLTAGE_INDEX,
	NTC_TABLE_INDEX,
	ADC_CHANNEL_INDEX,
};
#ifdef CONFIG_HORAE_THERMAL_SHELL
void mtk_ts_ntc_cust_parse_dt(struct device_node *node, int ntc_index);
void mtk_ts_ntc_overide_by_cust_if_needed(int *src, int param_index, int ntc_index);
int mtk_ts_ntc_cust_get(int param_index, int ntc_index);
#else
void mtk_ts_ntc_cust_parse_dt(struct device_node *node, int ntc_index){}
void mtk_ts_ntc_overide_by_cust_if_needed(int *src, int param_index, int ntc_index){}
int mtk_ts_ntc_cust_get(int param_index, int ntc_index){return -1;}
#endif
#endif
