#include "mtk_ts_ntc_cust.h"

static struct mtk_ntc_cust mtk_ntc_cust_infos[NTC_NUM_MAX];

void mtk_ts_ntc_cust_parse_dt(struct device_node *node, int ntc_index)
{
	int rc;
	struct mtk_ntc_cust *mtk_ntc_cust_ptr = NULL;

	if (!node) {
		pr_err("mtk_ts_ntc_cust_parse_dt err, node null. ntc_index: %d\n", ntc_index);
		return;
	}

	if (ntc_index < 0 || ntc_index >= NTC_NUM_MAX) {
		pr_err("mtk_ts_ntc_cust_parse_dt err, ntc_index err. ntc_index: %d\n", ntc_index);
		return;
	}

	mtk_ntc_cust_ptr = &mtk_ntc_cust_infos[ntc_index];

	memset(mtk_ntc_cust_ptr, -1, sizeof(struct mtk_ntc_cust));

	rc = of_property_read_u32(node, "oplus,ntc_cust_support",
			&mtk_ntc_cust_ptr->ntc_cust_support);
	if (rc) {
		mtk_ntc_cust_ptr->ntc_cust_support = -1;

	}

	if (mtk_ntc_cust_ptr->ntc_cust_support != 1) {
		pr_err("mtk_ts_ntc_cust_parse_dt: ntc cust not support, force return. ntc_index: %d\n", ntc_index);
		return;
	}

	rc = of_property_read_u32(node, "oplus,ntc_pull_up_r",
			&mtk_ntc_cust_ptr->pull_up_r);
	if (rc) {
		mtk_ntc_cust_ptr->pull_up_r = -1;

	}

	rc = of_property_read_u32(node, "oplus,over_critical_low",
			&mtk_ntc_cust_ptr->over_critical_low);
	if (rc) {
		mtk_ntc_cust_ptr->over_critical_low = -1;

	}

	rc = of_property_read_u32(node, "oplus,pull_up_voltage",
			&mtk_ntc_cust_ptr->pull_up_voltage);
	if (rc) {
		mtk_ntc_cust_ptr->pull_up_voltage = -1;

	}

	rc = of_property_read_u32(node, "oplus,ntc_table",
			&mtk_ntc_cust_ptr->ntc_table);
	if (rc) {
		mtk_ntc_cust_ptr->ntc_table = -1;

	}

	rc = of_property_read_u32(node, "oplus,adc_channel",
			&mtk_ntc_cust_ptr->adc_channel);
	if (rc) {
		mtk_ntc_cust_ptr->adc_channel = -1;

	}
	pr_err("mtk_ts_ntc_cust_parse_dt[%d]->pull_up_r: %d over_critical_low: %d pull_up_voltage: %d ntc_table: %d adc_channel: %d\n",
		ntc_index, mtk_ntc_cust_ptr->pull_up_r, mtk_ntc_cust_ptr->over_critical_low, mtk_ntc_cust_ptr->pull_up_voltage, mtk_ntc_cust_ptr->ntc_table, mtk_ntc_cust_ptr->adc_channel);
}

void mtk_ts_ntc_overide_by_cust_if_needed(int *src, int param_index, int ntc_index)
{
	struct mtk_ntc_cust *mtk_ntc_cust_ptr = NULL;
	int *param_prt = NULL;
	int ori_value = *src;

	if (ntc_index < 0 || ntc_index >= NTC_NUM_MAX) {
		pr_err("mtk_ts_ntc_overide_by_cust_if_needed err, ntc_index err. ntc_index: %d\n", ntc_index);
		return;
	}

	if (param_index < 0 || param_index > ADC_CHANNEL_INDEX) {
		pr_err("mtk_ts_ntc_overide_by_cust_if_needed err, param_index err. ntc_index: %d\n", ntc_index);
		return;
	}

	mtk_ntc_cust_ptr = &mtk_ntc_cust_infos[ntc_index];
	param_prt = &mtk_ntc_cust_infos[ntc_index].ntc_cust_support + param_index;


	if (mtk_ntc_cust_ptr->ntc_cust_support != 1) {
		pr_err("mtk_ts_ntc_overide_by_cust_if_needed: ntc cust not support, force return. ntc_index: %d\n", ntc_index);
		return;
	}

	if (param_prt && *param_prt != -1) {
		*src = *param_prt;
		pr_err("mtk_ts_ntc_overide_by_cust_if_needed: overide by cust vaule: %d, ori: %d, ntc_index: %d, param_index: %d\n",
			*param_prt, ori_value, ntc_index, param_index);
	}
}

int mtk_ts_ntc_cust_get(int param_index, int ntc_index)
{
	struct mtk_ntc_cust *mtk_ntc_cust_ptr = NULL;
	int *param_prt = NULL;

	if (ntc_index < 0 || ntc_index >= NTC_NUM_MAX) {
		pr_err("mtk_ts_ntc_cust_get err, ntc_index err. ntc_index: %d\n", ntc_index);
		return -1;
	}

	if (param_index < 0 || param_index > ADC_CHANNEL_INDEX) {
		pr_err("mtk_ts_ntc_cust_get err, param_index err. ntc_index: %d\n", ntc_index);
		return -1;
	}

	mtk_ntc_cust_ptr = &mtk_ntc_cust_infos[ntc_index];
	param_prt = &mtk_ntc_cust_infos[ntc_index].ntc_cust_support + param_index;


	if (mtk_ntc_cust_ptr->ntc_cust_support != 1) {
		pr_err("mtk_ts_ntc_overide_by_cust_if_needed: ntc cust not support, force return. ntc_index: %d\n", ntc_index);
		return -1;
	}

	if (param_prt && *param_prt != -1) {
		pr_err("mtk_ts_ntc_cust_get: get cust vaule: %d, ntc_index: %d, param_index: %d\n",
			*param_prt, ntc_index, param_index);
		return *param_prt;
	}

	return -1;
}

