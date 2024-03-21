// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2016 MediaTek Inc.
 */
#include <linux/kobject.h>
#include "ccci_fsm_internal.h"
#include "ccci_fsm_sys.h"

#define CCCI_KOBJ_NAME "md"

struct mdee_info_collect mdee_collect;

void fsm_sys_mdee_info_notify(char *buf)
{
	int ret = 0;

	spin_lock(&mdee_collect.mdee_info_lock);
	memset(mdee_collect.mdee_info, 0x0, AED_STR_LEN);
	ret = snprintf(mdee_collect.mdee_info, AED_STR_LEN, "%s", buf);
	if (ret < 0 || ret >= AED_STR_LEN)
		CCCI_ERROR_LOG(-1, FSM,
			"%s-%d:snprintf fail,ret = %d\n", __func__, __LINE__, ret);
	spin_unlock(&mdee_collect.mdee_info_lock);
}

static struct kobject fsm_kobj;

struct ccci_attribute {
	struct attribute attr;
	ssize_t (*show)(char *buf);
	ssize_t (*store)(const char *buf, size_t count);
};

#define CCCI_ATTR(_name, _mode, _show, _store)			\
static struct ccci_attribute ccci_attr_##_name = {		\
	.attr = {.name = __stringify(_name), .mode = _mode },	\
	.show = _show,						\
	.store = _store,					\
}

static void fsm_obj_release(struct kobject *kobj)
{
	CCCI_NORMAL_LOG(-1, FSM, "release fsm kobject\n");
}

static ssize_t ccci_attr_show(struct kobject *kobj,
				struct attribute *attr, char *buf)
{
	ssize_t len = 0;

	struct ccci_attribute *a
		= container_of(attr, struct ccci_attribute, attr);

	if (a->show)
		len = a->show(buf);

	return len;
}

static ssize_t ccci_attr_store(struct kobject *kobj,
		struct attribute *attr, const char *buf, size_t count)
{
	ssize_t len = 0;

	struct ccci_attribute *a
		= container_of(attr, struct ccci_attribute, attr);

	if (a->store)
		len = a->store(buf, count);

	return len;
}

static const struct sysfs_ops fsm_sysfs_ops = {
	.show = ccci_attr_show,
	.store = ccci_attr_store,
};

static ssize_t ccci_mdee_info_show(char *buf)
{
	int curr = 0;

	spin_lock(&mdee_collect.mdee_info_lock);
	curr = snprintf(buf, AED_STR_LEN, "%s\n", mdee_collect.mdee_info);
	if (curr < 0 || curr >= AED_STR_LEN) {
		CCCI_ERROR_LOG(-1, FSM,
			"%s-%d:snprintf fail,curr = %d\n", __func__, __LINE__, curr);
		spin_unlock(&mdee_collect.mdee_info_lock);
		return -1;
	}
	spin_unlock(&mdee_collect.mdee_info_lock);

	return curr;
}

CCCI_ATTR(mdee, 0444, &ccci_mdee_info_show, NULL);

/* Sys -- Add to group */
static struct attribute *ccci_default_attrs[] = {
	&ccci_attr_mdee.attr,
	NULL
};

static struct kobj_type fsm_ktype = {
	.release = fsm_obj_release,
	.sysfs_ops = &fsm_sysfs_ops,
	.default_attrs = ccci_default_attrs
};

int fsm_sys_init(void)
{
	int ret = 0;

	spin_lock_init(&mdee_collect.mdee_info_lock);

	ret = kobject_init_and_add(&fsm_kobj, &fsm_ktype,
			kernel_kobj, CCCI_KOBJ_NAME);
	if (ret < 0) {
		kobject_put(&fsm_kobj);
		CCCI_ERROR_LOG(-1, FSM, "fail to add fsm kobject\n");
		return ret;
	}

	return ret;
}

//#ifdef OPLUS_FEATURE_MODEM_MINIDUMP
//#ifdef OPLUS_FEATURE_MODEM_MINIDUMP
int getSubstrIndex(char *srcStr, char *subStr) {
	int index = 0;
	char* myStr = srcStr;
	char* mySub = subStr;
	char* temp_mystr = NULL;
	char* temp_mysub = NULL;

	while (*myStr != '\0')
	{
		if (*myStr != *mySub) {
			++myStr;
			index++;
			continue;
		}
		temp_mystr = myStr;
		temp_mysub = mySub;
		while (*temp_mysub != '\0') {

			if (*temp_mystr != *temp_mysub) {
				/* The following characters are not equal,
				so we need to skip them directly because we have already
				compared them and there is no need to compare them again. */
				++myStr;
				index++;
				break;
			}
			/* Two characters are equal, and both pointers move backwards at the same time */
			++temp_mysub;
			++temp_mystr;
		}
		if (*temp_mysub == '\0') {
			return index;
		}
	}
	return -1;
}

void deleteChar(char* str, int strLen, int index) {
	int i = index;
	for (i = index; i + 1 < strLen; i++) {
		str[i] = str[i + 1];
	}
	str[strLen-1] = '\0';
}

unsigned int BKDRHash(const char* str, unsigned int len, int md_id) {
	unsigned int seed = 131u; /* 31 131 1313 13131 131313 etc.. */
	unsigned int hash = 0u;
	int i = 0;
	CCCI_ERROR_LOG(md_id, FSM, "BKDRHash str: %s, len: %d\n", str, len);

	if (str == NULL) {
		return 0;
	}

	for (i = 0; i < len; str++, i++) {
		hash = (hash * seed) + (unsigned int)(*str);
		hash = hash & 0xffffffff;
	}
	CCCI_ERROR_LOG(md_id, FSM, "BKDRHash hash: %u\n", hash);
	return hash;
}
//#endif /*OPLUS_FEATURE_MODEM_MINIDUMP*/