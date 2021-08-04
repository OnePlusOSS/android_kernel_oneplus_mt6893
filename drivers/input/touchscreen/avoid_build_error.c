
#include <linux/module.h>

#ifndef CONFIG_TOUCHPANEL_OPLUS
int g_tp_dev_vendor = 0;

void switch_usb_state(bool usb_state)
{
    return;
}
EXPORT_SYMBOL(switch_usb_state);

void lcd_queue_load_tp_fw(void)
{
    return;
}
EXPORT_SYMBOL(lcd_queue_load_tp_fw);

int tp_gesture_enable_flag(void)
{
    return 1;
}
EXPORT_SYMBOL(tp_gesture_enable_flag);
#endif

int tp_usb_notifier_call_chain(unsigned long val, void *v)
{
    return 1;
}

#ifndef CONFIG_TOUCHSCREEN_NT36525
int NVT_TP = 0;

void lcd_resume_load_nvt_fw(void)
{
        return;
}
EXPORT_SYMBOL(lcd_resume_load_nvt_fw);
#endif

MODULE_AUTHOR("liunianliang@vanyol.com");
MODULE_DESCRIPTION("Avoid build for none oplus TP Architecture");
MODULE_LICENSE("GPL");
MODULE_ALIAS("avoid build");
