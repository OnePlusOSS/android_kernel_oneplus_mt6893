#ifndef _OPLUS_TEMPNTC_H_
#define _OPLUS_TEMPNTC_H_

#include <soc/oplus/system/oplus_project.h>

extern bool oplus_voocphy_get_bidirect_cp_support(void);

static inline int is_ntc_switch_projects(void)
{
       if (get_project() == 21061) {
               return 1;
       } else if ((get_project() == 21015) && (oplus_voocphy_get_bidirect_cp_support() == TRUE)) {
               return 1;
       }
       return 0;
}

/* 4G PA NTC */
int oplus_get_pa1_con_temp(void);
/* 5G PA NTC */
int oplus_get_pa2_con_temp(void);
#endif
