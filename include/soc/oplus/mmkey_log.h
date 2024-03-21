/************************************************************************************
** File: - android\kernel\include\soc\oplus\mmkey_log.h
** Copyright (C), 2008-2015, OPLUS Mobile Comm Corp., Ltd
**
** Description:
**      oplus key log multimedia issue id header file
** Version: 1.0
** --------------------------- Revision History: --------------------------------
**  <author><data>      <desc>
************************************************************************************/

#ifndef MMKEYLOG_H_
#define MMKEYLOG_H_

//MultiMedia issue type range is 200~399,
//mediaser and surfaceflinger use 200~299
//kernel use 300~399
enum mmkeylog_issue{
    TYPE_SOUND_CARD_REGISTER_FAIL = 300,
    TYPE_ADSP_LOAD_FAIL,
    TYPE_SMART_PA_EXCEPTION,
    TYPE_NO_DATA_TO_SHOW,
    TYPE_KGSL_EXCEPTION,
    TYPE_VSYNC_EXCEPTION,
    TYPE_ESD_EXCEPTION,
    TYPE_GPU_EXCEPTION,
    TYPE_IOMMU_ERROR,
    TYPE_FENCE_TIMEOUT,
    TYPE_BL_EXCEPTION,
    TYPE_ADSP_CLK_OPEN_TIMEOUT,
    TYPE_HP_PA_EXCEPTION,
};


enum conkeylog_issue{
    TYPE_SYMBOL_VERSION_DISAGREE = 803,
    TYPE_WDI_EXCEPTION,
};



//record subSystem crash
enum androidlog_issue{
    TYPE_SUBSYSTEM_RESTART = 1001,
};


extern void mm_keylog_write(const char *logmessage, const char *cause, int id);
/* #ifdef OPLUS_FEATURE_MODEM_MINIDUMP */
extern void mm_keylog_write_modemdump(unsigned int hashId, const char *cause, int id, char *subsys);
/* #endif OPLUS_FEATURE_MODEM_MINIDUMP */
#endif
