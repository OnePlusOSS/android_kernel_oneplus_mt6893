/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef AC102_P_3_A0010_FHDP_DSI_VDO_H
#define AC102_P_3_A0010_FHDP_DSI_VDO_H

#define REGFLAG_DELAY           0xFFFC
#define REGFLAG_UDELAY          0xFFFB
#define REGFLAG_END_OF_TABLE    0xFFFD
#define REGFLAG_RESET_LOW       0xFFFE
#define REGFLAG_RESET_HIGH      0xFFFF

#define FRAME_WIDTH                 1080
#define FRAME_HEIGHT                2400

#define PHYSICAL_WIDTH              68364
#define PHYSICAL_HEIGHT             152300

#define MIPI_CLK                    374
#define DATA_RATE                   748
#define HSA                         10
#define HBP                         30
#define VSA                         10
#define VBP                         20
#define HFP                         136
#define HOPPING_MIPI_CLK            379
#define HOPPING_DATA_RATE           758
#define HOPPING_HBP                 38
/*Parameter setting for mode 0 Start*/
#define MODE_0_FPS                  60
#define MODE_0_VFP                  1296
/*Parameter setting for mode 0 End*/

/*Parameter setting for mode 1 Start*/
#define MODE_1_FPS                  90
#define MODE_1_VFP                  54
/*Parameter setting for mode 1 End*/

/*Parameter setting for mode 2 Start*/
#define MODE_2_FPS                  50
#define MODE_2_VFP                  2040
/*Parameter setting for mode 2 End*/

/*Parameter setting for mode 3 Start*/
#define MODE_3_FPS                  48
#define MODE_3_VFP                  2230
/*Parameter setting for mode 3 End*/

/*Parameter setting for mode 4 Start*/
#define MODE_4_FPS                  30
#define MODE_4_VFP                  5020
/*Parameter setting for mode 4 End*/

#define LFR_EN                      0
/* DSC RELATED */

#define DSC_ENABLE                  1
#define DSC_VER                     17
#define DSC_SLICE_MODE              1
#define DSC_RGB_SWAP                0
#define DSC_DSC_CFG                 34
#define DSC_RCT_ON                  1
#define DSC_BIT_PER_CHANNEL         8
#define DSC_DSC_LINE_BUF_DEPTH      9
#define DSC_BP_ENABLE               1
#define DSC_BIT_PER_PIXEL           128
#define DSC_PIC_HEIGHT              2400
#define DSC_PIC_WIDTH               1080
#define DSC_SLICE_HEIGHT            12
#define DSC_SLICE_WIDTH             540
#define DSC_CHUNK_SIZE              540
#define DSC_XMIT_DELAY              170
#define DSC_DEC_DELAY               526
#define DSC_SCALE_VALUE             32
#define DSC_INCREMENT_INTERVAL      67
#define DSC_DECREMENT_INTERVAL      7
#define DSC_LINE_BPG_OFFSET         12
#define DSC_NFL_BPG_OFFSET          2235
#define DSC_SLICE_BPG_OFFSET        2170
#define DSC_INITIAL_OFFSET          6144
#define DSC_FINAL_OFFSET            7072
#define DSC_FLATNESS_MINQP          3
#define DSC_FLATNESS_MAXQP          12
#define DSC_RC_MODEL_SIZE           8192
#define DSC_RC_EDGE_FACTOR          6
#define DSC_RC_QUANT_INCR_LIMIT0    11
#define DSC_RC_QUANT_INCR_LIMIT1    11
#define DSC_RC_TGT_OFFSET_HI        3
#define DSC_RC_TGT_OFFSET_LO        3

#endif //end of AC102_P_3_A0010_FHDP_DSI_VDO_H
