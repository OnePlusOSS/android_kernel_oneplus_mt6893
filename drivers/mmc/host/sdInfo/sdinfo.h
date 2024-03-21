/****************************************************************
 ** Copyright (C), 2021-2030, OPLUS Mobile Comm Corp., Ltd.
 ** File: - sdinfo.h
 ** Description: record sdcard information
 ** Version: 1.0
 ** Date : 2021-07-13
 ** Author:
 **
 ** ----------------------Revision History: --------------------
 **  <author>      <date>      <version>      <desc>
 **                2021-07-13  1.0            sdcard info
 ****************************************************************/


#define SET_SDCARD_QUICK_RETURN		1
#define RESET_SDCARD_QUICK_RETURN	2
#define SET_DMA_DATA_TIMEOUT		3
#define RESET_DMA_DATA_TIMEOUT		4

struct sd_info {
	unsigned int	manfid;
	unsigned int	csd_version;	/* 0 or 1 */
	unsigned long	capacity;	/* sectors */
	unsigned int	supported_bus_mode;
					/**
					SD_MODE_UHS_SDR104
					SD_MODE_UHS_SDR50
					SD_MODE_UHS_DDR50
					SD_MODE_UHS_SDR25
					SD_MODE_UHS_SDR12
					**/
	unsigned int 	sd_bus_speed;	/** real bus speed
					UHS_SDR104_BUS_SPEED
					UHS_SDR50_BUS_SPEED
					UHS_DDR50_BUS_SPEED
					UHS_SDR25_BUS_SPEED
					UHS_SDR12_BUS_SPEED
					**/
	unsigned int	taac_ns;	/* read data access time (ns per block) */
	unsigned short	taac_clks;	/* clock-dependent read data access time */
					/* the total access time is the sum of taac_ns and taac_clks */
	unsigned int	r2w_factor;	/* write speed factor (read access time times 1<<r2w_factor) */

	int 		runtime_resume_error_count;
	int		cmd_timeout_count;
	int		cmd_crc_err_count;
	int		data_timeout_int_count;		/* data timeout reported by interruption */
	int		data_crc_err_count;
	int		data_timeout_count;	/* data timeout reported by soft-timer or delay work */
	int		vold_timeout_count;
};

extern void sdcard_remove_attr_init_sysfs(struct device *dev);
extern void reset_sdinfo(void);
extern int get_sdcard_remove(void);
extern int get_dma_data_timeout(void);
extern struct sd_info sdinfo;

