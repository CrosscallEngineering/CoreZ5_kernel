#ifndef MMC_HEALTH_DIAG_H
#define MMC_HEALTH_DIAG_H

#include <linux/genhd.h>

#define MAX_ERR_TIMES 10
#define MAX_ERR_TYPE 6
#define MAX_REPORT_TIMES 10

enum SD_ERR
{
   SD_STUCK_IN_PRG_ERR = 0x20,
   SD_SPEED_LOW,
   SD_RW_ERR,
   SD_RESERVE_ERR,
   SD_PLUG_OUT,
   SD_RO_ERR,
};

struct sd_err_info
{
   unsigned int err_num;
   char err_info[12];
};


void sd_err_status_report(struct gendisk *disk,int status);
void sd_status_reinit(void);
int sd_trans_status(int status);
void sd_ro_st(struct gendisk *disk,unsigned int read_only);
ssize_t
show_sd_speed(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t
show_sd_report_set(struct device *dev, struct device_attribute *attr, char *buf);
ssize_t
store_sd_report_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
#endif
