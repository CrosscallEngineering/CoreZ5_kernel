/*
 *  kernel/drivers/mmc/card/sd_err_check.c
 *
 */
#define pr_fmt(fmt) "mmc1:" fmt

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/kdev_t.h>
#include <linux/blkdev.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/compat.h>
#include <linux/syscalls.h>
#include <linux/proc_fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/mmc/core.h>
#include <trace/events/mmc.h>
#include <linux/mmc/ioctl.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include "sd_err_check.h"


/*===========================================================================*/
/*????class 6?Ŀ???Ϊ??????*/
#define SLOW_CARD_CLASS_N 6
/*???????ߵ?T??????*/
#define SLOW_CARD_CLASS_MAX 10


/*SD_STATUS=1:T????д?쳣*/
/*SD_STATUS=2:????*/
/*SD_STATUS=3:reserve*/
/*SD_STATUS=4:T??ֻ??*/
/*SD_STATUS=5:????ӳ??Ϊ???豸?Ļ????γ??¼?*/
struct sd_err_info sd_err_table[MAX_ERR_TYPE] = {
  {SD_STUCK_IN_PRG_ERR, "SD_STATUS=1"},
  {SD_SPEED_LOW, "SD_STATUS=2"},//??Ч???ϲ?ͨ???ڵ???ȡ
  {SD_RW_ERR, "SD_STATUS=1"},
  {SD_RESERVE_ERR, "SD_STATUS=3"},//??Ч
  {SD_PLUG_OUT, "SD_STATUS=5"},
  {SD_RO_ERR, "SD_STATUS=4"},
};
static int g_bad_sd_report_times = 0;
static int g_sd_write_times = 0;

struct gendisk *sd_card_disk;
extern struct gendisk *virtual_T_disk;
extern unsigned int sd_class;

/*===========================================================================*/


/*T??״̬??ʼ??*/
void sd_status_reinit(void)
{
   g_bad_sd_report_times = 0;
   g_sd_write_times = 0;
}


/*T??????״̬????*/
int sd_err_adap(int num)
{
   int i;
   int index = -1;
   for( i = 0; i < MAX_ERR_TYPE; i++ ) {
      if( sd_err_table[i].err_num == num ) {
        index = i;
	 break;
      }
   }
   return index;
}

/*T???쳣״̬?ϱ?*/
void sd_err_status_report(struct gendisk *disk, int status)
{
  int index;
  if(status >= SD_STUCK_IN_PRG_ERR && status <= SD_RO_ERR){
    if(g_bad_sd_report_times < MAX_REPORT_TIMES){
       index = sd_err_adap(status);
       if((index != -1)&& (index < MAX_ERR_TYPE)){
           set_sd_disk_health_status(disk,sd_err_table[index].err_info);
           g_bad_sd_report_times++;
       }
	else
    	    pr_buf_err("sd_report index = %d!\n",index);
    }
  }
}


/*T???????쳣״̬?ϱ?*/
int sd_trans_status(int status)
{
    static int err_num = 0;
    int ret = 0;
    /*if err is rw err,must be 10 times continous,so can report!*/
    if(status >= MMC_BLK_CMD_ERR && status <= MMC_BLK_ECC_ERR){
        err_num++;
        if(err_num >= MAX_ERR_TIMES){
            ret = SD_RW_ERR;
            pr_buf_err("%s,err = %d, num = %d, trans status to USER!!\n",__func__,status,err_num);
            err_num = 0;
        }
        else{
            pr_buf_err("%s,err = %d, num = %d!!\n",__func__,status,err_num);
        }
    }
    else{
        if(err_num != 0){
            pr_buf_err("%s, fail (num=%d)---> success, reset err status\n",__func__,err_num);
            err_num = 0;
        }
    }
    return ret;
}
/*T??ֻ??״̬?ϱ?*/
void sd_ro_st(struct gendisk *disk, unsigned int read_only)
{   
   
   if(g_sd_write_times == 0){
   	sd_card_disk = disk;
   }
   g_sd_write_times = 1;
   if(read_only){
      sd_err_status_report(disk, SD_RO_ERR);
   }
}

/*?ϲ????غ???ȡT??????*/
ssize_t
show_sd_speed(struct device *dev, struct device_attribute *attr, char *buf)
{
   unsigned int speed = 0;
   switch(sd_class){
   case 0x00:
        speed = 0;
        break;
   case 0x01:
        speed = 2;
        break;
   case 0x02:
        speed = 4;
        break;
   case 0x03:
        speed = 6;
        break;
   case 0x04:
        speed = 10;
        break;
   default:
        speed = SLOW_CARD_CLASS_N;
   }
   
   pr_buf_err("%s:sd speed class is %d\n", __func__, speed);
   if(speed < SLOW_CARD_CLASS_N) 
   	return snprintf(buf, PAGE_SIZE, "LOW speed sd!\n");
   else
   	return snprintf(buf, PAGE_SIZE, "Normal speed sd!\n");
}

/*???Խڵ?*/
ssize_t
show_sd_report_set(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "help: echo 1 : sd rw err, when sd card plugin;\n"
		        "echo 2 : low speed sd;\n"
		        "echo 3 : run ro check;\n"
		        "echo 4 : read-only sd;\n"
		        "echo 5 : sd card which init failed is plugout;\n"
		        "echo 0 : error reprot times;\n"
		        "echo other : do nothing.\n");
}
#define cls_dev_to_mmc_host(d)	container_of(d, struct mmc_host, class_dev)
ssize_t
store_sd_report_set(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	unsigned long value;

	if (!host || kstrtoul(buf, 0, &value) || !sd_card_disk)
		return -EINVAL;

	if(value == 1)
      		sd_err_status_report(sd_card_disk, SD_STUCK_IN_PRG_ERR);
	else if(value == 2)
      		sd_err_status_report(sd_card_disk, SD_SPEED_LOW);
	else if(value == 3)
      		sd_err_status_report(sd_card_disk, SD_RESERVE_ERR);
	else if(value == 4)
      		sd_err_status_report(sd_card_disk, SD_RO_ERR);
	else if(value == 5)
      		sd_err_status_report(virtual_T_disk, SD_PLUG_OUT);
	else if(value == 0)
      		pr_buf_err("g_bad_sd_report_times = %d,"
      		                   "g_sd_write_times = %d\n", g_bad_sd_report_times, g_sd_write_times);
	else
      		printk("Invalid parameter : do nothing\n");

	return count;
}
