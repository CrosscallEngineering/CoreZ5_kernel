#define LOG_TAG         "Test"

#include "cts_config.h"
#include "cts_platform.h"
#include "cts_core.h"
#include "cts_strerror.h"
#include "cts_test.h"
#include "cts_firmware.h"
#include <linux/productinfo.h>
#include "../ts_func_test.h"

extern int cts_driver_suspend(struct chipone_ts_data *cts_data);
extern bool cts_probed;

const char *cts_test_item_str(int test_item)
{
#define case_test_item(item) \
    case CTS_TEST_ ## item: return #item "-TEST"

    switch (test_item) {
        case_test_item(RESET_PIN);
        case_test_item(INT_PIN);
        case_test_item(RAWDATA);
        case_test_item(NOISE);
        case_test_item(OPEN);
        case_test_item(SHORT);
        case_test_item(COMPENSATE_CAP);
        case_test_item(GESTURE_RAWDATA);
        case_test_item(GESTURE_LP_RAWDATA);
        case_test_item(GESTURE_NOISE);
        case_test_item(GESTURE_LP_NOISE);
        default: return "INVALID";
    }
#undef case_test_item
}

#define CTS_FIRMWARE_WORK_MODE_NORMAL   (0x00)
#define CTS_FIRMWARE_WORK_MODE_FACTORY  (0x01)
#define CTS_FIRMWARE_WORK_MODE_CONFIG   (0x02)
#define CTS_FIRMWARE_WORK_MODE_TEST     (0x03)

#define CTS_TEST_SHORT                  (0x01)
#define CTS_TEST_OPEN                   (0x02)

#define CTS_SHORT_TEST_UNDEFINED        (0x00)
#define CTS_SHORT_TEST_BETWEEN_COLS     (0x01)
#define CTS_SHORT_TEST_BETWEEN_ROWS     (0x02)
#define CTS_SHORT_TEST_BETWEEN_GND      (0x03)

#define TEST_RESULT_BUFFER_SIZE(cts_dev) \
    (cts_dev->fwdata.rows * cts_dev->fwdata.cols * 2)

#define RAWDATA_BUFFER_SIZE(cts_dev) \
        (cts_dev->fwdata.rows * cts_dev->fwdata.cols * 2)

static int disable_fw_monitor_mode(struct cts_device *cts_dev)
{
    int ret;
    u8 value;

    ret = cts_fw_reg_readb(cts_dev, CTS_DEVICE_FW_REG_FLAG_BITS, &value);
    if (ret) {
        return ret;
    }

    if (value & BIT(0)) {
        return cts_fw_reg_writeb(cts_dev,
            CTS_DEVICE_FW_REG_FLAG_BITS, value & (~BIT(0)));
    }

    return 0;
}

static int disable_fw_auto_compensate(struct cts_device *cts_dev)
{
    return cts_fw_reg_writeb(cts_dev,
        CTS_DEVICE_FW_REG_AUTO_CALIB_COMP_CAP_ENABLE, 0);
}

static int set_fw_work_mode(struct cts_device *cts_dev, u8 mode)
{
    int ret, retries;
    u8  pwr_mode;

    cts_info("Set firmware work mode to %u", mode);

    ret = cts_fw_reg_writeb(cts_dev, CTS_DEVICE_FW_REG_WORK_MODE, mode);
    if (ret) {
        cts_err("Write firmware work mode register failed %d(%s)",
            ret, cts_strerror(ret));
        return ret;
    }

    ret = cts_fw_reg_readb(cts_dev, CTS_DEVICE_FW_REG_POWER_MODE,
        &pwr_mode);
    if (ret) {
        cts_err("Read firmware power mode register failed %d(%s)",
            ret, cts_strerror(ret));
        return ret;
    }

    if (pwr_mode == 1) {
        ret = cts_send_command(cts_dev, CTS_CMD_QUIT_GESTURE_MONITOR);
        if (ret) {
            cts_err("Send cmd QUIT_GESTURE_MONITOR failed %d(%s)",
                ret, cts_strerror(ret));
            return ret;
        }

        msleep(50);
    }

    retries = 0;
    do {
        u8 sys_busy, curr_mode;

        msleep(10);

        ret = cts_fw_reg_readb(cts_dev, CTS_DEVICE_FW_REG_SYS_BUSY,
            &sys_busy);
        if (ret) {
            cts_err("Read firmware system busy register failed %d(%s)",
                ret, cts_strerror(ret));
            //return ret;
            continue;
        }
        if (sys_busy)
            continue;

        ret = cts_fw_reg_readb(cts_dev, CTS_DEVICE_FW_REG_GET_WORK_MODE,
            &curr_mode);
        if (ret) {
            cts_err("Read firmware current work mode failed %d(%s)",
                ret, cts_strerror(ret));
            //return ret;
            continue;
        }

        if (curr_mode == mode /*|| curr_mode == 0xFF*/) {
            break;
        }
    } while (retries++ < 1000);

    return (retries >= 1000 ? -ETIMEDOUT : 0);
}


static int wait_test_complete(struct cts_device *cts_dev, int skip_frames)
{
    int ret, i, j;

    cts_info("Wait test complete skip %d frames", skip_frames);

    for (i = 0; i < (skip_frames + 1); i++) {
        u8 ready;

        for (j = 0; j < 1000; j++) {
            mdelay(1);

            ready = 0;
            ret = cts_get_data_ready_flag(cts_dev, &ready);
            if (ret) {
                cts_err("Get data ready flag failed %d(%s)",
                    ret, cts_strerror(ret));
                return ret;
            }

            if (ready) {
                break;
            }
        }

        if (ready == 0) {
            cts_err("Wait test complete timeout");
            return -ETIMEDOUT;
        }
        if (i < skip_frames) {
            ret = cts_clr_data_ready_flag(cts_dev);
            if (ret) {
                cts_err("Clr data ready flag failed %d(%s)",
                    ret, cts_strerror(ret));
                return ret;
            }
        }
    }

    return 0;
}

static int get_test_result(struct cts_device *cts_dev, u16 *result)
{
    int ret;

    ret = cts_fw_reg_readsb(cts_dev, CTS_DEVICE_FW_REG_RAW_DATA, result,
            TEST_RESULT_BUFFER_SIZE(cts_dev));
    if (ret) {
        cts_err("Get test result data failed %d(%s)",
            ret, cts_strerror(ret));
        return ret;
    }

    ret = cts_clr_data_ready_flag(cts_dev);
    if (ret) {
        cts_err("Clear data ready flag failed %d(%s)",
            ret, cts_strerror(ret));
        return ret;
    }

    return 0;
}

static int set_fw_test_type(struct cts_device *cts_dev, u8 type)
{
    int ret, retries = 0;
    u8  sys_busy;
    u8  type_readback;

    cts_info("Set test type %d", type);

    ret = cts_fw_reg_writeb(cts_dev, 0x34, type);
    if (ret) {
        cts_err("Write test type register to failed %d(%s)",
            ret, cts_strerror(ret));
        return ret;
    }

    do {
        msleep(1);

        ret = cts_fw_reg_readb(cts_dev, 0x01, &sys_busy);
        if (ret) {
            cts_err("Read system busy register failed %d(%s)",
                ret, cts_strerror(ret));
            return ret;
        }
    } while (sys_busy && retries++ < 1000);

    if (retries >= 1000) {
        cts_err("Wait system ready timeout");
        return -ETIMEDOUT;
    }

    ret = cts_fw_reg_readb(cts_dev, 0x34, &type_readback);
    if (ret) {
        cts_err("Read test type register failed %d(%s)",
            ret, cts_strerror(ret));
        return ret;
    }

    if (type != type_readback) {
        cts_err("Set test type %u != readback %u", type, type_readback);
        return -EFAULT;
    }

    return 0;
}

static bool set_short_test_type(struct cts_device *cts_dev, u8 type)
{
    static struct fw_short_test_param {
        u8  type;
        u32 col_pattern[2];
        u32 row_pattern[2];
    } param = {
        .type = CTS_SHORT_TEST_BETWEEN_COLS,
        .col_pattern = {0, 0},
        .row_pattern = {0, 0}
    };
    int i, ret;

    cts_info("Set short test type to %u", type);

    param.type = type;
    for (i = 0; i < 5; i++) {
        u8 type_readback;

        ret = cts_fw_reg_writesb(cts_dev, 0x5000, &param, sizeof(param));
        if (ret) {
            cts_err("Set short test type to %u failed %d(%s)",
                type, ret, cts_strerror(ret));
            continue;
        }
        ret = cts_fw_reg_readb(cts_dev, 0x5000, &type_readback);
        if (ret) {
            cts_err("Get short test type failed %d(%s)",
                ret, cts_strerror(ret));
            continue;
        }
        if (type == type_readback) {
            return 0;
        } else {
            cts_err("Set test type %u != readback %u",
                type, type_readback);
            continue;
        }
    }

    return ret;
}

int cts_write_file(struct file *filp, const void *data, size_t size)
{
    loff_t  pos;
    ssize_t ret = -1;

    pos = filp->f_pos;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,0)
	#if defined(CONFIG_TOUCHSCREEN_CHIPONE_IN_QGKI)
    ret = kernel_write(filp, data, size, &pos);
	#endif
#else
	#if defined(CONFIG_TOUCHSCREEN_CHIPONE_IN_QGKI)
    ret = kernel_write(filp, data, size, pos);
	#endif
#endif

    if (ret >= 0) {
        filp->f_pos += ret;
    }

    return ret;
}

/* Make directory for filepath
 * If filepath = "/A/B/C/D.file", it will make dir /A/B/C recursive
 * like userspace mkdir -p
 */
int cts_mkdir_for_file(const char *filepath, umode_t mode)
{
    char *dirname = NULL;
    int   dirname_len;
    char *s;
    int   ret = 0;
    mm_segment_t fs;

    if (filepath == NULL) {
        cts_err("Create dir for file with filepath = NULL");
        return -EINVAL;
    }

    if (filepath[0] == '\0' || filepath[0] != '/') {
        cts_err("Create dir for file with invalid filepath[0]: %c",
            filepath[0]);
        return -EINVAL;
    }

    dirname_len = strrchr(filepath, '/') - filepath;
    if (dirname_len == 0) {
        cts_warn("Create dir for file '%s' in root dir", filepath);
        return 0;
    }

    dirname = kstrndup(filepath, dirname_len, GFP_KERNEL);
    if (dirname == NULL) {
        cts_err("Create dir alloc mem for dirname failed");
        return -ENOMEM;
    }

    cts_info("Create dir '%s' for file '%s'", dirname, filepath);

    fs = get_fs();
    set_fs(KERNEL_DS);

    s = dirname + 1;   /* Skip leading '/' */
    while (1) {
        char c = '\0';

        /* Bypass leading non-'/'s and then subsequent '/'s */
        while (*s) {
            if (*s == '/') {
                do {
                    ++s;
                } while (*s == '/');
                c = *s;     /* Save current char */
                *s = '\0';  /* and replace it with nul */
                break;
            }
            ++s;
        }

        cts_dbg(" - Create dir '%s'", dirname);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,17,0)
		#if defined(CONFIG_TOUCHSCREEN_CHIPONE_IN_QGKI)
        ret = ksys_mkdir(dirname, 0777);
		#endif
#else
		#if defined(CONFIG_TOUCHSCREEN_CHIPONE_IN_QGKI)
        ret = sys_mkdir(dirname, 0777);
		#endif
#endif
        if (ret < 0 && ret != -EEXIST) {
            cts_info("Create dir '%s' failed %d(%s)",
                dirname, ret, cts_strerror(ret));
            /* Remove any inserted nul from the path */
            *s = c;
            break;
        }
        /* Reset ret to 0 if return -EEXIST */
        ret = 0;

        if (c) {
            /* Remove any inserted nul from the path */
            *s = c;
        } else {
            break;
        }
    }

    set_fs(fs);

    if (dirname) {
        kfree(dirname);
    }

    return ret;
}

struct file *cts_test_data_filp = NULL;
int cts_start_dump_test_data_to_file(const char *filepath, bool append_to_file)
{
    int ret;

#define START_BANNER \
        ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n"

    cts_info("Start dump test data to file '%s'", filepath);

    ret = cts_mkdir_for_file(filepath, 0777);
    if (ret) {
        cts_err("Create dir for test data file failed %d(%s)",
            ret, cts_strerror(ret));
        return ret;
    }
	#if defined(CONFIG_TOUCHSCREEN_CHIPONE_IN_QGKI)
    cts_test_data_filp = filp_open(filepath,
        O_WRONLY | O_CREAT | (append_to_file ? O_APPEND : O_TRUNC),
        S_IRUGO | S_IWUGO);
	#endif
    if (IS_ERR(cts_test_data_filp)) {
        ret = PTR_ERR(cts_test_data_filp);
        cts_test_data_filp = NULL;
        cts_err("Open file '%s' for test data failed %d(%s)",
            cts_test_data_filp, ret, cts_strerror(ret));
        return ret;
    }

    cts_write_file(cts_test_data_filp, START_BANNER, strlen(START_BANNER));

    return 0;
#undef START_BANNER
}

void cts_stop_dump_test_data_to_file(void)
{
#define END_BANNER \
    "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n"
    int r = 0;

    cts_info("Stop dump test data to file");

    if (cts_test_data_filp) {
        cts_write_file(cts_test_data_filp,
            END_BANNER, strlen(END_BANNER));
		#if defined(CONFIG_TOUCHSCREEN_CHIPONE_IN_QGKI)
        r = filp_close(cts_test_data_filp, NULL);
		#endif
        if (r) {
            cts_err("Close test data file failed %d(%s)",
                r, cts_strerror(r));
        }
        cts_test_data_filp = NULL;
    } else {
        cts_warn("Stop dump tsdata to file with filp = NULL");
    }
#undef END_BANNER
}

static void cts_dump_tsdata(struct cts_device *cts_dev,
        const char *desc, const u16 *data, bool to_console)
{
#define SPLIT_LINE_STR \
    "---------------------------------------------------------------------------------------------------------------"
#define ROW_NUM_FORMAT_STR  "%2d | "
#define COL_NUM_FORMAT_STR  "%-5u "
#define DATA_FORMAT_STR     "%-5u "

    int r, c;
    u32 max, min, sum, average;
    int max_r, max_c, min_r, min_c;
    char line_buf[128];
    int count = 0;

    max = min = data[0];
    sum = 0;
    max_r = max_c = min_r = min_c = 0;
    for (r = 0; r < cts_dev->fwdata.rows; r++) {
        for (c = 0; c < cts_dev->fwdata.cols; c++) {
            u16 val = data[r * cts_dev->fwdata.cols + c];

            sum += val;
            if (val > max) {
                max = val;
                max_r = r;
                max_c = c;
            } else if (val < min) {
                min = val;
                min_r = r;
                min_c = c;
            }
        }
    }
    average = sum / (cts_dev->fwdata.rows * cts_dev->fwdata.cols);

    count = 0;
    count += scnprintf(line_buf + count, sizeof(line_buf) - count,
        " %s test data MIN: [%u][%u]=%u, MAX: [%u][%u]=%u, AVG=%u",
        desc, min_r, min_c, min, max_r, max_c, max, average);
    if (to_console) {
        cts_info(SPLIT_LINE_STR);
        cts_info("%s", line_buf);
        cts_info(SPLIT_LINE_STR);
    }
    if (cts_test_data_filp) {
        cts_write_file(cts_test_data_filp, SPLIT_LINE_STR, strlen(SPLIT_LINE_STR));
        cts_write_file(cts_test_data_filp, "\n", 1);
        cts_write_file(cts_test_data_filp, line_buf, count);
        cts_write_file(cts_test_data_filp, "\n", 1);
        cts_write_file(cts_test_data_filp, SPLIT_LINE_STR, strlen(SPLIT_LINE_STR));
        cts_write_file(cts_test_data_filp, "\n", 1);
    }

    count = 0;
    count += scnprintf(line_buf + count, sizeof(line_buf) - count, "   |  ");
    for (c = 0; c < cts_dev->fwdata.cols; c++) {
        count += scnprintf(line_buf + count, sizeof(line_buf) - count,
            COL_NUM_FORMAT_STR, c);
    }
    if (to_console) {
        cts_info("%s", line_buf);
        cts_info(SPLIT_LINE_STR);
    }
    if (cts_test_data_filp) {
        cts_write_file(cts_test_data_filp, line_buf, count);
        cts_write_file(cts_test_data_filp, "\n", 1);
        cts_write_file(cts_test_data_filp, SPLIT_LINE_STR, strlen(SPLIT_LINE_STR));
        cts_write_file(cts_test_data_filp, "\n", 1);
    }

    for (r = 0; r < cts_dev->fwdata.rows; r++) {
        count = 0;
        count += scnprintf(line_buf + count, sizeof(line_buf) - count,
            ROW_NUM_FORMAT_STR, r);
        for (c = 0; c < cts_dev->fwdata.cols; c++) {
            count +=
                scnprintf(line_buf + count, sizeof(line_buf) - count,
                    DATA_FORMAT_STR,
                    data[r * cts_dev->fwdata.cols + c]);
        }
        if (to_console) {
            cts_info("%s", line_buf);
        }
        if (cts_test_data_filp) {
            cts_write_file(cts_test_data_filp, line_buf, count);
            cts_write_file(cts_test_data_filp, "\n", 1);
        }
    }
    if (to_console) {
        cts_info(SPLIT_LINE_STR);
    }
    if (cts_test_data_filp) {
        cts_write_file(cts_test_data_filp, SPLIT_LINE_STR, strlen(SPLIT_LINE_STR));
        cts_write_file(cts_test_data_filp, "\n", 1);
    }

#undef SPLIT_LINE_STR
#undef ROW_NUM_FORMAT_STR
#undef COL_NUM_FORMAT_STR
#undef DATA_FORMAT_STR
}

static bool is_invalid_node(u32 *invalid_nodes, u32 num_invalid_nodes,
    u16 row, u16 col)
{
    int i;

    for (i = 0; i < num_invalid_nodes; i++) {
        if (MAKE_INVALID_NODE(row,col)== invalid_nodes[i]) {
            return true;
        }
    }

    return false;
}

static int validate_tsdata(struct cts_device *cts_dev,
    const char *desc, u16 *data,
    u32 *invalid_nodes, u32 num_invalid_nodes,
    bool per_node, int *min, int *max)
{
#define SPLIT_LINE_STR \
    "------------------------------"

    int r, c;
    int failed_cnt = 0;

    cts_info("%s validate data: %s, num invalid node: %u, thresh[0]=[%d, %d]",
        desc, per_node ? "Per-Node" : "Uniform-Threshold",
        num_invalid_nodes, min ? min[0] : INT_MIN, max ? max[0] : INT_MAX);

    for (r = 0; r < cts_dev->fwdata.rows; r++) {
        for (c = 0; c < cts_dev->fwdata.cols; c++) {
            int offset = r * cts_dev->fwdata.cols + c;

            if (num_invalid_nodes &&
                is_invalid_node(invalid_nodes, num_invalid_nodes, r,c)) {
                continue;
            }

            if ((min != NULL && data[offset] < min[per_node ? offset : 0]) ||
                (max != NULL && data[offset] > max[per_node ? offset : 0])) {
                if (failed_cnt == 0) {
                    cts_info(SPLIT_LINE_STR);
                    cts_info("%s failed nodes:", desc);
                }
                failed_cnt++;

                cts_info("  %3d: [%-2d][%-2d] = %u",
                    failed_cnt, r, c, data[offset]);
            }
        }
    }

    if (failed_cnt) {
        cts_info(SPLIT_LINE_STR);
        cts_info("%s test %d node total failed", desc, failed_cnt);
    }

    return failed_cnt;

#undef SPLIT_LINE_STR
}

static int validate_comp_cap(struct cts_device *cts_dev,
    const char *desc, u8 *cap,
    u32 *invalid_nodes, u32 num_invalid_nodes,
    bool per_node, int *min, int *max)
{
#define SPLIT_LINE_STR \
    "------------------------------"

    int r, c;
    int failed_cnt = 0;

    cts_info("Validate %s data: %s, num invalid node: %u, thresh[0]=[%d, %d]",
        desc, per_node ? "Per-Node" : "Uniform-Threshold",
        num_invalid_nodes, min ? min[0] : INT_MIN, max ? max[0] : INT_MAX);

    for (r = 0; r < cts_dev->fwdata.rows; r++) {
        for (c = 0; c < cts_dev->fwdata.cols; c++) {
            int offset = r * cts_dev->fwdata.cols + c;

            if (num_invalid_nodes &&
                is_invalid_node(invalid_nodes, num_invalid_nodes, r,c)) {
                continue;
            }

            if ((min != NULL && cap[offset] < min[per_node ? offset : 0]) ||
                (max != NULL && cap[offset] > max[per_node ? offset : 0])) {
                if (failed_cnt == 0) {
                    cts_info(SPLIT_LINE_STR);
                    cts_info("%s failed nodes:", desc);
                }
                failed_cnt++;

                cts_info("  %3d: [%-2d][%-2d] = %u",
                    failed_cnt, r, c, cap[offset]);
            }
        }
    }

    if (failed_cnt) {
        cts_info(SPLIT_LINE_STR);
        cts_info("%s test %d node total failed", desc, failed_cnt);
    }

    return failed_cnt;

#undef SPLIT_LINE_STR
}

static int wait_fw_to_normal_work(struct cts_device *cts_dev)
{
    int i = 0;
    int ret;

    cts_info ("Wait fw to normal work");

    do {
        u8 work_mode;

        ret = cts_fw_reg_readb(cts_dev,
            CTS_DEVICE_FW_REG_GET_WORK_MODE, &work_mode);
        if (ret) {
            cts_err("Get fw curr work mode failed %d(%s)",
                work_mode, ret, cts_strerror(ret));
            continue;
        } else {
            if (work_mode == CTS_FIRMWARE_WORK_MODE_NORMAL) {
                return 0;
            }
        }

        mdelay (10);
    } while (++i < 100);

    return -ETIMEDOUT;
}

static int prepare_test(struct cts_device *cts_dev)
{
    int ret;

    cts_info("Prepare test");

    cts_plat_reset_device(cts_dev->pdata);

    ret = cts_set_dev_esd_protection(cts_dev, false);
    if (ret) {
        cts_err("Disable firmware ESD protection failed %d(%s)",
            ret, cts_strerror(ret));
        return ret;
    }

    ret = disable_fw_monitor_mode(cts_dev);
    if (ret) {
        cts_err("Disable firmware monitor mode failed %d(%s)",
            ret, cts_strerror(ret));
        return ret;
    }

    ret = disable_fw_auto_compensate(cts_dev);
    if (ret) {
        cts_err("Disable firmware auto compensate failed %d(%s)",
            ret, cts_strerror(ret));
        return ret;
    }

    ret = set_fw_work_mode(cts_dev, CTS_FIRMWARE_WORK_MODE_CONFIG);
    if (ret) {
        cts_err("Set firmware work mode to WORK_MODE_CONFIG failed %d(%s)",
            ret, cts_strerror(ret));
        return ret;
    }

    cts_dev->rtdata.testing = true;

    return 0;
}

static void post_test(struct cts_device *cts_dev)
{
    int ret;

    cts_info("Post test");

    cts_plat_reset_device(cts_dev->pdata);

    ret = set_fw_work_mode(cts_dev, CTS_FIRMWARE_WORK_MODE_NORMAL);
    if (ret) {
        cts_err("Set firmware work mode to WORK_MODE_NORMAL failed %d(%s)",
            ret, cts_strerror(ret));
    }

    ret = wait_fw_to_normal_work(cts_dev);
    if (ret) {
        cts_err("Wait fw to normal work failed %d(%s)",
            ret, cts_strerror(ret));
        //return ret;
    }

    cts_dev->rtdata.testing = false;
}

void cts_print_test_result_to_console(
    const char *test_case, int result, s64 elapsed_time_ms)
{
    if (result > 0) {
        cts_info("%-10s test: %d nodes FAIL, ELAPSED TIME: %lldms",
            test_case, result, elapsed_time_ms);
    } else if (result < 0) {
        cts_info("%-10s test: FAIL %d(%s), ELAPSED TIME: %lldms",
            test_case, result, cts_strerror(result), elapsed_time_ms);
    } else {
        cts_info("%-10s test: PASS, ELAPSED TIME: %lldms",
            test_case, elapsed_time_ms);
    }
}

#ifdef CONFIG_CTS_SYSFS
int cts_print_test_result_to_buffer(char *buf, size_t size,
    const char *test_case, int result, s64 elapsed_time_ms)
{
    if (result > 0) {
        return scnprintf(buf, size,
            "%-10s test: %d nodes FAIL, ELAPSED TIME: %lldms\n",
            test_case, result, elapsed_time_ms);
    } else if (result < 0) {
        return scnprintf(buf, size,
            "%-10s test: FAIL %d(%s), ELAPSED TIME: %lldms\n",
            test_case, result, cts_strerror(result), elapsed_time_ms);
    } else {
        return scnprintf(buf, size,
            "%-10s test: PASS, ELAPSED TIME: %lldms\n",
            test_case, elapsed_time_ms);
    }
}
#endif /* CONFIG_CTS_SYSFS */

/* Return 0 success
    negative value while error occurs
    positive value means how many nodes fail */
int cts_test_short(struct cts_device *cts_dev,
    struct cts_test_param *param)
{
    bool driver_validate_data = false;
    bool validate_data_per_node = false;
    bool stop_if_failed = false;
    bool dump_test_data_to_user = false;
    bool dump_test_data_to_console = false;
    bool dump_test_data_to_file = false;
    int  num_nodes;
    int  tsdata_frame_size;
    int  loopcnt;
    int  ret;
    u16 *test_result = NULL;
    u8   feature_ver;
    ktime_t start_time;
    s64 elapsed_time_ms;

    if (cts_dev == NULL || param == NULL) {
        cts_err("Short test with invalid param: cts_dev: %p test param: %p",
            cts_dev, param);
        return -EINVAL;
    }

    num_nodes = cts_dev->fwdata.rows * cts_dev->fwdata.cols;
    tsdata_frame_size = 2 * num_nodes;

    driver_validate_data =
        !!(param->flags & CTS_TEST_FLAG_VALIDATE_DATA);
    validate_data_per_node =
        !!(param->flags & CTS_TEST_FLAG_VALIDATE_PER_NODE);
    dump_test_data_to_user =
        !!(param->flags & CTS_TEST_FLAG_DUMP_TEST_DATA_TO_USERSPACE);
    dump_test_data_to_console =
        !!(param->flags & CTS_TEST_FLAG_DUMP_TEST_DATA_TO_CONSOLE);
    dump_test_data_to_file =
        !!(param->flags & CTS_TEST_FLAG_DUMP_TEST_DATA_TO_FILE);
    stop_if_failed =
        !!(param->flags & CTS_TEST_FLAG_STOP_TEST_IF_VALIDATE_FAILED);

    cts_info("Short test, flags: 0x%08x,"
               "num invalid node: %u, "
               "test data file: '%s' buf size: %d, "
               "drive log file: '%s' buf size: %d",
        param->flags, param->num_invalid_node,
        param->test_data_filepath, param->test_data_buf_size,
        param->driver_log_filepath, param->driver_log_buf_size);

    start_time = ktime_get();

    if (dump_test_data_to_user) {
        test_result = (u16 *)param->test_data_buf;
    } else {
        test_result = (u16 *)kmalloc(tsdata_frame_size, GFP_KERNEL);
        if (test_result == NULL) {
            cts_err("Allocate test result buffer failed");
            ret = -ENOMEM;
            goto show_test_result;
        }
    }

    ret = cts_stop_device(cts_dev);
    if (ret) {
        cts_err("Stop device failed %d(%s)",
            ret, cts_strerror(ret));
        goto free_mem;
    }

    cts_lock_device(cts_dev);

    ret = prepare_test(cts_dev);
    if (ret) {
        cts_err("Prepare test failed %d(%s)",
            ret, cts_strerror(ret));
        goto unlock_device;
    }

    cts_info("Test short to GND");

    ret = cts_sram_readb(cts_dev, 0xE8, &feature_ver);
    if (ret) {
        cts_err("Read firmware feature version failed %d(%s)",
            ret, cts_strerror(ret));
        goto post_test;
    }
    cts_info("Feature version: %u", feature_ver);

    if (feature_ver > 0) {
        ret = set_short_test_type(cts_dev, CTS_SHORT_TEST_UNDEFINED);
        if (ret) {
            cts_err("Set short test type to UNDEFINED failed %d(%s)",
            ret, cts_strerror(ret));
            goto post_test;
        }

        ret = set_fw_test_type(cts_dev, CTS_TEST_SHORT);
        if (ret) {
            cts_err("Set test type to SHORT failed %d(%s)",
            ret, cts_strerror(ret));
            goto post_test;
        }

        ret = set_fw_work_mode(cts_dev, CTS_FIRMWARE_WORK_MODE_TEST);
        if (ret) {
            cts_err("Set firmware work mode to WORK_MODE_TEST failed %d(%s)",
            ret, cts_strerror(ret));
            goto post_test;
        }

        if (feature_ver <= 3) {
            u8 val;

            cts_info("Patch short test issue");

            ret = cts_hw_reg_readb(cts_dev, 0x350E2, &val);
            if (ret) {
                cts_err("Read 0x350E2 failed %d(%s)",
                    ret, cts_strerror(ret));
                goto post_test;
            }
            if ((val & (BIT(2) | BIT(5))) != 0) {
                ret = cts_hw_reg_writeb(cts_dev, 0x350E2, val & 0xDB);
                if (ret) {
                    cts_err("Write 0x350E2 failed %d(%s)",
                        ret, cts_strerror(ret));
                    goto post_test;
                }
            }
        }

        ret = set_short_test_type(cts_dev, CTS_SHORT_TEST_BETWEEN_GND);
        if (ret) {
            cts_err("Set short test type to SHORT_TO_GND failed %d(%s)",
                ret, cts_strerror(ret));
            goto post_test;
        }

        ret = wait_test_complete(cts_dev, 0);
        if (ret) {
            cts_err("Wait test complete failed %d(%s)",
                ret, cts_strerror(ret));
            goto post_test;
        }
    } else {
        ret = cts_send_command(cts_dev, CTS_CMD_RECOVERY_TX_VOL);
        if (ret) {
            cts_err("Send command RECOVERY_TX_VOL failed %d(%s)",
                ret, cts_strerror(ret));
            goto post_test;
        }

        ret = wait_test_complete(cts_dev, 2);
        if (ret) {
            cts_err("Wait test complete failed %d(%s)",
                ret, cts_strerror(ret));
            goto post_test;
        }

        // TODO: In factory mode
    }

    ret = get_test_result(cts_dev, test_result);
    if (ret) {
        cts_err("Read test result failed %d(%s)",
            ret, cts_strerror(ret));
        goto post_test;
    }

    if (dump_test_data_to_user) {
        *param->test_data_wr_size += tsdata_frame_size;
    }

    if (dump_test_data_to_file) {
        int r = cts_start_dump_test_data_to_file(param->test_data_filepath,
            !!(param->flags & CTS_TEST_FLAG_DUMP_TEST_DATA_TO_FILE_APPEND));
        if (r) {
            cts_err("Start dump test data to file failed %d(%s)",
                r, cts_strerror(r));
        }
    }

    if (dump_test_data_to_console || dump_test_data_to_file) {
        cts_dump_tsdata(cts_dev, "GND-short", test_result,
            dump_test_data_to_console);
    }

    if (driver_validate_data) {
        ret = validate_tsdata(cts_dev, "GND-short",
            test_result, param->invalid_nodes, param->num_invalid_node,
            validate_data_per_node, param->min, param->max);
        if (ret) {
            cts_err("Short to GND test failed %d(%s)",
                ret, cts_strerror(ret));
            if (stop_if_failed) {
                goto stop_dump_test_data_to_file;
            }
        }
    }

    if (dump_test_data_to_user) {
        test_result += num_nodes;
    }


    /*
     * Short between colums
     */
    cts_info("Test short between columns");

    ret = set_short_test_type(cts_dev, CTS_SHORT_TEST_BETWEEN_COLS);
    if (ret) {
        cts_err("Set short test type to BETWEEN_COLS failed %d(%s)",
            ret, cts_strerror(ret));
        goto stop_dump_test_data_to_file;
    }

    ret = wait_test_complete(cts_dev, 0);
    if (ret) {
        cts_err("Wait test complete failed %d(%s)",
            ret, cts_strerror(ret));
        goto stop_dump_test_data_to_file;
    }

    ret = get_test_result(cts_dev, test_result);
    if (ret) {
        cts_err("Read test result failed %d(%s)",
            ret, cts_strerror(ret));
        goto stop_dump_test_data_to_file;
    }

    if (dump_test_data_to_user) {
        *param->test_data_wr_size += tsdata_frame_size;
    }

    if (dump_test_data_to_console || dump_test_data_to_file) {
        cts_dump_tsdata(cts_dev, "Col-short", test_result,
            dump_test_data_to_console);
    }

    if (driver_validate_data) {
        ret = validate_tsdata(cts_dev, "Col-short",
            test_result, param->invalid_nodes, param->num_invalid_node,
            validate_data_per_node, param->min, param->max);
        if (ret) {
            cts_err("Short between columns test failed %d(%s)",
                ret, cts_strerror(ret));
            if (stop_if_failed) {
                goto stop_dump_test_data_to_file;
            }
        }
    }

    if (dump_test_data_to_user) {
        test_result += num_nodes;
    }

    /*
     * Short between colums
     */
    cts_info("Test short between rows");

    ret = set_short_test_type(cts_dev, CTS_SHORT_TEST_BETWEEN_ROWS);
    if (ret) {
        cts_err("Set short test type to BETWEEN_ROWS failed %d(%s)",
            ret, cts_strerror(ret));
        goto stop_dump_test_data_to_file;
    }

    loopcnt = cts_dev->hwdata->num_row;
    while (loopcnt > 1) {
        ret = wait_test_complete(cts_dev, 0);
        if (ret) {
            cts_err("Wait test complete failed %d(%s)",
                ret, cts_strerror(ret));
            goto stop_dump_test_data_to_file;
        }

        ret = get_test_result(cts_dev, test_result);
        if (ret) {
            cts_err("Read test result failed %d(%s)",
                ret, cts_strerror(ret));
            goto stop_dump_test_data_to_file;
        }

        if (dump_test_data_to_user) {
            *param->test_data_wr_size += tsdata_frame_size;
        }

        if (dump_test_data_to_console || dump_test_data_to_file) {
            cts_dump_tsdata(cts_dev, "Row-short", test_result,
                dump_test_data_to_console);
        }

        if (driver_validate_data) {
            ret = validate_tsdata(cts_dev, "Row-short",
                test_result, param->invalid_nodes, param->num_invalid_node,
                validate_data_per_node, param->min, param->max);
            if (ret) {
                cts_err("Short between columns test failed %d(%s)",
                    ret, cts_strerror(ret));
                if (stop_if_failed) {
                    goto stop_dump_test_data_to_file;
                }
            }
        }

        if (dump_test_data_to_user) {
            test_result += num_nodes;
        }

        loopcnt += loopcnt % 2;
        loopcnt = loopcnt >> 1;
    }

stop_dump_test_data_to_file:
    if (dump_test_data_to_file) {
        cts_stop_dump_test_data_to_file();
    }

post_test:
    post_test(cts_dev);

unlock_device:
    cts_unlock_device(cts_dev);

    cts_start_device(cts_dev);

free_mem:
    if (!dump_test_data_to_user && test_result) {
        kfree(test_result);
    }

show_test_result:
    elapsed_time_ms = ktime_ms_delta(ktime_get(), start_time);
    if (param->elapsed_time_ms) {
        *param->elapsed_time_ms = elapsed_time_ms;
    }
    if (param->test_result) {
        *param->test_result = ret;
    }
    cts_print_test_result_to_console("Short", ret, elapsed_time_ms);

    return ret;
}

/* Return 0 success
    negative value while error occurs
    positive value means how many nodes fail */
int cts_test_open(struct cts_device *cts_dev,
    struct cts_test_param *param)
{
    bool driver_validate_data = false;
    bool validate_data_per_node = false;
    bool dump_test_data_to_user = false;
    bool dump_test_data_to_console = false;
    bool dump_test_data_to_file = false;
    int  num_nodes;
    int  tsdata_frame_size;
    int  ret;
    u16 *test_result = NULL;
    ktime_t start_time;
    s64 elapsed_time_ms;

    if (cts_dev == NULL || param == NULL) {
        cts_err("Open test with invalid param: cts_dev: %p test param: %p",
            cts_dev, param);
        return -EINVAL;
    }

    num_nodes = cts_dev->fwdata.rows * cts_dev->fwdata.cols;
    tsdata_frame_size = 2 * num_nodes;

    driver_validate_data =
        !!(param->flags & CTS_TEST_FLAG_VALIDATE_DATA);
    validate_data_per_node =
        !!(param->flags & CTS_TEST_FLAG_VALIDATE_PER_NODE);
    dump_test_data_to_user =
        !!(param->flags & CTS_TEST_FLAG_DUMP_TEST_DATA_TO_USERSPACE);
    dump_test_data_to_console =
        !!(param->flags & CTS_TEST_FLAG_DUMP_TEST_DATA_TO_CONSOLE);
    dump_test_data_to_file =
        !!(param->flags & CTS_TEST_FLAG_DUMP_TEST_DATA_TO_FILE);

    cts_info("Open test, flags: 0x%08x,"
               "num invalid node: %u, "
               "test data file: '%s' buf size: %d, "
               "drive log file: '%s' buf size: %d",
        param->flags, param->num_invalid_node,
        param->test_data_filepath, param->test_data_buf_size,
        param->driver_log_filepath, param->driver_log_buf_size);

    start_time = ktime_get();

    if (dump_test_data_to_user) {
        test_result = (u16 *)param->test_data_buf;
    } else {
        test_result = (u16 *) kmalloc(tsdata_frame_size, GFP_KERNEL);
        if (test_result == NULL) {
            cts_err("Allocate memory for test result faild");
            ret = -ENOMEM;
            goto show_test_result;
        }
    }

    ret = cts_stop_device(cts_dev);
    if (ret) {
        cts_err("Stop device failed %d(%s)",
            ret, cts_strerror(ret));
        goto free_mem;
    }

    cts_lock_device(cts_dev);

    ret = prepare_test(cts_dev);
    if (ret) {
        cts_err("Prepare test failed %d(%s)",
            ret, cts_strerror(ret));
        goto unlock_device;
    }

    ret = cts_send_command(cts_dev, CTS_CMD_RECOVERY_TX_VOL);
    if (ret) {
        cts_err("Recovery tx voltage failed %d(%s)",
            ret, cts_strerror(ret));
        goto post_test;
    }

    ret = set_fw_test_type(cts_dev, CTS_TEST_OPEN);
    if (ret) {
        cts_err("Set test type to OPEN_TEST failed %d(%s)",
            ret, cts_strerror(ret));
        goto post_test;
    }

    ret = set_fw_work_mode(cts_dev, CTS_FIRMWARE_WORK_MODE_TEST);
    if (ret) {
        cts_err("Set firmware work mode to WORK_MODE_TEST failed %d(%s)",
            ret, cts_strerror(ret));
        goto post_test;
    }

    ret = wait_test_complete(cts_dev, 2);
    if (ret) {
        cts_err("Wait test complete failed %d(%s)",
            ret, cts_strerror(ret));
        goto post_test;
    }

    ret = get_test_result(cts_dev, test_result);
    if (ret) {
        cts_err("Read test result failed %d(%s)",
            ret, cts_strerror(ret));
        goto post_test;
    }

    if (dump_test_data_to_user) {
        *param->test_data_wr_size += tsdata_frame_size;
    }

    if (dump_test_data_to_file) {
        int r = cts_start_dump_test_data_to_file(param->test_data_filepath,
            !!(param->flags & CTS_TEST_FLAG_DUMP_TEST_DATA_TO_FILE_APPEND));
        if (r) {
            cts_err("Start dump test data to file failed %d(%s)",
                r, cts_strerror(r));
        }
    }

    if (dump_test_data_to_console || dump_test_data_to_file) {
        cts_dump_tsdata(cts_dev, "Open-circuit", test_result,
            dump_test_data_to_console);
    }

    if (dump_test_data_to_file) {
        cts_stop_dump_test_data_to_file();
    }

    if (driver_validate_data) {
        ret = validate_tsdata(cts_dev, "Open-circuit",
            test_result, param->invalid_nodes, param->num_invalid_node,
            validate_data_per_node, param->min, param->max);
    }

post_test:
    post_test(cts_dev);

unlock_device:
    cts_unlock_device(cts_dev);

    cts_start_device(cts_dev);

free_mem:
    if (!dump_test_data_to_user && test_result) {
        kfree(test_result);
    }

show_test_result:
    elapsed_time_ms = ktime_ms_delta(ktime_get(), start_time);
    if (param->elapsed_time_ms) {
        *param->elapsed_time_ms = elapsed_time_ms;
    }
    if (param->test_result) {
        *param->test_result = ret;
    }
    cts_print_test_result_to_console("Open", ret, elapsed_time_ms);

    return ret;
}

#ifdef CFG_CTS_HAS_RESET_PIN
int cts_test_reset_pin(struct cts_device *cts_dev, struct cts_test_param *param)
{
    ktime_t start_time;
    s64 elapsed_time_ms;
    int ret = 0;

    if (cts_dev == NULL || param == NULL) {
        cts_err("Reset-pin test with invalid param: cts_dev: %p test param: %p",
            cts_dev, param);
        return -EINVAL;
    }

    cts_info("Reset-Pin test, flags: 0x%08x, "
               "drive log file: '%s' buf size: %d",
        param->flags,
        param->driver_log_filepath, param->driver_log_buf_size);

    start_time = ktime_get();

    ret = cts_stop_device(cts_dev);
    if (ret) {
        cts_err("Stop device failed %d(%s)", ret, cts_strerror(ret));
        goto show_test_result;
    }

    cts_lock_device(cts_dev);

    cts_plat_set_reset(cts_dev->pdata, 0);
    mdelay(50);
#ifdef CONFIG_CTS_I2C_HOST
    /* Check whether device is in normal mode */
    if (cts_dev->bus_type == CTS_I2C_BUS &&
        cts_plat_is_i2c_online(cts_dev->pdata, CTS_DEV_NORMAL_MODE_I2CADDR)) {
        ret = -EIO;
    }
#endif
#ifdef CONFIG_CTS_SPI_HOST
    if (cts_dev->bus_type == CTS_SPI_BUS &&
        cts_plat_is_normal_mode(cts_dev->pdata)) {
        ret = -EIO;
    }
#endif /* CONFIG_CTS_SPI_HOST */
    if (ret) {
        cts_err("Device is alive while reset is low");
    }
    cts_plat_set_reset(cts_dev->pdata, 1);
    mdelay(50);

    {
        int r = wait_fw_to_normal_work(cts_dev);
        if (r) {
            cts_err("Wait fw to normal work failed %d(%s)",
                r, cts_strerror(r));
        }
    }

#ifdef CONFIG_CTS_I2C_HOST
    /* Check whether device is in normal mode */
    if (cts_dev->bus_type == CTS_I2C_BUS &&
        !cts_plat_is_i2c_online(cts_dev->pdata, CTS_DEV_NORMAL_MODE_I2CADDR)) {
        ret = -EIO;
    }
#endif
#ifdef CONFIG_CTS_SPI_HOST$
    if (cts_dev->bus_type == CTS_SPI_BUS &&
        !cts_plat_is_normal_mode(cts_dev->pdata)) {
        ret = -EIO;
    }
#endif /* CONFIG_CTS_SPI_HOST */
    if (ret) {
        cts_err("Device is offline while reset is high");
    }

#ifdef CONFIG_CTS_CHARGER_DETECT
    if (cts_is_charger_exist(cts_dev)) {
        int r = cts_set_dev_charger_attached(cts_dev, true);
        if (r) {
            cts_err("Set dev charger attached failed %d(%s)",
                r, cts_strerror(r));
        }
    }
#endif /* CONFIG_CTS_CHARGER_DETECT */

#ifdef CONFIG_CTS_EARJACK_DETECT
    if (cts_dev->fwdata.supp_headphone_cable_reject &&
        cts_is_earjack_exist(cts_dev)) {
        int r = cts_set_dev_earjack_attached(cts_dev, true);
        if (r) {
            cts_err("Set dev earjack attached failed %d(%s)",
                r, cts_strerror(r));
        }
    }
#endif /* CONFIG_CTS_EARJACK_DETECT */

#ifdef CONFIG_TOUCHSCREEN_CHIPONE_GLOVE
    if (cts_is_glove_enabled(cts_dev)) {
        cts_enter_glove_mode(cts_dev);
    }
#endif

#ifdef CFG_CTS_FW_LOG_REDIRECT
    if (cts_is_fw_log_redirect(cts_dev)) {
        cts_enable_fw_log_redirect(cts_dev);
    }
#endif

    cts_unlock_device(cts_dev);

    {
        int r = cts_start_device(cts_dev);
        if (r) {
            cts_err("Start device failed %d(%s)",
                r, cts_strerror(r));
        }
    }

    if (!cts_dev->rtdata.program_mode) {
        cts_set_normal_addr(cts_dev);
    }

show_test_result:
    elapsed_time_ms = ktime_ms_delta(ktime_get(), start_time);
    if (param->elapsed_time_ms) {
        *param->elapsed_time_ms = elapsed_time_ms;
    }
    if (param->test_result) {
        *param->test_result = ret;
    }
    cts_print_test_result_to_console("Reset-Pin", ret, elapsed_time_ms);

    return ret;
}
#endif

int cts_test_int_pin(struct cts_device *cts_dev, struct cts_test_param *param)
{
    ktime_t start_time;
    s64 elapsed_time_ms;
    int ret;

    if (cts_dev == NULL || param == NULL) {
        cts_err("Int-pin test with invalid param: cts_dev: %p test param: %p",
            cts_dev, param);
        return -EINVAL;
    }

    cts_info("Int-Pin test, flags: 0x%08x, "
             "drive log file: '%s' buf size: %d",
        param->flags,
        param->driver_log_filepath, param->driver_log_buf_size);

    start_time = ktime_get();

    ret = cts_stop_device(cts_dev);
    if (ret) {
        cts_err("Stop device failed %d(%s)", ret, cts_strerror(ret));
        goto show_test_result;
    }

    cts_lock_device(cts_dev);

    ret = cts_send_command(cts_dev, CTS_CMD_WRTITE_INT_HIGH);
    if (ret) {
        cts_err("Send command WRTITE_INT_HIGH failed %d(%s)",
            ret, cts_strerror(ret));
        goto unlock_device;
    }
    mdelay(10);
    if (cts_plat_get_int_pin(cts_dev->pdata) == 0) {
        cts_err("INT pin state != HIGH");
        ret = -EFAULT;
        goto exit_int_test;
    }

    ret = cts_send_command(cts_dev, CTS_CMD_WRTITE_INT_LOW);
    if (ret) {
        cts_err("Send command WRTITE_INT_LOW failed %d(%s)",
            ret, cts_strerror(ret));
        goto exit_int_test;
    }
    mdelay(10);
    if (cts_plat_get_int_pin(cts_dev->pdata) != 0) {
        cts_err("INT pin state != LOW");
        ret = -EFAULT;
        goto exit_int_test;
    }

exit_int_test:
    {
        int r = cts_send_command(cts_dev, CTS_CMD_RELASE_INT_TEST);
        if (r) {
            cts_err("Send command RELASE_INT_TEST failed %d(%s)",
                r, cts_strerror(r));
        }
    }
    mdelay(10);

unlock_device:
    cts_unlock_device(cts_dev);

    {
        int r = cts_start_device(cts_dev);
        if (r) {
            cts_err("Start device failed %d(%s)",
                r, cts_strerror(r));
        }
    }

show_test_result:
    elapsed_time_ms = ktime_ms_delta(ktime_get(), start_time);
    if (param->elapsed_time_ms) {
        *param->elapsed_time_ms = elapsed_time_ms;
    }
    if (param->test_result) {
        *param->test_result = ret;
    }
    cts_print_test_result_to_console("Int-Pin", ret, elapsed_time_ms);

    return ret;
}

void cts_dump_comp_cap(struct cts_device *cts_dev, u8 *cap, bool to_console)
{
#define SPLIT_LINE_STR \
            "-----------------------------------------------------------------------------"
#define ROW_NUM_FORMAT_STR  "%2d | "
#define COL_NUM_FORMAT_STR  "%3u "
#define DATA_FORMAT_STR     "%4d"

    int r, c;
    u32 max, min, sum, average;
    int max_r, max_c, min_r, min_c;
    char line_buf[128];
    int count;

    max = min = cap[0];
    sum = 0;
    max_r = max_c = min_r = min_c = 0;
    for (r = 0; r < cts_dev->fwdata.rows; r++) {
        for (c = 0; c < cts_dev->fwdata.cols; c++) {
            u16 val = cap[r * cts_dev->fwdata.cols + c];

            sum += val;
            if (val > max) {
                max = val;
                max_r = r;
                max_c = c;
            } else if (val < min) {
                min = val;
                min_r = r;
                min_c = c;
            }
        }
    }
    average = sum / (cts_dev->fwdata.rows * cts_dev->fwdata.cols);

    count = 0;
    count += scnprintf(line_buf + count, sizeof(line_buf) - count,
              " Compensate Cap MIN: [%u][%u]=%u, MAX: [%u][%u]=%u, AVG=%u",
              min_r, min_c, min, max_r, max_c, max, average);
    if (to_console) {
        cts_info(SPLIT_LINE_STR);
        cts_info("%s", line_buf);
        cts_info(SPLIT_LINE_STR);
    }
    if (cts_test_data_filp) {
        cts_write_file(cts_test_data_filp, SPLIT_LINE_STR, strlen(SPLIT_LINE_STR));
        cts_write_file(cts_test_data_filp, "\n", 1);
        cts_write_file(cts_test_data_filp, line_buf, count);
        cts_write_file(cts_test_data_filp, "\n", 1);
        cts_write_file(cts_test_data_filp, SPLIT_LINE_STR, strlen(SPLIT_LINE_STR));
        cts_write_file(cts_test_data_filp, "\n", 1);
    }

    count = 0;
    count += scnprintf(line_buf + count, sizeof(line_buf) - count, "      ");
    for (c = 0; c < cts_dev->fwdata.cols; c++) {
        count += scnprintf(line_buf + count, sizeof(line_buf) - count,
                  COL_NUM_FORMAT_STR, c);
    }
    if (to_console) {
        cts_info("%s", line_buf);
        cts_info(SPLIT_LINE_STR);
    }
    if (cts_test_data_filp) {
        cts_write_file(cts_test_data_filp, line_buf, count);
        cts_write_file(cts_test_data_filp, "\n", 1);
        cts_write_file(cts_test_data_filp, SPLIT_LINE_STR, strlen(SPLIT_LINE_STR));
        cts_write_file(cts_test_data_filp, "\n", 1);
    }

    for (r = 0; r < cts_dev->fwdata.rows; r++) {
        count = 0;
        count += scnprintf(line_buf + count, sizeof(line_buf) - count,
                  ROW_NUM_FORMAT_STR, r);
        for (c = 0; c < cts_dev->fwdata.cols; c++) {
            count += scnprintf(line_buf + count,
                      sizeof(line_buf) - count,
                      DATA_FORMAT_STR,
                      cap[r * cts_dev->fwdata.cols + c]);
        }
        if (to_console) {
            cts_info("%s", line_buf);
        }
        if (cts_test_data_filp) {
            cts_write_file(cts_test_data_filp, line_buf, count);
            cts_write_file(cts_test_data_filp, "\n", 1);
        }
    }

    if (to_console) {
        cts_info(SPLIT_LINE_STR);
    }
    if (cts_test_data_filp) {
        cts_write_file(cts_test_data_filp, SPLIT_LINE_STR, strlen(SPLIT_LINE_STR));
        cts_write_file(cts_test_data_filp, "\n", 1);
    }
#undef SPLIT_LINE_STR
#undef ROW_NUM_FORMAT_STR
#undef COL_NUM_FORMAT_STR
#undef DATA_FORMAT_STR
}

int cts_test_compensate_cap(struct cts_device *cts_dev,
    struct cts_test_param *param)
{
    bool driver_validate_data = false;
    bool validate_data_per_node = false;
    bool dump_test_data_to_user = false;
    bool dump_test_data_to_console = false;
    bool dump_test_data_to_file = false;
    int  num_nodes;
    u8 * cap = NULL;
    int  ret = 0;
    ktime_t start_time;
    s64 elapsed_time_ms;

    if (cts_dev == NULL || param == NULL) {
        cts_err("Compensate cap test with invalid param: cts_dev: %p test param: %p",
            cts_dev, param);
        return -EINVAL;
    }

    num_nodes = cts_dev->hwdata->num_row * cts_dev->hwdata->num_col;

    driver_validate_data =
        !!(param->flags & CTS_TEST_FLAG_VALIDATE_DATA);
    if (driver_validate_data) {
        validate_data_per_node =
            !!(param->flags & CTS_TEST_FLAG_VALIDATE_PER_NODE);
    }
    dump_test_data_to_user =
        !!(param->flags & CTS_TEST_FLAG_DUMP_TEST_DATA_TO_USERSPACE);
    dump_test_data_to_console =
        !!(param->flags & CTS_TEST_FLAG_DUMP_TEST_DATA_TO_CONSOLE);
    dump_test_data_to_file =
        !!(param->flags & CTS_TEST_FLAG_DUMP_TEST_DATA_TO_FILE);

    cts_info("Compensate cap test, flags: 0x%08x "
               "num invalid node: %u, "
               "test data file: '%s' buf size: %d, "
               "drive log file: '%s' buf size: %d",
        param->flags, param->num_invalid_node,
        param->test_data_filepath, param->test_data_buf_size,
        param->driver_log_filepath, param->driver_log_buf_size);

    start_time = ktime_get();

    if (dump_test_data_to_user) {
        cap = (u8 *)param->test_data_buf;
    } else {
        cap = (u8 *)kzalloc(num_nodes, GFP_KERNEL);
        if (cap == NULL) {
            cts_err("Alloc mem for compensate cap failed");
            ret = -ENOMEM;
            goto show_test_result;
        }
    }

    /* Stop device to avoid un-wanted interrrupt */
    ret = cts_stop_device(cts_dev);
    if (ret) {
        cts_err("Stop device failed %d(%s)", ret, cts_strerror(ret));
        goto free_mem;
    }

    cts_lock_device(cts_dev);
    ret = cts_get_compensate_cap(cts_dev, cap);
    cts_unlock_device(cts_dev);
    if (ret) {
        cts_err("Get compensate cap failed %d(%s)",
            ret, cts_strerror(ret));
        goto start_device;
    }

    if (dump_test_data_to_user) {
        *param->test_data_wr_size = num_nodes;
    }

    if (dump_test_data_to_file) {
        int r = cts_start_dump_test_data_to_file(param->test_data_filepath,
            !!(param->flags & CTS_TEST_FLAG_DUMP_TEST_DATA_TO_FILE_APPEND));
        if (r) {
            cts_err("Start dump test data to file failed %d(%s)",
                r, cts_strerror(r));
        }
    }

    if (dump_test_data_to_console || dump_test_data_to_file) {
        cts_dump_comp_cap(cts_dev, cap,
            dump_test_data_to_console);
    }

    if (dump_test_data_to_file) {
        cts_stop_dump_test_data_to_file();
    }

    if (driver_validate_data) {
        ret = validate_comp_cap(cts_dev, "Compensate-Cap",
            cap, param->invalid_nodes, param->num_invalid_node,
            validate_data_per_node, param->min, param->max);
    }

start_device:
    {
        int r = cts_start_device(cts_dev);
        if (r) {
            cts_err("Start device failed %d(%s)",
                r, cts_strerror(r));
        }
    }

free_mem:
    if (!dump_test_data_to_user && cap) {
        kfree(cap);
    }

show_test_result:
    elapsed_time_ms = ktime_ms_delta(ktime_get(), start_time);
    if (param->elapsed_time_ms) {
        *param->elapsed_time_ms = elapsed_time_ms;
    }
    if (param->test_result) {
        *param->test_result = ret;
    }
    cts_print_test_result_to_console("Comp-CAP", ret, elapsed_time_ms);

    return ret;
}

int cts_test_rawdata(struct cts_device *cts_dev,
    struct cts_test_param *param)
{
    struct cts_rawdata_test_priv_param *priv_param;
    bool driver_validate_data = false;
    bool validate_data_per_node = false;
    bool stop_test_if_validate_fail = false;
    bool dump_test_data_to_user = false;
    bool dump_test_data_to_console = false;
    bool dump_test_data_to_file = false;
    int  num_nodes;
    int  tsdata_frame_size;
    int  frame;
    u16 *rawdata = NULL;
    ktime_t start_time;
    s64 elapsed_time_ms;
    int  i;
    int  ret;

    if (cts_dev == NULL || param == NULL ||
        param->priv_param_size != sizeof(*priv_param) ||
        param->priv_param == NULL) {
        cts_err("Rawdata test with invalid param: priv param: %p size: %d",
            param->priv_param, param->priv_param_size);
        return -EINVAL;
    }

    priv_param = param->priv_param;
    if (priv_param->frames <= 0) {
        cts_info("Rawdata test with too little frame %u",
            priv_param->frames);
        return -EINVAL;
    }

    num_nodes = cts_dev->fwdata.rows * cts_dev->fwdata.cols;
    tsdata_frame_size = 2 * num_nodes;

    driver_validate_data =
        !!(param->flags & CTS_TEST_FLAG_VALIDATE_DATA);
    validate_data_per_node =
        !!(param->flags & CTS_TEST_FLAG_VALIDATE_PER_NODE);
    dump_test_data_to_user =
        !!(param->flags & CTS_TEST_FLAG_DUMP_TEST_DATA_TO_USERSPACE);
    dump_test_data_to_console =
        !!(param->flags & CTS_TEST_FLAG_DUMP_TEST_DATA_TO_CONSOLE);
    dump_test_data_to_file =
        !!(param->flags & CTS_TEST_FLAG_DUMP_TEST_DATA_TO_FILE);
    stop_test_if_validate_fail =
        !!(param->flags & CTS_TEST_FLAG_STOP_TEST_IF_VALIDATE_FAILED);

    cts_info("Rawdata test, flags: 0x%08x, frames: %d, "
               "num invalid node: %u, "
               "test data file: '%s' buf size: %d, "
               "drive log file: '%s' buf size: %d",
        param->flags, priv_param->frames, param->num_invalid_node,
        param->test_data_filepath, param->test_data_buf_size,
        param->driver_log_filepath, param->driver_log_buf_size);

    start_time = ktime_get();

    if (dump_test_data_to_user) {
        rawdata = (u16 *)param->test_data_buf;
    } else {
        rawdata = (u16 *)kmalloc(tsdata_frame_size, GFP_KERNEL);
        if (rawdata == NULL) {
            cts_err("Allocate memory for rawdata failed");
            ret = -ENOMEM;
            goto show_test_result;
        }
    }

    /* Stop device to avoid un-wanted interrrupt */
    ret = cts_stop_device(cts_dev);
    if (ret) {
        cts_err("Stop device failed %d(%s)", ret, cts_strerror(ret));
        goto free_mem;
    }

    cts_lock_device(cts_dev);

    for (i = 0; i < 5; i++) {
        int r;
        u8 val;
        r = cts_enable_get_rawdata(cts_dev);
        if (r) {
            cts_err("Enable get tsdata failed %d(%s)",
                r, cts_strerror(r));
            continue;
        }
        mdelay(1);
        r = cts_fw_reg_readb(cts_dev, 0x12, &val);
        if (r) {
            cts_err("Read enable get tsdata failed %d(%s)",
                r, cts_strerror(r));
            continue;
        }
        if (val != 0) {
            break;
        }
    }

    if (i >= 5) {
        cts_err("Enable read tsdata failed");
        ret = -EIO;
        goto unlock_device;
    }

    if (dump_test_data_to_file) {
        int r = cts_start_dump_test_data_to_file(param->test_data_filepath,
            !!(param->flags & CTS_TEST_FLAG_DUMP_TEST_DATA_TO_FILE_APPEND));
        if (r) {
            cts_err("Start dump test data to file failed %d(%s)",
                r, cts_strerror(r));
        }
    }

    for (frame = 0; frame < priv_param->frames; frame++) {
        bool data_valid = false;
        int  r;

        //if (cts_dev.fwdata.monitor_mode) {
            r = cts_send_command(cts_dev, CTS_CMD_QUIT_GESTURE_MONITOR);
            if (r) {
                cts_err("Send CMD_QUIT_GESTURE_MONITOR failed %d(%s)",
                    r, cts_strerror(r));
            }
        //}

        for (i = 0; i < 3; i++) {
            r = cts_get_rawdata(cts_dev, rawdata);
            if (r) {
                cts_err("Get rawdata failed %d(%s)", r, cts_strerror(r));
                mdelay(30);
            } else {
                data_valid = true;
                break;
            }
        }

        if (!data_valid) {
            ret = -EIO;
            break;
        }

        if (dump_test_data_to_user) {
            *param->test_data_wr_size += tsdata_frame_size;
        }

        if (dump_test_data_to_console || dump_test_data_to_file) {
            cts_dump_tsdata(cts_dev, "Rawdata", rawdata,
                dump_test_data_to_console);
        }

        if (driver_validate_data) {
            ret = validate_tsdata(cts_dev,
                "Rawdata", rawdata,
                param->invalid_nodes, param->num_invalid_node,
                validate_data_per_node, param->min, param->max);
            if (ret) {
                cts_err("Rawdata test failed %d(%s)",
                    ret, cts_strerror(ret));
                if (stop_test_if_validate_fail) {
                    break;
                }
            }
        }

        if (dump_test_data_to_user) {
            rawdata += num_nodes;
        }
    }

    if (dump_test_data_to_file) {
        cts_stop_dump_test_data_to_file();
    }

    for (i = 0; i < 5; i++) {
        int r = cts_disable_get_rawdata(cts_dev);
        if (r) {
            cts_err("Disable get rawdata failed %d(%s)",
                r, cts_strerror(r));
            continue;
        } else {
            break;
        }
    }

unlock_device:
    cts_unlock_device(cts_dev);

    {
        int r = cts_start_device(cts_dev);
        if (r) {
            cts_err("Start device failed %d(%s)", r, cts_strerror(r));
        }
    }

free_mem:
    if (!dump_test_data_to_user && rawdata != NULL) {
        kfree(rawdata);
    }

show_test_result:
    elapsed_time_ms = ktime_ms_delta(ktime_get(), start_time);
    if (param->elapsed_time_ms) {
        *param->elapsed_time_ms = elapsed_time_ms;
    }
    if (param->test_result) {
        *param->test_result = ret;
    }
    cts_print_test_result_to_console("Rawdata", ret, elapsed_time_ms);

    return ret;
}

int cts_test_noise(struct cts_device *cts_dev,
        struct cts_test_param *param)
{
    struct cts_noise_test_priv_param *priv_param;
    bool driver_validate_data = false;
    bool validate_data_per_node = false;
    bool dump_test_data_to_user = false;
    bool dump_test_data_to_console = false;
    bool dump_test_data_to_file = false;
    int  num_nodes;
    int  tsdata_frame_size;
    int  frame;
    u16 *buffer = NULL;
    int  buf_size = 0;
    u16 *curr_rawdata = NULL;
    u16 *max_rawdata = NULL;
    u16 *min_rawdata = NULL;
    u16 *noise = NULL;
    bool first_frame = true;
    ktime_t start_time;
    s64 elapsed_time_ms;
    int  i;
    int  ret;

    if (cts_dev == NULL || param == NULL ||
        param->priv_param_size != sizeof(*priv_param) ||
        param->priv_param == NULL) {
        cts_err("Noise test with invalid param: priv param: %p size: %d",
            param->priv_param, param->priv_param_size);
        return -EINVAL;
    }

    priv_param = param->priv_param;
    if (priv_param->frames < 2) {
        cts_err("Noise test with too little frame %u",
            priv_param->frames);
        return -EINVAL;
    }

    num_nodes = cts_dev->fwdata.rows * cts_dev->fwdata.cols;
    tsdata_frame_size = 2 * num_nodes;

    driver_validate_data =
        !!(param->flags & CTS_TEST_FLAG_VALIDATE_DATA);
    validate_data_per_node =
        !!(param->flags & CTS_TEST_FLAG_VALIDATE_PER_NODE);
    dump_test_data_to_user =
        !!(param->flags & CTS_TEST_FLAG_DUMP_TEST_DATA_TO_USERSPACE);
    dump_test_data_to_console =
        !!(param->flags & CTS_TEST_FLAG_DUMP_TEST_DATA_TO_CONSOLE);
    dump_test_data_to_file =
        !!(param->flags & CTS_TEST_FLAG_DUMP_TEST_DATA_TO_FILE);

    cts_info("Noise test, flags: 0x%08x, frames: %d, "
               "num invalid node: %u, "
               "test data file: '%s' buf size: %d, "
               "drive log file: '%s' buf size: %d",
        param->flags, priv_param->frames, param->num_invalid_node,
        param->test_data_filepath, param->test_data_buf_size,
        param->driver_log_filepath, param->driver_log_buf_size);

    start_time = ktime_get();

    buf_size = (driver_validate_data ? 4 : 1) * tsdata_frame_size;
    buffer = (u16 *)kmalloc(buf_size, GFP_KERNEL);
    if (buffer == NULL) {
        cts_err("Alloc mem for touch data failed");
        ret = -ENOMEM;
        goto show_test_result;
    }

    curr_rawdata = buffer;
    if (driver_validate_data) {
        max_rawdata = curr_rawdata + 1 * num_nodes;
        min_rawdata = curr_rawdata + 2 * num_nodes;
        noise       = curr_rawdata + 3 * num_nodes;
    }

    /* Stop device to avoid un-wanted interrrupt */
    ret = cts_stop_device(cts_dev);
    if (ret) {
        cts_err("Stop device failed %d(%s)", ret, cts_strerror(ret));
        goto free_mem;
    }

    cts_lock_device(cts_dev);

    for (i = 0; i < 5; i++) {
        int r;
        u8 val;
        r = cts_enable_get_rawdata(cts_dev);
        if (r) {
            cts_err("Enable get ts data failed %d(%s)",
                r, cts_strerror(r));
            continue;
        }
        mdelay(1);
        r = cts_fw_reg_readb(cts_dev, 0x12, &val);
        if (r) {
            cts_err("Read enable get ts data failed %d(%s)",
                r, cts_strerror(r));
            continue;
        }
        if (val != 0) {
            break;
        }
    }

    if (i >= 5) {
        cts_err("Enable read tsdata failed");
        ret = -EIO;
        goto unlock_device;
    }

    if (dump_test_data_to_file) {
        int r = cts_start_dump_test_data_to_file(param->test_data_filepath,
            !!(param->flags & CTS_TEST_FLAG_DUMP_TEST_DATA_TO_FILE_APPEND));
        if (r) {
            cts_err("Start dump test data to file failed %d(%s)",
                r, cts_strerror(r));
        }
    }

    msleep(50);

    for (frame = 0; frame < priv_param->frames; frame++) {
        int r;

        r = cts_send_command(cts_dev, CTS_CMD_QUIT_GESTURE_MONITOR);
        if (r) {
            cts_err("send quit gesture monitor failed %d(%s)",
                r, cts_strerror(r));
            // Ignore this error
        }

        for (i = 0; i < 3; i++) {
            r = cts_get_rawdata(cts_dev, curr_rawdata);
            if (r) {
                cts_err("Get rawdata failed %d(%s)",
                    r, cts_strerror(r));
                mdelay(30);
            } else {
                break;
            }
        }

        if (i >= 3) {
            cts_err("Read rawdata failed");
            ret = -EIO;
            goto disable_get_tsdata;
        }

        if (dump_test_data_to_console || dump_test_data_to_file) {
            cts_dump_tsdata(cts_dev, "Noise-rawdata", curr_rawdata,
                dump_test_data_to_console);
        }

        if (dump_test_data_to_user) {
            memcpy(param->test_data_buf + *param->test_data_wr_size,
                curr_rawdata, tsdata_frame_size);
            *param->test_data_wr_size += tsdata_frame_size;
        }

        if (driver_validate_data) {
            if (unlikely(first_frame)) {
                memcpy(max_rawdata, curr_rawdata, tsdata_frame_size);
                memcpy(min_rawdata, curr_rawdata, tsdata_frame_size);
                first_frame = false;
            } else {
                for (i = 0; i < num_nodes; i++) {
                    if (curr_rawdata[i] > max_rawdata[i]) {
                        max_rawdata[i] = curr_rawdata[i];
                    } else if (curr_rawdata[i] < min_rawdata[i]) {
                        min_rawdata[i] = curr_rawdata[i];
                    }
                }
            }
        }
    }

    if (driver_validate_data) {
        for (i = 0; i < num_nodes; i++) {
            noise[i] = max_rawdata[i] - min_rawdata[i];
        }

        if (dump_test_data_to_user &&
            param->test_data_buf_size >=
                (*param->test_data_wr_size + tsdata_frame_size)) {
           memcpy(param->test_data_buf + *param->test_data_wr_size,
               noise, tsdata_frame_size);
           *param->test_data_wr_size += tsdata_frame_size;
       }

        if (dump_test_data_to_console || dump_test_data_to_file) {
            cts_dump_tsdata(cts_dev, "Noise", noise,
                dump_test_data_to_console);
        }

        ret = validate_tsdata(cts_dev, "Noise test",
            noise, param->invalid_nodes, param->num_invalid_node,
            validate_data_per_node, param->min, param->max);
    }

    if (dump_test_data_to_file) {
        cts_stop_dump_test_data_to_file();
    }

disable_get_tsdata:
    for (i = 0; i < 5; i++) {
        int r = cts_disable_get_rawdata(cts_dev);
        if (r) {
            cts_err("Disable get rawdata failed %d(%s)",
                r, cts_strerror(r));
            continue;
        } else {
            break;
        }
    }

unlock_device:
    cts_unlock_device(cts_dev);

    {
        int r = cts_start_device(cts_dev);
        if (r) {
            cts_err("Start device failed %d(%s)",
                r, cts_strerror(r));
        }
    }

free_mem:
    if (buffer) {
        kfree(buffer);
    }

show_test_result:
    elapsed_time_ms = ktime_ms_delta(ktime_get(), start_time);
    if (param->elapsed_time_ms) {
        *param->elapsed_time_ms = elapsed_time_ms;
    }
    if (param->test_result) {
        *param->test_result = ret;
    }
    cts_print_test_result_to_console("Noise", ret, elapsed_time_ms);

    return ret;
}

static bool set_gesture_raw_type(struct cts_device *cts_dev, u8 type)
{
    u8 val = 0xff, r;

    r = cts_fw_reg_writeb(cts_dev, 0x45, type);
    if (r) {
        cts_err("Set gesture raw type failed %d(%s)", r, cts_strerror(r));
        return false;
    }

    r = cts_fw_reg_readb(cts_dev, 0x45, &val);
    if (r) {
        cts_err("Get gesture raw type failed %d(%s)", r, cts_strerror(r));
        return false;
    }
    return val == type;
}

int cts_test_gesture_rawdata(struct cts_device *cts_dev,
    struct cts_test_param *param)
{
    struct cts_rawdata_test_priv_param *priv_param;
    bool driver_validate_data = false;
    bool validate_data_per_node = false;
    bool stop_test_if_validate_fail = false;
    bool dump_test_data_to_user = false;
    bool dump_test_data_to_console = false;
    bool dump_test_data_to_file = false;
    int  num_nodes;
    int  tsdata_frame_size;
    int  frame;
    int  idle_mode;
    u16 *gesture_rawdata = NULL;
    ktime_t start_time;
    s64 elapsed_time_ms;
    int  i;
    int  ret;

    if (cts_dev == NULL || param == NULL ||
        param->priv_param_size != sizeof(*priv_param) ||
        param->priv_param == NULL) {
        cts_err("Gesture rawdata test with invalid priv param: %p size: %d",
            param->priv_param, param->priv_param_size);
        return -EINVAL;
    }

    priv_param = param->priv_param;
    if (priv_param->frames <= 0) {
        cts_info("Gesture rawdata test with too little frame %u",
            priv_param->frames);
        return -EINVAL;
    }

    num_nodes = cts_dev->fwdata.rows * cts_dev->fwdata.cols;
    tsdata_frame_size = 2 * num_nodes;

    driver_validate_data =
        !!(param->flags & CTS_TEST_FLAG_VALIDATE_DATA);
    validate_data_per_node =
        !!(param->flags & CTS_TEST_FLAG_VALIDATE_PER_NODE);
    dump_test_data_to_user =
        !!(param->flags & CTS_TEST_FLAG_DUMP_TEST_DATA_TO_USERSPACE);
    dump_test_data_to_console =
        !!(param->flags & CTS_TEST_FLAG_DUMP_TEST_DATA_TO_CONSOLE);
    dump_test_data_to_file =
        !!(param->flags & CTS_TEST_FLAG_DUMP_TEST_DATA_TO_FILE);
    stop_test_if_validate_fail =
        !!(param->flags & CTS_TEST_FLAG_STOP_TEST_IF_VALIDATE_FAILED);

    cts_info("Gesture rawdata test, flags: 0x%08x, frames: %d, "
               "num invalid node: %u, "
               "test data file: '%s' buf size: %d, "
               "drive log file: '%s' buf size: %d",
        param->flags, priv_param->frames, param->num_invalid_node,
        param->test_data_filepath, param->test_data_buf_size,
        param->driver_log_filepath, param->driver_log_buf_size);

    start_time = ktime_get();

    if (dump_test_data_to_user) {
        gesture_rawdata = (u16 *)param->test_data_buf;
    } else {
        gesture_rawdata = (u16 *)kmalloc(tsdata_frame_size, GFP_KERNEL);
        if (gesture_rawdata == NULL) {
            cts_err("Allocate memory for rawdata failed");
            ret = -ENOMEM;
            goto show_test_result;
        }
    }

    /* Stop device to avoid un-wanted interrrupt */
    ret = cts_stop_device(cts_dev);
    if (ret) {
        cts_err("Stop device failed %d(%s)", ret, cts_strerror(ret));
        goto free_mem;
    }

    cts_lock_device(cts_dev);

    idle_mode = priv_param->work_mode;

    for (i = 0; i < 5; i++) {
        int r;
        u8 val;

        r = cts_enable_get_rawdata(cts_dev);
        if (r) {
            cts_err("Enable get tsdata failed %d(%s)",
                r, cts_strerror(r));
            continue;
        }
        mdelay(1);
        r = cts_fw_reg_readb(cts_dev, 0x12, &val);
        if (r) {
            cts_err("Read enable get tsdata failed %d(%s)",
                r, cts_strerror(r));
            continue;
        }
        if (val != 0) {
            break;
        }
    }

    if (i >= 5) {
        cts_err("Enable read tsdata failed");
        ret = -EIO;
        goto unlock_device;
    }

    if (dump_test_data_to_file) {
        int r = cts_start_dump_test_data_to_file(param->test_data_filepath,
            !!(param->flags & CTS_TEST_FLAG_DUMP_TEST_DATA_TO_FILE_APPEND));
        if (r) {
            cts_err("Start dump test data to file failed %d(%s)",
                r, cts_strerror(r));
        }
    }

    for (frame = 0; frame < priv_param->frames; frame++) {
        bool data_valid = false;
        int  r;

        r = cts_set_work_mode(cts_dev, CTS_WORK_MODE_GESTURE_ACTIVE);
        if (r) {
            cts_err("Set work mode:%d failed %d(%s)",
                CTS_WORK_MODE_GESTURE_ACTIVE, r, cts_strerror(r));
            continue;
        }

        r = cts_send_command(cts_dev, CTS_CMD_QUIT_GESTURE_MONITOR);
        if (r) {
            cts_err("Send CMD_QUIT_GESTURE_MONITOR failed %d(%s)",
                r, cts_strerror(r));
        }

        if (!set_gesture_raw_type(cts_dev, idle_mode)) {
           cts_err("Set gesture raw type failed");
           continue;
        }

        for (i = 0; i < 3; i++) {
            r = cts_get_rawdata(cts_dev, gesture_rawdata);
            if (r) {
                cts_err("Get gesture rawdata failed %d(%s)",
                    r, cts_strerror(r));
                mdelay(30);
            } else {
                data_valid = true;
                break;
            }
        }

        if (!data_valid) {
            ret = -EIO;
            break;
        }

        if (dump_test_data_to_user) {
            *param->test_data_wr_size += tsdata_frame_size;
        }

        if (dump_test_data_to_console || dump_test_data_to_file) {
            cts_dump_tsdata(cts_dev,
                idle_mode ? "Gesture Rawdata" : "Gesture LP Rawdata",
                gesture_rawdata, dump_test_data_to_console);
        }

        if (driver_validate_data) {
            ret = validate_tsdata(cts_dev,
                idle_mode ? "Gesture Rawdata" : "Gesture LP Rawdata",
                gesture_rawdata, param->invalid_nodes, param->num_invalid_node,
                validate_data_per_node, param->min, param->max);
            if (ret) {
                cts_err("Gesture Rawdata test failed %d(%s)",
                    ret, cts_strerror(ret));
                if (stop_test_if_validate_fail) {
                    break;
                }
            }
        }

        if (dump_test_data_to_user) {
            gesture_rawdata += num_nodes;
        }
    }

    if (dump_test_data_to_file) {
        cts_stop_dump_test_data_to_file();
    }

    for (i = 0; i < 5; i++) {
        int r = cts_disable_get_rawdata(cts_dev);
        if (r) {
            cts_err("Disable get rawdata failed %d(%s)",
                r, cts_strerror(r));
            continue;
        } else {
            break;
        }
    }

unlock_device:
    cts_unlock_device(cts_dev);

    {
        int r = cts_start_device(cts_dev);
        if (r) {
            cts_err("Start device failed %d(%s)", r, cts_strerror(r));
        }
    }

free_mem:
    if (!dump_test_data_to_user && gesture_rawdata != NULL) {
        kfree(gesture_rawdata);
    }

show_test_result:
    elapsed_time_ms = ktime_ms_delta(ktime_get(), start_time);
    if (param->elapsed_time_ms) {
        *param->elapsed_time_ms = elapsed_time_ms;
    }
    if (param->test_result) {
        *param->test_result = ret;
    }
    cts_print_test_result_to_console("Gesture Rawdata", ret, elapsed_time_ms);

    return ret;
}

int cts_test_gesture_noise(struct cts_device *cts_dev,
        struct cts_test_param *param)
{
    struct cts_noise_test_priv_param *priv_param;
    bool driver_validate_data = false;
    bool validate_data_per_node = false;
    bool dump_test_data_to_user = false;
    bool dump_test_data_to_console = false;
    bool dump_test_data_to_file = false;
    int  num_nodes;
    int  tsdata_frame_size;
    int  frame;
    int  idle_mode;
    u16 *buffer = NULL;
    int  buf_size = 0;
    u16 *curr_rawdata = NULL;
    u16 *max_rawdata = NULL;
    u16 *min_rawdata = NULL;
    u16 *gesture_noise = NULL;
    bool first_frame = true;
    ktime_t start_time;
    s64 elapsed_time_ms;
    int  i;
    int  ret;

    if (cts_dev == NULL || param == NULL ||
        param->priv_param_size != sizeof(*priv_param) ||
        param->priv_param == NULL) {
        cts_err("Noise test with invalid param: priv param: %p size: %d",
            param->priv_param, param->priv_param_size);
        return -EINVAL;
    }

    priv_param = param->priv_param;
    if (priv_param->frames < 2) {
        cts_err("Noise test with too little frame %u",
            priv_param->frames);
        return -EINVAL;
    }

    num_nodes = cts_dev->fwdata.rows * cts_dev->fwdata.cols;
    tsdata_frame_size = 2 * num_nodes;

    driver_validate_data =
        !!(param->flags & CTS_TEST_FLAG_VALIDATE_DATA);
    validate_data_per_node =
        !!(param->flags & CTS_TEST_FLAG_VALIDATE_PER_NODE);
    dump_test_data_to_user =
        !!(param->flags & CTS_TEST_FLAG_DUMP_TEST_DATA_TO_USERSPACE);
    dump_test_data_to_console =
        !!(param->flags & CTS_TEST_FLAG_DUMP_TEST_DATA_TO_CONSOLE);
    dump_test_data_to_file =
        !!(param->flags & CTS_TEST_FLAG_DUMP_TEST_DATA_TO_FILE);

    cts_info("Noise test, flags: 0x%08x, frames: %d, "
               "num invalid node: %u, "
               "test data file: '%s' buf size: %d, "
               "drive log file: '%s' buf size: %d",
        param->flags, priv_param->frames, param->num_invalid_node,
        param->test_data_filepath, param->test_data_buf_size,
        param->driver_log_filepath, param->driver_log_buf_size);

    start_time = ktime_get();

    buf_size = (driver_validate_data ? 4 : 1) * tsdata_frame_size;
    buffer = (u16 *)kmalloc(buf_size, GFP_KERNEL);
    if (buffer == NULL) {
        cts_err("Alloc mem for touch data failed");
        ret = -ENOMEM;
        goto show_test_result;
    }

    curr_rawdata = buffer;
    if (driver_validate_data) {
        max_rawdata = curr_rawdata + 1 * num_nodes;
        min_rawdata = curr_rawdata + 2 * num_nodes;
        gesture_noise = curr_rawdata + 3 * num_nodes;
    }

    /* Stop device to avoid un-wanted interrrupt */
    ret = cts_stop_device(cts_dev);
    if (ret) {
        cts_err("Stop device failed %d(%s)", ret, cts_strerror(ret));
        goto free_mem;
    }

    cts_lock_device(cts_dev);

    idle_mode = priv_param->work_mode;

    for (i = 0; i < 5; i++) {
        int r;
        u8 val;

        r = cts_enable_get_rawdata(cts_dev);
        if (r) {
            cts_err("Enable get ts data failed %d(%s)",
                r, cts_strerror(r));
            continue;
        }
        mdelay(1);
        r = cts_fw_reg_readb(cts_dev, 0x12, &val);
        if (r) {
            cts_err("Read enable get ts data failed %d(%s)",
                r, cts_strerror(r));
            continue;
        }
        if (val != 0) {
            break;
        }
    }

    if (i >= 5) {
        cts_err("Enable read tsdata failed");
        ret = -EIO;
        goto unlock_device;
    }

    if (dump_test_data_to_file) {
        int r = cts_start_dump_test_data_to_file(param->test_data_filepath,
            !!(param->flags & CTS_TEST_FLAG_DUMP_TEST_DATA_TO_FILE_APPEND));
        if (r) {
            cts_err("Start dump test data to file failed %d(%s)",
                r, cts_strerror(r));
        }
    }

    msleep(50);

    for (frame = 0; frame < priv_param->frames; frame++) {
        int r;

        r = cts_set_work_mode(cts_dev, CTS_WORK_MODE_GESTURE_ACTIVE);
        if (r) {
            cts_err("Set work mode:%d failed %d(%s)",
                CTS_WORK_MODE_GESTURE_ACTIVE, r, cts_strerror(r));
            continue;
        }

        r = cts_send_command(cts_dev, CTS_CMD_QUIT_GESTURE_MONITOR);
        if (r) {
            cts_err("send quit gesture monitor failed %d(%s)",
                r, cts_strerror(r));
            // Ignore this error
        }

        if (!set_gesture_raw_type(cts_dev, idle_mode)) {
           cts_err("Set gesture raw type failed");
           continue;
        }

        for (i = 0; i < 3; i++) {
            r = cts_get_rawdata(cts_dev, curr_rawdata);
            if (r) {
                cts_err("Get rawdata failed %d(%s)",
                    r, cts_strerror(r));
                mdelay(30);
            } else {
                break;
            }
        }

        if (i >= 3) {
            cts_err("Read rawdata failed");
            ret = -EIO;
            goto disable_get_tsdata;
        }

        if (dump_test_data_to_console || dump_test_data_to_file) {
            cts_dump_tsdata(cts_dev,
                idle_mode ? "Gstr Noise-Raw" : "Gstr LP Noise-Raw",
                curr_rawdata, dump_test_data_to_console);
        }

        if (dump_test_data_to_user) {
            memcpy(param->test_data_buf + *param->test_data_wr_size,
                curr_rawdata, tsdata_frame_size);
            *param->test_data_wr_size += tsdata_frame_size;
        }

        if (driver_validate_data) {
            if (unlikely(first_frame)) {
                memcpy(max_rawdata, curr_rawdata, tsdata_frame_size);
                memcpy(min_rawdata, curr_rawdata, tsdata_frame_size);
                first_frame = false;
            } else {
                for (i = 0; i < num_nodes; i++) {
                    if (curr_rawdata[i] > max_rawdata[i]) {
                        max_rawdata[i] = curr_rawdata[i];
                    } else if (curr_rawdata[i] < min_rawdata[i]) {
                        min_rawdata[i] = curr_rawdata[i];
                    }
                }
            }
        }
    }

    if (driver_validate_data) {
        for (i = 0; i < num_nodes; i++) {
            gesture_noise[i] = max_rawdata[i] - min_rawdata[i];
        }

        if (dump_test_data_to_user &&
            param->test_data_buf_size >=
                (*param->test_data_wr_size + tsdata_frame_size)) {
           memcpy(param->test_data_buf + *param->test_data_wr_size,
               gesture_noise, tsdata_frame_size);
           *param->test_data_wr_size += tsdata_frame_size;
       }

        if (dump_test_data_to_console || dump_test_data_to_file) {
            cts_dump_tsdata(cts_dev,
                idle_mode ? "Gesture Noise" : "Gesture LP Noise",
                gesture_noise, dump_test_data_to_console);
        }

        ret = validate_tsdata(cts_dev,
            idle_mode ? "Gesture Noise" : "Gesture LP Noise",
            gesture_noise, param->invalid_nodes, param->num_invalid_node,
            validate_data_per_node, param->min, param->max);
    }

    if (dump_test_data_to_file) {
        cts_stop_dump_test_data_to_file();
    }

disable_get_tsdata:
    for (i = 0; i < 5; i++) {
        int r = cts_disable_get_rawdata(cts_dev);
        if (r) {
            cts_err("Disable get rawdata failed %d(%s)",
                r, cts_strerror(r));
            continue;
        } else {
            break;
        }
    }

unlock_device:
    cts_unlock_device(cts_dev);

    {
        int r = cts_start_device(cts_dev);
        if (r) {
            cts_err("Start device failed %d(%s)",
                r, cts_strerror(r));
        }
    }

free_mem:
    if (buffer) {
        kfree(buffer);
    }

show_test_result:
    elapsed_time_ms = ktime_ms_delta(ktime_get(), start_time);
    if (param->elapsed_time_ms) {
        *param->elapsed_time_ms = elapsed_time_ms;
    }
    if (param->test_result) {
        *param->test_result = ret;
    }
    cts_print_test_result_to_console("Noise", ret, elapsed_time_ms);

    return ret;
}

bool cts_display_on(struct cts_device *cts_dev, bool on)
{
    u8 buf = 0x55;
    u16 addr;
    int retry = 4;
    int ret;

    while (--retry > 0) {
        addr = on ? CTS_DEVICE_FW_REG_SCREEN_ON1
                 : CTS_DEVICE_FW_REG_SCREEN_OFF1;
        ret = cts_fw_reg_writeb(cts_dev, addr, buf);
        if (ret)
            continue;
        mdelay(10);

        addr = on ? CTS_DEVICE_FW_REG_SCREEN_ON2
                 : CTS_DEVICE_FW_REG_SCREEN_OFF2;
        ret = cts_fw_reg_writeb(cts_dev, addr, buf);
        if (ret)
            continue;
        mdelay(10);

        buf = on ? 0x2A : 0x22;
        ret = cts_fw_reg_writeb(cts_dev, CTS_DEVICE_FW_REG_SCREEN, buf);
        if (ret)
            continue;
        break;
    }

    if (retry == 0) {
        cts_err("retry too much times, display %s failed", on ? "on" : "off");
        return false;
    }

    return true;
}
void cts_ts_register_productinfo(struct chipone_ts_data *cts_data)
{
	/* format as flow: version:0x01 Module id:0x57 */
	char deviceinfo[64];
	u16 cust_version =0;
	u8 modid = 0;
	int ret = 0;

	cts_get_firmware_version(&cts_data->cts_dev, &cust_version);

	ret = cts_fw_reg_readb_retry(&cts_data->cts_dev,
	          CTS_DEVICE_FW_REG_MODULE_ID, &modid, 5, 0);
	if (ret) {
		cts_err("Read device module id failed %d\n", ret);
		modid = 0;
	} else {
		cts_info("Device module id: %02x", modid);
	}
	snprintf(deviceinfo, ARRAY_SIZE(deviceinfo), "FW version:0x%04x Module id:0x%02x",
			cust_version, modid);
	productinfo_register(PRODUCTINFO_CTP_ID, NULL, deviceinfo);
}
int cts_get_fw_filename(struct chipone_ts_data *data, char *buf, int size)
{
	const char *str = "NULL";
	char tmp[16] = {0};

	strlcpy(buf, "/vendor/firmware/fw_", size);
	strlcat(buf, CONFIG_HISENSE_PRODUCT_NAME, size);
	if (!strcmp(str, data->pdata->factory_info)) {
		strlcat(buf, "_chipone.bin", size);
	} else {
		strlcat(buf, "_", size);
		strlcat(buf, data->pdata->factory_info, size);
		snprintf(tmp, ARRAY_SIZE(tmp), "_0x%02x", data->cts_dev.fwdata.modid);
		strlcat(buf, tmp, size);
		strlcat(buf, ".bin", size);
	}
	return 0;
}
static int factory_get_fw_update_progress(struct device *dev)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

	return cts_data->cts_dev.rtdata.updating? FW_IS_UPDATETING : FW_UPGRADE_SUCCESS;
}

static int factory_proc_fw_bin_update(struct device  *dev, const char *buf)
{
	int ret = 0;
	unsigned int len;
	char fw_path[256];
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
	if (cts_data->cts_dev.rtdata.suspended) {
		printk("%s Chipone tddi IC enter suspend.\n", __func__);
		return -EPERM;
	}
	/* Get firmware path */
	strlcpy(fw_path, buf, sizeof(fw_path));
	len = strlen(fw_path);
	if (len > 0) {
		if (fw_path[len-1] == '\n')
			fw_path[len-1] = '\0';
	}

	printk("%s: Chipone_tddi fw_path = %s\n", __func__, fw_path);

	mutex_lock(&cts_data->cts_dev.pdata->ts_input_dev->mutex);
	if (!cts_data->cts_dev.rtdata.updating) {
		cts_stop_device(&cts_data->cts_dev);
		ret = cts_update_firmware_from_file(&cts_data->cts_dev, fw_path, true);
		cts_start_device(&cts_data->cts_dev);
		if (ret < 0) {
			cts_err(" %s:upgrade failed.\n", __func__);
		}
	}
	mutex_unlock(&cts_data->cts_dev.pdata->ts_input_dev->mutex);
	cts_ts_register_productinfo(cts_data);

	return ret;
}

static int factory_get_fs_fw_version(struct device *dev, char *buf)
{
	int fw_ver = 0;
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

	fw_ver = cts_data->buildin_fw_version;

	return snprintf(buf, CTS_VERSION_SIZE, "0x%04X\n", fw_ver);
}
static int factory_get_ic_fw_version(struct device *dev, char *buf)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
	u16 fw_version;
	int ret = 0;

	ret = cts_get_firmware_version(&cts_data->cts_dev, &fw_version);
	if(ret < 0){
		printk("%s read cts version failed with ret = %d.\n", __func__, ret);
	} else {
		printk("%s Chipone_tddi version: %04x.\n", __func__, fw_version);
		return sprintf(buf, "0x%04X\n", fw_version);
	}

	return 0;
}
static int factory_get_module_id(struct device *dev, char *buf)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

	return sprintf(buf, "0x%02X\n", cts_data->cts_dev.fwdata.modid);
}
static int factory_get_chip_type(struct device *dev, char *buf)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "chipone %s\n",
			cts_data->cts_dev.hwdata ? cts_data->cts_dev.hwdata->name : "Unknown");
}

static int factory_get_factory_info(struct device *dev, char *buf)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%s\n", cts_data->pdata->factory_info);
}

static int factory_get_chip_settings(struct device *dev, char *buf)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
	char temp[400]= {0};
	char temp1[CTS_VERSION_SIZE] = {0};
	char firmware_name[128] = {0};
	int fw_ver = 0;
	const struct cts_firmware *firmware;

	cts_info("enter");
	strlcpy(temp, "\n### chipone icnl9911c ###\n", sizeof(temp));
	cts_get_fw_filename(cts_data, firmware_name, sizeof(firmware_name));
	strlcat(temp, firmware_name, sizeof(temp));
	strlcat(temp, "\n", sizeof(temp));
	firmware = cts_request_firmware_from_fs(firmware_name);
	if (firmware == NULL) {
		cts_err("Request firmware from %s failed", firmware_name);
		return -EAGAIN;
	}

	fw_ver = FIRMWARE_VERSION(firmware);
	snprintf(temp1, ARRAY_SIZE(temp1), "IC_FW_version=0x%02X\nIC_host_version=", fw_ver);
	strlcat(temp, temp1, sizeof(temp));
	factory_get_fs_fw_version(dev, temp1);
	strlcat(temp, temp1, sizeof(temp));
	snprintf(temp1, ARRAY_SIZE(temp1),"moduleid=0x%02x\n"\
		"reset_gpio=%d, irq_gpio=%d\n"\
		"x=%d, y=%d\n",
		cts_data->cts_dev.fwdata.modid,
		cts_data->cts_dev.pdata->rst_gpio,cts_data->cts_dev.pdata->int_gpio,
		cts_data->cts_dev.pdata->res_x, cts_data->cts_dev.pdata->res_y);
	strlcat(temp, temp1, sizeof(temp));
	strcpy(buf,temp);
	cts_info(" leave");
	return snprintf(buf, PAGE_SIZE, "%s\n\n", buf);
}

static int factory_proc_hibernate_test(struct device *dev)
{
	int err = 0;
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_test_param test_param = {
        .test_item = CTS_TEST_RESET_PIN,
        .flags = 0,
    };

	err = cts_test_reset_pin(&cts_data->cts_dev, &test_param);
	if (err) {
		dev_err(dev, "CTS %s: Chipone_tddi reset test failed.\n", __func__);
		return 0;
	}else{
		printk("CTS %s Chipone_tddi reset test success!\n", __func__);
		return 1;
	}
}
static int factory_get_short_test(struct device *dev, char *buf)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
	struct cts_device *cts_dev = &cts_data->cts_dev;
	int short_min = 500;
	int short_test_result = 0;

    struct cts_test_param short_test_param = {
        .test_item = CTS_TEST_SHORT,
        .flags = CTS_TEST_FLAG_VALIDATE_DATA |
                 CTS_TEST_FLAG_VALIDATE_MIN |
                 CTS_TEST_FLAG_STOP_TEST_IF_VALIDATE_FAILED |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_CONSOLE |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_FILE,
        .test_data_filepath = NULL,
        .num_invalid_node = 0,
        .invalid_nodes = NULL,
    };
	short_test_param.min = &short_min;
	short_test_result = cts_test_short(cts_dev, &short_test_param);
	if (short_test_result) {
		cts_err("Chipone_tddi_short_test failed!\n");
	}
	return !short_test_result;
}

static int factory_get_open_test(struct device *dev, char *buf)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
	struct cts_device *cts_dev = &cts_data->cts_dev;
	int open_min = 500;
	int open_test_result = 0;
    struct cts_test_param open_test_param = {
        .test_item = CTS_TEST_OPEN,
        .flags = CTS_TEST_FLAG_VALIDATE_DATA |
                 CTS_TEST_FLAG_VALIDATE_MIN |
                 CTS_TEST_FLAG_STOP_TEST_IF_VALIDATE_FAILED |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_CONSOLE |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_FILE,
        .test_data_filepath = NULL,
        .num_invalid_node = 0,
        .invalid_nodes = NULL,
    };
	open_test_param.min = &open_min;
	open_test_result = cts_test_open(cts_dev, &open_test_param);
	if (open_test_result) {
		cts_err("Chipone_tddi_open_test failed!\n");
	}
	return !open_test_result;
}

int factory_get_rawdata(struct device *dev, char *buf)
{
	int ret;
	int i, j, num_read_chars = 0;
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
	u16 *rawdata = NULL;
	bool data_valid = true;

	rawdata = (u16 *)kmalloc(cts_data->cts_dev.fwdata.rows *
								cts_data->cts_dev.fwdata.cols * 2, GFP_KERNEL);
	if (rawdata == NULL) {
		cts_err("Allocate memory for rawdata failed.\n");
		return -ENOMEM;
	}

	ret = cts_send_command(&cts_data->cts_dev, CTS_CMD_QUIT_GESTURE_MONITOR);
	if (ret) {
		cts_err("Send cmd QUIT_GESTURE_MONITOR failed %d\n", ret);
		goto err_free_rawdata;
	}
	msleep(30);

	ret = cts_enable_get_rawdata(&cts_data->cts_dev);
	if (ret) {
		cts_err("Chipone_tddi Enable read raw data failed %d\n", ret);
		goto err_free_rawdata;
	}

	mdelay(1);
	ret = cts_get_rawdata(&cts_data->cts_dev, rawdata);
	if (ret) {
		cts_err("Chipone_tddi Get raw data failed %d\n", ret);
		data_valid = false;
	}

	printk("Chipone_tddi rows = %d,cols = %d\n", cts_data->cts_dev.fwdata.rows, cts_data->cts_dev.fwdata.cols);
	ret = cts_disable_get_rawdata(&cts_data->cts_dev);
	if (ret) {
		cts_err("Chipone_tddi Disable read raw data failed %d\n", ret);
		goto err_free_rawdata;
	}

	if (data_valid) {
		for (i = 0; i < cts_data->cts_dev.fwdata.rows; i++) {
			for (j = 0; j < cts_data->cts_dev.fwdata.cols; j++) {
				num_read_chars += sprintf(&(buf[num_read_chars]),"%u ",
					rawdata[i*cts_data->cts_dev.fwdata.cols + j]);
			}
			buf[num_read_chars++] = '\n';
		}
		buf[num_read_chars-1] = '\n';
	}

	err_free_rawdata:
		kfree(rawdata);

	return (data_valid ? num_read_chars : ret);
}

static int factory_get_rawdata_info(struct device *dev, char *buf)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
	char  row, column;

	row = cts_data->cts_dev.fwdata.rows;
	column = cts_data->cts_dev.fwdata.cols;

	return sprintf(buf, "RX:%d TX:%d HIGH:%d LOW:%d\n", row, column, 0, 0);
}

static int factory_get_diff(struct device *dev, char *buf)
{
	int ret;
	int i, j, num_read_chars = 0;
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
	s16 *diffdata = NULL;
	bool data_valid = true;

	diffdata = (s16 *)kmalloc(cts_data->cts_dev.fwdata.rows *
								cts_data->cts_dev.fwdata.cols * 2, GFP_KERNEL);
	if (diffdata == NULL) {
		cts_err("Allocate memory for diffdata failed.\n");
		return -ENOMEM;
	}

	ret = cts_send_command(&cts_data->cts_dev, CTS_CMD_QUIT_GESTURE_MONITOR);
	if (ret) {
		cts_err("Send cmd QUIT_GESTURE_MONITOR failed %d\n", ret);
		goto err_free_diffdata;
	}
	msleep(30);

	ret = cts_enable_get_rawdata(&cts_data->cts_dev);
	if (ret) {
		cts_err("Chipone_tddi Enable read diff data failed %d\n", ret);
		goto err_free_diffdata;
	}

	mdelay(1);
	ret = cts_get_diffdata(&cts_data->cts_dev, diffdata);
	if (ret) {
		cts_err("Chipone_tddi Get diff data failed %d\n", ret);
		data_valid = false;
	}

	ret = cts_disable_get_rawdata(&cts_data->cts_dev);
	if (ret) {
		cts_err("Chipone_tddi Disable read diff data failed %d\n", ret);
		goto err_free_diffdata;
	}

	if (data_valid) {
		for (i = 0; i < cts_data->cts_dev.fwdata.rows; i++) {
			for (j = 0; j < cts_data->cts_dev.fwdata.cols; j++) {
				num_read_chars += sprintf(&(buf[num_read_chars]),"%5d ",
					diffdata[i*cts_data->cts_dev.fwdata.cols + j]);
			}
			buf[num_read_chars++] = '\n';
		}
		buf[num_read_chars-1] = '\n';
	}

	err_free_diffdata:
		kfree(diffdata);

	return (data_valid ? num_read_chars : ret);
}

static bool get_tp_enable_switch(struct device *dev)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

	return cts_data->cts_dev.enabled;
}

static int set_tp_enable_switch(struct device *dev, bool enable)
{
	int retval = 0;
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
	static bool is_the_first_set = 1;
#if defined(CONFIG_TOUCHSCREEN_CHIPONE_GESTURE)
	static bool gesture_switch;
#endif

	if (is_the_first_set) {
#if defined(CONFIG_TOUCHSCREEN_CHIPONE_GESTURE)
		gesture_switch = cts_data->cts_dev.rtdata.gesture_wakeup_enabled;
#endif
		is_the_first_set = 0;
	}

	if (enable) {
#if defined(CONFIG_TOUCHSCREEN_CHIPONE_GESTURE)
		cts_data->cts_dev.rtdata.gesture_wakeup_enabled = gesture_switch;
#endif
		retval = cts_start_device(&cts_data->cts_dev);
		if (retval)
			cts_err("failed to enter active power mode");
	} else {
#if defined(CONFIG_TOUCHSCREEN_CHIPONE_GESTURE)
		gesture_switch = cts_data->cts_dev.rtdata.gesture_wakeup_enabled;
		cts_data->cts_dev.rtdata.gesture_wakeup_enabled = 0;
#endif
		retval = cts_stop_device(&cts_data->cts_dev);
		if (retval)
			cts_err("failed to enter low power mode");
	}

	return retval;
}
#ifdef CONFIG_TOUCHSCREEN_CHIPONE_GESTURE
static unsigned int asic_to_hex(unsigned char val)
{
	if ((val >= '0') && (val <= '9'))
		val -= '0';
	else if ((val >= 'a') && (val <= 'z'))
		val = val - 'a' + 10;
	else if ((val >= 'A') && (val <= 'Z'))
		val = val - 'A' + 10;

	return (unsigned int)val;
}

bool get_gesture_switch(struct device *dev)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
	bool ret;

	ret = cts_data->cts_dev.rtdata.gesture_wakeup_enabled;

	return ret;
}

static int set_gesture_switch(struct device *dev, const char *buf)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
	unsigned char gesture[10], len;
	strlcpy(gesture, buf, sizeof(gesture));
	len = strlen(gesture);
	if (len > 0) {
		if ((gesture[len-1] == '\n') || (gesture[len-1] == '\0'))
			len--;
	}
	cts_info("Chipone_tddi set gesture state len: %d gesture_state: %d,%d,%d",
		len, gesture[0], gesture[1], gesture[2]);
	if (len == 1) {
		if (gesture[0] == '1')
			cts_data->gesture_state = 0xffff;
		else if (gesture[0] == '0')
			cts_data->gesture_state = 0x0;
	} else if (len == 4) {
		cts_data->gesture_state = asic_to_hex(gesture[0])*0x1000
		   + asic_to_hex(gesture[1]) * 0x100
		   + asic_to_hex(gesture[2]) * 0x10
		   + asic_to_hex(gesture[3]);
	} else {
		cts_err("Set gesture switch write wrong cmd");
		return 0;
	}
	if (cts_data->gesture_state)
		cts_enable_gesture_wakeup(&cts_data->cts_dev);
	else
		cts_disable_gesture_wakeup(&cts_data->cts_dev);
	cts_info("Gesture state %04x", cts_data->gesture_state);
	return 0;
}
#endif
#ifdef CONFIG_TOUCHSCREEN_CHIPONE_GLOVE
static bool get_glove_switch(struct device *dev)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

	printk("%s: Chipone_tddi glove_enable = %d\n", __func__, cts_data->cts_dev.rtdata.glove_enable);
	return cts_data->cts_dev.rtdata.glove_enable;
}

static int set_glove_switch(struct device *dev, bool enable)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

	cts_data->cts_dev.rtdata.glove_enable = enable;
	if (cts_data->cts_dev.rtdata.glove_enable)
		cts_enter_glove_mode(&cts_data->cts_dev);
	else
		cts_exit_glove_mode(&cts_data->cts_dev);

	return 0;
}
#endif

#ifdef CONFIG_TOUCHSCREEN_WORK_MODE
static int get_tp_work_mode(struct device *dev, char *buf)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
	u8  work_mode, pwr_mode;
	int ret;

	cts_info("Chipone tddi get TP_WORK_MODE");

	if (!cts_data->cts_dev.enabled) {
		return snprintf(buf, PAGE_SIZE, "IDLE\n");
	} else {
		ret = cts_get_work_mode(&cts_data->cts_dev, &work_mode);
		if (ret) {
			return snprintf(buf, PAGE_SIZE, "READ MODE ERROR\n");
		}

		if (work_mode != CTS_FIRMWARE_WORK_MODE_NORMAL) {
			return snprintf(buf, PAGE_SIZE, "IDLE\n");
		} else {
			ret = cts_fw_reg_readb(&cts_data->cts_dev, CTS_DEVICE_FW_REG_POWER_MODE, &pwr_mode);
			if (ret) {
				return snprintf(buf, PAGE_SIZE, "READ MODE ERROR\n");
			}

			if (pwr_mode == 0) {
				return snprintf(buf, PAGE_SIZE, "OPERATING\n");
			} else if (pwr_mode == 1) {
				return snprintf(buf, PAGE_SIZE, "MONITOR\n");
			} else {
				return snprintf(buf, PAGE_SIZE, "IDLE\n");
			}
		}
	}
}

static int set_tp_work_mode(struct device *dev, const char *mode)
{
	struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
	int ret;

	cts_info("Set TP_WORK_MODE to '%s'", mode);

	if (strncmp(mode, "IDLE", 4) == 0) {
		ret = cts_set_work_mode(&cts_data->cts_dev, CTS_FIRMWARE_WORK_MODE_CONFIG);
		if (ret) {
			cts_err("Set work mode to IDLE failed %d", ret);

			return ret;
		}
	} else if (strncmp(mode, "OPERATING", 9) == 0) {
		ret = cts_fw_disable_monitor_mode(&cts_data->cts_dev);
		if (ret) {
			cts_err("Disable monitor mode failed %d", ret);

			return ret;
		}

		ret = cts_fw_reg_writeb(&cts_data->cts_dev, CTS_DEVICE_FW_REG_POWER_MODE, 0);
		if (ret) {
			cts_err("Set power mode to active failed %d", ret);

			return ret;
		}

		ret = cts_set_work_mode(&cts_data->cts_dev, CTS_FIRMWARE_WORK_MODE_NORMAL);
		if (ret) {
			cts_err("Set work mode to OPERATING failed %d", ret);

			return ret;
		}
	} else if (strncmp(mode, "MONITOR", 7) == 0) {
		ret = cts_set_monitor_mode_count(&cts_data->cts_dev, 1);
		if (ret) {
			cts_err("Set monitor mode counter failed %d", ret);

			return ret;
		}

		ret = cts_fw_enable_monitor_mode(&cts_data->cts_dev);
		if (ret) {
			cts_err("Set work mode to MONITOR failed %d", ret);

			return ret;
		}
	} else {
		cts_err("Set mode to '%s' INVALID", mode);

		return -EINVAL;
	}

	return 0;
}
#endif

#if defined(CONFIG_TOUCHSCREEN_CHIPONE_INCELL_CHIP)
static int cts_ts_suspend_for_lcd_async_use(struct device *dev)
{
	struct chipone_ts_data *cts_data;

	cts_data = dev_get_drvdata(dev);
	if ((cts_data == NULL) || (!cts_probed))
		return -ENODEV;

	cancel_work_sync(&cts_data->resume_work);
	return cts_driver_suspend(cts_data);
}

static int cts_ts_resume_for_lcd_async_use(struct device *dev)
{
	struct chipone_ts_data *cts_data;

	cts_data = dev_get_drvdata(dev);
	if ((cts_data == NULL) || (!cts_probed))
		return -ENODEV;

	if (!work_pending(&cts_data->resume_work))
		schedule_work(&cts_data->resume_work);

	return 0;
}

static int cts_ts_provide_reset_control(struct device *dev)
{
	struct chipone_ts_data *cts_data;

	cts_data = dev_get_drvdata(dev);
	if ((cts_data == NULL) || (!cts_probed))
		return -ENODEV;

	gpio_set_value(cts_data->pdata->rst_gpio, 0);
	msleep(5);
	gpio_set_value(cts_data->pdata->rst_gpio, 1);

	return 0;
}
#endif
#if defined(CONFIG_TOUCHSCREEN_CHIPONE_INCELL_CHIP) && defined(CONFIG_TOUCHSCREEN_CHIPONE_GESTURE)
int cts_need_lcd_power_reset_keep_flag_get(struct device *dev)
{
	struct chipone_ts_data *cts_data;

	cts_data = dev_get_drvdata(dev);
	if ((cts_data == NULL) || (!cts_probed))
		return -ENODEV;

	return cts_data->cts_dev.rtdata.gesture_wakeup_enabled;
}
#endif

int factory_ts_func_test_register(struct chipone_ts_data* data)
{
	ts_gen_func_test_init();
	data->ts_test_dev.dev = data->device;

	data->ts_test_dev.get_fw_update_progress = factory_get_fw_update_progress;
#if 0	
	data->ts_test_dev.proc_fw_update = factory_proc_fw_update;
#endif
	data->ts_test_dev.proc_fw_update_with_given_file = factory_proc_fw_bin_update;
	data->ts_test_dev.get_fs_fw_version = factory_get_fs_fw_version;
	data->ts_test_dev.get_ic_fw_version = factory_get_ic_fw_version;
	data->ts_test_dev.get_module_id = factory_get_module_id;
	data->ts_test_dev.get_chip_type = factory_get_chip_type;
	data->ts_test_dev.get_factory_info = factory_get_factory_info;
	data->ts_test_dev.proc_hibernate_test = factory_proc_hibernate_test;
	data->ts_test_dev.get_short_test = factory_get_short_test;
	data->ts_test_dev.get_open_test = factory_get_open_test;
	data->ts_test_dev.get_rawdata = factory_get_rawdata;
	data->ts_test_dev.get_rawdata_info = factory_get_rawdata_info;
	data->ts_test_dev.get_diff = factory_get_diff;
	data->ts_test_dev.get_tp_settings_info = factory_get_chip_settings;
	data->ts_test_dev.get_tp_enable_switch = get_tp_enable_switch;
	data->ts_test_dev.set_tp_enable_switch = set_tp_enable_switch;
#if 0
	data->ts_test_dev.set_erase_flash_test = cts_set_erase_flash_test;
#endif
#if defined(CONFIG_TOUCHSCREEN_CHIPONE_GESTURE)
	data->ts_test_dev.get_gesture_switch = get_gesture_switch;
	data->ts_test_dev.set_gesture_switch = set_gesture_switch;
#endif

#if defined(CONFIG_TOUCHSCREEN_CHIPONE_GLOVE)
	data->ts_test_dev.get_glove_switch = get_glove_switch;
	data->ts_test_dev.set_glove_switch = set_glove_switch;
#endif
#ifdef CONFIG_TOUCHSCREEN_WORK_MODE
		data->ts_test_dev.get_tp_work_mode = get_tp_work_mode;
		data->ts_test_dev.set_tp_work_mode = set_tp_work_mode;
#endif
	#if defined(CONFIG_TOUCHSCREEN_CHIPONE_INCELL_CHIP)
	data->ts_test_dev.ts_async_suspend_for_lcd_use = cts_ts_suspend_for_lcd_async_use;
	data->ts_test_dev.ts_async_resume_for_lcd_use = cts_ts_resume_for_lcd_async_use;
	data->ts_test_dev.ts_reset_for_lcd_use = cts_ts_provide_reset_control;
#endif
#if defined(CONFIG_TOUCHSCREEN_CHIPONE_INCELL_CHIP) && defined(CONFIG_TOUCHSCREEN_CHIPONE_GESTURE)
	data->ts_test_dev.ts_suspend_need_lcd_power_reset_high = cts_need_lcd_power_reset_keep_flag_get;
#endif
#if 0
	data->ts_test_dev.check_fw_update_need = factory_check_fw_update_need;
	//data->ts_test_dev.get_calibration_ret = factory_get_calibration_ret;
#endif
	register_ts_func_test_device(&data->ts_test_dev);
	return 0;
}


