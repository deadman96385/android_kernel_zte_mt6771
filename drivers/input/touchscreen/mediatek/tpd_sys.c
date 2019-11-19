/***********************
 * file : tpd_fw.c
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/sched.h>
#include "tpd_sys.h"
#include "tpd.h"
//#include "tpd_common.h"

//#define TPD_SYS_DMESG(a, arg...) pr_info("tpd" ": " a, ##arg)


DECLARE_RWSEM(tp_firmware_list_lock);
LIST_HEAD(tp_firmware_list);

#define MAX_BUF_SIZE 256 * 1024
#define VENDOR_END 0xff

struct tpvendor_t synaptics_vendor_l[] ={
	{0x31, "TPK"},
	{0x32, "Truly"},
	{0x33, "Success"},
	{0x34, "Ofilm"},
	{0x35, "Lead"},
	{0x36, "Wintek"},
	{0x37, "Laibao"},
	{0x38, "CMI"},
	{0x39, "Ecw"},
	{0x41, "Goworld"},
	{0x42, "BaoMing"},
	{0x43, "Eachopto"},
	{0x44, "Mutto"},
	{0x45, "Junda"},
	{0x46, "Tdi"},
	{0x52, "Holitech"},
	{VENDOR_END, "Unkown"},
};

struct tpvendor_t focal_vendor_l[] ={
	{0x11, "TeMeiKe"},
	{0x51, "Ofilm"},
	{0x55, "LaiBao"},
	{0x57, "Goworld"},
	{0x5a, "Truly"},
	{0x5c, "TPK"},
	{0x5d, "BaoMing"},
	{0x5f, "Success"},
	{0x60, "Lead"},
	{0x67, "DiJing"},
	{0x80, "Eachopto"},
	{0x82, "HeLiTai"},
	{0x85, "JunDa"},
	{0x87, "LianChuang"},
	{0xda, "DiJing"},
	{VENDOR_END, "Unkown"},
};

struct tpvendor_t cypress_vendor_l[] ={
	{0x01, "TPK"},
	{0x02, "Truly"},
	{0x03, "Success"},
	{0x04, "Ofilm"},
	{0x05, "Lead"},
	{0x06, "Wintek"},
	{0x07, "Laibao"},
	{0x08, "CMI"},
	{0x09, "Ecw"},
	{0x0a, "Goworld"},
	{0x0b, "BaoMing"},
	{0x0c, "Eachopto"},
	{0x0d, "Mutto"},
	{0x0e, "Junda"},
	{VENDOR_END, "Unkown"},
};

struct tpvendor_t atmel_vendor_l[] ={
	{0x06, "Wintek"},
	{0x07, "Laibao"},
	{0x08, "CMI"},
	{0x09, "Ecw"},
	{VENDOR_END, "Unkown"},
};

struct tpvendor_t goodix_vendor_l[] ={
	{0x00, "Eachopto"},
	{0x01, "Success"},
	{0x02, "TPK"},
	{0x03, "BaoMing"},
	{0x04, "Ofilm"},
	{0x05, "Truly"},
	{0x06, "Wintek"},
	{0x07, "Laibao"},
	{0x08, "CMI"},
	{0x09, "Ecw"},
	{0x0a, "Goworld"},
	{0x0b, "Lead"},
	{0x0c, "TeMeiKe"},
	{0x0d, "Mutto"},
	{0x0e, "Junda"},
	{0x0f, "TianMa"},
	{0x10, "SanXing"},
	{VENDOR_END, "Unkown"},
};

struct tpvendor_t mstar_vendor_l[] ={
	{0x01, "FuNaYuanChuang"},
	{0x02, "TeMeiKe"},
	{VENDOR_END, "Unkown"},
};

struct tpvendor_t melfas_vendor_l[] ={
	{VENDOR_END, "Unkown"},
};

int tpd_power_already_on = 0;
struct tpd_classdev_t tpd_fw_cdev;

static struct class *tsp_fw_class;

//static unsigned char filepath[256];

static int tpd_alloc_buffer(struct tpd_classdev_t *cdev, int alloc_size)
{
	if(cdev->tp_fw.data == NULL) {
		cdev->tp_fw.data = (unsigned char *)vmalloc(alloc_size);
		if(!cdev->tp_fw.data) {
			dev_err(cdev->dev, "memory alloc failed\n");
			goto error;
		}
		cdev->tp_fw.length = alloc_size;
		printk("tpd: %s malloc %d byte success.\n", __func__, alloc_size);
	}

	return 0;
error:
	return -1;
}

static int tpd_free_buffer(struct tpd_classdev_t *cdev)
{
	if(cdev->tp_fw.data != NULL) {
		vfree(cdev->tp_fw.data);
		cdev->tp_fw.data = NULL;
		cdev->tp_fw.data_len = 0;
		cdev->tp_fw.length = 0;
		cdev->status = STATUS_OK;
	}

	return 0;
}

static int get_chip_vendor(struct tpvendor_t * vendor_l, int count, int vendor_id, char *vendor_name)
{
	int i = 0;
	printk("%s: count: %d.\n", __func__, count);

	for(i = 0; i < count; i ++) {
		if(vendor_l[i].vendor_id == vendor_id || VENDOR_END == vendor_l[i].vendor_id) {
			strcpy(vendor_name, vendor_l[i].vendor_name);
			break;
		}
	}

	return 0;
}

static void tpd_get_tp_module_name(struct tpd_classdev_t *cdev)
{
	unsigned int vendor_id = 0;
	int size = 0;
	
	printk("%s \n", __func__);

	vendor_id = cdev->ic_tpinfo.module_id;
	
	if(NULL != strstr(cdev->ic_tpinfo.tp_name, "Synaptics")) {
		size = sizeof(synaptics_vendor_l) / sizeof(struct tpvendor_t);
		get_chip_vendor(synaptics_vendor_l, size, cdev->ic_tpinfo.module_id, cdev->ic_tpinfo.vendor_name);
	} else if (NULL != strstr(cdev->ic_tpinfo.tp_name, "Atmel")) {
		size = sizeof(atmel_vendor_l) / sizeof(struct tpvendor_t);
		get_chip_vendor(atmel_vendor_l, size, cdev->ic_tpinfo.module_id, cdev->ic_tpinfo.vendor_name);
	} else if (NULL != strstr(cdev->ic_tpinfo.tp_name, "Cyttsp")) {
		size = sizeof(cypress_vendor_l) / sizeof(struct tpvendor_t);
		get_chip_vendor(cypress_vendor_l, size, cdev->ic_tpinfo.module_id, cdev->ic_tpinfo.vendor_name);
	} else if (NULL != strstr(cdev->ic_tpinfo.tp_name, "Focal"))	{
		size = sizeof(focal_vendor_l) / sizeof(struct tpvendor_t);
		get_chip_vendor(focal_vendor_l, size, cdev->ic_tpinfo.module_id, cdev->ic_tpinfo.vendor_name);
	} else if (NULL != strstr(cdev->ic_tpinfo.tp_name, "Goodix")) {
		size = sizeof(goodix_vendor_l) / sizeof(struct tpvendor_t);
		get_chip_vendor(goodix_vendor_l, size, cdev->ic_tpinfo.module_id, cdev->ic_tpinfo.vendor_name);
		vendor_id = 0;
	} else if (NULL != strstr(cdev->ic_tpinfo.tp_name, "Melfas")) {
		size = sizeof(melfas_vendor_l) / sizeof(struct tpvendor_t);
		get_chip_vendor(melfas_vendor_l, size, cdev->ic_tpinfo.module_id, cdev->ic_tpinfo.vendor_name);
	} else if (NULL != strstr(cdev->ic_tpinfo.tp_name, "Mstar")) {
		size = sizeof(mstar_vendor_l) / sizeof(struct tpvendor_t);
		get_chip_vendor(mstar_vendor_l, size, cdev->ic_tpinfo.module_id, cdev->ic_tpinfo.vendor_name);
	} else {
		strcpy(cdev->ic_tpinfo.vendor_name, "Unkown.");
	}
	strcpy(cdev->file_tpinfo.vendor_name, cdev->ic_tpinfo.vendor_name);

	printk("fun:%s module name:%s.\n",__func__, cdev->ic_tpinfo.vendor_name);
	
}

static int tpd_upgrade_firmware(struct tpd_classdev_t *cdev, int cmd, int forced)
{
	int retval = 0;

	if(!mutex_trylock(&cdev->upgrade_mutex))
	{
		printk("tpd: %s: Pre func execing.\n", __func__);
		retval = -1;
		goto out;
	}
	
	if(cdev->flash_fw &&( (STATUS_BUF_RDY == cdev->status)||  \
		(STATUS_UPG_FAILED == cdev->status)||(STATUS_UPG_NONEED == cdev->status))) {
		if(NULL == cdev->tp_fw.data || 0 == cdev->tp_fw.data_len) {
			printk("tpd: BUFFER is NULL.\n");
			cdev->status = STATUS_UPG_FAILED;
			retval = -1;
			goto out;
		}
		
		retval = cdev->flash_fw(cdev, cdev->tp_fw.data, cdev->tp_fw.data_len, forced);
		if(retval == 0) {
			cdev->status = STATUS_UPG_SUCC;
		} else if (retval == 0xff) {
			cdev->status = STATUS_UPG_NONEED;
		} else {
			cdev->status = STATUS_UPG_FAILED;
		}
	} else {
		printk("tpd: %s: status:%d\n", __func__, cdev->status);
	}

	mutex_unlock(&cdev->upgrade_mutex);

out:
	return retval;
}

//0: tp file version > ic, need upgrade. other: tp file version <= ic, not upgrade.
static int tpd_start_compare_tpinfo(struct tpd_classdev_t *cdev)
{
	int retval = -1;

	//init file tpinfo.
	strcpy(cdev->file_tpinfo.tp_name, cdev->ic_tpinfo.tp_name);
	cdev->file_tpinfo.chip_model_id = cdev->ic_tpinfo.chip_model_id;
	cdev->file_tpinfo.chip_part_id = cdev->ic_tpinfo.chip_part_id;
	cdev->file_tpinfo.chip_ver = cdev->ic_tpinfo.chip_ver;
	cdev->file_tpinfo.config_ver = cdev->ic_tpinfo.config_ver;
	cdev->file_tpinfo.firmware_ver = cdev->ic_tpinfo.firmware_ver;
	cdev->file_tpinfo.module_id = cdev->ic_tpinfo.module_id;
	cdev->fw_compare_result = 1;

	if(cdev->compare_tp_version) {
		retval = cdev->compare_tp_version(cdev, cdev->tp_fw.data);
		if(0 == retval) {
			cdev->fw_compare_result = 0;
		} else {
			cdev->fw_compare_result = 1;
		}
		printk("%s  result:%d\ntpd  0: tp file version > ic, need upgrade. other: tp file version <= ic, not upgrade.\n",
			__func__, cdev->fw_compare_result );
	}
		
	return 0;
}

static ssize_t tsp_cmd_show(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct tpd_classdev_t *tsp_fw_cdev = dev_get_drvdata(dev);

	return sprintf(buf, "Current cmd is 0x%x.\n", tsp_fw_cdev->cmd);
}

static ssize_t tsp_cmd_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	int cmd = 0, param = 0;

	mutex_lock(&cdev->cmd_mutex);
	sscanf(buf, "%d %d", &cmd, &param);

	printk("tpd: %s command:%d, param:%d. \n", __func__, cmd, param);
	
	cdev->cmd = cmd;
	switch(cmd) {
	case CMD_WRITE_IMAGE:
		cdev->status = STATUS_BUF_NULL;
		tpd_free_buffer(cdev);
		break;
	case CMD_WRITE_FINISH:
		if(cdev->tp_fw.data_len == param) {
			cdev->status = STATUS_BUF_RDY;
		} else {
			printk("tpd: %s error: not entire content of image. \n", __func__);
		}
		printk("tpd: %s image write 0x%x bytes, should 0x%x bytes.\n ", __func__, cdev->tp_fw.data_len, param);
		break;
	case CMD_READ_IMAGE:

		break;
	case CMD_PORCE_UPG:
		tpd_upgrade_firmware(cdev, cmd, 1);
		break;
	case CMD_PERFORM_UPG:
		tpd_upgrade_firmware(cdev, cmd, 0);
		break;
	case CMD_CLEAN_BUF:
		tpd_free_buffer(cdev);
		break;
	case CMD_SET_PART_ID:
		cdev->ic_tpinfo.chip_part_id = param;
		break;
	case CMD_COMPARE_FIRMWARE:
		tpd_start_compare_tpinfo(cdev);
		break;
	default:
		printk("tpd: %s Not support this cmd:0x%x\n", __func__, cmd);
		break;
	}
	mutex_unlock(&cdev->cmd_mutex);

	return size;
}
static ssize_t tsp_upg_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);

	memcpy(buf, &cdev->status, sizeof(int));

	return sizeof(int);
}
static ssize_t tsp_upg_status_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	//struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	
	//memcpy(buf, &cdev->status, sizeof(int));

	return sizeof(int);
}
static ssize_t tsp_fw_ic_tpinfo_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);

	if(cdev->get_tpinfo) {
		cdev->get_tpinfo(cdev);
	}

	return sprintf(buf, "%u 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x %s\n",
		cdev->ic_tpinfo.chip_part_id,   cdev->ic_tpinfo.chip_model_id,
		cdev->ic_tpinfo.chip_ver,        cdev->ic_tpinfo.module_id, 
		cdev->ic_tpinfo.firmware_ver, cdev->ic_tpinfo.config_ver, 
		cdev->ic_tpinfo.i2c_type,        cdev->ic_tpinfo.i2c_addr, 
		cdev->ic_tpinfo.tp_name);
}
static ssize_t tsp_fw_ic_tpinfo_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	return size;
}

static ssize_t tsp_get_bsg_value_show(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	int retval = 1;

	mutex_lock(&cdev->cmd_mutex);
	if(cdev->get_gesture) {
		retval = cdev->get_gesture(cdev);
	}
	printk("tpd: %s val:%d.\n", __func__, retval);
	//memcpy(buf, &retval, sizeof(int));
	retval = sprintf(buf, "0x%02x\n", retval);
	mutex_unlock(&cdev->cmd_mutex);
	
	//retval = sizeof(int);
	return retval;
}
static ssize_t tsp_get_bsg_value_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	return size;
}

static ssize_t tsp_gesture_enable_show(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	int retval = 0;

	mutex_lock(&cdev->cmd_mutex);
	printk("tpd: %s val:%d.\n", __func__, cdev->b_gesture_enable);

	//memcpy(buf, &cdev->b_gesture_enable, sizeof(int));
	retval = sprintf(buf, "0x%02x\n", cdev->b_gesture_enable);
	mutex_unlock(&cdev->cmd_mutex);

	//retval = sizeof(int);
	return retval;
}

static ssize_t tsp_gesture_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	int ret = -1;
	char *after;
	unsigned long enable = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	if (isspace(*after))
		count++;

	if (count == size) {
		ret = count;
	}
	
	printk("tpd: %s val %ld.\n", __func__, enable);

	mutex_lock(&cdev->cmd_mutex);
	if(cdev->wake_gesture) {
		cdev->wake_gesture(cdev, enable);
	}
	cdev->b_gesture_enable = enable;
	mutex_unlock(&cdev->cmd_mutex);

	return size;
}
static ssize_t tsp_fw_file_tpinfo_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	
	return sprintf(buf, "%u 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x %s\n",
		cdev->file_tpinfo.chip_part_id,   cdev->file_tpinfo.chip_model_id,
		cdev->file_tpinfo.chip_ver,        cdev->file_tpinfo.module_id, 
		cdev->file_tpinfo.firmware_ver, cdev->file_tpinfo.config_ver, 
		cdev->file_tpinfo.i2c_type,        cdev->file_tpinfo.i2c_addr, 
		cdev->file_tpinfo.tp_name);
}

static ssize_t tsp_ic_tpinfo_show_for_pv(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);

	if(cdev->get_tpinfo) {
		cdev->get_tpinfo(cdev);
	}
	tpd_get_tp_module_name(cdev);
	
	return sprintf(buf, "%s%u config version:0x%x\nManufacturer:%s firmware_version:0x%x\nLcm:%s\n", 
		cdev->ic_tpinfo.tp_name, cdev->ic_tpinfo.chip_part_id, 
		cdev->ic_tpinfo.config_ver, cdev->ic_tpinfo.vendor_name, cdev->ic_tpinfo.firmware_ver, cdev->lcm_info);
}

static ssize_t tsp_file_tpinfo_show_for_pv(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	tpd_get_tp_module_name(cdev);
	
	return sprintf(buf, "%s%u config version:0x%x\nManufacturer:%s firmware_version:0x%x\nLcm:%s\n", 
		cdev->ic_tpinfo.tp_name, cdev->ic_tpinfo.chip_part_id, 
		cdev->ic_tpinfo.config_ver, cdev->ic_tpinfo.vendor_name, cdev->ic_tpinfo.firmware_ver, cdev->lcm_info);
}

//0: tp file version > ic, need upgrade. 1: tp file version <= ic, not upgrade.
static ssize_t tsp_tpinfo_compare_result_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	unsigned int compare_ret;
	
	if(cdev->fw_compare_result == 0) {
		compare_ret = 0;
	} else {
		compare_ret = 1;
	}

	return sprintf(buf, "%u\n",compare_ret);
}

static unsigned char *i2c_test_buffer = NULL;
static unsigned int i2c_test_size = 0;

static ssize_t tsp_i2c_test_show(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	int retval = i2c_test_size;
	char i2c_test_buf1[64], i2c_test_buf2[64];
	
	snprintf(i2c_test_buf1, 64, "%u 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x %s\n", cdev->ic_tpinfo.chip_part_id, 
		cdev->ic_tpinfo.chip_ver, cdev->ic_tpinfo.module_id, cdev->ic_tpinfo.firmware_ver, cdev->ic_tpinfo.config_ver, 
		cdev->ic_tpinfo.i2c_type, cdev->ic_tpinfo.i2c_addr, 
		cdev->ic_tpinfo.tp_name);
	if(cdev->get_tpinfo) {
		cdev->get_tpinfo(cdev);
	}

	snprintf(i2c_test_buf2, 64, "%u 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x %s\n", cdev->ic_tpinfo.chip_part_id, 
		cdev->ic_tpinfo.chip_ver, cdev->ic_tpinfo.module_id, cdev->ic_tpinfo.firmware_ver, cdev->ic_tpinfo.config_ver, 
		cdev->ic_tpinfo.i2c_type, cdev->ic_tpinfo.i2c_addr, 
		cdev->ic_tpinfo.tp_name);
	if(strncmp(i2c_test_buf1, i2c_test_buf2, 64) == 0 ) {
		memcpy(buf, i2c_test_buffer, i2c_test_size);
	} else {
		memset(i2c_test_buffer, 0, i2c_test_size);
		memcpy(buf, i2c_test_buffer, i2c_test_size);
	}
	printk("tpd %s", i2c_test_buf1);
	printk("tpd %s", i2c_test_buf2);

	printk("tpd: %s read val:0x%x.\n", __func__, buf[0]);

	retval = i2c_test_size;

	return retval;
}

static ssize_t tsp_i2c_test_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	//struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	//printk("tpd: %s write %d byte.\n", __func__, size);

	if(i2c_test_buffer == NULL) {
		i2c_test_buffer = (unsigned char *)vmalloc(256);
		printk("tpd i2c_test_buffer already vmalloc.\n");
	}
	memcpy(i2c_test_buffer, buf, size > 256 ? 256 : size);
	i2c_test_size = size > 256 ? 256 : size;

	printk("tpd: %s write val:0x%x.\n", __func__, i2c_test_buffer[0]);

	return size > 256 ? 256 : size;
}

static ssize_t tsp_lcminfo_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);

	return sprintf(buf, "%s", cdev->lcm_info);
}
static DEVICE_ATTR(cmd, 0644, tsp_cmd_show, tsp_cmd_store);
static DEVICE_ATTR(status, 0644, tsp_upg_status_show, tsp_upg_status_store);
static DEVICE_ATTR(tpinfo, 0644, tsp_fw_ic_tpinfo_show, tsp_fw_ic_tpinfo_store);
static DEVICE_ATTR(fileinfo, 0444, tsp_fw_file_tpinfo_show, NULL);
static DEVICE_ATTR(gesture, 0644, tsp_get_bsg_value_show, tsp_get_bsg_value_store);
static DEVICE_ATTR(gesture_enable, 0644, tsp_gesture_enable_show, tsp_gesture_enable_store);
static DEVICE_ATTR(i2c_test, 0644, tsp_i2c_test_show, tsp_i2c_test_store);
static DEVICE_ATTR(lcminfo, 0644, tsp_lcminfo_show, NULL);

//for pv tpinfo compare.
static DEVICE_ATTR(ic_info, 0444, tsp_ic_tpinfo_show_for_pv, NULL );
static DEVICE_ATTR(file_info, 0444, tsp_file_tpinfo_show_for_pv,  NULL);
static DEVICE_ATTR(compare_result, 0444, tsp_tpinfo_compare_result_show, NULL);


static struct attribute *tsp_dev_attrs[] = {
	&dev_attr_cmd.attr,
	&dev_attr_status.attr,
	&dev_attr_tpinfo.attr,
	&dev_attr_fileinfo.attr,
	&dev_attr_gesture.attr,
	&dev_attr_gesture_enable.attr,
	&dev_attr_i2c_test.attr,
	&dev_attr_lcminfo.attr,
	
	&dev_attr_ic_info.attr,
	&dev_attr_file_info.attr,
	&dev_attr_compare_result.attr,
	NULL,
};

//static struct bin_attribute *tsp_dev_bin_attributes[] = {
//	&bin_attr_button,
//	&bin_attr_info,
//	NULL,
//};

static const struct attribute_group tsp_dev_attribute_group = {
	.attrs = tsp_dev_attrs,
	//.bin_attrs = tsp_dev_bin_attributes,
};

static const struct attribute_group *tsp_dev_attribute_groups[] = {
	&tsp_dev_attribute_group,
	NULL,
};

static ssize_t tsp_fw_data_read(struct file *filp, struct kobject *kobj,
				  struct bin_attribute *bin_attr,
				  char *buffer, loff_t offset, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	int retval = -1;

	//printk("tpd: %s offset %lld byte, count %ld byte.\n", __func__, offset, count);
	
	if(offset > MAX_BUF_SIZE || cdev->tp_fw.data == NULL) {
		dev_err(cdev->dev, "[TSP]firmware size overflow\n");
		retval = -1;
		goto error;
	}

	if(offset + count > MAX_BUF_SIZE) {
		count = MAX_BUF_SIZE - offset;
	}

	memcpy(buffer, cdev->tp_fw.data + offset, count);

	retval = count;

error:
	return retval;
}

static ssize_t tsp_fw_data_write(struct file *filp, struct kobject *kobj,
				   struct bin_attribute *bin_attr,
				   char *buffer, loff_t offset, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	int retval = -1;
	int i = 0;

	//printk("tpd: %s offset %lld byte, count %ld byte.\n", __func__, offset, count);

	if(0 != tpd_alloc_buffer(cdev, MAX_BUF_SIZE)) {
		retval = -1;
		goto error;
	}

	if(offset + count > MAX_BUF_SIZE) {
		dev_err(cdev->dev, "[TSP]firmware size overflow\n");
		retval = -1;
		goto error;
	}
	memcpy(cdev->tp_fw.data + offset, buffer, count);
	cdev->tp_fw.data_len= cdev->tp_fw.data_len+ count;
	retval = count;

	printk("tpd: %s:", __func__);
	for(i = 0; i < 16; i++) {
		printk("tpd: 0x%x ", cdev->tp_fw.data[i]);
	}
	printk("tpd: \n");
	cdev->status = STATUS_BUF_ING;

error:
	return retval;
}

static struct bin_attribute firmware_attr_data = {
	.attr = { .name = "data", .mode = 0644 },
	.size = 0,
	.read = tsp_fw_data_read,
	.write = tsp_fw_data_write,
};

static ssize_t tsp_fw_reg_read(struct file *filp, struct kobject *kobj,
				  struct bin_attribute *bin_attr,
				  char *buffer, loff_t offset, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	int retval = -1;

	//printk("tpd: %s offset %lld byte, count %ld byte.\n", __func__, offset, count);
	
	if(cdev->read_block) {
		cdev->read_block(cdev, offset, buffer , count);
	}

	retval = count;

	return retval;
}

static ssize_t tsp_fw_reg_write(struct file *filp, struct kobject *kobj,
				   struct bin_attribute *bin_attr,
				   char *buffer, loff_t offset, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct tpd_classdev_t *cdev = dev_get_drvdata(dev);
	int retval = -1;

	//printk("tpd: %s offset %lld byte, count %ld byte.\n", __func__, offset, count);

	if(cdev->write_block) {
		cdev->write_block(cdev, offset, buffer, count);
	}

	retval = count;

	return retval;
}

static struct bin_attribute firmware_attr_reg = {
	.attr = { .name = "reg", .mode = 0644 },
	.size = 0,
	.read = tsp_fw_reg_read,
	.write = tsp_fw_reg_write,
};


/*
*
* add for proc/driver/lcd_id & proc/driver/tsc_id
*
*/

#ifdef CONFIG_PROC_FS
#define	LCD_PROC_FILE	"driver/lcd_id"
#define	TPD_PROC_FILE	"ts_information"
#define	WAKE_GESTURE_PROC_FILE "wake_gesture"

static struct proc_dir_entry *lcd_proc_entry;
static struct proc_dir_entry *tpd_proc_entry;
static struct proc_dir_entry *touchscreen_proc_entry;
static struct proc_dir_entry *wake_gesture_proc_entry;

static ssize_t tpd_proc_read_val(struct file *file,
	char __user *buffer, size_t count, loff_t *offset)
{
	ssize_t len = 0;
	char data[800];
	struct tpd_classdev_t *cdev = &tpd_fw_cdev;
	tpd_get_tp_module_name(cdev);

	if(cdev->get_tpinfo) {
		cdev->get_tpinfo(cdev);
	}
	tpd_get_tp_module_name(cdev);

	len = snprintf(data, 800, "TP module:%s(0x%x): IC type:%s; I2C address:0x%x; Firmware version:0x%x; Config version:0x%x\n", 
		cdev->ic_tpinfo.vendor_name, cdev->ic_tpinfo.module_id, 
		cdev->ic_tpinfo.tp_name, cdev->ic_tpinfo.i2c_addr,
		cdev->ic_tpinfo.firmware_ver, cdev->ic_tpinfo.config_ver);

	return simple_read_from_buffer(buffer, count, offset, data, len);
}

static ssize_t tpd_proc_write_val(struct file *filp,
					 const char *buff, size_t len,
					 loff_t * off)
{
	return len;
}

static struct file_operations tpd_proc_ops = {
	.read = tpd_proc_read_val,
	.write = tpd_proc_write_val,
};


static ssize_t lcd_proc_read_val(struct file *file,
	char __user *buffer, size_t count, loff_t *offset)
{
	ssize_t len = 0;
	char data[800];
	struct tpd_classdev_t *cdev = &tpd_fw_cdev;

	len += sprintf(data + len, "IC:%s; Vendor:%s; Resolution:%d*%d.\n", cdev->lcm_chip_info, cdev->lcm_info, 
		DISP_GetScreenWidth(), DISP_GetScreenHeight());

	return simple_read_from_buffer(buffer, count, offset, data, len);
}

static ssize_t lcd_proc_write_val(struct file *filp,
					 const char *buff, size_t len,
					 loff_t * off)
{
	return len;
}

static struct file_operations lcd_proc_ops = {
	.read = lcd_proc_read_val,
	.write = lcd_proc_write_val,
};

static void create_lcd_proc_entry(void)
{
	lcd_proc_entry = proc_create(LCD_PROC_FILE, 0644, NULL, &lcd_proc_ops);
	if (lcd_proc_entry) {
		printk(KERN_INFO "create proc file sucess!\n");
	} else
		printk(KERN_INFO "create proc file failed!\n");
}



static ssize_t wake_gesture_proc_read_val(struct file *file,
	char __user *buffer, size_t count, loff_t *offset)
{


	ssize_t len = 0;
	char data[800];
	struct tpd_classdev_t *cdev = &tpd_fw_cdev;

	len += sprintf(data + len, "wake_gesture:%d.\n", cdev->b_gesture_enable );

	return simple_read_from_buffer(buffer, count, offset, data, len);
}

static ssize_t wake_gesture_proc_write_val(struct file *filp,
					 const char *buff, size_t len,
					 loff_t * off)
{
	unsigned int input;
	char value[128];
	int max = 127;
	struct tpd_classdev_t *cdev = &tpd_fw_cdev;

	if(len >= max)
		len = max;

	  if(copy_from_user(value, buff, len))
   		return -EFAULT;

	  value[len] = '\0';
	
	if (sscanf(value, "%u", &input) != 1)
		return -EINVAL;

	input = input > 0 ? 1 : 0;
	
	printk("tpd: %s val %d.\n", __func__, input);

	mutex_lock(&cdev->cmd_mutex);
	if(cdev->wake_gesture) {
		cdev->wake_gesture(cdev, input);
	}
	cdev->b_gesture_enable = input;
	mutex_unlock(&cdev->cmd_mutex);
	
	return len;
}

static struct file_operations wake_gesture_proc_ops = {
	.read = wake_gesture_proc_read_val,
	.write = wake_gesture_proc_write_val,
};


static void create_touchscreen_proc_entry(void)
{
	touchscreen_proc_entry = proc_mkdir("touchscreen",NULL);

	tpd_proc_entry = proc_create(TPD_PROC_FILE, 0644, touchscreen_proc_entry, &tpd_proc_ops);
	if (tpd_proc_entry) {
		printk(KERN_INFO "create proc file sucess!\n");
	} else
		printk(KERN_INFO "create proc file failed!\n");
	wake_gesture_proc_entry = proc_create(WAKE_GESTURE_PROC_FILE, 0644, touchscreen_proc_entry, &wake_gesture_proc_ops);
	if (wake_gesture_proc_entry) {
		printk(KERN_INFO "create proc file sucess!\n");
	} else
		printk(KERN_INFO "create proc file failed!\n");


}
#endif


/**
 * tpd_classdev_register - register a new object of tpd_classdev_t class.
 * @parent: The device to register.
 * @tsp_fw_cdev: the tpd_classdev_t structure for this device.
 */
int tpd_classdev_register(struct device *parent, struct tpd_classdev_t *tsp_fw_cdev)
{
	int error = 0;
	
	tsp_fw_cdev->dev = device_create(tsp_fw_class, NULL, 0, tsp_fw_cdev,
					  "%s", tsp_fw_cdev->name);
	if (IS_ERR(tsp_fw_cdev->dev))
		return PTR_ERR(tsp_fw_cdev->dev);

	error = device_create_bin_file(tsp_fw_cdev->dev, &firmware_attr_data);
	if (error) {
		dev_err(tsp_fw_cdev->dev, "%s: sysfs_create_bin_file failed\n", __func__);
	}
	error = device_create_bin_file(tsp_fw_cdev->dev, &firmware_attr_reg);
	if (error) {
		dev_err(tsp_fw_cdev->dev, "%s: sysfs_create_bin_file failed\n", __func__);
	}

	/* add to the list of tp_firmware */
	down_write(&tp_firmware_list_lock);
	list_add_tail(&tsp_fw_cdev->node, &tp_firmware_list);
	up_write(&tp_firmware_list_lock);

	mutex_init(&tsp_fw_cdev->upgrade_mutex);
	mutex_init(&tsp_fw_cdev->cmd_mutex);

	printk("tpd: Registered tsp_fw device: %s\n",
			tsp_fw_cdev->name);


	return 0;
}
EXPORT_SYMBOL_GPL(tpd_classdev_register);

/**
 * tpd_classdev_unregister - unregisters a object of tsp_fw_properties class.
 * @tsp_fw_cdev: the tsp_fw device to unregister
 *
 * Unregisters a previously registered via tpd_classdev_register object.
 */
void tpd_classdev_unregister(struct tpd_classdev_t *tsp_fw_cdev)
{
	device_unregister(tsp_fw_cdev->dev);

	down_write(&tp_firmware_list_lock);
	list_del(&tsp_fw_cdev->node);
	up_write(&tp_firmware_list_lock);
}
EXPORT_SYMBOL_GPL(tpd_classdev_unregister);

extern char* mtkfb_find_lcm_driver(void);
static void get_lcm_info(struct tpd_classdev_t *cdev)
{
	char * tmp_name = NULL;
	char lcm_info[16],lcm_chip_info[16];
	
	tmp_name = mtkfb_find_lcm_driver();
	printk("tpd check lcm name = %s", tmp_name);
	if(NULL != strstr(tmp_name, "txd") || NULL != strstr(tmp_name, "tongxingda") ){
		strncpy(lcm_info, "TongXingDa", 15);
	} else if(NULL != strstr(tmp_name, "lead")){
		strncpy(lcm_info, "Lead", 15);
	} else if(NULL != strstr(tmp_name, "tianma")){
		strncpy(lcm_info, "TianMa", 15);
	} else if(NULL != strstr(tmp_name, "sanxing")){
		strncpy(lcm_info, "SanXing", 15);
	} else if(NULL != strstr(tmp_name, "helitai")){
		strncpy(lcm_info, "HeLiTai", 15);
	} else if(NULL != strstr(tmp_name, "lcetron")){
		strncpy(lcm_info, "Lcetron", 15);
	} else if(NULL != strstr(tmp_name, "skyworth")){
		strncpy(lcm_info, "Skyworth", 15);
	} else if(NULL != strstr(tmp_name, "holitech")){
		strncpy(lcm_info, "Holitech", 15);
	} else if(NULL != strstr(tmp_name, "dijing")){
		strncpy(lcm_info, "DiJing", 15);
	} else if(NULL != strstr(tmp_name, "yassy")){
		strncpy(lcm_info, "YaShi", 15);
	} else if(NULL != strstr(tmp_name, "cmi")){
		strncpy(lcm_info, "YaShi", 15);
	} else {
		strncpy(lcm_info, "unkown", 15);
	}

	if(NULL != strstr(tmp_name, "8394") ){
		strncpy(lcm_chip_info, "HX8394", 15);
	} else if(NULL != strstr(tmp_name, "9881")){
		strncpy(lcm_chip_info, "ILI9881", 15);
	} else if(NULL != strstr(tmp_name, "8399")){
		strncpy(lcm_chip_info, "HX8399", 15);
	} else if(NULL != strstr(tmp_name, "35532")){
		strncpy(lcm_chip_info, "NT35532", 15);
	} else if(NULL != strstr(tmp_name, "68200")){
		strncpy(lcm_chip_info, "RM68200", 15);
	} else if(NULL != strstr(tmp_name, "7703")){
		strncpy(lcm_chip_info, "ST7703", 15);
	} else if(NULL != strstr(tmp_name, "hx83112a")){
		strncpy(lcm_chip_info, "HX83112A", 15);
	} else {
		strncpy(lcm_chip_info, "lcd chip", 15);
	}

	strncpy(cdev->lcm_info, lcm_info, 64);
	strncpy(cdev->lcm_chip_info, lcm_chip_info, 64);

}

static int __init tpd_class_init(void)
{
	tsp_fw_class = class_create(THIS_MODULE, "tsp_fw");
	if (IS_ERR(tsp_fw_class))
		return PTR_ERR(tsp_fw_class);
	tsp_fw_class->dev_groups = tsp_dev_attribute_groups;

	tpd_fw_cdev.tp_fw.data = NULL;
	tpd_fw_cdev.tp_fw.data_len = 0;
	tpd_fw_cdev.tp_fw.length = 0;
	tpd_fw_cdev.fw_compare_result = 1;

	get_lcm_info(&tpd_fw_cdev);

#ifdef CONFIG_PROC_FS
	create_lcd_proc_entry();
	create_touchscreen_proc_entry();
#endif
	
	return 0;
}

static void __exit tpd_class_exit(void)
{
	tpd_free_buffer(&tpd_fw_cdev);
	class_destroy(tsp_fw_class);
}

subsys_initcall(tpd_class_init);
module_exit(tpd_class_exit);

MODULE_AUTHOR("John Lenz, Richard Purdie");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("TSP FW Class Interface");


