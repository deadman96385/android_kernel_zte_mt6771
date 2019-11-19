#ifndef _AW8891_H_
#define _AW8891_H_


/*
 * i2c transaction on Linux limited to 64k
 * (See Linux kernel documentation: Documentation/i2c/writing-clients)
*/
#define MAX_I2C_BUFFER_SIZE 65536

#define AW8891_FLAG_START_ON_MUTE   (1 << 0)
#define AW8891_FLAG_SKIP_INTERRUPTS     (1 << 1)
#define AW8891_FLAG_SAAM_AVAILABLE      (1 << 2)
#define AW8891_FLAG_STEREO_DEVICE       (1 << 3)
#define AW8891_FLAG_MULTI_MIC_INPUTS    (1 << 4)

#define AW8891_NUM_RATES                9

#define AW8891_MAX_REGISTER             0xff

/*#define AW8891_VBAT_MONITOR*/

#ifdef AW8891_VBAT_MONITOR
#define SYS_BAT_DEV "/sys/class/power_supply/battery/batt_vol"
#define AW8891_SYS_VBAT_LIMIT           3600
#define AW8891_SYS_VBAT_MIN             3000
#endif

enum aw8891_chipid {
		AW8891_ID,
};

enum aw8891_mode_spk_rcv {
		AW8891_SPEAKER_MODE = 0,
		AW8891_RECEIVER_MODE = 1,
};

struct aw8891 {
	struct regmap *regmap;
	struct i2c_client *i2c;
	struct snd_soc_codec *codec;
	struct device *dev;
	struct mutex cfg_lock;
#ifdef AW8891_VBAT_MONITOR
	struct hrtimer vbat_monitor_timer;
	struct work_struct vbat_monitor_work;
#endif
	int sysclk;
	int rate;
	int pstream;
	int cstream;

	int reset_gpio;
	int irq_gpio;

#ifdef CONFIG_DEBUG_FS
	struct dentry *dbg_dir;
#endif
	u8 reg;

	unsigned int flags;
	unsigned int chipid;
	unsigned int init;
	unsigned int spk_rcv_mode;

	unsigned int bst_ilimit;
};

struct aw8891_container {
	int len;
	unsigned char data[];
};


int aw8891_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id);
int aw8891_i2c_remove(struct i2c_client *client);

#endif
