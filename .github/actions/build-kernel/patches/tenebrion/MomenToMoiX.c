// SPDX-License-Identifier: GPL-2.0
/*
 * MomenToMoiX (Hybrid Engine)
 *
 * Screen-off power saver: cap CPU max freq, isolate background cpusets,
 * switch I/O scheduler, with a thermal hold after screen-on.
 */
#define pr_fmt(fmt) "momx: " fmt

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/fs.h>
#include <linux/ctype.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/limits.h>
#include <linux/math64.h>
#include <linux/cpumask.h>
#include <linux/cpufreq.h>
#include <linux/pm_qos.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/wait.h>
#include <linux/jiffies.h>
#include <linux/suspend.h>
#include <linux/notifier.h>

MODULE_IMPORT_NS(VFS_internal_I_am_really_a_filesystem_and_am_NOT_a_driver);

/* ------------------------------------------------------------------ */
/* Tunables                                                            */
/* ------------------------------------------------------------------ */

static int poll_interval_ms = 1000;
module_param(poll_interval_ms, int, 0644);
MODULE_PARM_DESC(poll_interval_ms, "Screen poll interval (100-10000 ms)");

static int boot_delay_ms = 40000;
module_param(boot_delay_ms, int, 0644);
MODULE_PARM_DESC(boot_delay_ms, "Delay before engine starts (0-600000 ms)");

static int thermal_threshold_mc = 65000;
module_param(thermal_threshold_mc, int, 0644);
MODULE_PARM_DESC(thermal_threshold_mc, "Arm thermal hold at/above this temp (mC)");

static int thermal_hysteresis_mc = 5000;
module_param(thermal_hysteresis_mc, int, 0644);
MODULE_PARM_DESC(thermal_hysteresis_mc, "Release hold below threshold - hysteresis (mC)");

static int thermal_hold_ms = 3000;
module_param(thermal_hold_ms, int, 0644);
MODULE_PARM_DESC(thermal_hold_ms, "Minimum hold time after screen-on (ms)");

static int thermal_hold_bias_percent = 50;
module_param(thermal_hold_bias_percent, int, 0644);
MODULE_PARM_DESC(thermal_hold_bias_percent, "Freq cap while thermal hold is active with screen on (0-100)");

static int charging_freq_bias_percent = 20;
module_param(charging_freq_bias_percent, int, 0644);
MODULE_PARM_DESC(charging_freq_bias_percent, "Screen-off freq cap while charging (0-100)");

static int doze_active_freq_bias_percent = 10;
module_param(doze_active_freq_bias_percent, int, 0644);
MODULE_PARM_DESC(doze_active_freq_bias_percent, "Screen-off freq cap on battery (0-100)");

/* ------------------------------------------------------------------ */
/* Paths & constants                                                   */
/* ------------------------------------------------------------------ */

#define DPMS_PATH            "/sys/class/drm/card0-DSI-1/dpms"
#define BACKLIGHT_PATH       "/sys/class/backlight/panel0-backlight/brightness"

#define CPUSET_SYSBG_PATH    "/dev/cpuset/system-background/cpus"
#define CPUSET_BG_PATH       "/dev/cpuset/background/cpus"

#define IOSCHED_TARGET       "none"

#define MOMX_MAX_ZONES       64
#define MOMX_TEMP_MIN_MC     (-40000)
#define MOMX_TEMP_MAX_MC     150000
#define MOMX_TEMP_INVALID    INT_MIN
#define MOMX_CHG_RECHECK_MS  5000

static const char * const iosched_candidates[] = {
	"/sys/block/sda/queue/scheduler",
	"/sys/block/mmcblk0/queue/scheduler",
	"/sys/block/nvme0n1/queue/scheduler",
	NULL,
};

static const char * const power_status_candidates[] = {
	"/sys/class/power_supply/battery/status",
	"/sys/class/power_supply/bms/status",
	"/sys/class/power_supply/BAT0/status",
	NULL,
};

/* ------------------------------------------------------------------ */
/* State (only touched by the watcher thread, and by exit after stop)  */
/* ------------------------------------------------------------------ */

enum momx_screen_src { SRC_UNKNOWN, SRC_DPMS, SRC_BACKLIGHT };

struct momx_cpuset {
	const char *path;
	char orig[32];
	bool restricted;
};

static struct momx_cpuset cpusets[] = {
	{ .path = CPUSET_SYSBG_PATH },
	{ .path = CPUSET_BG_PATH },
};

static struct freq_qos_request momx_max_req[NR_CPUS];
static bool qos_active[NR_CPUS];
static int cur_cap = -1;

static struct task_struct *watcher_thread;
static DECLARE_WAIT_QUEUE_HEAD(momx_wq);
static bool momx_kick;
static bool in_deep_sleep;

static bool is_screen_off;
static bool thermal_hold_active;
static unsigned long thermal_hold_expire;
static unsigned long next_chg_check;
static bool last_charging;

static struct file *dpms_f;
static struct file *bl_f;
static struct file *power_f;
static enum momx_screen_src screen_src = SRC_UNKNOWN;

static char power_status_path[64];
static bool power_status_available;

static char iosched_path[64];
static char default_iosched[32];
static bool iosched_available;
static bool iosched_restricted;

static struct file *zone_f[MOMX_MAX_ZONES];
static int zone_id[MOMX_MAX_ZONES];
static int nr_zones;

/* ------------------------------------------------------------------ */
/* File helpers                                                        */
/* ------------------------------------------------------------------ */

static void momx_close(struct file **fp)
{
	if (*fp) {
		filp_close(*fp, NULL);
		*fp = NULL;
	}
}

/* Read via a cached fd (sysfs supports re-reading from pos 0). Trims whitespace. */
static ssize_t momx_read_cached(struct file **fp, const char *path,
				char *buf, size_t size)
{
	loff_t pos = 0;
	ssize_t ret;

	if (!*fp) {
		struct file *f = filp_open(path, O_RDONLY, 0);

		if (IS_ERR(f))
			return PTR_ERR(f);
		*fp = f;
	}

	ret = kernel_read(*fp, buf, size - 1, &pos);
	if (ret <= 0) {
		momx_close(fp);
		return ret ? ret : -ENODATA;
	}

	buf[ret] = '\0';
	while (ret > 0 && isspace(buf[ret - 1]))
		buf[--ret] = '\0';
	return ret;
}

static ssize_t momx_read_once(const char *path, char *buf, size_t size)
{
	struct file *f = NULL;
	ssize_t ret = momx_read_cached(&f, path, buf, size);

	momx_close(&f);
	return ret;
}

static int momx_write_file(const char *path, const char *buf)
{
	struct file *f;
	loff_t pos = 0;
	ssize_t ret;

	f = filp_open(path, O_WRONLY, 0);
	if (IS_ERR(f))
		return PTR_ERR(f);

	ret = kernel_write(f, buf, strlen(buf), &pos);
	filp_close(f, NULL);
	return ret > 0 ? 0 : (ret < 0 ? (int)ret : -EIO);
}

/* ------------------------------------------------------------------ */
/* Sensors                                                             */
/* ------------------------------------------------------------------ */

static void momx_thermal_init(void)
{
	char path[64], buf[16];
	long val;
	int i;

	for (i = 0; i < MOMX_MAX_ZONES; i++) {
		struct file *f = NULL;

		snprintf(path, sizeof(path),
			 "/sys/class/thermal/thermal_zone%d/temp", i);
		if (momx_read_cached(&f, path, buf, sizeof(buf)) > 0 &&
		    !kstrtol(buf, 10, &val)) {
			zone_f[nr_zones] = f;
			zone_id[nr_zones] = i;
			nr_zones++;
		} else {
			momx_close(&f);
		}
	}
	pr_info("Thermal: %d usable zones\n", nr_zones);
}

static int momx_read_max_temp(void)
{
	char path[64], buf[16];
	int i, max_temp = MOMX_TEMP_INVALID;
	long val;

	for (i = 0; i < nr_zones; i++) {
		snprintf(path, sizeof(path),
			 "/sys/class/thermal/thermal_zone%d/temp", zone_id[i]);
		if (momx_read_cached(&zone_f[i], path, buf, sizeof(buf)) <= 0)
			continue;
		if (kstrtol(buf, 10, &val))
			continue;
		if (val < MOMX_TEMP_MIN_MC || val > MOMX_TEMP_MAX_MC)
			continue;	/* bogus / virtual sensor */
		if ((int)val > max_temp)
			max_temp = (int)val;
	}
	return max_temp;
}

static void momx_charge_detect(void)
{
	char buf[32];
	int i;

	for (i = 0; power_status_candidates[i]; i++) {
		if (momx_read_cached(&power_f, power_status_candidates[i],
				     buf, sizeof(buf)) > 0) {
			strscpy(power_status_path, power_status_candidates[i],
				sizeof(power_status_path));
			power_status_available = true;
			pr_info("Battery status node: %s\n", power_status_path);
			return;
		}
	}
	pr_warn("No battery status node found\n");
}

static bool momx_is_charging(void)
{
	char buf[32];

	if (!power_status_available ||
	    momx_read_cached(&power_f, power_status_path, buf, sizeof(buf)) <= 0)
		return false;
	return !strcmp(buf, "Charging") || !strcmp(buf, "Full");
}

/* Returns 1 = on, 0 = off, -1 = unknown (keep previous state) */
static int momx_read_screen_state(void)
{
	char buf[64];
	long bl;

	if (screen_src != SRC_BACKLIGHT) {
		if (momx_read_cached(&dpms_f, DPMS_PATH, buf, sizeof(buf)) > 0) {
			screen_src = SRC_DPMS;
			return !strcasecmp(buf, "On");
		}
		if (screen_src == SRC_DPMS)
			return -1;
	}

	if (momx_read_cached(&bl_f, BACKLIGHT_PATH, buf, sizeof(buf)) > 0 &&
	    !kstrtol(buf, 10, &bl)) {
		screen_src = SRC_BACKLIGHT;
		return bl > 0;
	}
	return -1;
}

/* ------------------------------------------------------------------ */
/* Cpuset                                                              */
/* ------------------------------------------------------------------ */

static void momx_cpuset_init(void)
{
	char buf[32];
	size_t i;

	for (i = 0; i < ARRAY_SIZE(cpusets); i++) {
		if (momx_read_once(cpusets[i].path, buf, sizeof(buf)) > 0)
			strscpy(cpusets[i].orig, buf, sizeof(cpusets[i].orig));
		else
			snprintf(cpusets[i].orig, sizeof(cpusets[i].orig),
				 "0-%u", num_possible_cpus() - 1);
	}
}

static void momx_cpuset_restrict(void)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(cpusets); i++)
		if (!momx_write_file(cpusets[i].path, "0\n"))
			cpusets[i].restricted = true;
	pr_info("Cpuset: background groups isolated to CPU0\n");
}

static void momx_cpuset_restore(void)
{
	char val[40];
	size_t i;

	for (i = 0; i < ARRAY_SIZE(cpusets); i++) {
		if (!cpusets[i].restricted)
			continue;
		snprintf(val, sizeof(val), "%s\n", cpusets[i].orig);
		if (!momx_write_file(cpusets[i].path, val))
			cpusets[i].restricted = false;
		pr_info("Cpuset: %s restored to %s\n", cpusets[i].path,
			cpusets[i].orig);
	}
}

/* ------------------------------------------------------------------ */
/* I/O scheduler                                                       */
/* ------------------------------------------------------------------ */

static void momx_iosched_detect(void)
{
	char buf[128];
	char *start, *end;
	size_t len;
	int i;

	for (i = 0; iosched_candidates[i]; i++) {
		if (momx_read_once(iosched_candidates[i], buf, sizeof(buf)) <= 0)
			continue;

		start = strchr(buf, '[');
		end = strchr(buf, ']');
		if (!start || !end || end <= start)
			break;

		len = end - start - 1;
		if (!len || len >= sizeof(default_iosched))
			break;

		memcpy(default_iosched, start + 1, len);
		default_iosched[len] = '\0';
		strscpy(iosched_path, iosched_candidates[i], sizeof(iosched_path));

		if (!strcmp(default_iosched, IOSCHED_TARGET)) {
			pr_info("I/O sched already '%s', nothing to switch\n",
				default_iosched);
			return;
		}
		iosched_available = true;
		pr_info("I/O sched node: %s (stock: %s)\n", iosched_path,
			default_iosched);
		return;
	}
	pr_warn("I/O sched: no compatible node\n");
}

static void momx_iosched_restrict(void)
{
	if (!iosched_available || iosched_restricted)
		return;
	if (!momx_write_file(iosched_path, IOSCHED_TARGET)) {
		iosched_restricted = true;
		pr_info("I/O sched -> %s\n", IOSCHED_TARGET);
	}
}

static void momx_iosched_restore(void)
{
	if (!iosched_available || !iosched_restricted)
		return;
	if (!momx_write_file(iosched_path, default_iosched)) {
		iosched_restricted = false;
		pr_info("I/O sched restored -> %s\n", default_iosched);
	}
}

/* ------------------------------------------------------------------ */
/* CPU freq cap (max-only; min stays stock so idle floor isn't raised) */
/* ------------------------------------------------------------------ */

static void momx_set_cap(int percent)
{
	cpumask_t done;
	unsigned int cpu, n = 0;

	percent = clamp(percent, 0, 100);
	if (percent == cur_cap)
		return;

	cpumask_clear(&done);
	for_each_possible_cpu(cpu) {
		struct cpufreq_policy *policy;
		unsigned int target;
		int ret;

		if (cpumask_test_cpu(cpu, &done))
			continue;
		policy = cpufreq_cpu_get(cpu);
		if (!policy)
			continue;
		cpumask_or(&done, &done, policy->related_cpus);

		target = policy->cpuinfo.min_freq +
			 div_u64((u64)(policy->cpuinfo.max_freq -
				       policy->cpuinfo.min_freq) * percent, 100);

		if (qos_active[cpu]) {
			ret = freq_qos_update_request(&momx_max_req[cpu], target);
		} else {
			ret = freq_qos_add_request(&policy->constraints,
						   &momx_max_req[cpu],
						   FREQ_QOS_MAX, target);
			if (ret >= 0)
				qos_active[cpu] = true;
		}

		if (ret < 0)
			pr_warn("cpu%u: freq cap failed (%d)\n", cpu, ret);
		else
			n++;
		cpufreq_cpu_put(policy);
	}

	cur_cap = percent;
	pr_info("Freq cap %d%% on %u policies\n", percent, n);
}

static void momx_release_cap(void)
{
	unsigned int i;

	for (i = 0; i < NR_CPUS; i++) {
		if (qos_active[i]) {
			freq_qos_remove_request(&momx_max_req[i]);
			qos_active[i] = false;
		}
	}
	if (cur_cap >= 0)
		pr_info("Freq cap released\n");
	cur_cap = -1;
}

/* ------------------------------------------------------------------ */
/* State machine                                                       */
/* ------------------------------------------------------------------ */

static int momx_offscreen_cap(bool charging)
{
	return charging ? charging_freq_bias_percent
			: doze_active_freq_bias_percent;
}

static void momx_on_screen_off(void)
{
	bool charging = momx_is_charging();
	int temp = momx_read_max_temp();

	pr_info("Display OFF | charging=%s | max temp=%d mC\n",
		charging ? "yes" : "no", temp);

	is_screen_off = true;
	last_charging = charging;
	next_chg_check = jiffies + msecs_to_jiffies(MOMX_CHG_RECHECK_MS);
	thermal_hold_active = temp != MOMX_TEMP_INVALID &&
			      temp >= thermal_threshold_mc;

	momx_set_cap(momx_offscreen_cap(charging));
	momx_cpuset_restrict();
	momx_iosched_restrict();

	if (thermal_hold_active)
		pr_info("Thermal hold armed (%d >= %d mC)\n", temp,
			thermal_threshold_mc);
}

static void momx_on_screen_on(void)
{
	is_screen_off = false;
	pr_info("Display ON\n");

	momx_cpuset_restore();
	momx_iosched_restore();

	if (thermal_hold_active) {
		thermal_hold_expire = jiffies + msecs_to_jiffies(thermal_hold_ms);
		momx_set_cap(thermal_hold_bias_percent);
	} else {
		momx_release_cap();
	}
}

static void momx_tick(void)
{
	int temp;

	if (is_screen_off) {
		bool chg;

		if (time_before(jiffies, next_chg_check))
			return;
		next_chg_check = jiffies + msecs_to_jiffies(MOMX_CHG_RECHECK_MS);

		chg = momx_is_charging();
		if (chg != last_charging) {
			last_charging = chg;
			momx_set_cap(momx_offscreen_cap(chg));
		}
		return;
	}

	if (!thermal_hold_active || time_before(jiffies, thermal_hold_expire))
		return;

	temp = momx_read_max_temp();
	if (temp == MOMX_TEMP_INVALID ||
	    temp < thermal_threshold_mc - thermal_hysteresis_mc) {
		pr_info("Thermal hold released (%d mC)\n", temp);
		momx_release_cap();
		thermal_hold_active = false;
	} else {
		thermal_hold_expire = jiffies +
			msecs_to_jiffies(clamp(poll_interval_ms, 100, 10000));
		pr_debug("Thermal hold extended (%d mC)\n", temp);
	}
}

/* ------------------------------------------------------------------ */
/* PM notifier + watcher                                               */
/* ------------------------------------------------------------------ */

static int momx_pm_notifier(struct notifier_block *nb, unsigned long action,
			    void *data)
{
	switch (action) {
	case PM_SUSPEND_PREPARE:
		WRITE_ONCE(in_deep_sleep, true);
		break;
	case PM_POST_SUSPEND:
		WRITE_ONCE(in_deep_sleep, false);
		WRITE_ONCE(momx_kick, true);	/* re-evaluate right after resume */
		wake_up(&momx_wq);
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block momx_pm_nb = {
	.notifier_call = momx_pm_notifier,
};

/* Freezable, stop-aware sleep. Returns true if the thread should exit. */
static bool momx_sleep(unsigned int ms)
{
	(void)wait_event_freezable_timeout(momx_wq,
		kthread_should_stop() || READ_ONCE(momx_kick),
		msecs_to_jiffies(ms));
	WRITE_ONCE(momx_kick, false);
	return kthread_should_stop();
}

static int momx_watcher(void *data)
{
	int last_state = -1;

	set_freezable();

	pr_info("Watcher spawned, boot delay %d ms\n", boot_delay_ms);
	if (momx_sleep(clamp(boot_delay_ms, 0, 600000)))
		return 0;

	momx_thermal_init();
	momx_charge_detect();
	momx_cpuset_init();
	momx_iosched_detect();

	pr_info("Watcher loop active (sysfs polling)\n");

	do {
		if (!READ_ONCE(in_deep_sleep)) {
			int state = momx_read_screen_state();

			if (state >= 0 && state != last_state) {
				last_state = state;
				if (!state && !is_screen_off)
					momx_on_screen_off();
				else if (state && is_screen_off)
					momx_on_screen_on();
			}
			momx_tick();
		}
	} while (!momx_sleep(clamp(poll_interval_ms, 100, 10000)));

	pr_info("Watcher exiting\n");
	return 0;
}

/* ------------------------------------------------------------------ */
/* Module init / exit                                                  */
/* ------------------------------------------------------------------ */

static int __init momx_init(void)
{
	pr_info("Loading MomenToMoiX (Hybrid Engine)\n");

	register_pm_notifier(&momx_pm_nb);

	watcher_thread = kthread_run(momx_watcher, NULL, "momx_watch");
	if (IS_ERR(watcher_thread)) {
		int err = PTR_ERR(watcher_thread);

		watcher_thread = NULL;
		pr_err("Failed to start watcher kthread (%d)\n", err);
		unregister_pm_notifier(&momx_pm_nb);
		return err;
	}
	return 0;
}

static void __exit momx_exit(void)
{
	int i;

	if (watcher_thread)
		kthread_stop(watcher_thread);

	unregister_pm_notifier(&momx_pm_nb);

	/* Everything below runs single-threaded; undo all runtime changes. */
	momx_release_cap();
	momx_cpuset_restore();
	momx_iosched_restore();

	momx_close(&dpms_f);
	momx_close(&bl_f);
	momx_close(&power_f);
	for (i = 0; i < nr_zones; i++)
		momx_close(&zone_f[i]);

	pr_info("MomenToMoiX unloaded cleanly\n");
}

module_init(momx_init);
module_exit(momx_exit);

MODULE_DESCRIPTION("MomenToMoiX: screen-off power/thermal engine");
MODULE_LICENSE("GPL");