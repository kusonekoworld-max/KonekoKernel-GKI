// SPDX-License-Identifier: GPL-2.0-only
/*
 * dynamic_fsync - Screen-state aware fsync toggle
 *
 * Disables fsync while the screen is on to eliminate write-back
 * stalls during UI-critical moments. Re-enables fsync when the
 * screen turns off to guarantee data integrity before suspend.
 *
 * Screen-state detection: polling-based, same approach as
 * MomenToMoiX's momx_display_node_detect() -- probe a DPMS node
 * first (/sys/class/drm/cardN-DSI-1/dpms), fall back to a raw
 * backlight brightness node if no DPMS node is found or readable.
 * This replaces the previous backlight_register_notifier() based
 * detection, which silently never fires on devices that expose
 * DRM-panel backlight without wiring backlight_update_status()
 * through the notifier chain -- on such devices dynamic_fsync_state
 * would stay pinned at its boot default forever, either always
 * skipping fsync (data-integrity risk) or never skipping it.
 * Polling reads sysfs directly, so it works regardless of whether
 * any notifier chain is actually driven by the vendor's backlight
 * driver.
 *
 * The core fsync path (fs/sync.c, do_fsync()) checks
 * dynamic_fsync_should_skip() (see include/linux/dynamic_fsync.h)
 * before issuing any writeback.
 *
 * Userspace control:
 *   /sys/module/dynamic_fsync_driver/parameters/enabled       (1/0, def 1)
 *   /sys/module/dynamic_fsync_driver/parameters/poll_interval_ms (def 1000)
 */
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/dynamic_fsync.h>

/*
 * Module parameter stored directly in the kernel's global so the
 * fs/sync.c hot path always reads the authoritative value.
 */
module_param_named(enabled, dynamic_fsync_enabled, bool, 0644);
MODULE_PARM_DESC(enabled, "Enable dynamic fsync (default: true)");

static int poll_interval_ms = 1000;
module_param(poll_interval_ms, int, 0644);
MODULE_PARM_DESC(poll_interval_ms, "Screen-state poll interval in ms (default: 1000)");

/* Same candidate lists / rationale as momx_display_node_detect():
 * display-state exposure isn't standardized across vendors, so we
 * probe a list once at boot instead of hardcoding a single path. */
static const char *dpms_candidates[] = {
	"/sys/class/drm/card0-DSI-1/dpms",
	"/sys/class/drm/card1-DSI-1/dpms",
	NULL,
};

static const char *backlight_candidates[] = {
	"/sys/class/backlight/panel0-backlight/brightness",
	"/sys/class/leds/lcd-backlight/brightness",
	"/sys/class/backlight/panel0/brightness",
	NULL,
};

static char dpms_path[64] = "";
static bool dpms_available = false;
static char backlight_path[64] = "";
static bool backlight_available = false;

static struct task_struct *dfsync_thread;

static int dfsync_read_file(const char *path, char *buf, size_t size)
{
	struct file *f;
	loff_t pos = 0;
	int ret;

	f = filp_open(path, O_RDONLY, 0);
	if (IS_ERR(f))
		return -1;
	ret = kernel_read(f, buf, size - 1, &pos);
	filp_close(f, NULL);
	if (ret > 0) {
		if (buf[ret - 1] == '\n')
			buf[ret - 1] = '\0';
		else
			buf[ret] = '\0';
	} else {
		ret = -1;
	}
	return ret;
}

static void dfsync_node_detect(void)
{
	char buf[32];
	int i;

	for (i = 0; dpms_candidates[i] != NULL; i++) {
		if (dfsync_read_file(dpms_candidates[i], buf, sizeof(buf)) > 0) {
			strscpy(dpms_path, dpms_candidates[i], sizeof(dpms_path));
			dpms_available = true;
			pr_info("dynamic_fsync: DPMS node detected at: %s\n", dpms_path);
			break;
		}
	}
	if (!dpms_available)
		pr_warn("dynamic_fsync: no DPMS node found, falling back to backlight\n");

	for (i = 0; backlight_candidates[i] != NULL; i++) {
		if (dfsync_read_file(backlight_candidates[i], buf, sizeof(buf)) > 0) {
			strscpy(backlight_path, backlight_candidates[i], sizeof(backlight_path));
			backlight_available = true;
			pr_info("dynamic_fsync: backlight node detected at: %s\n", backlight_path);
			break;
		}
	}
	if (!backlight_available)
		pr_warn("dynamic_fsync: no backlight node found either\n");

	if (!dpms_available && !backlight_available)
		pr_err("dynamic_fsync: no usable display-state node -- screen detection disabled, fsync will never be skipped\n");
}

static int dfsync_watcher(void *data)
{
	int last_state = -1;

	dfsync_node_detect();

	while (!kthread_should_stop()) {
		int current_state = -1;
		char buf[32];

		if (dpms_available && dfsync_read_file(dpms_path, buf, sizeof(buf)) > 0) {
			current_state = strstr(buf, "On") ? 1 : 0;
		} else if (backlight_available &&
			   dfsync_read_file(backlight_path, buf, sizeof(buf)) > 0) {
			long bl_val;

			if (!kstrtol(buf, 10, &bl_val))
				current_state = (bl_val > 0) ? 1 : 0;
		}

		if (current_state != -1 && current_state != last_state) {
			WRITE_ONCE(dynamic_fsync_state, current_state ? 2 : 1);
			pr_info("dynamic_fsync: screen %s\n", current_state ? "ON" : "OFF");
			last_state = current_state;
		}

		msleep_interruptible(poll_interval_ms);
	}

	return 0;
}

static int __init dfsync_init(void)
{
	/* Default: assume screen is on at boot, until first poll lands */
	WRITE_ONCE(dynamic_fsync_state, 2);

	dfsync_thread = kthread_run(dfsync_watcher, NULL, "dfsync_watch");
	if (IS_ERR(dfsync_thread)) {
		pr_err("dynamic_fsync: failed to spawn watcher thread\n");
		return PTR_ERR(dfsync_thread);
	}

	pr_info("dynamic_fsync: enabled=%d, screen=ON (default)\n",
		(int)READ_ONCE(dynamic_fsync_enabled));
	return 0;
}

static void __exit dfsync_exit(void)
{
	if (dfsync_thread)
		kthread_stop(dfsync_thread);
	WRITE_ONCE(dynamic_fsync_state, 0);
}

module_init(dfsync_init);
module_exit(dfsync_exit);

MODULE_DESCRIPTION("Dynamic fsync — disable fsync when screen is on");
MODULE_AUTHOR("XTENSEI");
MODULE_LICENSE("GPL v2");