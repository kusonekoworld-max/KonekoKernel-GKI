// SPDX-License-Identifier: GPL-2.0-only
/*
 * dynamic_fsync - Screen-state aware fsync toggle
 *
 * Disables fsync while the screen is on to eliminate write-back
 * stalls during UI-critical moments. Re-enables fsync when the
 * screen turns off to guarantee data integrity before suspend.
 *
 * Uses backlight_register_notifier() instead of the legacy fbdev
 * fb_notifier/FB_EVENT_BLANK path, since DRM-panel devices only
 * expose fbdev as a thin compat shim and BLANK events are not
 * reliably delivered through it on those devices. The backlight
 * class device is present on virtually all Android panels and its
 * BACKLIGHT_UPDATE_STATUS event fires on every power-state change.
 *
 * The core fsync path (fs/sync.c, do_fsync()) checks
 * dynamic_fsync_should_skip() (see include/linux/dynamic_fsync.h)
 * before issuing any writeback.
 *
 * Userspace control:
 *   /sys/module/dynamic_fsync_driver/parameters/enabled  (1/0, def 1)
 */
#include <linux/module.h>
#include <linux/backlight.h>
#include <linux/notifier.h>
#include <linux/dynamic_fsync.h>

/*
 * Module parameter stored directly in the kernel's global so the
 * fs/sync.c hot path always reads the authoritative value.
 */
module_param_named(enabled, dynamic_fsync_enabled, bool, 0644);
MODULE_PARM_DESC(enabled, "Enable dynamic fsync (default: true)");

static int dfsync_bl_notifier(struct notifier_block *nb,
			       unsigned long event, void *data)
{
	struct backlight_device *bd = data;

	if (event != BACKLIGHT_UPDATE_STATUS)
		return NOTIFY_OK;

	if (!bd)
		return NOTIFY_OK;

	/*
	 * props.power: FB_BLANK_UNBLANK (0) = screen on, anything else
	 * (FB_BLANK_POWERDOWN, etc.) = screen off.
	 */
	WRITE_ONCE(dynamic_fsync_state,
		   (bd->props.power == FB_BLANK_UNBLANK) ? 2 : 1);

	return NOTIFY_OK;
}

static struct notifier_block dfsync_bl_nb = {
	.notifier_call = dfsync_bl_notifier,
};

static int __init dfsync_init(void)
{
	/* Default: assume screen is on at boot */
	WRITE_ONCE(dynamic_fsync_state, 2);

	backlight_register_notifier(&dfsync_bl_nb);

	pr_info("dynamic_fsync: enabled=%d, screen=ON\n",
		(int)READ_ONCE(dynamic_fsync_enabled));
	return 0;
}

static void __exit dfsync_exit(void)
{
	backlight_unregister_notifier(&dfsync_bl_nb);
	WRITE_ONCE(dynamic_fsync_state, 0);
}

module_init(dfsync_init);
module_exit(dfsync_exit);

MODULE_DESCRIPTION("Dynamic fsync — disable fsync when screen is on");
MODULE_AUTHOR("XTENSEI");
MODULE_LICENSE("GPL v2");
