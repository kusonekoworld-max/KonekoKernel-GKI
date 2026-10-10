// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2026 kusonekoworld
 *
 * CPUFreq governor "bara-no-seidou"
 *
 * Schedutil-derived governor tuned for battery efficiency and thermal
 * headroom during normal use, with an automatic switch to a
 * low-latency responsive profile when gaming mode is active
 * (sched_gaming_active, the same global driven by the fair.c
 * gaming-bias patch and readable/writable here via gaming_mode).
 *
 * Profiles / behavior:
 *  - Normal: long rate-limit windows, tiered headroom, thermal-pressure
 *    aware util scaling, capped upward frequency steps, per-cluster
 *    daily caps/floors. Little cluster is capped when UI is idle.
 *  - Touch boost: on a touchscreen event, step limiting is suspended
 *    and headroom is skipped for a short window so the first frames
 *    after a tap/scroll aren't throttled.
 *  - UI ramp assist: a sharp rise in util% re-arms the same "UI is
 *    active" window (fling scroll, open/close animations).
 *  - Gaming: rate limits collapse, up-rate-limit is zero, generous
 *    headroom and a hispeed floor apply. An exit-hysteresis window
 *    keeps the profile from flapping during loading screens.
 *  - Screen off: the global screen_on tunable (fed by userspace or by
 *    the fsync driver) hard-caps frequency for doze battery saving.
 *
 * The governor takes the max util across all CPUs of a policy (like
 * schedutil's shared path), so CPUs in the same cluster no longer
 * overwrite each other's frequency request.
 *
 * GPU devfreq locking is NOT done here: the devfreq API used by the
 * old code does not exist on 5.15 GKI (no lookup by name, no
 * min_freq/max_freq in struct devfreq). Do the GPU cap from userspace
 * through /sys/class/devfreq/<gpu>/{min,max}_freq when gaming_mode flips.
 *
 * All numeric knobs are sysfs tunables under
 * /sys/devices/system/cpu/cpufreq/policyX/bara-no-seidou/.
 * touch_boost_ms, screen_on and gaming_mode are global: writing them
 * under any policy affects every policy.
 */

#define pr_fmt(fmt) "bara-no-seidou: " fmt

#include <linux/cpufreq.h>
#include <linux/kthread.h>
#include <linux/percpu-defs.h>
#include <linux/slab.h>
#include <linux/irq_work.h>
#include <linux/input.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/sched/cpufreq.h>
#include <linux/sched/loadavg.h>
#include <linux/tick.h>
#ifdef CONFIG_PSI
#include <linux/psi.h>
#endif
#include <trace/events/power.h>
#include <uapi/linux/sched/types.h>

/* Internal scheduler header (kernel/sched/sched.h) - this is where
 * map_util_freq() and arch_scale_thermal_pressure() actually live.
 */
#include "sched.h"

extern int sched_gaming_active;

/* ---- Defaults (overridable at runtime via sysfs) --------------------- */

#define BARA_NO_SEIDOU_UP_RATE_LIMIT_NORMAL_US		(8000)
#define BARA_NO_SEIDOU_DOWN_RATE_LIMIT_NORMAL_US	(35000)
#define BARA_NO_SEIDOU_DOWN_RATE_LIMIT_GAME_US		(8000)

#define BARA_NO_SEIDOU_HEADROOM_GAMING_LITTLE_PCT_DEFAULT	(25)
#define BARA_NO_SEIDOU_HEADROOM_GAMING_BIG_PCT_DEFAULT		(15)
#define BARA_NO_SEIDOU_MAX_STEP_PCT_DEFAULT			(80)	/* normal profile */

#define BARA_NO_SEIDOU_HISPEED_LOAD_DEFAULT		(85)	/* pct, gaming only */
#define BARA_NO_SEIDOU_HISPEED_FREQ_PCT_DEFAULT		(70)	/* pct of max_freq */

#define BARA_NO_SEIDOU_THERMAL_DISCOUNT_MAX_PCT		(25)	/* extra util cut at full pressure */
#define BARA_NO_SEIDOU_GAMING_EXIT_DELAY_MS_DEFAULT	(3000)	/* gaming exit hysteresis */

#define BARA_NO_SEIDOU_TOUCH_BOOST_MS_DEFAULT		(220)	/* daily-use smoothness */
#define BARA_NO_SEIDOU_SCREEN_OFF_CAP_PCT_DEFAULT	(35)	/* pct of max_freq */

/* Daily UI ramp-assist: a sharp rise in util% re-arms the "UI is
 * active" floor even without a fresh touch event.
 */
#define BARA_NO_SEIDOU_RAMP_DELTA_PCT_DEFAULT		(15)	/* upct jump to re-arm */
#define BARA_NO_SEIDOU_UI_BOOST_MS_DEFAULT		(200)	/* how long the floor holds */
#define BARA_NO_SEIDOU_LITTLE_CAP_PCT_DEFAULT		(55)	/* little cap, UI idle */
#define BARA_NO_SEIDOU_LITTLE_BOOST_CAP_PCT_DEFAULT	(85)	/* little cap, UI active */
#define BARA_NO_SEIDOU_LITTLE_UI_FLOOR_PCT_DEFAULT	(35)	/* little floor, UI active */
#define BARA_NO_SEIDOU_BIG_UI_FLOOR_PCT_DEFAULT		(25)	/* big floor, UI active */

/* PSI pressure floor/cap - opt-in, off by default (0 = disabled). */
#define BARA_NO_SEIDOU_PSI_CPU_FLOOR_THRESH_DEFAULT	(0)
#define BARA_NO_SEIDOU_PSI_CPU_FLOOR_PCT_DEFAULT	(70)
#define BARA_NO_SEIDOU_PSI_MEM_CAP_THRESH_DEFAULT	(0)
#define BARA_NO_SEIDOU_PSI_MEM_CAP_PCT_DEFAULT		(80)

/* Directional EMA on policy util: rises fast (kills PELT-lag stutter),
 * decays slowly (no inter-frame sag). Shift 0 = no smoothing, 3 =
 * eighth-weight on the new sample.
 */
#define BARA_NO_SEIDOU_EMA_UP_SHIFT_DEFAULT		(0)
#define BARA_NO_SEIDOU_EMA_DOWN_SHIFT_DEFAULT		(3)

/* Smallest IOWait boost step, doubles on repeated IOWait ticks,
 * halves when they stop.
 */
#define IOWAIT_BOOST_MIN	(SCHED_CAPACITY_SCALE / 8)

/* ---- Global state (shared across all policies/clusters) ---------------- */

static atomic64_t bara_no_seidou_touch_boost_until_ns = ATOMIC64_INIT(0);
static unsigned int bara_no_seidou_touch_boost_ms __read_mostly =
	BARA_NO_SEIDOU_TOUCH_BOOST_MS_DEFAULT;
static bool bara_no_seidou_screen_on __read_mostly = true;

static inline bool bara_no_seidou_touch_boosted(u64 time)
{
	return (s64)(atomic64_read(&bara_no_seidou_touch_boost_until_ns) - time) > 0;
}

static void bara_no_seidou_touch_event(struct input_handle *handle,
				       unsigned int type, unsigned int code,
				       int value)
{
	/* Only events that mean "finger is on / moving on the panel".
	 * Everything else on the device (sync, slot, pressure, ...) is noise.
	 */
	if (type == EV_ABS) {
		if (code != ABS_MT_POSITION_X && code != ABS_MT_POSITION_Y &&
		    code != ABS_MT_TRACKING_ID)
			return;
	} else if (type == EV_KEY) {
		if (code != BTN_TOUCH)
			return;
	} else {
		return;
	}

	atomic64_set(&bara_no_seidou_touch_boost_until_ns,
		     local_clock() +
		     (u64)READ_ONCE(bara_no_seidou_touch_boost_ms) * NSEC_PER_MSEC);
}

static int bara_no_seidou_input_connect(struct input_handler *handler,
					struct input_dev *dev,
					const struct input_device_id *id)
{
	struct input_handle *handle;
	int ret;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "bara_no_seidou_touch";

	ret = input_register_handle(handle);
	if (ret)
		goto err_free;

	ret = input_open_device(handle);
	if (ret)
		goto err_unregister;

	return 0;

err_unregister:
	input_unregister_handle(handle);
err_free:
	kfree(handle);
	return ret;
}

static void bara_no_seidou_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

/* Match multi-touch touchscreens only. A bare EV_ABS match also attaches
 * to sensors/gamepads and floods the handler with irrelevant events.
 */
static const struct input_device_id bara_no_seidou_input_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			 INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] =
			    BIT_MASK(ABS_MT_POSITION_X) |
			    BIT_MASK(ABS_MT_POSITION_Y) },
	},
	{ },
};

static struct input_handler bara_no_seidou_input_handler = {
	.event		= bara_no_seidou_touch_event,
	.connect	= bara_no_seidou_input_connect,
	.disconnect	= bara_no_seidou_input_disconnect,
	.name		= "bara_no_seidou_touch",
	.id_table	= bara_no_seidou_input_ids,
};

/* Registered once for all policies. Tracks whether registration really
 * succeeded so exit never unregisters a handler that was never added.
 */
static DEFINE_MUTEX(bara_no_seidou_input_lock);
static unsigned int bara_no_seidou_input_users;
static bool bara_no_seidou_input_registered;

static void bara_no_seidou_input_get(void)
{
	int ret;

	mutex_lock(&bara_no_seidou_input_lock);
	bara_no_seidou_input_users++;
	if (!bara_no_seidou_input_registered) {
		ret = input_register_handler(&bara_no_seidou_input_handler);
		if (ret)
			pr_warn("touch boost unavailable (input_register_handler failed: %d)\n",
				ret);
		else
			bara_no_seidou_input_registered = true;
	}
	mutex_unlock(&bara_no_seidou_input_lock);
}

static void bara_no_seidou_input_put(void)
{
	mutex_lock(&bara_no_seidou_input_lock);
	if (bara_no_seidou_input_users &&
	    !--bara_no_seidou_input_users &&
	    bara_no_seidou_input_registered) {
		input_unregister_handler(&bara_no_seidou_input_handler);
		bara_no_seidou_input_registered = false;
	}
	mutex_unlock(&bara_no_seidou_input_lock);
}

/* ---- Types ------------------------------------------------------------ */

struct bara_no_seidou_tunables {
	struct kobject kobj;
	struct bara_no_seidou_policy *gd_policy;

	unsigned int up_rate_limit_normal_us;
	unsigned int down_rate_limit_normal_us;
	unsigned int down_rate_limit_game_us;

	unsigned int headroom_gaming_little_pct;
	unsigned int headroom_gaming_big_pct;
	unsigned int max_step_pct;

	unsigned int hispeed_load;
	unsigned int hispeed_freq_pct;

	unsigned int gaming_exit_delay_ms;
	unsigned int screen_off_cap_pct;
	unsigned int ramp_delta_pct;
	unsigned int ui_boost_ms;
	unsigned int little_cap_pct;
	unsigned int little_boost_cap_pct;
	unsigned int little_ui_floor_pct;
	unsigned int big_ui_floor_pct;
	unsigned int psi_cpu_floor_thresh;
	unsigned int psi_cpu_floor_pct;
	unsigned int psi_mem_cap_thresh;
	unsigned int psi_mem_cap_pct;
	unsigned int ema_up_shift;
	unsigned int ema_down_shift;

	bool thermal_aware;
};

struct bara_no_seidou_policy {
	struct cpufreq_policy *policy;
	struct bara_no_seidou_tunables *tunables;
	bool is_little;

	raw_spinlock_t update_lock;
	u64 last_freq_update_time;
	s64 up_rate_delay_ns;
	s64 down_rate_delay_ns;
	unsigned int next_freq;
	unsigned int cached_raw_freq;
	unsigned int cached_resolved_freq;
	int last_gaming_state;
	u64 last_gaming_active_ns;

	/* policy-wide (max over CPUs) util state */
	unsigned long smoothed_util;
	unsigned int prev_upct;
	u64 ui_boost_end_ns;

	/* thermal step controller state */
	int thermal_applied_pct;
	u64 thermal_step_ns;

	bool screen_on_last;

	struct irq_work irq_work;
	struct kthread_work work;
	struct kthread_worker worker;
	struct task_struct *thread;
	struct mutex work_lock;
	bool work_in_progress;
	bool limits_changed;
	bool need_freq_update;
};

struct bara_no_seidou_cpu {
	struct update_util_data update_util;
	struct bara_no_seidou_policy *gd_policy;
	unsigned int cpu;

	/* Decaying IOWait boost */
	bool iowait_boost_pending;
	unsigned int iowait_boost;
	u64 last_update;

#ifdef CONFIG_NO_HZ_COMMON
	unsigned long saved_idle_calls;
#endif
};

static DEFINE_PER_CPU(struct bara_no_seidou_cpu, bara_no_seidou_cpu_list);

/* ---- Rate-limit delays ---------------------------------------------------- */

static void bara_no_seidou_set_delays(struct bara_no_seidou_policy *gd_policy,
				      bool gaming)
{
	struct bara_no_seidou_tunables *t = gd_policy->tunables;
	unsigned int up, down;

	/* Instant up while gaming - never delay a frame that needs more
	 * clock, only the downward step stays rate-limited.
	 */
	up = gaming ? 0 : READ_ONCE(t->up_rate_limit_normal_us);
	down = gaming ? READ_ONCE(t->down_rate_limit_game_us) :
			READ_ONCE(t->down_rate_limit_normal_us);

	gd_policy->up_rate_delay_ns = (s64)up * NSEC_PER_USEC;
	gd_policy->down_rate_delay_ns = (s64)down * NSEC_PER_USEC;
}

/* Called from sysfs after any tunable write, so rate limits and the
 * rest take effect immediately instead of at the next gaming edge.
 */
static void bara_no_seidou_tunables_changed(struct bara_no_seidou_tunables *t)
{
	struct bara_no_seidou_policy *gd_policy = t->gd_policy;
	unsigned long flags;

	raw_spin_lock_irqsave(&gd_policy->update_lock, flags);
	bara_no_seidou_set_delays(gd_policy, gd_policy->last_gaming_state == 1);
	gd_policy->need_freq_update = true;
	raw_spin_unlock_irqrestore(&gd_policy->update_lock, flags);
}

/* ---- Gaming state (raw + exit-hysteresis) ------------------------------ */

static bool bara_no_seidou_gaming(struct bara_no_seidou_policy *gd_policy,
				  u64 time)
{
	struct bara_no_seidou_tunables *t = gd_policy->tunables;

	if (READ_ONCE(sched_gaming_active)) {
		gd_policy->last_gaming_active_ns = time;
		return true;
	}

	if (!gd_policy->last_gaming_active_ns)
		return false;

	/* signed compare: rq clocks of different CPUs can be slightly skewed */
	return (s64)(time - gd_policy->last_gaming_active_ns) <
	       (s64)READ_ONCE(t->gaming_exit_delay_ms) * NSEC_PER_MSEC;
}

/* ---- PSI sustained-pressure floor / cap ---------------------------------- */

/*
 * System-wide CPU "some" pressure, 10s EWMA, as an integer percentage.
 * Returns 0 when CONFIG_PSI is off or PSI is disabled at runtime, which
 * makes the floor below a permanent no-op rather than a build failure.
 */
static inline unsigned int bara_no_seidou_psi_cpu_pct(void)
{
#ifdef CONFIG_PSI
	unsigned long avg;

	if (static_branch_likely(&psi_disabled))
		return 0;

	avg = READ_ONCE(psi_system.avg[PSI_CPU_SOME][0]);
	return (unsigned int)LOAD_INT(avg);
#else
	return 0;
#endif
}

static inline unsigned int bara_no_seidou_psi_mem_pct(void)
{
#ifdef CONFIG_PSI
	unsigned long avg;

	if (static_branch_likely(&psi_disabled))
		return 0;

	avg = READ_ONCE(psi_system.avg[PSI_MEM_SOME][0]);
	return (unsigned int)LOAD_INT(avg);
#else
	return 0;
#endif
}

/*
 * PSI's 10s smoothing window only makes sense for *sustained* CPU
 * pressure (background sync fighting the foreground app, etc.).
 * Opt-in (threshold 0 = off). Never applied during gaming.
 */
static unsigned int bara_no_seidou_psi_floor(struct bara_no_seidou_policy *gd_policy,
					     bool gaming)
{
	struct bara_no_seidou_tunables *t = gd_policy->tunables;
	unsigned int max_freq = gd_policy->policy->cpuinfo.max_freq;

	if (gaming || !t->psi_cpu_floor_thresh)
		return 0;

	if (bara_no_seidou_psi_cpu_pct() < t->psi_cpu_floor_thresh)
		return 0;

	return mult_frac(max_freq, t->psi_cpu_floor_pct, 100);
}

/*
 * Inverse of the floor: sustained memory pressure means the CPU is
 * stalled on paging, not short on clock - a cap, not a floor. Opt-in.
 * Applies during gaming too, more clock cannot fix a memory stall.
 */
static unsigned int bara_no_seidou_psi_mem_cap(struct bara_no_seidou_policy *gd_policy)
{
	struct bara_no_seidou_tunables *t = gd_policy->tunables;
	unsigned int max_freq = gd_policy->policy->cpuinfo.max_freq;

	if (!t->psi_mem_cap_thresh)
		return 0;

	if (bara_no_seidou_psi_mem_pct() < t->psi_mem_cap_thresh)
		return 0;

	return mult_frac(max_freq, t->psi_mem_cap_pct, 100);
}

/* ---- Resume / profile refresh --------------------------------------------- */

/*
 * On the screen-off -> screen-on edge, stale hysteresis/decay state from
 * before a (possibly long) suspend must not leak into the first
 * post-resume ticks.
 */
static void bara_no_seidou_resume_reset(struct bara_no_seidou_policy *gd_policy)
{
	bool screen_on = READ_ONCE(bara_no_seidou_screen_on);

	if (!gd_policy->screen_on_last && screen_on) {
		gd_policy->last_gaming_active_ns = 0;
		gd_policy->ui_boost_end_ns = 0;
		gd_policy->prev_upct = 0;
		gd_policy->smoothed_util = 0;
		gd_policy->thermal_applied_pct = 0;
		gd_policy->thermal_step_ns = 0;
		gd_policy->need_freq_update = true;
	}
	gd_policy->screen_on_last = screen_on;
}

static bool bara_no_seidou_refresh_profile(struct bara_no_seidou_policy *gd_policy,
					   u64 time)
{
	bool gaming;

	bara_no_seidou_resume_reset(gd_policy);
	gaming = bara_no_seidou_gaming(gd_policy, time);

	if (gd_policy->last_gaming_state != (int)gaming) {
		bara_no_seidou_set_delays(gd_policy, gaming);
		gd_policy->last_gaming_state = gaming;
	}

	return gaming;
}

static bool bara_no_seidou_rate_limited(struct bara_no_seidou_policy *gd_policy,
					u64 time, unsigned int next_freq)
{
	s64 delta_ns = time - gd_policy->last_freq_update_time;

	/* An external limits change (e.g. thermal engine lowering max_freq)
	 * must take effect immediately - consume the flag once so we don't
	 * bypass rate-limiting on every subsequent call.
	 */
	if (gd_policy->limits_changed) {
		gd_policy->limits_changed = false;
		return false;
	}

	if (next_freq > gd_policy->next_freq &&
	    delta_ns < gd_policy->up_rate_delay_ns)
		return true;

	if (next_freq < gd_policy->next_freq &&
	    delta_ns < gd_policy->down_rate_delay_ns)
		return true;

	return false;
}

/* ---- Thermal step controller -------------------------------------------- */

#define BARA_NO_SEIDOU_THERMAL_STEP_NS		(200000000ULL)	/* 200ms tick */
#define BARA_NO_SEIDOU_THERMAL_STEP_DOWN_PCT	(4)	/* enter throttle fast */
#define BARA_NO_SEIDOU_THERMAL_STEP_UP_PCT	(2)	/* recover gently, no bounce */
#define BARA_NO_SEIDOU_THERMAL_MIN_CAP_PCT	(60)

/*
 * Returns a percentage (60..100) the requested util is scaled by.
 *
 * arch_scale_thermal_pressure() is in capacity units (0..cpu capacity),
 * so it is normalised to a percentage first. The result scales util
 * down: shrinking the *capacity* argument of map_util_freq() instead
 * would RAISE the requested frequency, the opposite of the intent.
 *
 * The applied value walks toward the target in small rate-limited
 * steps (down faster than up) for a smooth throttle entry and a gentle
 * recovery with no oscillation around the trip point.
 */
static unsigned int bara_no_seidou_thermal_scale_pct(struct bara_no_seidou_policy *gd_policy,
						     int cpu, unsigned long cap,
						     u64 time)
{
	unsigned long pressure_pct;
	int target, applied;

	if (!gd_policy->tunables->thermal_aware || !cap)
		return 100;

	pressure_pct = min_t(unsigned long, 100,
			     arch_scale_thermal_pressure(cpu) * 100 / cap);
	target = 100 - (int)mult_frac(pressure_pct,
				      BARA_NO_SEIDOU_THERMAL_DISCOUNT_MAX_PCT, 100);
	target = clamp(target, BARA_NO_SEIDOU_THERMAL_MIN_CAP_PCT, 100);

	applied = gd_policy->thermal_applied_pct;
	if (!applied)
		applied = 100;

	if ((s64)(time - gd_policy->thermal_step_ns) >=
	    (s64)BARA_NO_SEIDOU_THERMAL_STEP_NS) {
		if (applied > target)
			applied -= BARA_NO_SEIDOU_THERMAL_STEP_DOWN_PCT;
		else if (applied < target)
			applied += BARA_NO_SEIDOU_THERMAL_STEP_UP_PCT;
		applied = clamp(applied, BARA_NO_SEIDOU_THERMAL_MIN_CAP_PCT, 100);
		gd_policy->thermal_applied_pct = applied;
		gd_policy->thermal_step_ns = time;
	}

	return (unsigned int)applied;
}

/* ---- Frequency selection ----------------------------------------------- */

static unsigned int bara_no_seidou_apply_step_limit(struct bara_no_seidou_policy *gd_policy,
						    unsigned int cur,
						    unsigned int target,
						    bool gaming, bool boosted)
{
	unsigned int max_freq = gd_policy->policy->cpuinfo.max_freq;
	unsigned int step_cap;

	if (gaming || boosted || target <= cur || cur >= max_freq)
		return target;

	step_cap = cur + mult_frac(max_freq - cur,
				   gd_policy->tunables->max_step_pct, 100);
	return min(target, step_cap);
}

static unsigned int bara_no_seidou_hispeed_floor(struct bara_no_seidou_policy *gd_policy,
						 unsigned long util, unsigned long cap,
						 bool gaming)
{
	struct bara_no_seidou_tunables *t = gd_policy->tunables;
	unsigned int max_freq = gd_policy->policy->cpuinfo.max_freq;

	if (!gaming)
		return 0;

	if (util * 100 < cap * t->hispeed_load)
		return 0;

	return mult_frac(max_freq, t->hispeed_freq_pct, 100);
}

static unsigned int bara_no_seidou_screen_off_cap(struct bara_no_seidou_policy *gd_policy)
{
	struct bara_no_seidou_tunables *t = gd_policy->tunables;
	unsigned int max_freq = gd_policy->policy->cpuinfo.max_freq;

	if (READ_ONCE(bara_no_seidou_screen_on))
		return 0;

	return mult_frac(max_freq, t->screen_off_cap_pct, 100);
}

/*
 * Headroom, not discount: request slightly MORE capacity than measured
 * so the resolved OPP has slack. Daily uses a tiered curve - negligible
 * headroom at low util (battery), more as util climbs (responsiveness).
 * Gaming uses a flat generous headroom.
 */
static unsigned long bara_no_seidou_apply_headroom(struct bara_no_seidou_policy *gd_policy,
						   unsigned long util,
						   unsigned long cap, bool gaming)
{
	struct bara_no_seidou_tunables *t = gd_policy->tunables;
	unsigned int upct;

	if (!cap || util >= cap)
		return cap;

	upct = (unsigned int)(util * 100 / cap);
	if (upct >= 95)
		return cap;

	if (gaming) {
		unsigned int h = gd_policy->is_little ?
			t->headroom_gaming_little_pct : t->headroom_gaming_big_pct;
		return min(util + util * h / 100, cap);
	}

	if (gd_policy->is_little) {
		if (upct >= 70)
			return min(util + (util >> 4), cap);
		if (upct >= 45)
			return min(util + (util >> 5), cap);
		return util;
	}

	if (upct >= 75)
		return min(util + (util >> 4), cap);
	if (upct >= 50)
		return min(util + (util >> 5), cap);
	return min(util + (util >> 6), cap);
}

/*
 * A sharp rise in util% - not just a fresh touch event - re-arms the
 * same "UI is active" window that touch-boost uses.
 */
static bool bara_no_seidou_ui_ramp_active(struct bara_no_seidou_policy *gd_policy,
					  unsigned long util, unsigned long cap,
					  u64 time)
{
	struct bara_no_seidou_tunables *t = gd_policy->tunables;
	unsigned int upct = cap ? (unsigned int)(util * 100 / cap) : 0;

	if (upct > gd_policy->prev_upct &&
	    upct - gd_policy->prev_upct >= t->ramp_delta_pct)
		gd_policy->ui_boost_end_ns = time +
			(u64)t->ui_boost_ms * NSEC_PER_MSEC;

	gd_policy->prev_upct = upct;

	return (s64)(gd_policy->ui_boost_end_ns - time) > 0;
}

/*
 * Explicit daily-profile shaping: little cluster gets a cap normally
 * (keeps it out of the power-hungry top end for background work) that
 * opens up while UI is active; both clusters get a floor while UI is
 * active so a fresh gesture never has to climb from a cold frequency.
 * Never applied during gaming - the hispeed floor owns that there.
 */
static unsigned int bara_no_seidou_apply_daily_shape(struct bara_no_seidou_policy *gd_policy,
						     unsigned int freq, bool ui_active)
{
	struct bara_no_seidou_tunables *t = gd_policy->tunables;
	unsigned int max_freq = gd_policy->policy->cpuinfo.max_freq;
	unsigned int cap_f, floor_f;

	if (gd_policy->is_little) {
		cap_f = mult_frac(max_freq, ui_active ? t->little_boost_cap_pct :
							t->little_cap_pct, 100);
		if (freq > cap_f)
			freq = cap_f;
		if (ui_active) {
			floor_f = mult_frac(max_freq, t->little_ui_floor_pct, 100);
			if (freq < floor_f)
				freq = floor_f;
		}
	} else if (ui_active) {
		floor_f = mult_frac(max_freq, t->big_ui_floor_pct, 100);
		if (freq < floor_f)
			freq = floor_f;
	}

	return freq;
}

/*
 * Returns the *raw* (not yet OPP-resolved) target. The caller resolves
 * it once at the very end: qcom-cpufreq-hw fast_switch applies
 * policy->cached_resolved_idx, so every floor/cap must be in place
 * before cpufreq_driver_resolve_freq() runs.
 */
static unsigned int bara_no_seidou_next_freq(struct bara_no_seidou_policy *gd_policy,
					     int cpu, unsigned long util,
					     unsigned long cap, u64 time,
					     bool gaming)
{
	struct cpufreq_policy *policy = gd_policy->policy;
	unsigned int base_freq = arch_scale_freq_invariant() ?
				 policy->cpuinfo.max_freq : policy->cur;
	bool boosted = bara_no_seidou_touch_boosted(time);
	bool ramp = bara_no_seidou_ui_ramp_active(gd_policy, util, cap, time);
	bool ui_active = boosted || ramp;
	unsigned int thermal_pct, freq, floor_freq, cap_freq;
	unsigned long next_util;

	next_util = boosted ? util :
		    bara_no_seidou_apply_headroom(gd_policy, util, cap, gaming);

	thermal_pct = bara_no_seidou_thermal_scale_pct(gd_policy, cpu, cap, time);
	if (thermal_pct < 100)
		next_util = mult_frac(next_util, thermal_pct, 100);

	freq = map_util_freq(next_util, base_freq, cap);
	freq = bara_no_seidou_apply_step_limit(gd_policy, policy->cur, freq,
					       gaming, boosted);

	floor_freq = bara_no_seidou_hispeed_floor(gd_policy, util, cap, gaming);
	if (floor_freq)
		freq = max(freq, floor_freq);

	if (!gaming)
		freq = bara_no_seidou_apply_daily_shape(gd_policy, freq, ui_active);

	floor_freq = bara_no_seidou_psi_floor(gd_policy, gaming);
	if (floor_freq)
		freq = max(freq, floor_freq);

	cap_freq = bara_no_seidou_psi_mem_cap(gd_policy);
	if (cap_freq)
		freq = min(freq, cap_freq);

	/* screen-off cap always wins - it's a hard ceiling, not a floor */
	cap_freq = bara_no_seidou_screen_off_cap(gd_policy);
	if (cap_freq)
		freq = min(freq, cap_freq);

	return freq;
}

/* ---- Deferred frequency change (kthread slow path) ---------------------- */

static void bara_no_seidou_work(struct kthread_work *work)
{
	struct bara_no_seidou_policy *gd_policy =
		container_of(work, struct bara_no_seidou_policy, work);
	unsigned int freq;
	unsigned long flags;

	mutex_lock(&gd_policy->work_lock);

	raw_spin_lock_irqsave(&gd_policy->update_lock, flags);
	freq = gd_policy->next_freq;
	gd_policy->work_in_progress = false;
	raw_spin_unlock_irqrestore(&gd_policy->update_lock, flags);

	__cpufreq_driver_target(gd_policy->policy, freq, CPUFREQ_RELATION_L);
	mutex_unlock(&gd_policy->work_lock);
}

static void bara_no_seidou_irq_work(struct irq_work *irq_work)
{
	struct bara_no_seidou_policy *gd_policy =
		container_of(irq_work, struct bara_no_seidou_policy, irq_work);

	kthread_queue_work(&gd_policy->worker, &gd_policy->work);
}

static void bara_no_seidou_queue_update(struct bara_no_seidou_policy *gd_policy)
{
	if (gd_policy->work_in_progress)
		return;

	gd_policy->work_in_progress = true;
	irq_work_queue(&gd_policy->irq_work);
}

/* ---- Directional EMA util smoothing (policy-wide) ----------------------- */

static unsigned long bara_no_seidou_ema_smooth(struct bara_no_seidou_policy *gd_policy,
					       unsigned long raw_util)
{
	struct bara_no_seidou_tunables *t = gd_policy->tunables;
	unsigned long prev = gd_policy->smoothed_util;
	unsigned int shift;

	if (!prev) {
		gd_policy->smoothed_util = raw_util;
		return raw_util;
	}

	shift = raw_util >= prev ? t->ema_up_shift : t->ema_down_shift;
	gd_policy->smoothed_util = prev + ((long)(raw_util - prev) >> shift);
	return gd_policy->smoothed_util;
}

/* ---- Hold frequency across brief idle blips ----------------------------- */

#ifdef CONFIG_NO_HZ_COMMON
static bool bara_no_seidou_hold_freq(struct bara_no_seidou_cpu *gd_cpu)
{
	unsigned long idle_calls;
	bool ret;

	idle_calls = tick_nohz_get_idle_calls_cpu(gd_cpu->cpu);
	ret = idle_calls == gd_cpu->saved_idle_calls;
	gd_cpu->saved_idle_calls = idle_calls;
	return ret;
}
#else
static inline bool bara_no_seidou_hold_freq(struct bara_no_seidou_cpu *gd_cpu) { return false; }
#endif

/* ---- Decaying IOWait boost ---------------------------------------------- */

static bool bara_no_seidou_iowait_reset(struct bara_no_seidou_cpu *gd_cpu, u64 time,
					bool set_iowait_boost)
{
	s64 delta_ns = time - gd_cpu->last_update;

	if (delta_ns <= TICK_NSEC)
		return false;

	gd_cpu->iowait_boost = set_iowait_boost ? IOWAIT_BOOST_MIN : 0;
	gd_cpu->iowait_boost_pending = set_iowait_boost;
	return true;
}

static void bara_no_seidou_iowait_boost(struct bara_no_seidou_cpu *gd_cpu, u64 time,
					unsigned int flags)
{
	bool set_iowait_boost = flags & SCHED_CPUFREQ_IOWAIT;

	if (gd_cpu->iowait_boost &&
	    bara_no_seidou_iowait_reset(gd_cpu, time, set_iowait_boost))
		return;

	if (!set_iowait_boost)
		return;

	if (gd_cpu->iowait_boost_pending)
		return;
	gd_cpu->iowait_boost_pending = true;

	if (gd_cpu->iowait_boost) {
		gd_cpu->iowait_boost = min_t(unsigned int,
					     gd_cpu->iowait_boost << 1,
					     SCHED_CAPACITY_SCALE);
		return;
	}

	gd_cpu->iowait_boost = IOWAIT_BOOST_MIN;
}

static unsigned long bara_no_seidou_iowait_apply(struct bara_no_seidou_cpu *gd_cpu,
						 u64 time, unsigned long cap)
{
	if (!gd_cpu->iowait_boost)
		return 0;

	if (bara_no_seidou_iowait_reset(gd_cpu, time, false))
		return 0;

	if (!gd_cpu->iowait_boost_pending) {
		gd_cpu->iowait_boost >>= 1;
		if (gd_cpu->iowait_boost < IOWAIT_BOOST_MIN) {
			gd_cpu->iowait_boost = 0;
			return 0;
		}
	}

	gd_cpu->iowait_boost_pending = false;
	return (gd_cpu->iowait_boost * cap) >> SCHED_CAPACITY_SHIFT;
}

/* ---- Fast path (per-cpu utilization callback) --------------------------- */

static void bara_no_seidou_update_single_cpu(struct bara_no_seidou_cpu *gd_cpu,
					     u64 time, unsigned int flags)
{
	struct bara_no_seidou_policy *gd_policy = gd_cpu->gd_policy;
	struct cpufreq_policy *policy = gd_policy->policy;
	unsigned long util = 0, cap, j_util, boost;
	unsigned int next_f;
	bool gaming;
	int j;

	gaming = bara_no_seidou_refresh_profile(gd_policy, time);

	cap = arch_scale_cpu_capacity(gd_cpu->cpu);

	bara_no_seidou_iowait_boost(gd_cpu, time, flags);
	gd_cpu->last_update = time;

	/* Policy-wide max util, like schedutil's shared path. Without this
	 * every CPU of a cluster would overwrite the others' request.
	 */
	for_each_cpu(j, policy->cpus) {
		struct bara_no_seidou_cpu *j_cpu = &per_cpu(bara_no_seidou_cpu_list, j);

		j_util = sched_cpu_util(j, cap);
		boost = bara_no_seidou_iowait_apply(j_cpu, time, cap);
		util = max(util, max(j_util, boost));
	}

	util = bara_no_seidou_ema_smooth(gd_policy, util);

	next_f = bara_no_seidou_next_freq(gd_policy, gd_cpu->cpu, util, cap,
					  time, gaming);

	/* Resolve once, after every floor/cap. cpufreq_driver_resolve_freq()
	 * also records the table index fast_switch drivers apply, and clamps
	 * to policy->min/max.
	 */
	if (next_f != gd_policy->cached_raw_freq || gd_policy->need_freq_update) {
		gd_policy->cached_raw_freq = next_f;
		next_f = cpufreq_driver_resolve_freq(policy, next_f);
		gd_policy->cached_resolved_freq = next_f;
	} else {
		next_f = gd_policy->cached_resolved_freq;
	}

	/* Don't let a brief idle blip drop freq below what's already
	 * running - avoids the measure-low / correct-next-tick sawtooth.
	 */
	if (bara_no_seidou_hold_freq(gd_cpu) && next_f < gd_policy->next_freq &&
	    !gd_policy->need_freq_update)
		next_f = gd_policy->next_freq;

	/* Nothing changed: skip the driver call entirely. */
	if (next_f == gd_policy->next_freq && !gd_policy->need_freq_update)
		return;

	if (bara_no_seidou_rate_limited(gd_policy, time, next_f))
		return;

	gd_policy->need_freq_update = false;
	gd_policy->next_freq = next_f;
	gd_policy->last_freq_update_time = time;

	if (policy->fast_switch_enabled)
		cpufreq_driver_fast_switch(policy, next_f);
	else
		bara_no_seidou_queue_update(gd_policy);
}

static void bara_no_seidou_update_util(struct update_util_data *hook, u64 time,
				       unsigned int flags)
{
	struct bara_no_seidou_cpu *gd_cpu = container_of(hook, struct bara_no_seidou_cpu,
							 update_util);
	struct bara_no_seidou_policy *gd_policy = gd_cpu->gd_policy;

	/* Only the CPU actually entitled to drive this policy may act on
	 * it. Skips spurious/racy updates from a CPU mid-hotplug.
	 */
	if (!cpufreq_this_cpu_can_update(gd_policy->policy))
		return;

	raw_spin_lock(&gd_policy->update_lock);
	bara_no_seidou_update_single_cpu(gd_cpu, time, flags);
	raw_spin_unlock(&gd_policy->update_lock);
}

/* ---- sysfs tunables ------------------------------------------------------ */

#define to_bara_no_seidou_tunables(k) \
	container_of(k, struct bara_no_seidou_tunables, kobj)

#define BARA_NO_SEIDOU_ATTR_RW(_name, _max)					\
static ssize_t _name##_show(struct kobject *kobj,				\
			    struct kobj_attribute *attr, char *buf)		\
{										\
	struct bara_no_seidou_tunables *t = to_bara_no_seidou_tunables(kobj);	\
	return sysfs_emit(buf, "%u\n", READ_ONCE(t->_name));			\
}										\
static ssize_t _name##_store(struct kobject *kobj,				\
			     struct kobj_attribute *attr,			\
			     const char *buf, size_t count)			\
{										\
	struct bara_no_seidou_tunables *t = to_bara_no_seidou_tunables(kobj);	\
	unsigned int val;							\
	if (kstrtouint(buf, 10, &val) || val > (_max))				\
		return -EINVAL;							\
	WRITE_ONCE(t->_name, val);						\
	bara_no_seidou_tunables_changed(t);					\
	return count;								\
}										\
static struct kobj_attribute _name##_attr =					\
	__ATTR(_name, 0644, _name##_show, _name##_store)

#define BARA_NO_SEIDOU_ATTR_RW_BOOL(_name)					\
static ssize_t _name##_show(struct kobject *kobj,				\
			    struct kobj_attribute *attr, char *buf)		\
{										\
	struct bara_no_seidou_tunables *t = to_bara_no_seidou_tunables(kobj);	\
	return sysfs_emit(buf, "%u\n", READ_ONCE(t->_name));			\
}										\
static ssize_t _name##_store(struct kobject *kobj,				\
			     struct kobj_attribute *attr,			\
			     const char *buf, size_t count)			\
{										\
	struct bara_no_seidou_tunables *t = to_bara_no_seidou_tunables(kobj);	\
	unsigned int val;							\
	if (kstrtouint(buf, 10, &val))						\
		return -EINVAL;							\
	WRITE_ONCE(t->_name, !!val);						\
	bara_no_seidou_tunables_changed(t);					\
	return count;								\
}										\
static struct kobj_attribute _name##_attr =					\
	__ATTR(_name, 0644, _name##_show, _name##_store)

BARA_NO_SEIDOU_ATTR_RW(up_rate_limit_normal_us, 1000000);
BARA_NO_SEIDOU_ATTR_RW(down_rate_limit_normal_us, 1000000);
BARA_NO_SEIDOU_ATTR_RW(down_rate_limit_game_us, 1000000);
BARA_NO_SEIDOU_ATTR_RW(headroom_gaming_little_pct, 100);
BARA_NO_SEIDOU_ATTR_RW(headroom_gaming_big_pct, 100);
BARA_NO_SEIDOU_ATTR_RW(max_step_pct, 100);
BARA_NO_SEIDOU_ATTR_RW(hispeed_load, 100);
BARA_NO_SEIDOU_ATTR_RW(hispeed_freq_pct, 100);
BARA_NO_SEIDOU_ATTR_RW(ema_up_shift, 10);
BARA_NO_SEIDOU_ATTR_RW(ema_down_shift, 10);
BARA_NO_SEIDOU_ATTR_RW(gaming_exit_delay_ms, 60000);
BARA_NO_SEIDOU_ATTR_RW(screen_off_cap_pct, 100);
BARA_NO_SEIDOU_ATTR_RW(ramp_delta_pct, 100);
BARA_NO_SEIDOU_ATTR_RW(ui_boost_ms, 5000);
BARA_NO_SEIDOU_ATTR_RW(little_cap_pct, 100);
BARA_NO_SEIDOU_ATTR_RW(little_boost_cap_pct, 100);
BARA_NO_SEIDOU_ATTR_RW(little_ui_floor_pct, 100);
BARA_NO_SEIDOU_ATTR_RW(big_ui_floor_pct, 100);
BARA_NO_SEIDOU_ATTR_RW(psi_cpu_floor_thresh, 100);
BARA_NO_SEIDOU_ATTR_RW(psi_cpu_floor_pct, 100);
BARA_NO_SEIDOU_ATTR_RW(psi_mem_cap_thresh, 100);
BARA_NO_SEIDOU_ATTR_RW(psi_mem_cap_pct, 100);
BARA_NO_SEIDOU_ATTR_RW_BOOL(thermal_aware);

/* Global: one shared value, visible/writable from every policy dir. */
static ssize_t touch_boost_ms_show(struct kobject *kobj,
				   struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n", READ_ONCE(bara_no_seidou_touch_boost_ms));
}

static ssize_t touch_boost_ms_store(struct kobject *kobj,
				    struct kobj_attribute *attr,
				    const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val > 5000)
		return -EINVAL;

	WRITE_ONCE(bara_no_seidou_touch_boost_ms, val);
	return count;
}
static struct kobj_attribute touch_boost_ms_attr =
	__ATTR(touch_boost_ms, 0644, touch_boost_ms_show, touch_boost_ms_store);

/* Global: write once (e.g. from the fsync driver / a daemon), every
 * policy follows.
 */
static ssize_t screen_on_show(struct kobject *kobj,
			      struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n", READ_ONCE(bara_no_seidou_screen_on));
}

static ssize_t screen_on_store(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	WRITE_ONCE(bara_no_seidou_screen_on, !!val);
	return count;
}
static struct kobj_attribute screen_on_attr =
	__ATTR(screen_on, 0644, screen_on_show, screen_on_store);

/*
 * Manual gaming-mode override. Reads/writes sched_gaming_active
 * directly - the same global the fair.c gaming bias uses.
 */
static ssize_t gaming_mode_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%d\n", READ_ONCE(sched_gaming_active));
}

static ssize_t gaming_mode_store(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 const char *buf, size_t count)
{
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	WRITE_ONCE(sched_gaming_active, !!val);
	return count;
}
static struct kobj_attribute gaming_mode_attr =
	__ATTR(gaming_mode, 0644, gaming_mode_show, gaming_mode_store);

static struct attribute *bara_no_seidou_attrs[] = {
	&up_rate_limit_normal_us_attr.attr,
	&down_rate_limit_normal_us_attr.attr,
	&down_rate_limit_game_us_attr.attr,
	&headroom_gaming_little_pct_attr.attr,
	&headroom_gaming_big_pct_attr.attr,
	&max_step_pct_attr.attr,
	&hispeed_load_attr.attr,
	&hispeed_freq_pct_attr.attr,
	&ema_up_shift_attr.attr,
	&ema_down_shift_attr.attr,
	&gaming_exit_delay_ms_attr.attr,
	&touch_boost_ms_attr.attr,
	&screen_off_cap_pct_attr.attr,
	&ramp_delta_pct_attr.attr,
	&ui_boost_ms_attr.attr,
	&little_cap_pct_attr.attr,
	&little_boost_cap_pct_attr.attr,
	&little_ui_floor_pct_attr.attr,
	&big_ui_floor_pct_attr.attr,
	&psi_cpu_floor_thresh_attr.attr,
	&psi_cpu_floor_pct_attr.attr,
	&psi_mem_cap_thresh_attr.attr,
	&psi_mem_cap_pct_attr.attr,
	&thermal_aware_attr.attr,
	&screen_on_attr.attr,
	&gaming_mode_attr.attr,
	NULL
};
ATTRIBUTE_GROUPS(bara_no_seidou);

static void bara_no_seidou_tunables_release(struct kobject *kobj)
{
	kfree(to_bara_no_seidou_tunables(kobj));
}

static struct kobj_type bara_no_seidou_tunables_ktype = {
	.default_groups	= bara_no_seidou_groups,
	.sysfs_ops	= &kobj_sysfs_ops,
	.release	= bara_no_seidou_tunables_release,
};

static struct bara_no_seidou_tunables *
bara_no_seidou_tunables_alloc(struct cpufreq_policy *policy,
			      struct bara_no_seidou_policy *gd_policy)
{
	struct bara_no_seidou_tunables *t;
	int ret;

	t = kzalloc(sizeof(*t), GFP_KERNEL);
	if (!t)
		return NULL;

	t->up_rate_limit_normal_us	= BARA_NO_SEIDOU_UP_RATE_LIMIT_NORMAL_US;
	t->down_rate_limit_normal_us	= BARA_NO_SEIDOU_DOWN_RATE_LIMIT_NORMAL_US;
	t->down_rate_limit_game_us	= BARA_NO_SEIDOU_DOWN_RATE_LIMIT_GAME_US;
	t->headroom_gaming_little_pct	= BARA_NO_SEIDOU_HEADROOM_GAMING_LITTLE_PCT_DEFAULT;
	t->headroom_gaming_big_pct	= BARA_NO_SEIDOU_HEADROOM_GAMING_BIG_PCT_DEFAULT;
	t->max_step_pct			= BARA_NO_SEIDOU_MAX_STEP_PCT_DEFAULT;
	t->hispeed_load			= BARA_NO_SEIDOU_HISPEED_LOAD_DEFAULT;
	t->hispeed_freq_pct		= BARA_NO_SEIDOU_HISPEED_FREQ_PCT_DEFAULT;
	t->ema_up_shift			= BARA_NO_SEIDOU_EMA_UP_SHIFT_DEFAULT;
	t->ema_down_shift		= BARA_NO_SEIDOU_EMA_DOWN_SHIFT_DEFAULT;
	t->gaming_exit_delay_ms		= BARA_NO_SEIDOU_GAMING_EXIT_DELAY_MS_DEFAULT;
	t->screen_off_cap_pct		= BARA_NO_SEIDOU_SCREEN_OFF_CAP_PCT_DEFAULT;
	t->ramp_delta_pct		= BARA_NO_SEIDOU_RAMP_DELTA_PCT_DEFAULT;
	t->ui_boost_ms			= BARA_NO_SEIDOU_UI_BOOST_MS_DEFAULT;
	t->little_cap_pct		= BARA_NO_SEIDOU_LITTLE_CAP_PCT_DEFAULT;
	t->little_boost_cap_pct		= BARA_NO_SEIDOU_LITTLE_BOOST_CAP_PCT_DEFAULT;
	t->little_ui_floor_pct		= BARA_NO_SEIDOU_LITTLE_UI_FLOOR_PCT_DEFAULT;
	t->big_ui_floor_pct		= BARA_NO_SEIDOU_BIG_UI_FLOOR_PCT_DEFAULT;
	t->psi_cpu_floor_thresh		= BARA_NO_SEIDOU_PSI_CPU_FLOOR_THRESH_DEFAULT;
	t->psi_cpu_floor_pct		= BARA_NO_SEIDOU_PSI_CPU_FLOOR_PCT_DEFAULT;
	t->psi_mem_cap_thresh		= BARA_NO_SEIDOU_PSI_MEM_CAP_THRESH_DEFAULT;
	t->psi_mem_cap_pct		= BARA_NO_SEIDOU_PSI_MEM_CAP_PCT_DEFAULT;
	t->thermal_aware		= true;

	/* Set the backpointers BEFORE the kobject (and its sysfs files)
	 * become visible, so a write can never see a half-built policy.
	 */
	t->gd_policy = gd_policy;
	gd_policy->tunables = t;

	ret = kobject_init_and_add(&t->kobj, &bara_no_seidou_tunables_ktype,
				   get_governor_parent_kobj(policy),
				   "bara-no-seidou");
	if (ret) {
		gd_policy->tunables = NULL;
		kobject_put(&t->kobj);
		return NULL;
	}

	return t;
}

/* ---- Governor lifecycle -------------------------------------------------- */

static int bara_no_seidou_init(struct cpufreq_policy *policy)
{
	struct bara_no_seidou_policy *gd_policy;
	struct bara_no_seidou_tunables *tunables;
	int ret;

	if (policy->governor_data)
		return -EBUSY;

	cpufreq_enable_fast_switch(policy);

	gd_policy = kzalloc(sizeof(*gd_policy), GFP_KERNEL);
	if (!gd_policy) {
		ret = -ENOMEM;
		goto disable_fast_switch;
	}

	gd_policy->policy = policy;
	raw_spin_lock_init(&gd_policy->update_lock);
	gd_policy->last_gaming_state = -1;
	gd_policy->screen_on_last = true;
	/* Heuristic: the cluster containing logical CPU 0 is the little/
	 * primary cluster on every SM6225-class layout this kernel targets
	 * (creek/bengal). Used for the little-cluster caps and headroom.
	 */
	gd_policy->is_little = cpumask_test_cpu(0, policy->related_cpus);

	tunables = bara_no_seidou_tunables_alloc(policy, gd_policy);
	if (!tunables) {
		ret = -ENOMEM;
		goto free_policy;
	}

	bara_no_seidou_set_delays(gd_policy, false);

	/*
	 * The kthread + irq_work slow path is only needed when the driver
	 * can't fast-switch (frequency change may sleep).
	 */
	if (!policy->fast_switch_enabled) {
		struct sched_attr attr = {
			.size		= sizeof(struct sched_attr),
			.sched_policy	= SCHED_DEADLINE,
			.sched_flags	= SCHED_FLAG_SUGOV,
			.sched_nice	= 0,
			.sched_priority	= 0,
			.sched_runtime	= NSEC_PER_MSEC,
			.sched_deadline	= 10 * NSEC_PER_MSEC,
			.sched_period	= 10 * NSEC_PER_MSEC,
		};

		mutex_init(&gd_policy->work_lock);
		init_irq_work(&gd_policy->irq_work, bara_no_seidou_irq_work);
		kthread_init_worker(&gd_policy->worker);
		kthread_init_work(&gd_policy->work, bara_no_seidou_work);

		gd_policy->thread = kthread_create(kthread_worker_fn,
						   &gd_policy->worker,
						   "bara-no-seidou:%d", policy->cpu);
		if (IS_ERR(gd_policy->thread)) {
			ret = PTR_ERR(gd_policy->thread);
			gd_policy->thread = NULL;
			mutex_destroy(&gd_policy->work_lock);
			goto free_tunables;
		}

		/* On failure keep running at default priority. Do NOT
		 * kthread_stop() here: the thread is still used below.
		 */
		if (sched_setattr_nocheck(gd_policy->thread, &attr))
			pr_warn("failed to set SCHED_DEADLINE/SUGOV for kthread, using default priority\n");

		kthread_bind_mask(gd_policy->thread, policy->related_cpus);
		wake_up_process(gd_policy->thread);
	}

	policy->governor_data = gd_policy;

	bara_no_seidou_input_get();

	pr_info("governor attached to policy cpu%d (%s cluster, %s switch)\n",
		policy->cpu, gd_policy->is_little ? "little" : "big",
		policy->fast_switch_enabled ? "fast" : "kthread");
	return 0;

free_tunables:
	kobject_put(&tunables->kobj);
free_policy:
	kfree(gd_policy);
disable_fast_switch:
	cpufreq_disable_fast_switch(policy);
	pr_err("initialization failed (error %d)\n", ret);
	return ret;
}

static void bara_no_seidou_exit(struct cpufreq_policy *policy)
{
	struct bara_no_seidou_policy *gd_policy = policy->governor_data;

	bara_no_seidou_input_put();

	if (!policy->fast_switch_enabled && gd_policy->thread) {
		kthread_flush_worker(&gd_policy->worker);
		kthread_stop(gd_policy->thread);
		mutex_destroy(&gd_policy->work_lock);
	}

	policy->governor_data = NULL;
	kobject_put(&gd_policy->tunables->kobj);
	kfree(gd_policy);
	cpufreq_disable_fast_switch(policy);
}

/*
 * Update hooks live in start/stop (not init/exit): CPU hotplug stops and
 * restarts the governor with a different policy->cpus, and a hook left
 * on an offlined CPU would point into freed memory after exit.
 */
static int bara_no_seidou_start(struct cpufreq_policy *policy)
{
	struct bara_no_seidou_policy *gd_policy = policy->governor_data;
	unsigned int cpu;

	gd_policy->last_freq_update_time = 0;
	gd_policy->next_freq = policy->cur;
	gd_policy->cached_raw_freq = 0;
	gd_policy->cached_resolved_freq = 0;
	gd_policy->limits_changed = false;
	gd_policy->need_freq_update = true;
	gd_policy->work_in_progress = false;
	gd_policy->smoothed_util = 0;
	gd_policy->prev_upct = 0;
	gd_policy->ui_boost_end_ns = 0;
	gd_policy->last_gaming_active_ns = 0;
	gd_policy->thermal_applied_pct = 0;
	gd_policy->thermal_step_ns = 0;

	for_each_cpu(cpu, policy->cpus) {
		struct bara_no_seidou_cpu *gd_cpu = &per_cpu(bara_no_seidou_cpu_list, cpu);

		memset(gd_cpu, 0, sizeof(*gd_cpu));
		gd_cpu->cpu = cpu;
		gd_cpu->gd_policy = gd_policy;
	}

	for_each_cpu(cpu, policy->cpus) {
		struct bara_no_seidou_cpu *gd_cpu = &per_cpu(bara_no_seidou_cpu_list, cpu);

		cpufreq_add_update_util_hook(cpu, &gd_cpu->update_util,
					     bara_no_seidou_update_util);
	}

	return 0;
}

static void bara_no_seidou_stop(struct cpufreq_policy *policy)
{
	struct bara_no_seidou_policy *gd_policy = policy->governor_data;
	unsigned int cpu;

	for_each_cpu(cpu, policy->cpus)
		cpufreq_remove_update_util_hook(cpu);

	synchronize_rcu();

	if (!policy->fast_switch_enabled) {
		irq_work_sync(&gd_policy->irq_work);
		kthread_cancel_work_sync(&gd_policy->work);
	}
}

static void bara_no_seidou_limits(struct cpufreq_policy *policy)
{
	struct bara_no_seidou_policy *gd_policy = policy->governor_data;
	unsigned long flags;

	raw_spin_lock_irqsave(&gd_policy->update_lock, flags);
	gd_policy->limits_changed = true;
	gd_policy->need_freq_update = true;
	raw_spin_unlock_irqrestore(&gd_policy->update_lock, flags);
}

static struct cpufreq_governor bara_no_seidou_gov = {
	.name		= "bara-no-seidou",
	.owner		= THIS_MODULE,
	.flags		= CPUFREQ_GOV_DYNAMIC_SWITCHING,
	.init		= bara_no_seidou_init,
	.exit		= bara_no_seidou_exit,
	.start		= bara_no_seidou_start,
	.stop		= bara_no_seidou_stop,
	.limits		= bara_no_seidou_limits,
};

cpufreq_governor_init(bara_no_seidou_gov);
cpufreq_governor_exit(bara_no_seidou_gov);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("cpufreq governor: bara-no-seidou (power-save + gaming switch, touch-boosted, tunable)");
