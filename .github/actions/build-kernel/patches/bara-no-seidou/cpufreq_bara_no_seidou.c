// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2026 kusonekoworld
 *
 * CPUFreq governor "bara-no-seidou"
 *
 * Schedutil-derived governor tuned for battery efficiency and thermal
 * headroom during normal use, with an automatic switch to a
 * low-latency responsive profile when gaming mode is active
 * (sched_gaming_active, the same global driven by the existing fair.c
 * gaming-bias patch and readable/writable here too via gaming_mode).
 *
 * Profiles / behavior:
 *  - Normal: long rate-limit windows, bounded utilization discount,
 *    thermal-pressure-aware extra discount, capped upward frequency
 *    steps. Little cluster gets a milder discount than big cluster,
 *    since little-core starvation is what actually causes UI jank in
 *    daily use, while big-core discount is where the battery win is.
 *  - Touch boost: on any touchscreen event, discount is suspended for
 *    a short window so the first frames after a tap/scroll aren't
 *    throttled - independent of gaming mode, this is what keeps daily
 *    scrolling feeling immediate instead of springy.
 *  - Gaming: rate limits collapse toward stock-schedutil values, the
 *    discount is dropped, and a hispeed floor guarantees a minimum
 *    frequency once load crosses hispeed_load. An exit-hysteresis
 *    window keeps the profile from flapping during loading screens.
 *  - Screen off: an explicit screen_on tunable (fed by whatever
 *    screen-state source you already have, e.g. the fsync driver's
 *    polling) hard-caps frequency for background/doze battery saving.
 *
 * All numeric knobs are sysfs tunables under
 * /sys/devices/system/cpu/cpufreq/policyX/bara-no-seidou/.
 */

#define pr_fmt(fmt) "bara-no-seidou: " fmt

#include <linux/cpufreq.h>
#include <linux/kthread.h>
#include <linux/percpu-defs.h>
#include <linux/slab.h>
#include <linux/irq_work.h>
#include <linux/input.h>
#include <linux/sched/cpufreq.h>
#include <linux/tick.h>
#ifdef CONFIG_PSI
#include <linux/psi.h>
#endif
#include <trace/events/power.h>
#include <uapi/linux/sched/types.h>

/* Internal scheduler header (kernel/sched/sched.h) - this is where
 * map_util_freq() and arch_scale_thermal_pressure() actually live.
 * sched_cpu_util() itself is a public API (include/linux/sched.h)
 * but this file lives in kernel/sched/, where this header is needed
 * for the same reason as any other governor placed there.
 */
#include "sched.h"

extern int sched_gaming_active;

#include <linux/devfreq.h>

/* ---- GPU devfreq lock (game-mode max-clock) ---------------------------- */

static char gpu_devfreq_name[DEVFREQ_NAME_LEN] = "";
module_param_string(gpu_devfreq_name, gpu_devfreq_name,
                    sizeof(gpu_devfreq_name), 0644);
MODULE_PARM_DESC(gpu_devfreq_name,
                 "Nama devfreq GPU persis (kosong = auto-probe candidate list)");

/* Nama devfreq device GPU yang umum dipakai di GKI target. Devfreq
 * name matching itu exact-string, bukan substring, jadi platform yang
 * nge-prefix base address devicetree (kayak Bengal/SD685) butuh entry
 * sendiri. Tambah entry baru di sini kalau pindah ke SoC lain.
 */
static const char * const gpu_devfreq_candidates[] = {
	"5900000.qcom,kgsl-3d0",	/* Adreno 610, SD685/Bengal */
	"kgsl-3d0",			/* Adreno, nama polos (SoC lain) */
	"gpu",
	"mali",
	NULL
};

static struct devfreq *bara_no_seidou_gpu_devfreq_cached;

/*
 * Resolve devfreq device GPU sekali (cached) - device devfreq GPU gak
 * bakal muncul/hilang saat runtime, jadi cukup di-probe sekali lalu
 * dipakai ulang tiap kali dibutuhkan.
 */
static struct devfreq *bara_no_seidou_find_gpu_devfreq(void)
{
	struct devfreq *df;
	int i;

	if (bara_no_seidou_gpu_devfreq_cached)
		return bara_no_seidou_gpu_devfreq_cached;

	if (gpu_devfreq_name[0]) {
		df = devfreq_get_devfreq_by_name(gpu_devfreq_name);
		if (!IS_ERR_OR_NULL(df))
			goto found;
		pr_warn("gpu_devfreq_name='%s' tidak ditemukan, fallback auto-probe\n",
			gpu_devfreq_name);
	}

	for (i = 0; gpu_devfreq_candidates[i]; i++) {
		df = devfreq_get_devfreq_by_name(gpu_devfreq_candidates[i]);
		if (!IS_ERR_OR_NULL(df))
			goto found;
	}

	pr_warn("tidak ada devfreq GPU yang cocok, GPU lock nonaktif\n");
	return NULL;

found:
	bara_no_seidou_gpu_devfreq_cached = df;
	pr_info("GPU devfreq resolved: %s\n", dev_name(&df->dev));
	return df;
}

/*
 * Lock GPU devfreq ke scaling_max_freq selama gaming mode aktif; lepas
 * clamp begitu gaming berakhir. Dipanggil hanya sekali per edge
 * transisi dari bara_no_seidou_refresh_profile() (yang sudah filter
 * itu), bukan tiap tick.
 *
 * bara_no_seidou_gpu_lock() sendiri aman dipanggil dari context
 * manapun - atomic/fast-switch tick maupun kthread - karena cuma
 * nge-set state lalu schedule_work(); mutex_lock()/update_devfreq()
 * yang butuh sleep dieksekusi belakangan di workqueue context yang
 * boleh sleep. Ini menghindari perlu deteksi fast_switch_enabled
 * sama sekali - satu code path buat kedua kondisi.
 */
static atomic_t bara_no_seidou_gpu_lock_pending = ATOMIC_INIT(-1); /* -1=none, 0=off, 1=on */

static void bara_no_seidou_gpu_lock_apply(bool gaming)
{
	struct devfreq *gdf = bara_no_seidou_find_gpu_devfreq();

	if (!gdf)
		return;

	mutex_lock(&gdf->lock);
	if (gaming) {
		gdf->min_freq = gdf->scaling_max_freq;
		gdf->max_freq = gdf->scaling_max_freq;
	} else {
		gdf->min_freq = 0;
		gdf->max_freq = 0;	/* lepas cap, balik ke range penuh */
	}
	update_devfreq(gdf);
	mutex_unlock(&gdf->lock);
}

static void bara_no_seidou_gpu_lock_work_fn(struct work_struct *work)
{
	int state = atomic_xchg(&bara_no_seidou_gpu_lock_pending, -1);

	if (state < 0)
		return;
	bara_no_seidou_gpu_lock_apply(state);
}

static DECLARE_WORK(bara_no_seidou_gpu_lock_work, bara_no_seidou_gpu_lock_work_fn);

static void bara_no_seidou_gpu_lock(bool gaming)
{
	atomic_set(&bara_no_seidou_gpu_lock_pending, gaming ? 1 : 0);
	schedule_work(&bara_no_seidou_gpu_lock_work);
}

/* ---- Defaults (overridable at runtime via sysfs) --------------------- */

#define BARA_NO_SEIDOU_UP_RATE_LIMIT_NORMAL_US	(8000)
#define BARA_NO_SEIDOU_DOWN_RATE_LIMIT_NORMAL_US	(35000)
#define BARA_NO_SEIDOU_DOWN_RATE_LIMIT_GAME_US	(8000)

#define BARA_NO_SEIDOU_HEADROOM_GAMING_LITTLE_PCT_DEFAULT	(25)
#define BARA_NO_SEIDOU_HEADROOM_GAMING_BIG_PCT_DEFAULT	(15)
#define BARA_NO_SEIDOU_MAX_STEP_PCT_DEFAULT		(80)	/* normal profile */

#define BARA_NO_SEIDOU_HISPEED_LOAD_DEFAULT		(85)	/* pct, gaming only */
#define BARA_NO_SEIDOU_HISPEED_FREQ_PCT_DEFAULT	(70)	/* pct of max_freq */

#define BARA_NO_SEIDOU_THERMAL_DISCOUNT_MAX_PCT	(25)	/* extra cut at full throttle */
#define BARA_NO_SEIDOU_GAMING_EXIT_DELAY_MS_DEFAULT	(3000)	/* gaming exit hysteresis */

#define BARA_NO_SEIDOU_TOUCH_BOOST_MS_DEFAULT	(220)	/* daily-use smoothness */
#define BARA_NO_SEIDOU_SCREEN_OFF_CAP_PCT_DEFAULT	(35)	/* pct of max_freq */

/* Daily UI ramp-assist: a sharp rise in util% - not just a fresh touch
 * event - re-arms the same "UI is active" floor. Catches fling-scrolls,
 * open/close animations, and caption draws that touch-boost alone
 * misses because there's no touch event mid-gesture. Learned from
 * ramp detection + per-cluster daily caps/floors.
 */
#define BARA_NO_SEIDOU_RAMP_DELTA_PCT_DEFAULT	(15)	/* upct jump to re-arm */
#define BARA_NO_SEIDOU_UI_BOOST_MS_DEFAULT		(200)	/* how long the floor holds */
#define BARA_NO_SEIDOU_LITTLE_CAP_PCT_DEFAULT	(55)	/* little cap, UI idle */
#define BARA_NO_SEIDOU_LITTLE_BOOST_CAP_PCT_DEFAULT	(85)	/* little cap, UI active */
#define BARA_NO_SEIDOU_LITTLE_UI_FLOOR_PCT_DEFAULT	(35)	/* little floor, UI active */
#define BARA_NO_SEIDOU_BIG_UI_FLOOR_PCT_DEFAULT	(25)	/* big floor, UI active */

/* PSI-CPU sustained-pressure floor - opt-in, off by default (0 =
 * disabled). See bara_no_seidou_psi_floor() for why the threshold matters
 * and why the default is conservative.
 */
#define BARA_NO_SEIDOU_PSI_CPU_FLOOR_THRESH_DEFAULT	(0)
#define BARA_NO_SEIDOU_PSI_CPU_FLOOR_PCT_DEFAULT	(70)
#define BARA_NO_SEIDOU_PSI_MEM_CAP_THRESH_DEFAULT	(0)
#define BARA_NO_SEIDOU_PSI_MEM_CAP_PCT_DEFAULT	(80)

/* Directional EMA on util: rises fast (kills PELT-lag stutter), decays
 * slowly (no inter-frame sag / yoyo). Shift of 1 = half-weight on the
 * new sample; 3 = eighth-weight.
 */
#define BARA_NO_SEIDOU_EMA_UP_SHIFT_DEFAULT		(0)
#define BARA_NO_SEIDOU_EMA_DOWN_SHIFT_DEFAULT	(3)

/* Smallest IOWait boost step,
 * doubles on repeated IOWait ticks, halves when they stop.
 */
#define IOWAIT_BOOST_MIN			(SCHED_CAPACITY_SCALE / 8)

/* ---- Global touch-boost state (shared across all policies/clusters) --- */

static atomic64_t bara_no_seidou_touch_boost_until_ns = ATOMIC64_INIT(0);
static unsigned int bara_no_seidou_touch_boost_ms = BARA_NO_SEIDOU_TOUCH_BOOST_MS_DEFAULT;

static void bara_no_seidou_touch_event(struct input_handle *handle,
				  unsigned int type, unsigned int code,
				  int value)
{
	if (type != EV_ABS && type != EV_KEY)
		return;

	atomic64_set(&bara_no_seidou_touch_boost_until_ns,
		     local_clock() + (u64)bara_no_seidou_touch_boost_ms * NSEC_PER_MSEC);
}

static bool bara_no_seidou_touch_boosted(void)
{
	return local_clock() < atomic64_read(&bara_no_seidou_touch_boost_until_ns);
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

static const struct input_device_id bara_no_seidou_input_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_ABS) },
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

/* ---- Types ------------------------------------------------------------ */

struct bara_no_seidou_tunables {
	struct kobject kobj;
	struct bara_no_seidou_policy *gd_policy;	/* backpointer, set in init */

	unsigned int up_rate_limit_normal_us;
	unsigned int down_rate_limit_normal_us;
	unsigned int down_rate_limit_game_us;

	unsigned int headroom_gaming_little_pct;
	unsigned int headroom_gaming_big_pct;
	unsigned int max_step_pct;

	unsigned int hispeed_load;
	unsigned int hispeed_freq_pct;

	unsigned int gaming_exit_delay_ms;
	unsigned int touch_boost_ms;
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
	bool screen_on;
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
	int last_gaming_state;
	u64 last_gaming_active_ns;

	/* thermal step controller state - see bara_no_seidou_thermal_discount() */
	int thermal_applied_pct;
	u64 thermal_step_ns;

	/* daily UI ramp-assist state - see bara_no_seidou_apply_daily_shape() */
	unsigned int prev_upct;
	u64 ui_boost_end_ns;
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
	unsigned long smoothed_util;	/* directional EMA state */

	/* Decaying IOWait boost */
	bool iowait_boost_pending;
	unsigned int iowait_boost;
	u64 last_update;

#ifdef CONFIG_NO_HZ_COMMON
	unsigned long saved_idle_calls;
#endif
};

static DEFINE_PER_CPU(struct bara_no_seidou_cpu, bara_no_seidou_cpu_list);
static atomic_t bara_no_seidou_input_refcount = ATOMIC_INIT(0);

/* ---- Gaming state (raw + exit-hysteresis) ------------------------------ */

static inline bool bara_no_seidou_raw_gaming(void)
{
	return READ_ONCE(sched_gaming_active) != 0;
}

static bool bara_no_seidou_gaming(struct bara_no_seidou_policy *gd_policy)
{
	struct bara_no_seidou_tunables *t = gd_policy->tunables;
	bool raw = bara_no_seidou_raw_gaming();
	u64 now = local_clock();

	if (raw) {
		gd_policy->last_gaming_active_ns = now;
		return true;
	}

	if (!gd_policy->last_gaming_active_ns)
		return false;

	return now - gd_policy->last_gaming_active_ns <
	       (u64)t->gaming_exit_delay_ms * NSEC_PER_MSEC;
}

/* ---- PSI-CPU sustained-pressure floor ----------------------------------- */

/*
 * System-wide CPU "some" pressure, 10s EWMA, as an integer percentage.
 * Returns 0 when CONFIG_PSI is off, when PSI is disabled at runtime
 * (psi_disabled static key), or on any kernel where psi_system isn't
 * visible to this translation unit - all of which make the floor
 * below a permanent no-op rather than a build failure. Read pattern
 * (rcu_read_lock + READ_ONCE on the averaging word) is the standard
 * safe way to sample it, simplified here to system-wide only (no
 * cgroup-v2 group picking - Bara-no-Seidou has no equivalent config surface
 * for that yet).
 */
static inline unsigned int bara_no_seidou_psi_cpu_pct(void)
{
#ifdef CONFIG_PSI
	unsigned long avg;

	if (static_branch_likely(&psi_disabled))
		return 0;

	rcu_read_lock();
	avg = READ_ONCE(psi_system.avg[PSI_CPU_SOME][0]);
	rcu_read_unlock();
	return (unsigned int)LOAD_INT(avg);
#else
	return 0;
#endif
}

/* Same shape as bara_no_seidou_psi_cpu_pct() for the PSI_MEM_SOME dimension -
 * feeds bara_no_seidou_psi_mem_cap() below, the inverse of the CPU floor:
 * heavy paging means the CPU is waiting on memory, so pushing freq up
 * just deepens the stall for no gain.
 */
static inline unsigned int bara_no_seidou_psi_mem_pct(void)
{
#ifdef CONFIG_PSI
	unsigned long avg;

	if (static_branch_likely(&psi_disabled))
		return 0;

	rcu_read_lock();
	avg = READ_ONCE(psi_system.avg[PSI_MEM_SOME][0]);
	rcu_read_unlock();
	return (unsigned int)LOAD_INT(avg);
#else
	return 0;
#endif
}

/*
 * PSI's 10s smoothing window is too slow for sub-second decisions -
 * by design this only fires for *sustained* CPU pressure (background
 * sync fighting the foreground app, multitasking with a background
 * compile/export, etc.) where instantaneous util already looks "fine"
 * per-tick but tasks have been queueing for seconds. Opt-in (default
 * threshold 0 = off): PSI pressure is a real but blunt signal, and an
 * unconditional floor here would fight the whole headroom/thermal
 * design elsewhere in the file. Never applied during gaming - gaming
 * already gets its own generous headroom and hispeed floor.
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
 * Inverse of bara_no_seidou_psi_floor(): sustained memory pressure means the
 * CPU is stalled on paging/swap, not short on clock - a cap here, not
 * a floor. Opt-in (threshold 0 = off) for the same reason as the CPU
 * floor: PSI is a real but blunt 10s-smoothed signal. Applies during
 * gaming too (unlike the daily-only UI shaping) since a genuinely
 * thrashing device benefits from backing off regardless of mode -
 * more clock cannot fix a memory stall.
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

/*
 * On the screen-off -> screen-on edge, stale hysteresis/decay state
 * from before a (possibly long) suspend must not leak into the first
 * post-resume ticks - the device may have fully cooled, the gaming
 * exit-grace timer may be stamped from hours ago, and a stale
 * ui_boost_end_ns could either wrongly hold a floor or wrongly miss
 * arming one. This class of bug - stale pre-suspend state biasing
 * the first post-resume decision window - is easy to miss without
 * dedicated testing across a real suspend/resume cycle.
 */
static void bara_no_seidou_resume_reset(struct bara_no_seidou_policy *gd_policy)
{
	bool screen_on = gd_policy->tunables->screen_on;

	if (!gd_policy->screen_on_last && screen_on) {
		gd_policy->last_gaming_active_ns = 0;
		gd_policy->ui_boost_end_ns = 0;
		gd_policy->prev_upct = 0;
		gd_policy->thermal_applied_pct = 0;
		gd_policy->thermal_step_ns = 0;
	}
	gd_policy->screen_on_last = screen_on;
}

static void bara_no_seidou_refresh_profile(struct bara_no_seidou_policy *gd_policy)
{
	struct bara_no_seidou_tunables *t = gd_policy->tunables;
	bool gaming;

	bara_no_seidou_resume_reset(gd_policy);
	gaming = bara_no_seidou_gaming(gd_policy);

	if (gd_policy->last_gaming_state == gaming)
		return;

	if (gaming) {
		/* Instant up while gaming - never delay a frame that needs
		 * more clock, only the downward step stays rate-limited.
		 */
		gd_policy->up_rate_delay_ns = 0;
		gd_policy->down_rate_delay_ns =
			(s64)t->down_rate_limit_game_us * NSEC_PER_USEC;
		bara_no_seidou_gpu_lock(true);
	} else {
		gd_policy->up_rate_delay_ns =
			(s64)t->up_rate_limit_normal_us * NSEC_PER_USEC;
		gd_policy->down_rate_delay_ns =
			(s64)t->down_rate_limit_normal_us * NSEC_PER_USEC;
		bara_no_seidou_gpu_lock(false);
	}

	gd_policy->last_gaming_state = gaming;
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

#define BARA_NO_SEIDOU_THERMAL_STEP_NS		(200000000ULL) /* 200ms tick */
#define BARA_NO_SEIDOU_THERMAL_STEP_DOWN_PCT		(4)  /* enter throttle fast */
#define BARA_NO_SEIDOU_THERMAL_STEP_UP_PCT		(2)  /* recover gently, no bounce */
#define BARA_NO_SEIDOU_THERMAL_MIN_CAP_PCT		(60)

/*
 * Walk the applied thermal cap toward the pressure-derived target in
 * small, rate-limited steps rather than snapping to it every tick.
 * Stepping down faster than up gives a smooth throttle entry and a
 * gentle recovery with no oscillation right at the trip point. This
 * directly targets the "still a bit choppy" feedback during longer
 * gaming sessions, where an instant discount was jittering freq every
 * 2s tick.
 */
static unsigned long bara_no_seidou_thermal_discount(struct bara_no_seidou_policy *gd_policy,
						 int cpu, unsigned long max,
						 u64 time)
{
	unsigned long pressure;
	int target, applied;

	if (!gd_policy->tunables->thermal_aware)
		return max;

	pressure = arch_scale_thermal_pressure(cpu);
	target = 100 - (int)mult_frac(pressure, BARA_NO_SEIDOU_THERMAL_DISCOUNT_MAX_PCT, 100);
	target = clamp(target, BARA_NO_SEIDOU_THERMAL_MIN_CAP_PCT, 100);

	applied = gd_policy->thermal_applied_pct ? gd_policy->thermal_applied_pct : 100;

	if ((s64)(time - gd_policy->thermal_step_ns) >= (s64)BARA_NO_SEIDOU_THERMAL_STEP_NS) {
		if (applied > target)
			applied -= BARA_NO_SEIDOU_THERMAL_STEP_DOWN_PCT;
		else if (applied < target)
			applied += BARA_NO_SEIDOU_THERMAL_STEP_UP_PCT;
		applied = clamp(applied, BARA_NO_SEIDOU_THERMAL_MIN_CAP_PCT, 100);
		gd_policy->thermal_applied_pct = applied;
		gd_policy->thermal_step_ns = time;
	}

	if (applied >= 100)
		return max;

	return mult_frac(max, (unsigned int)applied, 100);
}

/* ---- Frequency selection ----------------------------------------------- */

static unsigned int bara_no_seidou_apply_step_limit(struct bara_no_seidou_policy *gd_policy,
					       unsigned int cur,
					       unsigned int target,
					       bool gaming)
{
	unsigned int max_freq = gd_policy->policy->cpuinfo.max_freq;
	unsigned int step_cap;

	if (gaming || bara_no_seidou_touch_boosted() || target <= cur)
		return target;

	step_cap = cur + mult_frac(max_freq - cur,
				    gd_policy->tunables->max_step_pct, 100);
	return min(target, step_cap);
}

static unsigned int bara_no_seidou_hispeed_floor(struct bara_no_seidou_policy *gd_policy,
					    unsigned long util, unsigned long max,
					    bool gaming)
{
	struct bara_no_seidou_tunables *t = gd_policy->tunables;
	unsigned int max_freq = gd_policy->policy->cpuinfo.max_freq;

	if (!gaming)
		return 0;

	if (util * 100 < max * t->hispeed_load)
		return 0;

	return mult_frac(max_freq, t->hispeed_freq_pct, 100);
}

static unsigned int bara_no_seidou_screen_off_cap(struct bara_no_seidou_policy *gd_policy)
{
	struct bara_no_seidou_tunables *t = gd_policy->tunables;
	unsigned int max_freq = gd_policy->policy->cpuinfo.max_freq;

	if (t->screen_on)
		return 0;

	return mult_frac(max_freq, t->screen_off_cap_pct, 100);
}

/*
 * Headroom, not discount: request slightly MORE capacity than measured
 * so the resolved OPP has slack, instead of shaving util down and
 * landing exactly on the edge of what's needed (which is what the old
 * util_bias_pct discount did - fine for battery, bad for smoothness on
 * borderline loads). Daily uses a tiered curve - negligible headroom
 * at low util (battery), more as util climbs (responsiveness). Gaming
 * uses a flat generous headroom.
 */
static unsigned long bara_no_seidou_apply_headroom(struct bara_no_seidou_policy *gd_policy,
					      unsigned long util,
					      unsigned long max, bool gaming)
{
	struct bara_no_seidou_tunables *t = gd_policy->tunables;
	unsigned int upct;

	if (!max || util >= max)
		return max;

	upct = (unsigned int)(util * 100 / max);
	if (upct >= 95)
		return max;

	if (gaming) {
		unsigned int h = gd_policy->is_little ?
			t->headroom_gaming_little_pct : t->headroom_gaming_big_pct;
		return min(util + util * h / 100, max);
	}

	if (gd_policy->is_little) {
		if (upct >= 70)
			return min(util + (util >> 4), max);
		if (upct >= 45)
			return min(util + (util >> 5), max);
		return util;
	}

	if (upct >= 75)
		return min(util + (util >> 4), max);
	if (upct >= 50)
		return min(util + (util >> 5), max);
	return min(util + (util >> 6), max);
}

/*
 * A sharp rise in util% - not just a fresh touch event - re-arms the
 * same "UI is active" window that touch-boost uses. Catches a fling
 * scroll or an open/close animation mid-gesture, where there's no new
 * touch event to trigger bara_no_seidou_touch_boosted() but the frame is
 * just as real.
 */
static bool bara_no_seidou_ui_ramp_active(struct bara_no_seidou_policy *gd_policy,
				     unsigned long util, unsigned long max,
				     u64 time)
{
	struct bara_no_seidou_tunables *t = gd_policy->tunables;
	unsigned int upct = max ? (unsigned int)(util * 100 / max) : 0;

	if (upct > gd_policy->prev_upct &&
	    upct - gd_policy->prev_upct >= t->ramp_delta_pct)
		gd_policy->ui_boost_end_ns = time +
			(u64)t->ui_boost_ms * NSEC_PER_MSEC;

	gd_policy->prev_upct = upct;

	return time < gd_policy->ui_boost_end_ns;
}

/*
 * Explicit daily-profile shaping, separate from headroom: little
 * cluster gets a hard cap normally (keeps it out of the power-hungry
 * top end for background/idle-ish work) that opens up while UI is
 * active; both clusters get a floor while UI is active so a fresh
 * gesture never has to climb from a cold/low frequency. Never applied
 * during gaming - hispeed floor already owns that job there.
 */
static unsigned int bara_no_seidou_apply_daily_shape(struct bara_no_seidou_policy *gd_policy,
						 unsigned int freq, bool ui_active)
{
	struct bara_no_seidou_tunables *t = gd_policy->tunables;
	unsigned int max_freq = gd_policy->policy->cpuinfo.max_freq;
	unsigned int cap, floor_f;

	if (gd_policy->is_little) {
		cap = mult_frac(max_freq, ui_active ? t->little_boost_cap_pct :
						       t->little_cap_pct, 100);
		if (freq > cap)
			freq = cap;
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

static unsigned int bara_no_seidou_next_freq(struct bara_no_seidou_policy *gd_policy,
					int cpu, unsigned long util,
					unsigned long max, u64 time)
{
	struct cpufreq_policy *policy = gd_policy->policy;
	unsigned int freq = arch_scale_freq_invariant() ?
			     policy->cpuinfo.max_freq : policy->cur;
	unsigned long adj_max = bara_no_seidou_thermal_discount(gd_policy, cpu, max, time);
	unsigned long next_util;
	unsigned int floor_freq, screen_cap, mem_cap, target;
	bool gaming = bara_no_seidou_gaming(gd_policy);
	bool boosted = bara_no_seidou_touch_boosted();
	bool ramp_active = bara_no_seidou_ui_ramp_active(gd_policy, util, max, time);
	bool ui_active = boosted || ramp_active;

	next_util = boosted ? util :
		    bara_no_seidou_apply_headroom(gd_policy, util, max, gaming);

	freq = map_util_freq(next_util, freq, adj_max);

	if (freq != gd_policy->cached_raw_freq || gd_policy->need_freq_update) {
		gd_policy->cached_raw_freq = freq;
		freq = cpufreq_driver_resolve_freq(policy, freq);
	} else {
		freq = gd_policy->next_freq;
	}

	target = bara_no_seidou_apply_step_limit(gd_policy, policy->cur, freq, gaming);

	floor_freq = bara_no_seidou_hispeed_floor(gd_policy, util, max, gaming);
	if (floor_freq)
		target = max(target, floor_freq);

	if (!gaming)
		target = bara_no_seidou_apply_daily_shape(gd_policy, target, ui_active);

	floor_freq = bara_no_seidou_psi_floor(gd_policy, gaming);
	if (floor_freq)
		target = max(target, floor_freq);

	mem_cap = bara_no_seidou_psi_mem_cap(gd_policy);
	if (mem_cap)
		target = min(target, mem_cap);

	/* screen-off cap always wins - it's a hard ceiling, not a floor */
	screen_cap = bara_no_seidou_screen_off_cap(gd_policy);
	if (screen_cap)
		target = min(target, screen_cap);

	return target;
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

/* ---- Directional EMA util smoothing ------------------------------------ */

static unsigned long bara_no_seidou_ema_smooth(struct bara_no_seidou_cpu *gd_cpu,
					  unsigned long raw_util)
{
	struct bara_no_seidou_tunables *t = gd_cpu->gd_policy->tunables;
	unsigned long prev = gd_cpu->smoothed_util;
	unsigned int shift;

	if (!prev) {
		gd_cpu->smoothed_util = raw_util;
		return raw_util;
	}

	shift = raw_util >= prev ? t->ema_up_shift : t->ema_down_shift;
	gd_cpu->smoothed_util = prev + ((long)(raw_util - prev) >> shift);
	return gd_cpu->smoothed_util;
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
					    u64 time, unsigned long max)
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
	return (gd_cpu->iowait_boost * max) >> SCHED_CAPACITY_SHIFT;
}

/* ---- Fast path (per-cpu utilization callback) --------------------------- */

static void bara_no_seidou_update_single_cpu(struct bara_no_seidou_cpu *gd_cpu,
					 u64 time, unsigned int flags)
{
	struct bara_no_seidou_policy *gd_policy = gd_cpu->gd_policy;
	unsigned long util, max, boost;
	unsigned int next_f;

	bara_no_seidou_refresh_profile(gd_policy);

	max = arch_scale_cpu_capacity(gd_cpu->cpu);

	bara_no_seidou_iowait_boost(gd_cpu, time, flags);
	boost = bara_no_seidou_iowait_apply(gd_cpu, time, max);
	gd_cpu->last_update = time;

	util = sched_cpu_util(gd_cpu->cpu, max);
	util = max(util, boost);
	util = bara_no_seidou_ema_smooth(gd_cpu, util);

	next_f = bara_no_seidou_next_freq(gd_policy, gd_cpu->cpu, util, max, time);

	/* Don't let a brief idle blip drop freq below what's already
	 * running - avoids the measure-low / correct-next-tick sawtooth
	 * that reads as micro-stutter.
	 */
	if (bara_no_seidou_hold_freq(gd_cpu) && next_f < gd_policy->next_freq &&
	    !gd_policy->need_freq_update)
		next_f = gd_policy->next_freq;

	if (bara_no_seidou_rate_limited(gd_policy, time, next_f))
		return;

	gd_policy->next_freq = next_f;
	gd_policy->last_freq_update_time = time;

	if (gd_policy->policy->fast_switch_enabled)
		cpufreq_driver_fast_switch(gd_policy->policy, next_f);
	else
		bara_no_seidou_queue_update(gd_policy);
}

static void bara_no_seidou_update_util(struct update_util_data *hook, u64 time,
				  unsigned int flags)
{
	struct bara_no_seidou_cpu *gd_cpu = container_of(hook, struct bara_no_seidou_cpu,
						    update_util);

	/* Only the CPU actually entitled to drive this policy (matters on
	 * shared little-cluster policies with several CPUs) may act on it.
	 * Skips spurious/racy updates from a CPU mid-hotplug or isolated.
	 */
	if (!cpufreq_this_cpu_can_update(gd_cpu->gd_policy->policy))
		return;

	raw_spin_lock(&gd_cpu->gd_policy->update_lock);
	bara_no_seidou_update_single_cpu(gd_cpu, time, flags);
	raw_spin_unlock(&gd_cpu->gd_policy->update_lock);
}

/* ---- sysfs tunables ------------------------------------------------------ */

#define BARA_NO_SEIDOU_ATTR_RW(_name)						\
static ssize_t _name##_show(struct kobject *kobj,				\
			     struct kobj_attribute *attr, char *buf)		\
{										\
	struct bara_no_seidou_tunables *t = container_of(kobj,			\
					struct bara_no_seidou_tunables, kobj);	\
	return sprintf(buf, "%u\n", t->_name);					\
}										\
static ssize_t _name##_store(struct kobject *kobj,				\
			      struct kobj_attribute *attr,			\
			      const char *buf, size_t count)			\
{										\
	struct bara_no_seidou_tunables *t = container_of(kobj,			\
					struct bara_no_seidou_tunables, kobj);	\
	unsigned int val;							\
	if (kstrtouint(buf, 10, &val))						\
		return -EINVAL;							\
	t->_name = val;								\
	return count;								\
}										\
static struct kobj_attribute _name##_attr =					\
	__ATTR(_name, 0644, _name##_show, _name##_store)

#define BARA_NO_SEIDOU_ATTR_RW_BOOL(_name)						\
static ssize_t _name##_show(struct kobject *kobj,				\
			     struct kobj_attribute *attr, char *buf)		\
{										\
	struct bara_no_seidou_tunables *t = container_of(kobj,			\
					struct bara_no_seidou_tunables, kobj);	\
	return sprintf(buf, "%u\n", t->_name);					\
}										\
static ssize_t _name##_store(struct kobject *kobj,				\
			      struct kobj_attribute *attr,			\
			      const char *buf, size_t count)			\
{										\
	struct bara_no_seidou_tunables *t = container_of(kobj,			\
					struct bara_no_seidou_tunables, kobj);	\
	unsigned int val;							\
	if (kstrtouint(buf, 10, &val))						\
		return -EINVAL;							\
	t->_name = !!val;							\
	return count;								\
}										\
static struct kobj_attribute _name##_attr =					\
	__ATTR(_name, 0644, _name##_show, _name##_store)

BARA_NO_SEIDOU_ATTR_RW(up_rate_limit_normal_us);
BARA_NO_SEIDOU_ATTR_RW(down_rate_limit_normal_us);
BARA_NO_SEIDOU_ATTR_RW(down_rate_limit_game_us);
BARA_NO_SEIDOU_ATTR_RW(headroom_gaming_little_pct);
BARA_NO_SEIDOU_ATTR_RW(headroom_gaming_big_pct);
BARA_NO_SEIDOU_ATTR_RW(max_step_pct);
BARA_NO_SEIDOU_ATTR_RW(hispeed_load);
BARA_NO_SEIDOU_ATTR_RW(hispeed_freq_pct);
BARA_NO_SEIDOU_ATTR_RW(ema_up_shift);
BARA_NO_SEIDOU_ATTR_RW(ema_down_shift);
BARA_NO_SEIDOU_ATTR_RW(gaming_exit_delay_ms);
BARA_NO_SEIDOU_ATTR_RW(screen_off_cap_pct);
BARA_NO_SEIDOU_ATTR_RW(ramp_delta_pct);
BARA_NO_SEIDOU_ATTR_RW(ui_boost_ms);
BARA_NO_SEIDOU_ATTR_RW(little_cap_pct);
BARA_NO_SEIDOU_ATTR_RW(little_boost_cap_pct);
BARA_NO_SEIDOU_ATTR_RW(little_ui_floor_pct);
BARA_NO_SEIDOU_ATTR_RW(big_ui_floor_pct);
BARA_NO_SEIDOU_ATTR_RW(psi_cpu_floor_thresh);
BARA_NO_SEIDOU_ATTR_RW(psi_cpu_floor_pct);
BARA_NO_SEIDOU_ATTR_RW(psi_mem_cap_thresh);
BARA_NO_SEIDOU_ATTR_RW(psi_mem_cap_pct);
BARA_NO_SEIDOU_ATTR_RW_BOOL(thermal_aware);
BARA_NO_SEIDOU_ATTR_RW_BOOL(screen_on);

/*
 * touch_boost_ms is stored per-policy (so it shows up under every
 * cluster's bara-no-seidou/ dir) but the touch handler itself is global -
 * writing it from any policy updates the one shared value that
 * actually gates bara_no_seidou_touch_boosted().
 */
static ssize_t touch_boost_ms_show(struct kobject *kobj,
				    struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", READ_ONCE(bara_no_seidou_touch_boost_ms));
}

static ssize_t touch_boost_ms_store(struct kobject *kobj,
				     struct kobj_attribute *attr,
				     const char *buf, size_t count)
{
	struct bara_no_seidou_tunables *t = container_of(kobj,
					struct bara_no_seidou_tunables, kobj);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	t->touch_boost_ms = val;
	WRITE_ONCE(bara_no_seidou_touch_boost_ms, val);
	return count;
}
static struct kobj_attribute touch_boost_ms_attr =
	__ATTR(touch_boost_ms, 0644, touch_boost_ms_show, touch_boost_ms_store);

/*
 * Manual gaming-mode override. Reads/writes sched_gaming_active
 * directly - the same global the fair.c gaming bias uses - so this
 * file (or whatever node actually drives detection) keeps everything
 * in sync from a single source of truth.
 */
static ssize_t gaming_mode_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", READ_ONCE(sched_gaming_active));
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
	struct bara_no_seidou_tunables *t = container_of(kobj,
					struct bara_no_seidou_tunables, kobj);
	kfree(t);
}

static struct kobj_type bara_no_seidou_tunables_ktype = {
	.default_groups	= bara_no_seidou_groups,
	.sysfs_ops	= &kobj_sysfs_ops,
	.release	= bara_no_seidou_tunables_release,
};

static struct bara_no_seidou_tunables *bara_no_seidou_tunables_alloc(struct cpufreq_policy *policy)
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
	t->touch_boost_ms		= BARA_NO_SEIDOU_TOUCH_BOOST_MS_DEFAULT;
	t->screen_off_cap_pct		= BARA_NO_SEIDOU_SCREEN_OFF_CAP_PCT_DEFAULT;
	t->ramp_delta_pct		= BARA_NO_SEIDOU_RAMP_DELTA_PCT_DEFAULT;
	t->ui_boost_ms			= BARA_NO_SEIDOU_UI_BOOST_MS_DEFAULT;
	t->little_cap_pct		= BARA_NO_SEIDOU_LITTLE_CAP_PCT_DEFAULT;
	t->little_boost_cap_pct	= BARA_NO_SEIDOU_LITTLE_BOOST_CAP_PCT_DEFAULT;
	t->little_ui_floor_pct		= BARA_NO_SEIDOU_LITTLE_UI_FLOOR_PCT_DEFAULT;
	t->big_ui_floor_pct		= BARA_NO_SEIDOU_BIG_UI_FLOOR_PCT_DEFAULT;
	t->psi_cpu_floor_thresh		= BARA_NO_SEIDOU_PSI_CPU_FLOOR_THRESH_DEFAULT;
	t->psi_cpu_floor_pct		= BARA_NO_SEIDOU_PSI_CPU_FLOOR_PCT_DEFAULT;
	t->psi_mem_cap_thresh		= BARA_NO_SEIDOU_PSI_MEM_CAP_THRESH_DEFAULT;
	t->psi_mem_cap_pct		= BARA_NO_SEIDOU_PSI_MEM_CAP_PCT_DEFAULT;
	t->thermal_aware		= true;
	t->screen_on			= true;

	ret = kobject_init_and_add(&t->kobj, &bara_no_seidou_tunables_ktype,
				    get_governor_parent_kobj(policy),
				    "bara-no-seidou");
	if (ret) {
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
	int cpu, ret;

	cpufreq_enable_fast_switch(policy);

	gd_policy = kzalloc(sizeof(*gd_policy), GFP_KERNEL);
	if (!gd_policy) {
		ret = -ENOMEM;
		goto disable_fast_switch;
	}

	tunables = bara_no_seidou_tunables_alloc(policy);
	if (!tunables) {
		ret = -ENOMEM;
		goto free_policy;
	}

	gd_policy->policy = policy;
	gd_policy->tunables = tunables;
	tunables->gd_policy = gd_policy;
	/* Heuristic: the cluster containing logical CPU 0 is the little/
	 * primary cluster on every SM6225-class big.LITTLE layout this
	 * kernel targets (creek/bengal). Used only to soften the daily
	 * util discount there - never touches gaming or battery logic.
	 */
	gd_policy->is_little = cpumask_test_cpu(0, policy->related_cpus);
	raw_spin_lock_init(&gd_policy->update_lock);
	gd_policy->last_gaming_state = -1;
	gd_policy->screen_on_last = true;
	gd_policy->next_freq = policy->cur;
	bara_no_seidou_refresh_profile(gd_policy);

	/*
	 * The kthread + irq_work slow path is only needed when the
	 * driver can't fast-switch (frequency change may sleep). When
	 * fast switch is available, skip it entirely - one less thread,
	 * one less thing that can go wrong, and lower latency on every
	 * frequency change since there's no work-queue hop.
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
			goto free_tunables;
		}

		ret = sched_setattr_nocheck(gd_policy->thread, &attr);
		if (ret) {
			kthread_stop(gd_policy->thread);
			pr_warn("failed to set SCHED_DEADLINE/SUGOV for kthread, using default priority\n");
		}

		wake_up_process(gd_policy->thread);
	}

	policy->governor_data = gd_policy;

	for_each_cpu(cpu, policy->cpus) {
		struct bara_no_seidou_cpu *gd_cpu = &per_cpu(bara_no_seidou_cpu_list, cpu);

		memset(gd_cpu, 0, sizeof(*gd_cpu));
		gd_cpu->cpu = cpu;
		gd_cpu->gd_policy = gd_policy;
		cpufreq_add_update_util_hook(cpu, &gd_cpu->update_util,
					      bara_no_seidou_update_util);
	}

	if (atomic_inc_return(&bara_no_seidou_input_refcount) == 1) {
		ret = input_register_handler(&bara_no_seidou_input_handler);
		if (ret)
			pr_warn("touch boost unavailable (input_register_handler failed: %d)\n",
				ret);
	}

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
	pr_err("bara-no-seidou: initialization failed (error %d)\n", ret);
	return ret;
}

static void bara_no_seidou_exit(struct cpufreq_policy *policy)
{
	struct bara_no_seidou_policy *gd_policy = policy->governor_data;
	int cpu;

	for_each_cpu(cpu, policy->cpus)
		cpufreq_remove_update_util_hook(cpu);

	synchronize_rcu();

	if (atomic_dec_return(&bara_no_seidou_input_refcount) == 0)
		input_unregister_handler(&bara_no_seidou_input_handler);

	if (!policy->fast_switch_enabled && gd_policy->thread) {
		kthread_flush_worker(&gd_policy->worker);
		kthread_stop(gd_policy->thread);
		mutex_destroy(&gd_policy->work_lock);
	}

	kobject_put(&gd_policy->tunables->kobj);
	kfree(gd_policy);
	policy->governor_data = NULL;
	cpufreq_disable_fast_switch(policy);
}

static int bara_no_seidou_start(struct cpufreq_policy *policy)
{
	struct bara_no_seidou_policy *gd_policy = policy->governor_data;

	gd_policy->last_freq_update_time = 0;
	gd_policy->next_freq = policy->cur;
	gd_policy->need_freq_update = true;
	return 0;
}

static void bara_no_seidou_stop(struct cpufreq_policy *policy)
{
	struct bara_no_seidou_policy *gd_policy = policy->governor_data;

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
