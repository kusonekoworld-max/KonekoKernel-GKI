// SPDX-License-Identifier: GPL-2.0
/*
 * Vorpal CPUFreq Governor v2.0 - Perfect Gaming & Thermal Edition
 * Based on schedutil — optimized for 120fps gaming & daily use
 *
 * Features:
 *   • Dual-Profile Operating Modes      	— Gaming (locked high-band) / Daily (power-efficient)
 *   • Tri-Cluster Topology Awareness    	— Independent tuning for Little / Big / Prime
 *   • Directional EMA Util Smoothing    	— Fast-rise / slow-decay anti-yoyo filter
 *   • Dynamic Capacity Headroom         	— Load-proportional OPP headroom allocation
 *   • Proactive Thermal Step Controller 	— Smooth 2%-down / 1%-up cap ramping
 *   • Thermal Zone Integration          	— Hardware sensor + userspace fallback
 *   • Frame Pacing & Miss Recovery      	— Bounded floor boost on 120fps overrun
 *   • Global Frame Boost                	— All-cluster sync on dropped frames
 *   • Touch Input Responsiveness Boost  	— 220ms touch-window floor lift
 *   • UI Ramp-Assist / Render Burst     	— Sharp util-rise detection for animations
 *   • Adaptive Floor (Idle/Busy)        	— Prime & Little dynamic floor switching
 *   • Directional Rate Limiting         	— Per-cluster up/down rate gates
 *   • IOWait Performance Boost          	— Schedutil-legacy IOWait handling
 *   • Deadline Bandwidth Awareness      	— DL task frequency bypass
 *   • Jank Telemetry & Statistics       	— Frame/jank ratio reporting
 *   • Deferred IRQ-Work Frequency Commit 	— Async non-fast-switch path
 *   • Global Policy State Reset         	— Clean gaming-off transition
 *   • GKI 5.10 Util Interface           	— rfx_get_util_gki510 / rfx_dl_bw_exceeded_gki510
 *   • Scheduler Coupling              		— BORE/CFS gaming biases via sched_gaming_active
 *
 * Author: Templar Dev (Steambot12)
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/sched/topology.h>
#include <linux/rcupdate.h>
#include <linux/sched/rt.h>
#include <linux/sched/cpufreq.h>
#include <uapi/linux/sched/types.h>
#include <linux/tick.h>
#include <linux/timekeeping.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/irq_work.h>
#include <linux/percpu.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/atomic.h>
#include <linux/list.h>
#ifdef CONFIG_THERMAL
#include <linux/thermal.h>
#endif

#define CPUFREQ_VORPAL_NAME     "vorpal"
#define CPUFREQ_VORPAL_VERSION  "2.0"
#define CPUFREQ_VORPAL_AUTHOR   "Templar Dev"

/*
 * Scheduler coupling symbol. Defined and EXPORT_SYMBOL_GPL'd in
 * kernel/sched/fair.c (file-scope int, NOT a task_struct field -> KMI safe).
 * Written here by gaming_mode_store; read by fair.c gaming biases.
 */
extern int sched_gaming_active;

/* Core-sched util getter / deadline-bandwidth check (owned by core sched). */
extern void rfx_get_util_gki510(int cpu, unsigned long boost,
				unsigned long *util, unsigned long *bwmin);
extern bool rfx_dl_bw_exceeded_gki510(int cpu, unsigned long bwmin);

/* ===================================================================== */
/* Tunable defaults (KMI-safe: plain #defines, no struct-layout changes). */
/* ===================================================================== */

/* Cluster identification by arch capacity. */
#define RFX_LITTLE_CAP_THRESHOLD	614
#define RFX_PRIME_CAP_THRESHOLD		1000

/* Per-cluster rate limits (microseconds). up=0 means "scale up instantly". */
#define RFX_LITTLE_RATE_US		1000
#define RFX_LITTLE_UP_US		0
#define RFX_LITTLE_DOWN_US		4000

#define RFX_BIG_RATE_US			500
#define RFX_BIG_UP_US			0
#define RFX_BIG_DOWN_US			8000

#define RFX_PRIME_RATE_US		500
#define RFX_PRIME_UP_US			0
#define RFX_PRIME_DOWN_US		8000

/*
 * Gaming down-rate-limit. While gaming, frequency may only step DOWN this
 * slowly (microseconds). Combined with the floors below this is what kills
 * the yoyo / sawtooth: the clock parks in the high band and decays gently.
 */
#define RFX_GAMING_DOWN_US		30000

/* ---- Gaming frequency band, percent of policy fmax ---- */
/*
 * Prime: holds the render thread WHEN one is scheduled here. Telemetry from
 * real titles (Delta Force) shows the game often spreads its threads across
 * Big/Little and leaves Prime near-idle; a static high floor on an idle Prime
 * is pure waste heat with no FPS benefit. So the floor is ADAPTIVE: lock high
 * only once Prime is genuinely busy (util >= BUSY_ENTER), otherwise drop to a
 * lower idle floor that still recovers instantly (up-rate is 0 + headroom +
 * frame boost) the moment a heavy thread lands here.
 */
#define RFX_G_PRIME_FLOOR_PCT		90	/* busy: locked high */
#define RFX_G_PRIME_IDLE_FLOOR_PCT	65	/* near-idle: save heat */
#define RFX_G_PRIME_BUSY_ENTER_PCT	30
#define RFX_G_PRIME_CAP_PCT		99
#define RFX_G_PRIME_FRAME_PCT		95
/*
 * Big: in practice this cluster carries most of the game load. A cap of 90%
 * made it saturate (load 80%+) and starve frames; raised to 96% so the cores
 * that actually do the work have headroom -> load drops, frames land on time.
 */
#define RFX_G_BIG_FLOOR_PCT		80
#define RFX_G_BIG_CAP_PCT		96
#define RFX_G_BIG_FRAME_PCT		92
/*
 * Little: kept dynamic, but it too gets overloaded by render work, so its cap
 * is raised (85->95) to stop the 60-90% saturation that telemetry showed, and
 * it now honours the frame-miss boost.
 */
#define RFX_G_LITTLE_CAP_PCT		95
#define RFX_G_LITTLE_FLOOR_PCT		42
#define RFX_G_LITTLE_FLOOR_ENTER_PCT	20
#define RFX_G_LITTLE_FRAME_PCT		80

/* ---- Daily frequency shaping, percent of policy fmax ---- */
/*
 * "UI active" = a touch is recent OR a render burst was just detected (see
 * the ramp-assist below). When active, LITTLE's cap is relaxed and LITTLE/Big
 * get a responsiveness floor so animations / captions / scrolls do not run at
 * a starved OPP. When idle, LITTLE is capped low for battery.
 */
#define RFX_D_LITTLE_CAP_PCT		65	/* battery: cap LITTLE (was 55) */
#define RFX_D_LITTLE_BOOST_CAP_PCT	90	/* relax cap while UI active */
#define RFX_D_LITTLE_UI_FLOOR_PCT	58	/* LITTLE floor while UI active */
#define RFX_D_BIG_UI_FLOOR_PCT		55	/* Big floor while UI active */

/*
 * Daily UI ramp-assist. PELT/WALT util lags the real frame work, and the most
 * stutter-prone UI moments (a video caption appearing, an app open/close
 * animation, a fling-scroll) are often NOT touch events, so a touch-only boost
 * misses them. Instead we watch the smoothed util for a sharp RISE: a jump of
 * >= RFX_D_RAMP_DELTA_PCT points arms a short floor that holds for
 * RFX_D_UI_BOOST_NS, re-arming as long as demand keeps climbing. This lifts the
 * first frames of a burst immediately, independent of input - the core fix for
 * the gaming_mode=0 display/UI stutter.
 */
#define RFX_D_RAMP_DELTA_PCT		12
#define RFX_D_UI_BOOST_NS		(150 * NSEC_PER_MSEC)

/*
 * Touch window, lengthened 90 -> 220 ms so a tap-initiated app open/close
 * animation (~300 ms) stays boosted through the whole transition instead of
 * sagging halfway when the old short window expired.
 */
#define RFX_INPUT_WINDOW_NS		(220 * NSEC_PER_MSEC)

/* ---- Util EMA (directional smoothing). new>old: up_shift, else down_shift ---- */
#define RFX_EMA_UP_SHIFT_DAILY		1	/* rise fast: kill PELT-lag stutter */
#define RFX_EMA_DN_SHIFT_DAILY		3	/* decay gently: no inter-frame sag */
#define RFX_EMA_UP_SHIFT_GAMING		1	/* react fast to a demand rise */
#define RFX_EMA_DN_SHIFT_GAMING		3	/* decay slowly -> anti-jitter */

/* ---- Headroom (extra capacity above demand) percent ---- */
#define RFX_HEADROOM_GAMING_BIG		25
#define RFX_HEADROOM_GAMING_LITTLE	15

/* ---- Thermal step controller ---- */
#define RFX_THERMAL_STEP_NS		(6 * NSEC_PER_MSEC)
#define RFX_THERMAL_STEP_DOWN_PCT	2
#define RFX_THERMAL_STEP_UP_PCT		1
#define RFX_THERMAL_MIN_CAP_PCT		50
#define RFX_THERMAL_POLL_GAMING_MS	50
#define RFX_THERMAL_POLL_IDLE_MS	200
/*
 * Temperature breakpoints (milli-Celsius) -> target cap percent. GREEN raised
 * 38->40 so the cap stays at 100% through normal gameplay temps; throttling
 * that starts too early is felt as the residual jitter under load.
 */
#define RFX_TEMP_GREEN_MC		40000
#define RFX_TEMP_YELLOW_MC		43000
#define RFX_TEMP_RED_MC			45000

/* ---- Frame pacing ---- */
#define RFX_FRAME_BUDGET_US_120		8333	/* 1e6/120 */
#define RFX_FRAME_BOOST_NS		(120 * NSEC_PER_MSEC)
#define RFX_JANK_WINDOW_NS		(2000 * NSEC_PER_MSEC)

#define IOWAIT_BOOST_MIN		(SCHED_CAPACITY_SCALE / 8)

enum rfx_cluster_type {
	RFX_CLUSTER_LITTLE = 0,
	RFX_CLUSTER_BIG,
	RFX_CLUSTER_PRIME,
};

/* ===================================================================== */
/* Global state                                                          */
/* ===================================================================== */

/* Master gaming switch, written by gaming_mode sysfs (Prime cluster only). */
static atomic_t rfx_gaming = ATOMIC_INIT(0);

static inline bool rfx_gaming_enabled(void)
{
	return atomic_read(&rfx_gaming) != 0;
}

/* Last input event timestamp (daily touch boost). */
static atomic64_t rfx_input_ts_ns = ATOMIC64_INIT(0);

/* Thermal: target cap published by the poller, consumed by the fast path. */
static atomic_t rfx_thermal_cap_pct = ATOMIC_INIT(100);
/* Userspace-fed temperature fallback (milli-Celsius); 0 = unavailable. */
static atomic_t rfx_temp_mc = ATOMIC_INIT(0);

/* Frame pacing telemetry (userspace feeder writes frame_time_us). */
static atomic_t rfx_frame_time_us = ATOMIC_INIT(0);
static atomic_t rfx_frame_budget_us = ATOMIC_INIT(RFX_FRAME_BUDGET_US_120);
static atomic_t rfx_frames_seen = ATOMIC_INIT(0);
static atomic_t rfx_janks_seen = ATOMIC_INIT(0);
static atomic_t rfx_jank_pct = ATOMIC_INIT(0);

/*
 * Frame-miss boost deadline (ns since boot), GLOBAL so every cluster reacts to
 * a dropped frame - not just Prime. A missed frame means SOME cluster was too
 * slow; since the render thread's placement is not known in-kernel, all of
 * Prime/Big/Little lift their floor together until this deadline.
 */
static atomic64_t rfx_frame_boost_end_ns = ATOMIC64_INIT(0);

/* All live policies, so gaming-off can reset every cluster (not just Prime). */
static LIST_HEAD(rfx_policy_list);
static DEFINE_SPINLOCK(rfx_policy_list_lock);

/* ===================================================================== */
/* Data structures                                                       */
/* ===================================================================== */

struct rfx_tunables {
	struct gov_attr_set attr_set;
	unsigned int rate_limit_us;
	unsigned int up_rate_limit_us;
	unsigned int down_rate_limit_us;
	enum rfx_cluster_type cluster_type;
	unsigned int gaming_mode;
};

struct rfx_policy {
	struct cpufreq_policy *policy;
	struct rfx_tunables *tunables;
	struct list_head tunables_hook;
	struct list_head gov_node;	/* on rfx_policy_list */

	raw_spinlock_t update_lock;

	u64 last_upfreq_time;
	u64 last_downfreq_time;
	s64 freq_update_delay_ns;
	s64 up_rate_delay_ns;
	s64 down_rate_delay_ns;

	unsigned int next_freq;
	unsigned int cached_raw_freq;

	struct irq_work irq_work;
	struct kthread_work work;
	struct mutex work_lock;
	struct kthread_worker worker;
	struct task_struct *thread;
	bool work_in_progress;

	bool limits_changed;
	bool need_freq_update;

	bool is_prime;			/* this policy is the Prime cluster */
	bool is_little;

	unsigned int prev_upct;		/* last util%, for daily ramp detect */
	u64 ui_boost_end_ns;		/* daily: UI render-burst floor hold */

	int thermal_applied_pct;	/* walked toward rfx_thermal_cap_pct */
	u64 thermal_step_ns;
};

struct rfx_cpu {
	struct update_util_data update_util;
	struct rfx_policy *rfx_policy;
	unsigned int cpu;

	bool iowait_boost_pending;
	unsigned int iowait_boost;
	u64 last_update;

	unsigned long util;
	unsigned long bwmin;
	unsigned long filt_util;	/* directional EMA of effective util */
};

static DEFINE_PER_CPU(struct rfx_cpu, rfx_cpu);

static inline struct rfx_tunables *to_rfx_tunables(struct gov_attr_set *attr_set)
{
	return container_of(attr_set, struct rfx_tunables, attr_set);
}

static inline struct gov_attr_set *rfx_to_gov_attr_set(struct kobject *kobj)
{
	return container_of(kobj, struct gov_attr_set, kobj);
}

static inline bool rfx_cap_is_little(unsigned long cap)
{
	return cap <= (unsigned long)RFX_LITTLE_CAP_THRESHOLD;
}

static inline bool rfx_cap_is_prime(unsigned long cap)
{
	return cap >= (unsigned long)RFX_PRIME_CAP_THRESHOLD;
}

/* fmax * pct / 100 */
static inline unsigned int rfx_pct(unsigned int fmax, unsigned int pct)
{
	return (unsigned int)((u64)fmax * pct / 100);
}

static inline bool rfx_input_active(u64 time)
{
	u64 ts = (u64)atomic64_read(&rfx_input_ts_ns);

	return ts && (time - ts) < RFX_INPUT_WINDOW_NS;
}

/* ===================================================================== */
/* Util smoothing                                                        */
/* ===================================================================== */

/*
 * Directional EMA. Rising demand is tracked quickly (small up_shift) so the
 * UI / a new frame is not starved; falling demand decays slowly (larger
 * down_shift) so the clock does not chase every micro-dip - that decay is the
 * core of the anti-jitter / anti-yoyo behaviour. A forced minimum step of 1
 * prevents the filter from stalling on tiny deltas.
 */
static unsigned long rfx_ema(unsigned long old, unsigned long val, bool gaming)
{
	unsigned long up = gaming ? RFX_EMA_UP_SHIFT_GAMING : RFX_EMA_UP_SHIFT_DAILY;
	unsigned long dn = gaming ? RFX_EMA_DN_SHIFT_GAMING : RFX_EMA_DN_SHIFT_DAILY;
	unsigned long diff;

	if (!old)
		return val;
	if (val > old) {
		diff = val - old;
		return old + max(diff >> up, 1UL);
	}
	if (val < old) {
		diff = old - val;
		return old - max(diff >> dn, 1UL);
	}
	return val;
}

/*
 * Headroom: request slightly more capacity than measured so we land on an OPP
 * with room to spare (avoids running pinned at 100% util, which is both a
 * latency and a load-percent problem). Gaming uses a flat generous headroom;
 * daily uses a tiered curve that adds little at low util (battery) and more as
 * util climbs (responsiveness).
 */
static unsigned long rfx_apply_headroom(unsigned long util, unsigned long max_cap,
					bool gaming, bool little)
{
	unsigned int upct;

	if (!max_cap || util >= max_cap)
		return max_cap;

	upct = (unsigned int)(util * 100 / max_cap);
	if (upct >= 95)
		return max_cap;

	if (gaming) {
		unsigned int h = little ? RFX_HEADROOM_GAMING_LITTLE :
					  RFX_HEADROOM_GAMING_BIG;
		return min(util + util * h / 100, max_cap);
	}

	if (little) {
		if (upct >= 70)
			return min(util + (util >> 4), max_cap);
		if (upct >= 45)
			return min(util + (util >> 5), max_cap);
		return util;
	}

	if (upct >= 75)
		return min(util + (util >> 4), max_cap);
	if (upct >= 50)
		return min(util + (util >> 5), max_cap);
	return min(util + (util >> 6), max_cap);
}

/* ===================================================================== */
/* Thermal step controller (final clamp)                                 */
/* ===================================================================== */

/* Map temperature (milli-Celsius) to a target cap percent. */
static int rfx_temp_to_cap(int t_mc)
{
	if (t_mc < RFX_TEMP_GREEN_MC)
		return 100;
	if (t_mc < RFX_TEMP_YELLOW_MC)			/* -5%/C */
		return 100 - (t_mc - RFX_TEMP_GREEN_MC) * 5 / 1000;
	if (t_mc < RFX_TEMP_RED_MC)			/* -10%/C */
		return 80 - (t_mc - RFX_TEMP_YELLOW_MC) * 10 / 1000;
	return 60;
}

/*
 * Walk the applied cap toward the published target in small, rate-limited
 * steps and clamp the requested frequency to it. Stepping down faster than up
 * gives a smooth throttle entry and a gentle recovery (no oscillation at the
 * trip point). This is the LAST clamp, so it always wins over the floors -
 * that is what bounds power/temperature under the FPS-first policy.
 */
static unsigned int rfx_thermal_clamp(struct rfx_policy *p, unsigned int freq,
				      unsigned int fmax, u64 time)
{
	int target = atomic_read(&rfx_thermal_cap_pct);
	int applied = p->thermal_applied_pct ? p->thermal_applied_pct : 100;

	if ((s64)(time - p->thermal_step_ns) >= (s64)RFX_THERMAL_STEP_NS) {
		if (applied > target)
			applied -= RFX_THERMAL_STEP_DOWN_PCT;
		else if (applied < target)
			applied += RFX_THERMAL_STEP_UP_PCT;
		applied = clamp(applied, RFX_THERMAL_MIN_CAP_PCT, 100);
		p->thermal_applied_pct = applied;
		p->thermal_step_ns = time;
	}

	if (applied < 100) {
		unsigned int cap = rfx_pct(fmax, applied);

		if (freq > cap)
			freq = cap;
	}
	return freq;
}

/* ===================================================================== */
/* Frame pacing                                                          */
/* ===================================================================== */

/*
 * Called for the Prime policy each gaming update. If the userspace feeder
 * reports a frame that overran 1.5x the budget, arm a short floor boost so the
 * next frames recover to 120fps. The boost lifts the floor (see target_freq),
 * never forces fmax, so load stays mid.
 */
static void rfx_frame_account(u64 time)
{
	unsigned int ft = atomic_read(&rfx_frame_time_us);
	unsigned int bud = atomic_read(&rfx_frame_budget_us);

	if (!ft || !bud)
		return;

	atomic_inc(&rfx_frames_seen);
	if (ft > bud + (bud >> 1)) {
		atomic_inc(&rfx_janks_seen);
		atomic64_set(&rfx_frame_boost_end_ns, time + RFX_FRAME_BOOST_NS);
	}
}

static inline bool rfx_frame_boost_active(u64 time)
{
	u64 end = (u64)atomic64_read(&rfx_frame_boost_end_ns);

	return end && time < end;
}

/* ===================================================================== */
/* Frequency decision                                                    */
/* ===================================================================== */

/*
 * Pure-ish frequency selection from a (smoothed) util value. Order:
 *   1. headroom -> base freq from util/capacity
 *   2. profile shaping (gaming band lock OR daily caps/floors)
 *   3. thermal step clamp (final ceiling)
 *   4. resolve to a real OPP (cached to skip redundant table walks)
 */
static unsigned int rfx_target_freq(struct rfx_policy *p, unsigned long util,
				    unsigned long max_cap, u64 time, bool gaming)
{
	struct cpufreq_policy *pol = p->policy;
	unsigned int fmax = pol->cpuinfo.max_freq;
	unsigned int fmin = pol->cpuinfo.min_freq;
	bool little = rfx_cap_is_little(max_cap);
	bool prime = rfx_cap_is_prime(max_cap);
	unsigned int freq, upct;

	if (!fmax)
		return pol->cur;

	util = rfx_apply_headroom(util, max_cap, gaming, little);
	upct = max_cap ? (unsigned int)(util * 100 / max_cap) : 0;

	freq = (unsigned int)((u64)fmax * util / max_cap);
	freq = clamp(freq, fmin, fmax);

	if (gaming) {
		bool fboost = rfx_frame_boost_active(time);

		if (prime) {
			/*
			 * Adaptive floor: only lock Prime high once it is doing
			 * real work, so a game that parks its threads elsewhere
			 * does not leave Prime burning power at 90% for nothing.
			 */
			unsigned int fl = (upct >= RFX_G_PRIME_BUSY_ENTER_PCT) ?
				rfx_pct(fmax, RFX_G_PRIME_FLOOR_PCT) :
				rfx_pct(fmax, RFX_G_PRIME_IDLE_FLOOR_PCT);
			unsigned int cap = rfx_pct(fmax, RFX_G_PRIME_CAP_PCT);

			if (fboost)
				fl = max(fl, rfx_pct(fmax, RFX_G_PRIME_FRAME_PCT));
			if (freq < fl)
				freq = fl;
			if (freq > cap)
				freq = cap;
		} else if (!little) {		/* Big: carries most load */
			unsigned int fl = rfx_pct(fmax, RFX_G_BIG_FLOOR_PCT);
			unsigned int cap = rfx_pct(fmax, RFX_G_BIG_CAP_PCT);

			if (fboost)
				fl = max(fl, rfx_pct(fmax, RFX_G_BIG_FRAME_PCT));
			if (freq < fl)
				freq = fl;
			if (freq > cap)
				freq = cap;
		} else {			/* Little: dynamic, soft floor */
			unsigned int cap = rfx_pct(fmax, RFX_G_LITTLE_CAP_PCT);
			unsigned int fl = 0;

			if (upct > RFX_G_LITTLE_FLOOR_ENTER_PCT)
				fl = rfx_pct(fmax, RFX_G_LITTLE_FLOOR_PCT);
			if (fboost)
				fl = max(fl, rfx_pct(fmax, RFX_G_LITTLE_FRAME_PCT));
			if (freq < fl)
				freq = fl;
			if (freq > cap)
				freq = cap;
		}
	} else {
		bool ui_active;

		/*
		 * Detect a render burst: a sharp rise in smoothed util re-arms
		 * the UI floor. Catches caption draws / open-close animations /
		 * fling-scrolls that touch detection alone would miss.
		 */
		if (upct > p->prev_upct &&
		    upct - p->prev_upct >= RFX_D_RAMP_DELTA_PCT)
			p->ui_boost_end_ns = time + RFX_D_UI_BOOST_NS;
		p->prev_upct = upct;

		ui_active = rfx_input_active(time) ||
			    (p->ui_boost_end_ns && time < p->ui_boost_end_ns);

		if (little) {
			unsigned int cap = ui_active ?
				rfx_pct(fmax, RFX_D_LITTLE_BOOST_CAP_PCT) :
				rfx_pct(fmax, RFX_D_LITTLE_CAP_PCT);

			if (freq > cap)
				freq = cap;
			if (ui_active) {
				unsigned int fl = rfx_pct(fmax, RFX_D_LITTLE_UI_FLOOR_PCT);

				if (freq < fl)
					freq = fl;
			}
		} else if (!prime && ui_active) {	/* Big UI floor */
			unsigned int fl = rfx_pct(fmax, RFX_D_BIG_UI_FLOOR_PCT);

			if (freq < fl)
				freq = fl;
		}
	}

	freq = rfx_thermal_clamp(p, freq, fmax, time);
	freq = clamp(freq, fmin, fmax);

	if (freq == p->cached_raw_freq && !p->need_freq_update)
		return p->next_freq;
	p->cached_raw_freq = freq;
	return cpufreq_driver_resolve_freq(pol, freq);
}

/* ===================================================================== */
/* IO-wait boost (unchanged behaviour from schedutil lineage)            */
/* ===================================================================== */

static bool rfx_iowait_reset(struct rfx_cpu *rfx_c, u64 time, bool set)
{
	s64 delta_ns = time - rfx_c->last_update;

	if (delta_ns <= TICK_NSEC)
		return false;

	rfx_c->iowait_boost = set ? IOWAIT_BOOST_MIN : 0;
	rfx_c->iowait_boost_pending = set;
	return true;
}

static void rfx_iowait_boost(struct rfx_cpu *rfx_c, u64 time, unsigned int flags)
{
	bool set = flags & SCHED_CPUFREQ_IOWAIT;
	unsigned long max_cap;
	unsigned int cap;

	if (rfx_c->iowait_boost) {
		if (!rfx_iowait_reset(rfx_c, time, set))
			rfx_c->iowait_boost_pending = set;
		return;
	}
	if (!set || rfx_c->iowait_boost_pending)
		return;

	rfx_c->iowait_boost_pending = true;
	max_cap = arch_scale_cpu_capacity(rfx_c->cpu);

	if (rfx_c->iowait_boost >= max_cap) {
		cap = rfx_cap_is_little(max_cap) ? (SCHED_CAPACITY_SCALE / 6) :
						   (SCHED_CAPACITY_SCALE * 3 / 4);
		rfx_c->iowait_boost = min_t(unsigned int,
					    rfx_c->iowait_boost << 1, cap);
		return;
	}
	rfx_c->iowait_boost = IOWAIT_BOOST_MIN;
}

static unsigned long rfx_iowait_apply(struct rfx_cpu *rfx_c, u64 time,
				      unsigned long max_cap)
{
	if (!rfx_c->iowait_boost)
		return 0;
	if (rfx_iowait_reset(rfx_c, time, false))
		return 0;
	if (!rfx_c->iowait_boost_pending) {
		rfx_c->iowait_boost >>= 1;
		if (rfx_c->iowait_boost < IOWAIT_BOOST_MIN) {
			rfx_c->iowait_boost = 0;
			return 0;
		}
	}
	rfx_c->iowait_boost_pending = false;
	return rfx_c->iowait_boost * max_cap >> SCHED_CAPACITY_SHIFT;
}

static void rfx_get_util(struct rfx_cpu *rfx_c, unsigned long boost)
{
	rfx_get_util_gki510(rfx_c->cpu, boost, &rfx_c->util, &rfx_c->bwmin);
}

static inline void rfx_ignore_dl_rate_limit(struct rfx_cpu *rfx_c)
{
	if (rfx_dl_bw_exceeded_gki510(rfx_c->cpu, rfx_c->bwmin))
		rfx_c->rfx_policy->need_freq_update = true;
}

/* ===================================================================== */
/* Rate limiting                                                         */
/* ===================================================================== */

/* Set the active down-rate-limit for this update (long while gaming). */
static inline void rfx_set_down_delay(struct rfx_policy *p, bool gaming)
{
	if (gaming)
		p->down_rate_delay_ns = (s64)RFX_GAMING_DOWN_US * NSEC_PER_USEC;
	else
		p->down_rate_delay_ns =
			(s64)p->tunables->down_rate_limit_us * NSEC_PER_USEC;
}

/* up-rate-limit: instant up while gaming, tunable otherwise. */
static inline void rfx_pol_up_delay(struct rfx_policy *p, bool gaming)
{
	if (gaming)
		p->up_rate_delay_ns = 0;
	else
		p->up_rate_delay_ns =
			(s64)p->tunables->up_rate_limit_us * NSEC_PER_USEC;
}

/* Evaluation gate: cheap throttle on how often we recompute at all. */
static bool rfx_should_update_freq(struct rfx_policy *p, u64 time)
{
	s64 delta;

	if (!p || !p->policy)
		return false;
	if (!cpufreq_this_cpu_can_update(p->policy))
		return false;

	if (unlikely(READ_ONCE(p->limits_changed))) {
		WRITE_ONCE(p->limits_changed, false);
		p->need_freq_update = true;
		smp_mb();
		return true;
	}
	if (p->need_freq_update)
		return true;

	delta = (s64)(time - max(p->last_upfreq_time, p->last_downfreq_time));
	return delta >= p->freq_update_delay_ns;
}

/* Commit next_freq subject to directional up/down rate limits. */
static bool rfx_commit_freq(struct rfx_policy *p, u64 time, unsigned int next_freq)
{
	s64 delta;

	if (p->need_freq_update) {
		p->need_freq_update = false;
		if (p->next_freq == next_freq)
			return false;
	} else if (p->next_freq == next_freq) {
		return false;
	}

	if (next_freq < p->next_freq) {
		delta = (s64)(time - p->last_downfreq_time);
		if (p->down_rate_delay_ns > 0 && delta < p->down_rate_delay_ns)
			return false;
		p->last_downfreq_time = time;
	} else {
		delta = (s64)(time - p->last_upfreq_time);
		if (p->up_rate_delay_ns > 0 && delta < p->up_rate_delay_ns)
			return false;
		p->last_upfreq_time = time;
	}

	p->next_freq = next_freq;
	return true;
}

/* ===================================================================== */
/* Update hooks                                                          */
/* ===================================================================== */

static void rfx_deferred_update(struct rfx_policy *p)
{
	if (!p->work_in_progress) {
		p->work_in_progress = true;
		irq_work_queue(&p->irq_work);
	}
}

static void rfx_update_single_freq(struct update_util_data *hook, u64 time,
				   unsigned int flags)
{
	struct rfx_cpu *rfx_c = container_of(hook, struct rfx_cpu, update_util);
	struct rfx_policy *p = rfx_c->rfx_policy;
	bool gaming = rfx_gaming_enabled();
	unsigned long max_cap, boost, eff;
	unsigned int next_f;

	max_cap = arch_scale_cpu_capacity(rfx_c->cpu);

	rfx_iowait_boost(rfx_c, time, flags);
	rfx_c->last_update = time;
	rfx_ignore_dl_rate_limit(rfx_c);

	if (!rfx_should_update_freq(p, time))
		return;

	boost = rfx_iowait_apply(rfx_c, time, max_cap);
	rfx_get_util(rfx_c, boost);
	eff = max(rfx_c->util, boost);
	rfx_c->filt_util = rfx_ema(rfx_c->filt_util, eff, gaming);

	if (gaming)
		rfx_frame_account(time);

	rfx_set_down_delay(p, gaming);
	rfx_pol_up_delay(p, gaming);

	next_f = rfx_target_freq(p, rfx_c->filt_util, max_cap, time, gaming);

	if (!rfx_commit_freq(p, time, next_f))
		return;

	if (p->policy->fast_switch_enabled) {
		cpufreq_driver_fast_switch(p->policy, p->next_freq);
	} else {
		raw_spin_lock(&p->update_lock);
		rfx_deferred_update(p);
		raw_spin_unlock(&p->update_lock);
	}
}

static unsigned int rfx_next_freq_shared(struct rfx_cpu *rfx_c, u64 time,
					 bool gaming)
{
	struct rfx_policy *p = rfx_c->rfx_policy;
	struct cpufreq_policy *policy = p->policy;
	unsigned long max_cap = arch_scale_cpu_capacity(rfx_c->cpu);
	unsigned long max_util = 0;
	unsigned int j;

	for_each_cpu(j, policy->cpus) {
		struct rfx_cpu *jc = per_cpu_ptr(&rfx_cpu, j);
		unsigned long jb, je;

		jb = rfx_iowait_apply(jc, time, max_cap);
		rfx_get_util(jc, jb);
		je = max(jc->util, jb);
		jc->filt_util = rfx_ema(jc->filt_util, je, gaming);
		if (jc->filt_util > max_util)
			max_util = jc->filt_util;
	}

	if (gaming)
		rfx_frame_account(time);

	rfx_set_down_delay(p, gaming);
	rfx_pol_up_delay(p, gaming);

	return rfx_target_freq(p, max_util, max_cap, time, gaming);
}

static void rfx_update_shared(struct update_util_data *hook, u64 time,
			      unsigned int flags)
{
	struct rfx_cpu *rfx_c = container_of(hook, struct rfx_cpu, update_util);
	struct rfx_policy *p = rfx_c->rfx_policy;
	bool gaming = rfx_gaming_enabled();
	unsigned int next_f;

	raw_spin_lock(&p->update_lock);

	rfx_iowait_boost(rfx_c, time, flags);
	rfx_c->last_update = time;
	rfx_ignore_dl_rate_limit(rfx_c);

	if (rfx_should_update_freq(p, time)) {
		next_f = rfx_next_freq_shared(rfx_c, time, gaming);
		if (rfx_commit_freq(p, time, next_f)) {
			if (p->policy->fast_switch_enabled)
				cpufreq_driver_fast_switch(p->policy, p->next_freq);
			else
				rfx_deferred_update(p);
		}
	}

	raw_spin_unlock(&p->update_lock);
}

static void rfx_work(struct kthread_work *work)
{
	struct rfx_policy *p = container_of(work, struct rfx_policy, work);
	unsigned int freq;
	unsigned long flags;

	raw_spin_lock_irqsave(&p->update_lock, flags);
	freq = p->next_freq;
	p->work_in_progress = false;
	raw_spin_unlock_irqrestore(&p->update_lock, flags);

	mutex_lock(&p->work_lock);
	cpufreq_driver_target(p->policy, freq, CPUFREQ_RELATION_L);
	mutex_unlock(&p->work_lock);
}

static void rfx_irq_work(struct irq_work *irq_work)
{
	struct rfx_policy *p = container_of(irq_work, struct rfx_policy, irq_work);

	kthread_queue_work(&p->worker, &p->work);
}

/* ===================================================================== */
/* Thermal poller (slow path, may sleep -> never in the util hook)       */
/* ===================================================================== */

#ifdef CONFIG_THERMAL
static struct thermal_zone_device *rfx_tz;
static char rfx_tz_name[THERMAL_NAME_LENGTH];
#endif
static struct delayed_work rfx_thermal_work;
static u64 rfx_jank_window_start;

static void rfx_thermal_fn(struct work_struct *w)
{
	int t_mc = 0;
	bool have = false;
	unsigned int delay_ms;
	int frames, janks;
	u64 now = ktime_get_ns();

#ifdef CONFIG_THERMAL
	if (rfx_tz && !thermal_zone_get_temp(rfx_tz, &t_mc))
		have = true;
#endif
	if (!have) {
		t_mc = atomic_read(&rfx_temp_mc);
		if (t_mc > 0)
			have = true;
	}

	atomic_set(&rfx_thermal_cap_pct, have ? rfx_temp_to_cap(t_mc) : 100);

	/* Jank window: publish jank percent roughly every RFX_JANK_WINDOW_NS. */
	if (!rfx_jank_window_start)
		rfx_jank_window_start = now;
	if (now - rfx_jank_window_start >= RFX_JANK_WINDOW_NS) {
		frames = atomic_xchg(&rfx_frames_seen, 0);
		janks = atomic_xchg(&rfx_janks_seen, 0);
		atomic_set(&rfx_jank_pct, frames ? janks * 100 / frames : 0);
		rfx_jank_window_start = now;
	}

	delay_ms = rfx_gaming_enabled() ? RFX_THERMAL_POLL_GAMING_MS :
					  RFX_THERMAL_POLL_IDLE_MS;
	schedule_delayed_work(&rfx_thermal_work, msecs_to_jiffies(delay_ms));
}

/* ===================================================================== */
/* Input handler (daily touch boost; off during gaming)                  */
/* ===================================================================== */

static void rfx_input_event(struct input_handle *handle, unsigned int type,
			    unsigned int code, int value)
{
	if (rfx_gaming_enabled())
		return;
	if (type == EV_ABS || type == EV_KEY)
		atomic64_set(&rfx_input_ts_ns, ktime_get_ns());
}

static int rfx_input_connect(struct input_handler *handler,
			     struct input_dev *dev,
			     const struct input_device_id *id)
{
	struct input_handle *handle;
	int err;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "vorpal";

	err = input_register_handle(handle);
	if (err)
		goto err_free;
	err = input_open_device(handle);
	if (err)
		goto err_unregister;
	return 0;

err_unregister:
	input_unregister_handle(handle);
err_free:
	kfree(handle);
	return err;
}

static void rfx_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id rfx_input_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			 INPUT_DEVICE_ID_MATCH_ABSBIT,
		.evbit = { BIT_MASK(EV_ABS) },
		.absbit = { [BIT_WORD(ABS_MT_POSITION_X)] =
			    BIT_MASK(ABS_MT_POSITION_X) },
	},
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT |
			 INPUT_DEVICE_ID_MATCH_KEYBIT,
		.evbit = { BIT_MASK(EV_KEY) },
		.keybit = { [BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) },
	},
	{ },
};

static struct input_handler rfx_input_handler = {
	.event		= rfx_input_event,
	.connect	= rfx_input_connect,
	.disconnect	= rfx_input_disconnect,
	.name		= "vorpal",
	.id_table	= rfx_input_ids,
};

/* ===================================================================== */
/* sysfs                                                                 */
/* ===================================================================== */

static struct rfx_tunables *rfx_global_tunables;
static DEFINE_MUTEX(rfx_global_tunables_lock);

static ssize_t rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_rfx_tunables(attr_set)->rate_limit_us);
}
static ssize_t rate_limit_us_store(struct gov_attr_set *attr_set,
				   const char *buf, size_t count)
{
	struct rfx_tunables *t = to_rfx_tunables(attr_set);
	struct rfx_policy *p;
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	t->rate_limit_us = val;
	list_for_each_entry(p, &attr_set->policy_list, tunables_hook)
		p->freq_update_delay_ns = (s64)val * NSEC_PER_USEC;
	return count;
}
static struct governor_attr rate_limit_us = __ATTR_RW(rate_limit_us);

static ssize_t up_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_rfx_tunables(attr_set)->up_rate_limit_us);
}
static ssize_t up_rate_limit_us_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct rfx_tunables *t = to_rfx_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	t->up_rate_limit_us = val;
	return count;
}
static struct governor_attr up_rate_limit_us = __ATTR_RW(up_rate_limit_us);

static ssize_t down_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_rfx_tunables(attr_set)->down_rate_limit_us);
}
static ssize_t down_rate_limit_us_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct rfx_tunables *t = to_rfx_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	t->down_rate_limit_us = val;
	return count;
}
static struct governor_attr down_rate_limit_us = __ATTR_RW(down_rate_limit_us);

/* Reset transient gaming residue on every live policy (all clusters). */
static void rfx_reset_all_policies(void)
{
	struct rfx_policy *p;
	unsigned long flags;

	atomic64_set(&rfx_frame_boost_end_ns, 0);

	spin_lock_irqsave(&rfx_policy_list_lock, flags);
	list_for_each_entry(p, &rfx_policy_list, gov_node)
		p->need_freq_update = true;
	spin_unlock_irqrestore(&rfx_policy_list_lock, flags);
}

static ssize_t gaming_mode_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%u\n", to_rfx_tunables(attr_set)->gaming_mode);
}
static ssize_t gaming_mode_store(struct gov_attr_set *attr_set,
				 const char *buf, size_t count)
{
	struct rfx_tunables *t = to_rfx_tunables(attr_set);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > 1)
		return -EINVAL;

	t->gaming_mode = val;
	atomic_set(&rfx_gaming, val);
	/* Drive the scheduler-side gaming biases in lockstep (KMI-safe int). */
	WRITE_ONCE(sched_gaming_active, (int)val);

	if (!val) {
		atomic_set(&rfx_frame_time_us, 0);
		atomic_set(&rfx_frames_seen, 0);
		atomic_set(&rfx_janks_seen, 0);
		atomic_set(&rfx_jank_pct, 0);
		rfx_reset_all_policies();
	} else {
		/* Default the budget to 120fps unless userspace set it. */
		if (!atomic_read(&rfx_frame_budget_us))
			atomic_set(&rfx_frame_budget_us, RFX_FRAME_BUDGET_US_120);
		/* Sample temperature sooner once gaming begins. */
		mod_delayed_work(system_wq, &rfx_thermal_work,
				 msecs_to_jiffies(RFX_THERMAL_POLL_GAMING_MS));
	}
	return count;
}
static struct governor_attr gaming_mode = __ATTR_RW(gaming_mode);

static ssize_t temp_mc_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&rfx_temp_mc));
}
static ssize_t temp_mc_store(struct gov_attr_set *attr_set,
			     const char *buf, size_t count)
{
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;
	atomic_set(&rfx_temp_mc, val);
	return count;
}
static struct governor_attr temp_mc = __ATTR_RW(temp_mc);

static ssize_t thermal_zone_show(struct gov_attr_set *attr_set, char *buf)
{
#ifdef CONFIG_THERMAL
	return sprintf(buf, "%s\n", rfx_tz_name[0] ? rfx_tz_name : "(none)");
#else
	return sprintf(buf, "(no CONFIG_THERMAL)\n");
#endif
}
static ssize_t thermal_zone_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
#ifdef CONFIG_THERMAL
	struct thermal_zone_device *tz;
	char name[THERMAL_NAME_LENGTH];

	strscpy(name, buf, sizeof(name));
	strim(name);
	tz = thermal_zone_get_zone_by_name(name);
	if (IS_ERR(tz))
		return -EINVAL;
	rfx_tz = tz;
	strscpy(rfx_tz_name, name, sizeof(rfx_tz_name));
	return count;
#else
	return -ENODEV;
#endif
}
static struct governor_attr thermal_zone = __ATTR_RW(thermal_zone);

static ssize_t frame_budget_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&rfx_frame_budget_us));
}
static ssize_t frame_budget_us_store(struct gov_attr_set *attr_set,
				     const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 10, &val) || val < 1000)
		return -EINVAL;
	atomic_set(&rfx_frame_budget_us, val);
	return count;
}
static struct governor_attr frame_budget_us = __ATTR_RW(frame_budget_us);

static ssize_t frame_time_us_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&rfx_frame_time_us));
}
static ssize_t frame_time_us_store(struct gov_attr_set *attr_set,
				   const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	atomic_set(&rfx_frame_time_us, val);
	return count;
}
static struct governor_attr frame_time_us = __ATTR_RW(frame_time_us);

static ssize_t jank_pct_show(struct gov_attr_set *attr_set, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&rfx_jank_pct));
}
static struct governor_attr jank_pct = __ATTR_RO(jank_pct);

static struct attribute *rfx_little_attrs[] = {
	&rate_limit_us.attr,
	&up_rate_limit_us.attr,
	&down_rate_limit_us.attr,
	NULL
};
ATTRIBUTE_GROUPS(rfx_little);

static struct attribute *rfx_big_attrs[] = {
	&rate_limit_us.attr,
	&up_rate_limit_us.attr,
	&down_rate_limit_us.attr,
	NULL
};
ATTRIBUTE_GROUPS(rfx_big);

static struct attribute *rfx_prime_attrs[] = {
	&rate_limit_us.attr,
	&up_rate_limit_us.attr,
	&down_rate_limit_us.attr,
	&gaming_mode.attr,
	&temp_mc.attr,
	&thermal_zone.attr,
	&frame_budget_us.attr,
	&frame_time_us.attr,
	&jank_pct.attr,
	NULL
};
ATTRIBUTE_GROUPS(rfx_prime);

static void rfx_tunables_free(struct kobject *kobj)
{
	kfree(to_rfx_tunables(rfx_to_gov_attr_set(kobj)));
}

static struct kobj_type rfx_little_ktype = {
	.default_groups = rfx_little_groups,
	.sysfs_ops = &governor_sysfs_ops,
	.release = rfx_tunables_free,
};
static struct kobj_type rfx_big_ktype = {
	.default_groups = rfx_big_groups,
	.sysfs_ops = &governor_sysfs_ops,
	.release = rfx_tunables_free,
};
static struct kobj_type rfx_prime_ktype = {
	.default_groups = rfx_prime_groups,
	.sysfs_ops = &governor_sysfs_ops,
	.release = rfx_tunables_free,
};

static struct cpufreq_governor vorpal_gov;

/* ===================================================================== */
/* Allocation / kthread                                                  */
/* ===================================================================== */

static struct rfx_policy *rfx_policy_alloc(struct cpufreq_policy *policy)
{
	struct rfx_policy *p;

	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (!p)
		return NULL;
	p->policy = policy;
	p->thermal_applied_pct = 100;
	raw_spin_lock_init(&p->update_lock);
	INIT_LIST_HEAD(&p->gov_node);
	return p;
}

static void rfx_policy_free(struct rfx_policy *p)
{
	kfree(p);
}

static int rfx_kthread_create(struct rfx_policy *p)
{
	struct task_struct *thread;
	struct cpufreq_policy *policy = p->policy;
	struct sched_param sp = { .sched_priority = MAX_RT_PRIO / 2 };
	int ret;

	if (policy->fast_switch_enabled)
		return 0;

	kthread_init_work(&p->work, rfx_work);
	kthread_init_worker(&p->worker);
	thread = kthread_create(kthread_worker_fn, &p->worker, "rfx_gov/%d",
				cpumask_first(policy->related_cpus));
	if (IS_ERR(thread)) {
		pr_err("vorpal: kthread create failed %ld\n", PTR_ERR(thread));
		return PTR_ERR(thread);
	}

	ret = sched_setscheduler_nocheck(thread, SCHED_FIFO, &sp);
	if (ret) {
		kthread_stop(thread);
		pr_warn("vorpal: failed to set SCHED_FIFO\n");
		return ret;
	}

	p->thread = thread;
	if (policy->dvfs_possible_from_any_cpu)
		set_cpus_allowed_ptr(thread, policy->related_cpus);
	else
		kthread_bind_mask(thread, policy->related_cpus);

	init_irq_work(&p->irq_work, rfx_irq_work);
	mutex_init(&p->work_lock);
	wake_up_process(thread);
	return 0;
}

static void rfx_kthread_stop(struct rfx_policy *p)
{
	if (p->policy->fast_switch_enabled)
		return;
	kthread_flush_worker(&p->worker);
	kthread_stop(p->thread);
	mutex_destroy(&p->work_lock);
}

static struct rfx_tunables *rfx_tunables_alloc(struct rfx_policy *p)
{
	struct rfx_tunables *t;

	t = kzalloc(sizeof(*t), GFP_KERNEL);
	if (t) {
		gov_attr_set_init(&t->attr_set, &p->tunables_hook);
		if (!have_governor_per_policy())
			rfx_global_tunables = t;
	}
	return t;
}

static void rfx_clear_global_tunables(void)
{
	if (!have_governor_per_policy())
		rfx_global_tunables = NULL;
}

/* ===================================================================== */
/* Governor callbacks                                                    */
/* ===================================================================== */

static int rfx_init(struct cpufreq_policy *policy)
{
	struct rfx_policy *p;
	struct rfx_tunables *t;
	unsigned long max_cap;
	struct kobj_type *ktype;
	int ret = 0;

	if (policy->governor_data)
		return -EBUSY;

	cpufreq_enable_fast_switch(policy);

	p = rfx_policy_alloc(policy);
	if (!p) {
		ret = -ENOMEM;
		goto disable_fast_switch;
	}

	ret = rfx_kthread_create(p);
	if (ret)
		goto free_p;

	max_cap = arch_scale_cpu_capacity(cpumask_first(policy->cpus));
	p->is_prime = rfx_cap_is_prime(max_cap);
	p->is_little = rfx_cap_is_little(max_cap);

	mutex_lock(&rfx_global_tunables_lock);

	if (rfx_global_tunables) {
		if (WARN_ON(have_governor_per_policy())) {
			ret = -EINVAL;
			goto stop_kthread;
		}
		policy->governor_data = p;
		p->tunables = rfx_global_tunables;
		gov_attr_set_get(&rfx_global_tunables->attr_set, &p->tunables_hook);
		goto out;
	}

	t = rfx_tunables_alloc(p);
	if (!t) {
		ret = -ENOMEM;
		goto stop_kthread;
	}

	if (p->is_little) {
		t->cluster_type = RFX_CLUSTER_LITTLE;
		t->rate_limit_us = RFX_LITTLE_RATE_US;
		t->up_rate_limit_us = RFX_LITTLE_UP_US;
		t->down_rate_limit_us = RFX_LITTLE_DOWN_US;
		ktype = &rfx_little_ktype;
	} else if (p->is_prime) {
		t->cluster_type = RFX_CLUSTER_PRIME;
		t->rate_limit_us = RFX_PRIME_RATE_US;
		t->up_rate_limit_us = RFX_PRIME_UP_US;
		t->down_rate_limit_us = RFX_PRIME_DOWN_US;
		ktype = &rfx_prime_ktype;
	} else {
		t->cluster_type = RFX_CLUSTER_BIG;
		t->rate_limit_us = RFX_BIG_RATE_US;
		t->up_rate_limit_us = RFX_BIG_UP_US;
		t->down_rate_limit_us = RFX_BIG_DOWN_US;
		ktype = &rfx_big_ktype;
	}

	policy->governor_data = p;
	p->tunables = t;

	ret = kobject_init_and_add(&t->attr_set.kobj, ktype,
				   get_governor_parent_kobj(policy),
				   "%s", vorpal_gov.name);
	if (ret)
		goto fail;

out:
	p->freq_update_delay_ns = (s64)p->tunables->rate_limit_us * NSEC_PER_USEC;
	p->up_rate_delay_ns = (s64)p->tunables->up_rate_limit_us * NSEC_PER_USEC;
	p->down_rate_delay_ns = (s64)p->tunables->down_rate_limit_us * NSEC_PER_USEC;
	mutex_unlock(&rfx_global_tunables_lock);
	return 0;

fail:
	kobject_put(&t->attr_set.kobj);
	policy->governor_data = NULL;
	rfx_clear_global_tunables();
stop_kthread:
	rfx_kthread_stop(p);
	mutex_unlock(&rfx_global_tunables_lock);
free_p:
	rfx_policy_free(p);
disable_fast_switch:
	cpufreq_disable_fast_switch(policy);
	pr_err("vorpal: init failed error %d\n", ret);
	return ret;
}

static void rfx_exit(struct cpufreq_policy *policy)
{
	struct rfx_policy *p = policy->governor_data;
	struct rfx_tunables *t = p->tunables;
	unsigned int count;

	mutex_lock(&rfx_global_tunables_lock);
	count = gov_attr_set_put(&t->attr_set, &p->tunables_hook);
	policy->governor_data = NULL;
	if (!count)
		rfx_clear_global_tunables();
	mutex_unlock(&rfx_global_tunables_lock);

	rfx_kthread_stop(p);
	rfx_policy_free(p);
	cpufreq_disable_fast_switch(policy);
}

static int rfx_start(struct cpufreq_policy *policy)
{
	struct rfx_policy *p = policy->governor_data;
	void (*uu)(struct update_util_data *data, u64 time, unsigned int flags);
	unsigned long flags;
	unsigned int cpu;
	u64 now = ktime_get_ns();

	p->freq_update_delay_ns = (s64)p->tunables->rate_limit_us * NSEC_PER_USEC;
	p->up_rate_delay_ns = (s64)p->tunables->up_rate_limit_us * NSEC_PER_USEC;
	p->down_rate_delay_ns = (s64)p->tunables->down_rate_limit_us * NSEC_PER_USEC;

	p->last_upfreq_time = now;
	p->last_downfreq_time = now;
	p->next_freq = policy->cur > 0 ? policy->cur : policy->cpuinfo.min_freq;
	p->cached_raw_freq = 0;
	p->work_in_progress = false;
	p->limits_changed = false;
	p->need_freq_update = false;
	p->prev_upct = 0;
	p->ui_boost_end_ns = 0;
	p->thermal_applied_pct = 100;
	p->thermal_step_ns = now;

	spin_lock_irqsave(&rfx_policy_list_lock, flags);
	list_add(&p->gov_node, &rfx_policy_list);
	spin_unlock_irqrestore(&rfx_policy_list_lock, flags);

	for_each_cpu(cpu, policy->cpus) {
		struct rfx_cpu *rfx_c = per_cpu_ptr(&rfx_cpu, cpu);

		memset(rfx_c, 0, sizeof(*rfx_c));
		rfx_c->cpu = cpu;
		rfx_c->rfx_policy = p;
	}

	uu = policy_is_shared(policy) ? rfx_update_shared : rfx_update_single_freq;
	for_each_cpu(cpu, policy->cpus)
		cpufreq_add_update_util_hook(cpu, &per_cpu_ptr(&rfx_cpu, cpu)->update_util, uu);
	return 0;
}

static void rfx_stop(struct cpufreq_policy *policy)
{
	struct rfx_policy *p = policy->governor_data;
	unsigned long flags;
	unsigned int cpu;

	for_each_cpu(cpu, policy->cpus)
		cpufreq_remove_update_util_hook(cpu);

	synchronize_rcu();

	spin_lock_irqsave(&rfx_policy_list_lock, flags);
	list_del(&p->gov_node);
	spin_unlock_irqrestore(&rfx_policy_list_lock, flags);

	if (!policy->fast_switch_enabled) {
		irq_work_sync(&p->irq_work);
		kthread_cancel_work_sync(&p->work);
	}
}

static void rfx_limits(struct cpufreq_policy *policy)
{
	struct rfx_policy *p = policy->governor_data;

	if (!policy->fast_switch_enabled) {
		mutex_lock(&p->work_lock);
		cpufreq_policy_apply_limits(policy);
		mutex_unlock(&p->work_lock);
	}
	smp_wmb();
	WRITE_ONCE(p->limits_changed, true);
}

static struct cpufreq_governor vorpal_gov = {
	.name = CPUFREQ_VORPAL_NAME,
	.owner = THIS_MODULE,
	.flags = CPUFREQ_GOV_DYNAMIC_SWITCHING,
	.init = rfx_init,
	.exit = rfx_exit,
	.start = rfx_start,
	.stop = rfx_stop,
	.limits = rfx_limits,
};

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_VORPAL
struct cpufreq_governor *cpufreq_default_governor(void)
{
	return &vorpal_gov;
}
#endif

static int __init vorpal_gov_init(void)
{
	int ret;

	pr_info("Vorpal Governor v%s by %s\n", CPUFREQ_VORPAL_VERSION,
		CPUFREQ_VORPAL_AUTHOR);

	INIT_DEFERRABLE_WORK(&rfx_thermal_work, rfx_thermal_fn);
	schedule_delayed_work(&rfx_thermal_work,
			      msecs_to_jiffies(RFX_THERMAL_POLL_IDLE_MS));

	if (input_register_handler(&rfx_input_handler))
		pr_warn("vorpal: input handler register failed (touch boost off)\n");

	ret = cpufreq_register_governor(&vorpal_gov);
	if (ret) {
		input_unregister_handler(&rfx_input_handler);
		cancel_delayed_work_sync(&rfx_thermal_work);
	}
	return ret;
}

static void __exit vorpal_gov_exit(void)
{
	cpufreq_unregister_governor(&vorpal_gov);
	input_unregister_handler(&rfx_input_handler);
	cancel_delayed_work_sync(&rfx_thermal_work);
}

module_init(vorpal_gov_init);
module_exit(vorpal_gov_exit);

MODULE_AUTHOR(CPUFREQ_VORPAL_AUTHOR);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Vorpal CPUFreq Governor v2.0");
