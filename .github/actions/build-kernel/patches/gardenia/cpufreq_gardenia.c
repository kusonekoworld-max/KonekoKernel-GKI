#include "sched.h"

extern int sched_gaming_active;

/* ---- Defaults (overridable at runtime via sysfs) --------------------- */

#define GARDENIA_UP_RATE_LIMIT_NORMAL_US	(8000)
#define GARDENIA_DOWN_RATE_LIMIT_NORMAL_US	(50000)
#define GARDENIA_UP_RATE_LIMIT_GAME_US		(2000)
#define GARDENIA_DOWN_RATE_LIMIT_GAME_US	(8000)

#define GARDENIA_UTIL_BIAS_PCT_DEFAULT		(92)	/* big cluster, normal */
#define GARDENIA_UTIL_BIAS_LITTLE_PCT_DEFAULT	(96)	/* little cluster, normal */
#define GARDENIA_MAX_STEP_PCT_DEFAULT		(80)	/* normal profile */

#define GARDENIA_HISPEED_LOAD_DEFAULT		(85)	/* pct, gaming only */
#define GARDENIA_HISPEED_FREQ_PCT_DEFAULT	(70)	/* pct of max_freq */

#define GARDENIA_THERMAL_DISCOUNT_MAX_PCT	(25)	/* extra cut at full throttle */
#define GARDENIA_GAMING_EXIT_DELAY_MS_DEFAULT	(3000)	/* gaming exit hysteresis */

#define GARDENIA_TOUCH_BOOST_MS_DEFAULT	(220)	/* daily-use smoothness */
#define GARDENIA_SCREEN_OFF_CAP_PCT_DEFAULT	(35)	/* pct of max_freq */

/* Directional EMA on util: rises fast (kills PELT-lag stutter), decays
 * slowly (no inter-frame sag / yoyo). Shift of 1 = half-weight on the
 * new sample; 3 = eighth-weight. Learned from Vorpal's util smoothing.
 */
#define GARDENIA_EMA_UP_SHIFT_DEFAULT		(0)
#define GARDENIA_EMA_DOWN_SHIFT_DEFAULT	(3)

/* Matches schedutil/Reflex convention: smallest IOWait boost step,
 * doubles on repeated IOWait ticks, halves when they stop.
 */
#define IOWAIT_BOOST_MIN			(SCHED_CAPACITY_SCALE / 8)

/* ---- Global touch-boost state (shared across all policies/clusters) --- */

static atomic64_t gardenia_touch_boost_until_ns = ATOMIC64_INIT(0);
static unsigned int gardenia_touch_boost_ms = GARDENIA_TOUCH_BOOST_MS_DEFAULT;

static void gardenia_touch_event(struct input_handle *handle,
				  unsigned int type, unsigned int code,
				  int value)
{
	if (type != EV_ABS && type != EV_KEY)
		return;

	atomic64_set(&gardenia_touch_boost_until_ns,
		     local_clock() + (u64)gardenia_touch_boost_ms * NSEC_PER_MSEC);
}

static bool gardenia_touch_boosted(void)
{
	return local_clock() < atomic64_read(&gardenia_touch_boost_until_ns);
}

static int gardenia_input_connect(struct input_handler *handler,
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
	handle->name = "gardenia_touch";

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

static void gardenia_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id gardenia_input_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_ABS) },
	},
	{ },
};

static struct input_handler gardenia_input_handler = {
	.event		= gardenia_touch_event,
	.connect	= gardenia_input_connect,
	.disconnect	= gardenia_input_disconnect,
	.name		= "gardenia_touch",
	.id_table	= gardenia_input_ids,
};

/* ---- Types ------------------------------------------------------------ */

struct gardenia_tunables {
	struct kobject kobj;
	struct gardenia_policy *gd_policy;	/* backpointer, set in init */

	unsigned int up_rate_limit_normal_us;
	unsigned int down_rate_limit_normal_us;
	unsigned int up_rate_limit_game_us;
	unsigned int down_rate_limit_game_us;

	unsigned int util_bias_pct;
	unsigned int max_step_pct;

	unsigned int hispeed_load;
	unsigned int hispeed_freq_pct;

	unsigned int gaming_exit_delay_ms;
	unsigned int touch_boost_ms;
	unsigned int screen_off_cap_pct;
	unsigned int ema_up_shift;
	unsigned int ema_down_shift;

	bool thermal_aware;
	bool screen_on;
};

struct gardenia_policy {
	struct cpufreq_policy *policy;
	struct gardenia_tunables *tunables;
	bool is_little;

	raw_spinlock_t update_lock;
	u64 last_freq_update_time;
	s64 up_rate_delay_ns;
	s64 down_rate_delay_ns;
	unsigned int next_freq;
	unsigned int cached_raw_freq;
	int last_gaming_state;
	u64 last_gaming_active_ns;

	struct irq_work irq_work;
	struct kthread_work work;
	struct kthread_worker worker;
	struct task_struct *thread;
	struct mutex work_lock;
	bool work_in_progress;
	bool limits_changed;
	bool need_freq_update;
};

struct gardenia_cpu {
	struct update_util_data update_util;
	struct gardenia_policy *gd_policy;
	unsigned int cpu;
	unsigned long smoothed_util;	/* directional EMA state */

	/* Decaying IOWait boost (Reflex/schedutil pattern) */
	bool iowait_boost_pending;
	unsigned int iowait_boost;
	u64 last_update;

#ifdef CONFIG_NO_HZ_COMMON
	unsigned long saved_idle_calls;
#endif
};

static DEFINE_PER_CPU(struct gardenia_cpu, gardenia_cpu_list);
static atomic_t gardenia_input_refcount = ATOMIC_INIT(0);

/* ---- Gaming state (raw + exit-hysteresis) ------------------------------ */

static inline bool gardenia_raw_gaming(void)
{
	return READ_ONCE(sched_gaming_active) != 0;
}

static bool gardenia_gaming(struct gardenia_policy *gd_policy)
{
	struct gardenia_tunables *t = gd_policy->tunables;
	bool raw = gardenia_raw_gaming();
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

static void gardenia_refresh_profile(struct gardenia_policy *gd_policy)
{
	struct gardenia_tunables *t = gd_policy->tunables;
	bool gaming = gardenia_gaming(gd_policy);

	if (gd_policy->last_gaming_state == gaming)
		return;

	if (gaming) {
		gd_policy->up_rate_delay_ns =
			(s64)t->up_rate_limit_game_us * NSEC_PER_USEC;
		gd_policy->down_rate_delay_ns =
			(s64)t->down_rate_limit_game_us * NSEC_PER_USEC;
	} else {
		gd_policy->up_rate_delay_ns =
			(s64)t->up_rate_limit_normal_us * NSEC_PER_USEC;
		gd_policy->down_rate_delay_ns =
			(s64)t->down_rate_limit_normal_us * NSEC_PER_USEC;
	}

	gd_policy->last_gaming_state = gaming;
}

static bool gardenia_rate_limited(struct gardenia_policy *gd_policy,
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

/* ---- Thermal pressure discount ---------------------------------------- */

static unsigned long gardenia_thermal_discount(struct gardenia_policy *gd_policy,
						 int cpu, unsigned long max)
{
	unsigned long pressure;

	if (!gd_policy->tunables->thermal_aware)
		return max;

	pressure = arch_scale_thermal_pressure(cpu);
	if (!pressure)
		return max;

	return max - mult_frac(pressure, GARDENIA_THERMAL_DISCOUNT_MAX_PCT, 100);
}

/* ---- Frequency selection ----------------------------------------------- */

static unsigned int gardenia_apply_step_limit(struct gardenia_policy *gd_policy,
					       unsigned int cur,
					       unsigned int target,
					       bool gaming)
{
	unsigned int max_freq = gd_policy->policy->cpuinfo.max_freq;
	unsigned int step_cap;

	if (gaming || gardenia_touch_boosted() || target <= cur)
		return target;

	step_cap = cur + mult_frac(max_freq - cur,
				    gd_policy->tunables->max_step_pct, 100);
	return min(target, step_cap);
}

static unsigned int gardenia_hispeed_floor(struct gardenia_policy *gd_policy,
					    unsigned long util, unsigned long max,
					    bool gaming)
{
	struct gardenia_tunables *t = gd_policy->tunables;
	unsigned int max_freq = gd_policy->policy->cpuinfo.max_freq;

	if (!gaming)
		return 0;

	if (util * 100 < max * t->hispeed_load)
		return 0;

	return mult_frac(max_freq, t->hispeed_freq_pct, 100);
}

static unsigned int gardenia_screen_off_cap(struct gardenia_policy *gd_policy)
{
	struct gardenia_tunables *t = gd_policy->tunables;
	unsigned int max_freq = gd_policy->policy->cpuinfo.max_freq;

	if (t->screen_on)
		return 0;

	return mult_frac(max_freq, t->screen_off_cap_pct, 100);
}

static unsigned int gardenia_next_freq(struct gardenia_policy *gd_policy,
					int cpu, unsigned long util,
					unsigned long max)
{
	struct cpufreq_policy *policy = gd_policy->policy;
	struct gardenia_tunables *t = gd_policy->tunables;
	unsigned int freq = arch_scale_freq_invariant() ?
			     policy->cpuinfo.max_freq : policy->cur;
	unsigned long adj_max = gardenia_thermal_discount(gd_policy, cpu, max);
	unsigned long next_util = util;
	unsigned int floor_freq, screen_cap, target;
	bool gaming = gardenia_gaming(gd_policy);
	bool boosted = gardenia_touch_boosted();

	if (!gaming && !boosted) {
		unsigned int bias = gd_policy->is_little ?
			max(t->util_bias_pct, GARDENIA_UTIL_BIAS_LITTLE_PCT_DEFAULT) :
			t->util_bias_pct;
		next_util = (util * bias) / 100;
	}

	freq = map_util_freq(next_util, freq, adj_max);

	if (freq != gd_policy->cached_raw_freq || gd_policy->need_freq_update) {
		gd_policy->cached_raw_freq = freq;
		freq = cpufreq_driver_resolve_freq(policy, freq);
	} else {
		freq = gd_policy->next_freq;
	}

	target = gardenia_apply_step_limit(gd_policy, policy->cur, freq, gaming);

	floor_freq = gardenia_hispeed_floor(gd_policy, util, max, gaming);
	if (floor_freq)
		target = max(target, floor_freq);

	/* screen-off cap always wins - it's a hard ceiling, not a floor */
	screen_cap = gardenia_screen_off_cap(gd_policy);
	if (screen_cap)
		target = min(target, screen_cap);

	return target;
}

/* ---- Deferred frequency change (kthread slow path) ---------------------- */

static void gardenia_work(struct kthread_work *work)
{
	struct gardenia_policy *gd_policy =
		container_of(work, struct gardenia_policy, work);
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

static void gardenia_irq_work(struct irq_work *irq_work)
{
	struct gardenia_policy *gd_policy =
		container_of(irq_work, struct gardenia_policy, irq_work);

	kthread_queue_work(&gd_policy->worker, &gd_policy->work);
}

static void gardenia_queue_update(struct gardenia_policy *gd_policy)
{
	if (gd_policy->work_in_progress)
		return;

	gd_policy->work_in_progress = true;
	irq_work_queue(&gd_policy->irq_work);
}

/* ---- Directional EMA util smoothing ------------------------------------ */

static unsigned long gardenia_ema_smooth(struct gardenia_cpu *gd_cpu,
					  unsigned long raw_util)
{
	struct gardenia_tunables *t = gd_cpu->gd_policy->tunables;
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

/* ---- Hold frequency across brief idle blips (schedutil/Reflex pattern) - */

#ifdef CONFIG_NO_HZ_COMMON
static bool gardenia_hold_freq(struct gardenia_cpu *gd_cpu)
{
	unsigned long idle_calls;
	bool ret;

	idle_calls = tick_nohz_get_idle_calls_cpu(gd_cpu->cpu);
	ret = idle_calls == gd_cpu->saved_idle_calls;
	gd_cpu->saved_idle_calls = idle_calls;
	return ret;
}
#else
static inline bool gardenia_hold_freq(struct gardenia_cpu *gd_cpu) { return false; }
#endif

/* ---- Decaying IOWait boost ---------------------------------------------- */

static bool gardenia_iowait_reset(struct gardenia_cpu *gd_cpu, u64 time,
				   bool set_iowait_boost)
{
	s64 delta_ns = time - gd_cpu->last_update;

	if (delta_ns <= TICK_NSEC)
		return false;

	gd_cpu->iowait_boost = set_iowait_boost ? IOWAIT_BOOST_MIN : 0;
	gd_cpu->iowait_boost_pending = set_iowait_boost;
	return true;
}

static void gardenia_iowait_boost(struct gardenia_cpu *gd_cpu, u64 time,
				   unsigned int flags)
{
	bool set_iowait_boost = flags & SCHED_CPUFREQ_IOWAIT;

	if (gd_cpu->iowait_boost &&
	    gardenia_iowait_reset(gd_cpu, time, set_iowait_boost))
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

static unsigned long gardenia_iowait_apply(struct gardenia_cpu *gd_cpu,
					    u64 time, unsigned long max)
{
	if (!gd_cpu->iowait_boost)
		return 0;

	if (gardenia_iowait_reset(gd_cpu, time, false))
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

static void gardenia_update_single_cpu(struct gardenia_cpu *gd_cpu,
					 u64 time, unsigned int flags)
{
	struct gardenia_policy *gd_policy = gd_cpu->gd_policy;
	unsigned long util, max, boost;
	unsigned int next_f;

	gardenia_refresh_profile(gd_policy);

	max = arch_scale_cpu_capacity(gd_cpu->cpu);

	gardenia_iowait_boost(gd_cpu, time, flags);
	boost = gardenia_iowait_apply(gd_cpu, time, max);
	gd_cpu->last_update = time;

	util = sched_cpu_util(gd_cpu->cpu, max);
	util = max(util, boost);
	util = gardenia_ema_smooth(gd_cpu, util);

	next_f = gardenia_next_freq(gd_policy, gd_cpu->cpu, util, max);

	/* Don't let a brief idle blip drop freq below what's already
	 * running - avoids the measure-low / correct-next-tick sawtooth
	 * that reads as micro-stutter.
	 */
	if (gardenia_hold_freq(gd_cpu) && next_f < gd_policy->next_freq &&
	    !gd_policy->need_freq_update)
		next_f = gd_policy->next_freq;

	if (gardenia_rate_limited(gd_policy, time, next_f))
		return;

	gd_policy->next_freq = next_f;
	gd_policy->last_freq_update_time = time;

	if (gd_policy->policy->fast_switch_enabled)
		cpufreq_driver_fast_switch(gd_policy->policy, next_f);
	else
		gardenia_queue_update(gd_policy);
}

static void gardenia_update_util(struct update_util_data *hook, u64 time,
				  unsigned int flags)
{
	struct gardenia_cpu *gd_cpu = container_of(hook, struct gardenia_cpu,
						    update_util);

	raw_spin_lock(&gd_cpu->gd_policy->update_lock);
	gardenia_update_single_cpu(gd_cpu, time, flags);
	raw_spin_unlock(&gd_cpu->gd_policy->update_lock);
}

/* ---- sysfs tunables ------------------------------------------------------ */

#define GARDENIA_ATTR_RW(_name)						\
static ssize_t _name##_show(struct kobject *kobj,				\
			     struct kobj_attribute *attr, char *buf)		\
{										\
	struct gardenia_tunables *t = container_of(kobj,			\
					struct gardenia_tunables, kobj);	\
	return sprintf(buf, "%u\n", t->_name);					\
}										\
static ssize_t _name##_store(struct kobject *kobj,				\
			      struct kobj_attribute *attr,			\
			      const char *buf, size_t count)			\
{										\
	struct gardenia_tunables *t = container_of(kobj,			\
					struct gardenia_tunables, kobj);	\
	unsigned int val;							\
	if (kstrtouint(buf, 10, &val))						\
		return -EINVAL;							\
	t->_name = val;								\
	return count;								\
}										\
static struct kobj_attribute _name##_attr =					\
	__ATTR(_name, 0644, _name##_show, _name##_store)

#define GARDENIA_ATTR_RW_BOOL(_name)						\
static ssize_t _name##_show(struct kobject *kobj,				\
			     struct kobj_attribute *attr, char *buf)		\
{										\
	struct gardenia_tunables *t = container_of(kobj,			\
					struct gardenia_tunables, kobj);	\
	return sprintf(buf, "%u\n", t->_name);					\
}										\
static ssize_t _name##_store(struct kobject *kobj,				\
			      struct kobj_attribute *attr,			\
			      const char *buf, size_t count)			\
{										\
	struct gardenia_tunables *t = container_of(kobj,			\
					struct gardenia_tunables, kobj);	\
	unsigned int val;							\
	if (kstrtouint(buf, 10, &val))						\
		return -EINVAL;							\
	t->_name = !!val;							\
	return count;								\
}										\
static struct kobj_attribute _name##_attr =					\
	__ATTR(_name, 0644, _name##_show, _name##_store)

GARDENIA_ATTR_RW(up_rate_limit_normal_us);
GARDENIA_ATTR_RW(down_rate_limit_normal_us);
GARDENIA_ATTR_RW(up_rate_limit_game_us);
GARDENIA_ATTR_RW(down_rate_limit_game_us);
GARDENIA_ATTR_RW(util_bias_pct);
GARDENIA_ATTR_RW(max_step_pct);
GARDENIA_ATTR_RW(hispeed_load);
GARDENIA_ATTR_RW(hispeed_freq_pct);
GARDENIA_ATTR_RW(ema_up_shift);
GARDENIA_ATTR_RW(ema_down_shift);
GARDENIA_ATTR_RW(gaming_exit_delay_ms);
GARDENIA_ATTR_RW(screen_off_cap_pct);
GARDENIA_ATTR_RW_BOOL(thermal_aware);
GARDENIA_ATTR_RW_BOOL(screen_on);

/*
 * touch_boost_ms is stored per-policy (so it shows up under every
 * cluster's gardenia/ dir) but the touch handler itself is global -
 * writing it from any policy updates the one shared value that
 * actually gates gardenia_touch_boosted().
 */
static ssize_t touch_boost_ms_show(struct kobject *kobj,
				    struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", READ_ONCE(gardenia_touch_boost_ms));
}

static ssize_t touch_boost_ms_store(struct kobject *kobj,
				     struct kobj_attribute *attr,
				     const char *buf, size_t count)
{
	struct gardenia_tunables *t = container_of(kobj,
					struct gardenia_tunables, kobj);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	t->touch_boost_ms = val;
	WRITE_ONCE(gardenia_touch_boost_ms, val);
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

static struct attribute *gardenia_attrs[] = {
	&up_rate_limit_normal_us_attr.attr,
	&down_rate_limit_normal_us_attr.attr,
	&up_rate_limit_game_us_attr.attr,
	&down_rate_limit_game_us_attr.attr,
	&util_bias_pct_attr.attr,
	&max_step_pct_attr.attr,
	&hispeed_load_attr.attr,
	&hispeed_freq_pct_attr.attr,
	&ema_up_shift_attr.attr,
	&ema_down_shift_attr.attr,
	&gaming_exit_delay_ms_attr.attr,
	&touch_boost_ms_attr.attr,
	&screen_off_cap_pct_attr.attr,
	&thermal_aware_attr.attr,
	&screen_on_attr.attr,
	&gaming_mode_attr.attr,
	NULL
};
ATTRIBUTE_GROUPS(gardenia);

static void gardenia_tunables_release(struct kobject *kobj)
{
	struct gardenia_tunables *t = container_of(kobj,
					struct gardenia_tunables, kobj);
	kfree(t);
}

static struct kobj_type gardenia_tunables_ktype = {
	.default_groups	= gardenia_groups,
	.sysfs_ops	= &kobj_sysfs_ops,
	.release	= gardenia_tunables_release,
};

static struct gardenia_tunables *gardenia_tunables_alloc(struct cpufreq_policy *policy)
{
	struct gardenia_tunables *t;
	int ret;

	t = kzalloc(sizeof(*t), GFP_KERNEL);
	if (!t)
		return NULL;

	t->up_rate_limit_normal_us	= GARDENIA_UP_RATE_LIMIT_NORMAL_US;
	t->down_rate_limit_normal_us	= GARDENIA_DOWN_RATE_LIMIT_NORMAL_US;
	t->up_rate_limit_game_us	= GARDENIA_UP_RATE_LIMIT_GAME_US;
	t->down_rate_limit_game_us	= GARDENIA_DOWN_RATE_LIMIT_GAME_US;
	t->util_bias_pct		= GARDENIA_UTIL_BIAS_PCT_DEFAULT;
	t->max_step_pct			= GARDENIA_MAX_STEP_PCT_DEFAULT;
	t->hispeed_load			= GARDENIA_HISPEED_LOAD_DEFAULT;
	t->hispeed_freq_pct		= GARDENIA_HISPEED_FREQ_PCT_DEFAULT;
	t->ema_up_shift			= GARDENIA_EMA_UP_SHIFT_DEFAULT;
	t->ema_down_shift		= GARDENIA_EMA_DOWN_SHIFT_DEFAULT;
	t->gaming_exit_delay_ms		= GARDENIA_GAMING_EXIT_DELAY_MS_DEFAULT;
	t->touch_boost_ms		= GARDENIA_TOUCH_BOOST_MS_DEFAULT;
	t->screen_off_cap_pct		= GARDENIA_SCREEN_OFF_CAP_PCT_DEFAULT;
	t->thermal_aware		= true;
	t->screen_on			= true;

	ret = kobject_init_and_add(&t->kobj, &gardenia_tunables_ktype,
				    get_governor_parent_kobj(policy),
				    "gardenia");
	if (ret) {
		kobject_put(&t->kobj);
		return NULL;
	}

	return t;
}

/* ---- Governor lifecycle -------------------------------------------------- */

static int gardenia_init(struct cpufreq_policy *policy)
{
	struct gardenia_policy *gd_policy;
	struct gardenia_tunables *tunables;
	int cpu, ret;

	cpufreq_enable_fast_switch(policy);

	gd_policy = kzalloc(sizeof(*gd_policy), GFP_KERNEL);
	if (!gd_policy) {
		ret = -ENOMEM;
		goto disable_fast_switch;
	}

	tunables = gardenia_tunables_alloc(policy);
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
	gd_policy->next_freq = policy->cur;
	gardenia_refresh_profile(gd_policy);

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
		init_irq_work(&gd_policy->irq_work, gardenia_irq_work);
		kthread_init_worker(&gd_policy->worker);
		kthread_init_work(&gd_policy->work, gardenia_work);

		gd_policy->thread = kthread_create(kthread_worker_fn,
						    &gd_policy->worker,
						    "gardenia:%d", policy->cpu);
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
		struct gardenia_cpu *gd_cpu = &per_cpu(gardenia_cpu_list, cpu);

		memset(gd_cpu, 0, sizeof(*gd_cpu));
		gd_cpu->cpu = cpu;
		gd_cpu->gd_policy = gd_policy;
		cpufreq_add_update_util_hook(cpu, &gd_cpu->update_util,
					      gardenia_update_util);
	}

	if (atomic_inc_return(&gardenia_input_refcount) == 1) {
		ret = input_register_handler(&gardenia_input_handler);
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
	pr_err("gardenia: initialization failed (error %d)\n", ret);
	return ret;
}

static void gardenia_exit(struct cpufreq_policy *policy)
{
	struct gardenia_policy *gd_policy = policy->governor_data;
	int cpu;

	for_each_cpu(cpu, policy->cpus)
		cpufreq_remove_update_util_hook(cpu);

	synchronize_rcu();

	if (atomic_dec_return(&gardenia_input_refcount) == 0)
		input_unregister_handler(&gardenia_input_handler);

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

static int gardenia_start(struct cpufreq_policy *policy)
{
	struct gardenia_policy *gd_policy = policy->governor_data;

	gd_policy->last_freq_update_time = 0;
	gd_policy->next_freq = policy->cur;
	gd_policy->need_freq_update = true;
	return 0;
}

static void gardenia_stop(struct cpufreq_policy *policy)
{
	struct gardenia_policy *gd_policy = policy->governor_data;

	if (!policy->fast_switch_enabled) {
		irq_work_sync(&gd_policy->irq_work);
		kthread_cancel_work_sync(&gd_policy->work);
	}
}

static void gardenia_limits(struct cpufreq_policy *policy)
{
	struct gardenia_policy *gd_policy = policy->governor_data;
	unsigned long flags;

	raw_spin_lock_irqsave(&gd_policy->update_lock, flags);
	gd_policy->limits_changed = true;
	gd_policy->need_freq_update = true;
	raw_spin_unlock_irqrestore(&gd_policy->update_lock, flags);
}

static struct cpufreq_governor gardenia_gov = {
	.name		= "gardenia",
	.owner		= THIS_MODULE,
	.flags		= CPUFREQ_GOV_DYNAMIC_SWITCHING,
	.init		= gardenia_init,
	.exit		= gardenia_exit,
	.start		= gardenia_start,
	.stop		= gardenia_stop,
	.limits		= gardenia_limits,
};

cpufreq_governor_init(gardenia_gov);
cpufreq_governor_exit(gardenia_gov);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("cpufreq governor: gardenia (power-save + gaming switch, touch-boosted, tunable)");
