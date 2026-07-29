#define pr_fmt(fmt) "gardenia: " fmt

#include <linux/cpufreq.h>
#include <linux/kthread.h>
#include <linux/percpu-defs.h>
#include <linux/slab.h>
#include <linux/irq_work.h>
#include <linux/input.h>
#include <linux/sched/cpufreq.h>
#include <linux/thermal_pressure.h>
#include <trace/events/power.h>

extern int sched_gaming_active;

/* ---- Defaults (overridable at runtime via sysfs) --------------------- */

#define GARDENIA_UP_RATE_LIMIT_NORMAL_US	(20000)
#define GARDENIA_DOWN_RATE_LIMIT_NORMAL_US	(50000)
#define GARDENIA_UP_RATE_LIMIT_GAME_US		(2000)
#define GARDENIA_DOWN_RATE_LIMIT_GAME_US	(8000)

#define GARDENIA_UTIL_BIAS_PCT_DEFAULT		(88)	/* big cluster, normal */
#define GARDENIA_UTIL_BIAS_LITTLE_PCT_DEFAULT	(96)	/* little cluster, normal */
#define GARDENIA_MAX_STEP_PCT_DEFAULT		(60)	/* normal profile */

#define GARDENIA_HISPEED_LOAD_DEFAULT		(85)	/* pct, gaming only */
#define GARDENIA_HISPEED_FREQ_PCT_DEFAULT	(70)	/* pct of max_freq */

#define GARDENIA_THERMAL_DISCOUNT_MAX_PCT	(25)	/* extra cut at full throttle */
#define GARDENIA_GAMING_EXIT_DELAY_MS_DEFAULT	(3000)	/* gaming exit hysteresis */

#define GARDENIA_TOUCH_BOOST_MS_DEFAULT	(150)	/* daily-use smoothness */
#define GARDENIA_SCREEN_OFF_CAP_PCT_DEFAULT	(35)	/* pct of max_freq */

/* Adaptive thermal-feedback bias: every ADAPT_PERIOD, if thermal
 * pressure has been consistently present, nudge the effective bias
 * further down (more discount, more headroom below throttle); if
 * pressure has been consistently absent, nudge it back up toward the
 * tunable baseline (more responsive). Bounded so it can never remove
 * more than ADAPT_MAX_DELTA points of bias, and never override the
 * gaming/touch-boost paths, which always bypass bias entirely.
 */
#define GARDENIA_ADAPT_PERIOD_NS		(2000000000ULL) /* 2s */
#define GARDENIA_ADAPT_STEP			(2)   /* pct per period */
#define GARDENIA_ADAPT_MAX_DELTA		(15)  /* pct, floor of discount */
#define GARDENIA_ADAPT_PRESSURE_THRESHOLD	(10)  /* pct of max capacity */

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

	bool thermal_aware;
	bool screen_on;
	bool adaptive_enabled;
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

	/* self-adjusting thermal-feedback bias, see gardenia_adapt() */
	int adaptive_bias_delta;
	u64 last_adapt_ns;

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

/*
 * Adaptive feedback: runs at most once per GARDENIA_ADAPT_PERIOD_NS
 * per policy. Reads current thermal pressure (already sampled for
 * gardenia_thermal_discount - this is a second, slower-timescale
 * read, cheap and fine to duplicate) and walks adaptive_bias_delta
 * one step toward more or less discount. This is a bounded control
 * loop, not a predictive/learned model - it only ever reacts to the
 * thermal state that already exists right now.
 */
static void gardenia_adapt(struct gardenia_policy *gd_policy, int cpu,
			    unsigned long max)
{
	struct gardenia_tunables *t = gd_policy->tunables;
	unsigned long pressure;
	u64 now;

	if (!t->adaptive_enabled)
		return;

	now = local_clock();
	if (now - gd_policy->last_adapt_ns < GARDENIA_ADAPT_PERIOD_NS)
		return;
	gd_policy->last_adapt_ns = now;

	pressure = t->thermal_aware ? arch_scale_thermal_pressure(cpu) : 0;

	if (pressure * 100 > max * GARDENIA_ADAPT_PRESSURE_THRESHOLD) {
		/* under thermal pressure: increase discount */
		gd_policy->adaptive_bias_delta =
			max_t(int, -GARDENIA_ADAPT_MAX_DELTA,
			      gd_policy->adaptive_bias_delta - GARDENIA_ADAPT_STEP);
	} else {
		/* no pressure: relax back toward the tunable baseline */
		gd_policy->adaptive_bias_delta =
			min_t(int, 0,
			      gd_policy->adaptive_bias_delta + GARDENIA_ADAPT_STEP);
	}
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

	gardenia_adapt(gd_policy, cpu, max);

	if (!gaming && !boosted) {
		unsigned int base_bias = gd_policy->is_little ?
			max(t->util_bias_pct, GARDENIA_UTIL_BIAS_LITTLE_PCT_DEFAULT) :
			t->util_bias_pct;
		int effective_bias = clamp_t(int,
					      (int)base_bias + gd_policy->adaptive_bias_delta,
					      50, 100);
		next_util = (util * (unsigned int)effective_bias) / 100;
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

/* ---- Fast path (per-cpu utilization callback) --------------------------- */

static void gardenia_update_single_cpu(struct gardenia_cpu *gd_cpu,
					 u64 time, unsigned int flags)
{
	struct gardenia_policy *gd_policy = gd_cpu->gd_policy;
	unsigned long util, max;
	unsigned int next_f;

	gardenia_refresh_profile(gd_policy);

	max = arch_scale_cpu_capacity(gd_cpu->cpu);
	util = cpu_util_cfs(gd_cpu->cpu);
	util = effective_cpu_util(gd_cpu->cpu, util, &max, NULL, NULL);
	util = max(util, boosted_cpu_util(gd_cpu->cpu));

	if (flags & SCHED_CPUFREQ_IOWAIT)
		util = max(util, mult_frac(max, 60, 100));

	next_f = gardenia_next_freq(gd_policy, gd_cpu->cpu, util, max);

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
GARDENIA_ATTR_RW(gaming_exit_delay_ms);
GARDENIA_ATTR_RW(screen_off_cap_pct);
GARDENIA_ATTR_RW_BOOL(thermal_aware);
GARDENIA_ATTR_RW_BOOL(screen_on);
GARDENIA_ATTR_RW_BOOL(adaptive_enabled);

/* Read-only: shows what the adaptive loop is currently doing, in
 * percentage points subtracted from util_bias_pct. Useful to confirm
 * it's actually reacting instead of just trusting it blindly.
 */
static ssize_t adaptive_bias_delta_show(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf)
{
	struct gardenia_tunables *t = container_of(kobj,
					struct gardenia_tunables, kobj);
	return sprintf(buf, "%d\n", READ_ONCE(t->gd_policy->adaptive_bias_delta));
}
static struct kobj_attribute adaptive_bias_delta_attr =
	__ATTR(adaptive_bias_delta, 0444, adaptive_bias_delta_show, NULL);

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
	&gaming_exit_delay_ms_attr.attr,
	&touch_boost_ms_attr.attr,
	&screen_off_cap_pct_attr.attr,
	&thermal_aware_attr.attr,
	&screen_on_attr.attr,
	&adaptive_enabled_attr.attr,
	&adaptive_bias_delta_attr.attr,
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
	t->gaming_exit_delay_ms		= GARDENIA_GAMING_EXIT_DELAY_MS_DEFAULT;
	t->touch_boost_ms		= GARDENIA_TOUCH_BOOST_MS_DEFAULT;
	t->screen_off_cap_pct		= GARDENIA_SCREEN_OFF_CAP_PCT_DEFAULT;
	t->thermal_aware		= true;
	t->screen_on			= true;
	t->adaptive_enabled		= true;

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
	struct sched_param param = { .sched_priority = MAX_RT_PRIO / 2 };
	int cpu, ret;

	gd_policy = kzalloc(sizeof(*gd_policy), GFP_KERNEL);
	if (!gd_policy)
		return -ENOMEM;

	tunables = gardenia_tunables_alloc(policy);
	if (!tunables) {
		kfree(gd_policy);
		return -ENOMEM;
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
	mutex_init(&gd_policy->work_lock);
	gd_policy->last_gaming_state = -1;
	gd_policy->next_freq = policy->cur;
	gardenia_refresh_profile(gd_policy);

	init_irq_work(&gd_policy->irq_work, gardenia_irq_work);
	kthread_init_worker(&gd_policy->worker);
	kthread_init_work(&gd_policy->work, gardenia_work);

	gd_policy->thread = kthread_create(kthread_worker_fn, &gd_policy->worker,
					    "gardenia:%d", policy->cpu);
	if (IS_ERR(gd_policy->thread)) {
		ret = PTR_ERR(gd_policy->thread);
		kobject_put(&tunables->kobj);
		kfree(gd_policy);
		return ret;
	}

	ret = sched_setscheduler_nocheck(gd_policy->thread, SCHED_FIFO, &param);
	if (ret)
		pr_warn("failed to set SCHED_FIFO for gardenia kthread\n");

	wake_up_process(gd_policy->thread);

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

	pr_info("governor attached to policy cpu%d (%s cluster)\n",
		policy->cpu, gd_policy->is_little ? "little" : "big");
	return 0;
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

	kthread_flush_worker(&gd_policy->worker);
	kthread_stop(gd_policy->thread);

	kobject_put(&gd_policy->tunables->kobj);
	kfree(gd_policy);
	policy->governor_data = NULL;
}

ick, flags);
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
