// tenebrion.c
// SPDX-License-Identifier: GPL-3.0-only
// Tenebrion — Screen state based CPU frequency throttler + cpuset limiter
// Author: Kanagawa Yamada

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/fs.h>
#include <linux/pm_qos.h>
#include <linux/slab.h>
#include <linux/cpumask.h>

#define POLL_INTERVAL_MS    3000
#define DPMS_PATH           "/sys/class/drm/card0-DSI-1/dpms"
#define BACKLIGHT_PATH      "/sys/class/leds/lcd-backlight/brightness"

/*
 * cpuset paths that get restricted when screen is off.
 * top-app / foreground are intentionally left alone — the system
 * scheduler already won't run heavy foreground work while the
 * screen is off, and touching those sets causes jank on wake.
 */
#define CPUSET_SYSBG_PATH   "/dev/cpuset/system-background/cpus"

MODULE_IMPORT_NS(VFS_internal_I_am_really_a_filesystem_and_am_NOT_a_driver);

enum tenebrion_path {
    PATH_NONE        = 0,   /* not yet found — keep retrying            */
    PATH_DPMS        = 1,   /* /sys/class/drm/.../dpms                  */
    PATH_BACKLIGHT   = 2,   /* /sys/class/leds/.../brightness           */
    PATH_UNSUPPORTED = 3,   /* tried and failed — stop searching        */
};

static enum tenebrion_path active_path = PATH_NONE;
static bool is_screen_off = false;
static DEFINE_MUTEX(tenebrion_lock);
static struct task_struct *watcher_thread;

/* QoS requests per policy CPU */
static struct freq_qos_request tenebrion_min_req[NR_CPUS];
static struct freq_qos_request tenebrion_max_req[NR_CPUS];
static bool qos_initialized[NR_CPUS];

/* ------------------------------------------------------------------ */
/* File helpers                                                         */
/* ------------------------------------------------------------------ */

static int tenebrion_read_file(const char *path, char *buf, size_t size)
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
        /* strip trailing newline so comparisons are clean */
        if (buf[ret - 1] == '\n')
            buf[ret - 1] = '\0';
        else
            buf[ret] = '\0';
    } else {
        ret = -1;
    }

    return ret;
}

static int tenebrion_write_file(const char *path, const char *buf)
{
    struct file *f;
    loff_t pos = 0;
    int ret;

    f = filp_open(path, O_WRONLY, 0);
    if (IS_ERR(f)) {
        pr_warn("tenebrion: cannot open %s for write\n", path);
        return -1;
    }

    ret = kernel_write(f, buf, strlen(buf), &pos);
    filp_close(f, NULL);

    return ret > 0 ? 0 : -1;
}

/* ------------------------------------------------------------------ */
/* Path auto-detection                                                  */
/* Tries both known paths once; if neither works, marks UNSUPPORTED.  */
/* ------------------------------------------------------------------ */

static enum tenebrion_path tenebrion_detect_path(void)
{
    char buf[64];

    if (tenebrion_read_file(DPMS_PATH, buf, sizeof(buf)) > 0) {
        pr_info("tenebrion: detected path → %s\n", DPMS_PATH);
        return PATH_DPMS;
    }

    if (tenebrion_read_file(BACKLIGHT_PATH, buf, sizeof(buf)) > 0) {
        pr_info("tenebrion: detected path → %s\n", BACKLIGHT_PATH);
        return PATH_BACKLIGHT;
    }

    pr_err("tenebrion: no supported screen state path found — disabling\n");
    return PATH_UNSUPPORTED;
}

/* ------------------------------------------------------------------ */
/* Screen state detection                                               */
/* Returns: 1 = on, 0 = off, -1 = unknown                             */
/* ------------------------------------------------------------------ */

static int tenebrion_get_screen_state(void)
{
    char buf[64];
    int len;

    switch (active_path) {
    case PATH_DPMS:
        len = tenebrion_read_file(DPMS_PATH, buf, sizeof(buf));
        if (len <= 0)
            return -1;
        if (strstr(buf, "On"))
            return 1;
        if (strstr(buf, "Off"))
            return 0;
        return -1;

    case PATH_BACKLIGHT:
        len = tenebrion_read_file(BACKLIGHT_PATH, buf, sizeof(buf));
        if (len <= 0)
            return -1;
        if (simple_strtol(buf, NULL, 10) > 0)
            return 1;
        return 0;

    default:
        return -1;
    }
}

/* ------------------------------------------------------------------ */
/* cpuset helpers                                                       */
/* ------------------------------------------------------------------ */

/*
 * Build a cpumask string that covers only CPU 0 — the safest single
 * core to leave for background work regardless of topology.
 * On screen-off we pin background and system-background cpusets to
 * CPU0 only; everything else stays as-is so foreground/top-app are
 * not affected.
 */
#define CPUSET_SCREEN_OFF   "0\n"

static char saved_sysbg_cpus[32] = "";

static void tenebrion_cpuset_restrict(void)
{
    /* Save current mask before overriding */
    tenebrion_read_file(CPUSET_SYSBG_PATH, saved_sysbg_cpus, sizeof(saved_sysbg_cpus));

    if (tenebrion_write_file(CPUSET_SYSBG_PATH, CPUSET_SCREEN_OFF) == 0)
        pr_info("tenebrion: system-background cpuset → 0\n");
}

static void tenebrion_cpuset_restore(void)
{
    char sysbg_buf[32];
    char fallback_mask[32];
    int total_cores = num_possible_cpus();

    /* Forge the dynamic mask just in case the read failed (e.g. "0-7", "0-3") */
    snprintf(fallback_mask, sizeof(fallback_mask), "0-%d", total_cores - 1);

    /* tenebrion_read_file stripped the newline, so we must add it back */
    snprintf(sysbg_buf, sizeof(sysbg_buf), "%s\n", 
             saved_sysbg_cpus[0] ? saved_sysbg_cpus : fallback_mask);

    if (tenebrion_write_file(CPUSET_SYSBG_PATH, sysbg_buf) == 0)
        pr_info("tenebrion: system-background cpuset restored → %s", sysbg_buf);
}

/* ------------------------------------------------------------------ */
/* QoS init — add requests for all online policy CPUs                  */
/* ------------------------------------------------------------------ */

static void tenebrion_qos_init(void)
{
    unsigned int cpu;
    struct cpufreq_policy *policy;

    for_each_online_cpu(cpu) {
        policy = cpufreq_cpu_get(cpu);
        if (!policy)
            continue;

        if (policy->cpu == cpu && !qos_initialized[cpu]) {
            freq_qos_add_request(&policy->constraints,
                                 &tenebrion_min_req[cpu],
                                 FREQ_QOS_MIN,
                                 policy->cpuinfo.min_freq);

            freq_qos_add_request(&policy->constraints,
                                 &tenebrion_max_req[cpu],
                                 FREQ_QOS_MAX,
                                 policy->cpuinfo.max_freq);

            qos_initialized[cpu] = true;

            pr_info("tenebrion: QoS initialized for policy%u "
                    "(min=%u max=%u KHz)\n",
                    cpu,
                    policy->cpuinfo.min_freq,
                    policy->cpuinfo.max_freq);
        }

        cpufreq_cpu_put(policy);
    }
}

/* ------------------------------------------------------------------ */
/* CPUFreq — drop to min via QoS                                       */
/* ------------------------------------------------------------------ */

static void tenebrion_set_min_freq(void)
{
    unsigned int cpu;
    struct cpufreq_policy *policy;

    for_each_online_cpu(cpu) {
        policy = cpufreq_cpu_get(cpu);
        if (!policy)
            continue;

        if (policy->cpu == cpu && qos_initialized[cpu]) {
            /*
             * Order matters: bring min_req DOWN first so the QoS
             * arbiter never sees min > max during the transition,
             * then clamp max_req down to min_freq.
             */
            freq_qos_update_request(&tenebrion_min_req[cpu],
                                    policy->cpuinfo.min_freq);
            freq_qos_update_request(&tenebrion_max_req[cpu],
                                    policy->cpuinfo.min_freq);

            pr_info("tenebrion: policy%u → %u KHz (screen off)\n",
                    cpu, policy->cpuinfo.min_freq);
        }

        cpufreq_cpu_put(policy);
    }
}

/* ------------------------------------------------------------------ */
/* CPUFreq — restore via QoS                                           */
/* ------------------------------------------------------------------ */

static void tenebrion_restore_freq(void)
{
    unsigned int cpu;
    struct cpufreq_policy *policy;

    for_each_online_cpu(cpu) {
        policy = cpufreq_cpu_get(cpu);
        if (!policy)
            continue;

        if (policy->cpu == cpu && qos_initialized[cpu]) {
            /*
             * Order matters: raise the max_req ceiling first, then
             * restore min_req floor.  Reversing this would momentarily
             * set min > max on the QoS arbiter.
             */
            freq_qos_update_request(&tenebrion_max_req[cpu],
                                    policy->cpuinfo.max_freq);
            freq_qos_update_request(&tenebrion_min_req[cpu],
                                    policy->cpuinfo.min_freq);

            pr_info("tenebrion: policy%u restored min=%u max=%u KHz\n",
                    cpu,
                    policy->cpuinfo.min_freq,
                    policy->cpuinfo.max_freq);
        }

        cpufreq_cpu_put(policy);
    }
}

/* ------------------------------------------------------------------ */
/* QoS cleanup                                                          */
/* ------------------------------------------------------------------ */

static void tenebrion_qos_cleanup(void)
{
    unsigned int cpu;

    for_each_possible_cpu(cpu) {
        if (qos_initialized[cpu]) {
            freq_qos_remove_request(&tenebrion_min_req[cpu]);
            freq_qos_remove_request(&tenebrion_max_req[cpu]);
            qos_initialized[cpu] = false;
        }
    }
}

/* ------------------------------------------------------------------ */
/* Screen-off / screen-on actions                                       */
/* Both cpuset restriction and freq throttle happen together.          */
/* ------------------------------------------------------------------ */

static void tenebrion_on_screen_off(void)
{
    tenebrion_set_min_freq();
    tenebrion_cpuset_restrict();
    is_screen_off = true;
    pr_info("tenebrion: screen OFF → CPUs throttled + cpuset restricted "
            "(online CPUs: %u)\n", num_online_cpus());
}

static void tenebrion_on_screen_on(void)
{
    tenebrion_restore_freq();
    tenebrion_cpuset_restore();
    is_screen_off = false;
    pr_info("tenebrion: screen ON → CPUs restored + cpuset restored\n");
}

/* ------------------------------------------------------------------ */
/* Watcher kthread                                                      */
/* ------------------------------------------------------------------ */

static int tenebrion_watcher(void *data)
{
    int current_state = -1;
    int last_state    = -1;

    pr_info("tenebrion: watcher started, polling every %dms\n",
            POLL_INTERVAL_MS);

    /* Wait for Android SELinux policy + KernelSU rules to be applied */
    msleep(40000);

    /*
     * If init already flagged UNSUPPORTED, there is nothing to do.
     * Exit the thread cleanly rather than burning cycles forever.
     */
    if (active_path == PATH_UNSUPPORTED) {
        pr_info("tenebrion: path unsupported — watcher exiting\n");
        return 0;
    }

    /* Initialize QoS requests after the boot delay */
    tenebrion_qos_init();

    while (!kthread_should_stop()) {
        /*
         * PATH_NONE: detection was inconclusive at init (race with
         * driver probe). Try once more. If it fails this time, mark
         * UNSUPPORTED and stop the loop — no point hammering sysfs
         * every 3 s forever.
         */
        if (active_path == PATH_NONE) {
            active_path = tenebrion_detect_path();
            if (active_path == PATH_UNSUPPORTED) {
                pr_info("tenebrion: retry failed — watcher exiting\n");
                break;
            }
        }

        current_state = tenebrion_get_screen_state();

        if (current_state != -1 && current_state != last_state) {
            mutex_lock(&tenebrion_lock);

            if (current_state == 0 && !is_screen_off)
                tenebrion_on_screen_off();
            else if (current_state == 1 && is_screen_off)
                tenebrion_on_screen_on();

            mutex_unlock(&tenebrion_lock);
            last_state = current_state;
        }

        msleep_interruptible(POLL_INTERVAL_MS);
    }

    return 0;
}

/* ------------------------------------------------------------------ */
/* Init / Exit                                                          */
/* ------------------------------------------------------------------ */

static int __init tenebrion_init(void)
{
    memset(qos_initialized, 0, sizeof(qos_initialized));
    active_path = PATH_NONE;

    watcher_thread = kthread_run(tenebrion_watcher, NULL, "tenebrion");
    if (IS_ERR(watcher_thread)) {
        pr_err("tenebrion: failed to start watcher thread: %ld\n",
               PTR_ERR(watcher_thread));
        return PTR_ERR(watcher_thread);
    }

    pr_info("tenebrion: active — path=%d poll=%dms possible_cpus=%u\n",
            active_path, POLL_INTERVAL_MS, num_possible_cpus());
    return 0;
}

static void __exit tenebrion_exit(void)
{
    kthread_stop(watcher_thread);

    if (is_screen_off) {
        mutex_lock(&tenebrion_lock);
        tenebrion_on_screen_on();
        mutex_unlock(&tenebrion_lock);
    }

    tenebrion_qos_cleanup();

    pr_info("tenebrion: unloaded\n");
}

module_init(tenebrion_init);
module_exit(tenebrion_exit);

MODULE_LICENSE("GPL v3");
MODULE_AUTHOR("Kanagawa Yamada");
MODULE_DESCRIPTION("Tenebrion: Screen state based CPU frequency throttler + cpuset limiter");