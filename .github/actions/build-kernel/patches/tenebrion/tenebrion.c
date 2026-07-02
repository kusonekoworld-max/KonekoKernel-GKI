// SPDX-License-Identifier: GPL-2.0
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
#include <linux/string.h>

#define POLL_INTERVAL_MS    3000
#define DPMS_PATH           "/sys/class/drm/card0-DSI-1/dpms"
#define BACKLIGHT_PATH      "/sys/class/backlight/panel0-backlight/brightness"

#define CPUSET_SYSBG_PATH   "/dev/cpuset/system-background/cpus"
#define CPUSET_BG_PATH      "/dev/cpuset/background/cpus"

#define ZRAM_COMP_PATH       "/sys/block/zram0/comp_algorithm"
#define ZRAM_COMP_ALGO       "zstd"
#define ZRAM_RETRY_COUNT     10
#define ZRAM_RETRY_DELAY_MS  500

static const char *iosched_candidates[] = {
    "/sys/block/sda/queue/scheduler",
    "/sys/block/mmcblk0/queue/scheduler",
    "/sys/block/nvme0n1/queue/scheduler",
    NULL,
};
#define IOSCHED_TARGET   "bfq"

#define BOOT_DELAY_MS        40000
#define BOOT_DELAY_STEP_MS   100

MODULE_IMPORT_NS(VFS_internal_I_am_really_a_filesystem_and_am_NOT_a_driver);

enum tenebrion_path {
    PATH_NONE        = 0,
    PATH_DPMS        = 1,
    PATH_BACKLIGHT   = 2,
    PATH_UNSUPPORTED = 3,
};

static enum tenebrion_path active_path = PATH_NONE;
static bool is_screen_off = false;
static DEFINE_MUTEX(tenebrion_lock);
static struct task_struct *watcher_thread;

static struct freq_qos_request tenebrion_min_req[NR_CPUS];
static struct freq_qos_request tenebrion_max_req[NR_CPUS];
static bool qos_initialized[NR_CPUS];

static char iosched_path[64] = "";
static char iosched_saved[32] = "";
static bool iosched_available = false;

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

static void enforce_zstd_compression(void)
{
    struct file *f;
    loff_t pos = 0;

    f = filp_open(ZRAM_COMP_PATH, O_WRONLY, 0);
    if (!IS_ERR(f)) {
        kernel_write(f, ZRAM_COMP_ALGO, strlen(ZRAM_COMP_ALGO), &pos);
        filp_close(f, NULL);
        pr_info("tenebrion: ZRAM enforced to %s success!\n", ZRAM_COMP_ALGO);
    } else {
        pr_warn("tenebrion: failed to enforce %s (is ZRAM enabled?)\n",
                ZRAM_COMP_ALGO);
    }
}

static int __init tenebrion_zram_init(void)
{
    struct file *f;
    int retry;

    for (retry = 0; retry < ZRAM_RETRY_COUNT; retry++) {
        f = filp_open(ZRAM_COMP_PATH, O_WRONLY, 0);
        if (!IS_ERR(f)) {
            filp_close(f, NULL);
            enforce_zstd_compression();
            return 0;
        }
        msleep(ZRAM_RETRY_DELAY_MS);
    }

    pr_warn("tenebrion: zram device never became available after %d retries\n",
            ZRAM_RETRY_COUNT);
    return 0;
}
late_initcall(tenebrion_zram_init);

static enum tenebrion_path tenebrion_detect_path(void)
{
    char buf[64];

    if (tenebrion_read_file(DPMS_PATH, buf, sizeof(buf)) > 0) {
        pr_info("tenebrion: detected path -> %s\n", DPMS_PATH);
        return PATH_DPMS;
    }

    if (tenebrion_read_file(BACKLIGHT_PATH, buf, sizeof(buf)) > 0) {
        pr_info("tenebrion: detected path -> %s\n", BACKLIGHT_PATH);
        return PATH_BACKLIGHT;
    }

    pr_err("tenebrion: no supported screen state path found - disabling\n");
    return PATH_UNSUPPORTED;
}

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

#define CPUSET_SCREEN_OFF   "0\n"

static char saved_sysbg_cpus[32] = "";
static char saved_bg_cpus[32]    = "";

static void tenebrion_cpuset_restrict_one(const char *path, char *saved_buf,
                                          size_t saved_size)
{
    tenebrion_read_file(path, saved_buf, saved_size);

    if (tenebrion_write_file(path, CPUSET_SCREEN_OFF) == 0)
        pr_info("tenebrion: cpuset %s -> 0\n", path);
}

static void tenebrion_cpuset_restore_one(const char *path, const char *saved_buf)
{
    char buf[32];
    char fallback_mask[32];
    int total_cores = num_possible_cpus();

    snprintf(fallback_mask, sizeof(fallback_mask), "0-%d", total_cores - 1);

    snprintf(buf, sizeof(buf), "%s\n",
             saved_buf[0] ? saved_buf : fallback_mask);

    if (tenebrion_write_file(path, buf) == 0)
        pr_info("tenebrion: cpuset %s restored -> %s", path, buf);
}

static void tenebrion_cpuset_restrict(void)
{
    tenebrion_cpuset_restrict_one(CPUSET_SYSBG_PATH, saved_sysbg_cpus,
                                  sizeof(saved_sysbg_cpus));
    tenebrion_cpuset_restrict_one(CPUSET_BG_PATH, saved_bg_cpus,
                                  sizeof(saved_bg_cpus));
}

static void tenebrion_cpuset_restore(void)
{
    tenebrion_cpuset_restore_one(CPUSET_SYSBG_PATH, saved_sysbg_cpus);
    tenebrion_cpuset_restore_one(CPUSET_BG_PATH, saved_bg_cpus);
}

static void tenebrion_iosched_parse_active(const char *raw, char *out, size_t out_size)
{
    const char *start = strchr(raw, '[');
    const char *end;

    out[0] = '\0';
    if (!start)
        return;

    start++;
    end = strchr(start, ']');
    if (!end)
        return;

    if ((size_t)(end - start) >= out_size)
        return;

    memcpy(out, start, end - start);
    out[end - start] = '\0';
}

static void tenebrion_iosched_detect(void)
{
    char buf[128];
    int i;

    for (i = 0; iosched_candidates[i] != NULL; i++) {
        if (tenebrion_read_file(iosched_candidates[i], buf, sizeof(buf)) > 0) {
            strscpy(iosched_path, iosched_candidates[i], sizeof(iosched_path));
            iosched_available = true;
            pr_info("tenebrion: I/O scheduler control -> %s\n", iosched_path);
            return;
        }
    }

    pr_warn("tenebrion: no known block device scheduler node found - "
            "I/O scheduler switching disabled\n");
    iosched_available = false;
}

static void tenebrion_iosched_restrict(void)
{
    char buf[128];

    if (!iosched_available)
        return;

    if (tenebrion_read_file(iosched_path, buf, sizeof(buf)) > 0)
        tenebrion_iosched_parse_active(buf, iosched_saved, sizeof(iosched_saved));

    if (tenebrion_write_file(iosched_path, IOSCHED_TARGET) == 0)
        pr_info("tenebrion: I/O scheduler -> %s (screen off)\n", IOSCHED_TARGET);
}

static void tenebrion_iosched_restore(void)
{
    if (!iosched_available)
        return;

    if (!iosched_saved[0]) {
        pr_warn("tenebrion: no saved I/O scheduler to restore, leaving as-is\n");
        return;
    }

    if (tenebrion_write_file(iosched_path, iosched_saved) == 0)
        pr_info("tenebrion: I/O scheduler restored -> %s\n", iosched_saved);
}

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

static void tenebrion_set_min_freq(void)
{
    unsigned int cpu;
    struct cpufreq_policy *policy;

    for_each_online_cpu(cpu) {
        policy = cpufreq_cpu_get(cpu);
        if (!policy)
            continue;

        if (policy->cpu == cpu && qos_initialized[cpu]) {
            freq_qos_update_request(&tenebrion_min_req[cpu],
                                    policy->cpuinfo.min_freq);
            freq_qos_update_request(&tenebrion_max_req[cpu],
                                    policy->cpuinfo.min_freq);

            pr_info("tenebrion: policy%u -> %u KHz (screen off)\n",
                    cpu, policy->cpuinfo.min_freq);
        }

        cpufreq_cpu_put(policy);
    }
}

static void tenebrion_restore_freq(void)
{
    unsigned int cpu;
    struct cpufreq_policy *policy;

    for_each_online_cpu(cpu) {
        policy = cpufreq_cpu_get(cpu);
        if (!policy)
            continue;

        if (policy->cpu == cpu && qos_initialized[cpu]) {
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

static void tenebrion_on_screen_off(void)
{
    tenebrion_set_min_freq();
    tenebrion_cpuset_restrict();
    tenebrion_iosched_restrict();
    is_screen_off = true;
    pr_info("tenebrion: screen OFF -> CPUs throttled + cpusets restricted "
            "+ I/O scheduler switched (online CPUs: %u)\n", num_online_cpus());
}

static void tenebrion_on_screen_on(void)
{
    tenebrion_restore_freq();
    tenebrion_cpuset_restore();
    tenebrion_iosched_restore();
    is_screen_off = false;
    pr_info("tenebrion: screen ON -> CPUs restored + cpusets restored "
            "+ I/O scheduler restored\n");
}

static int tenebrion_watcher(void *data)
{
    int current_state = -1;
    int last_state    = -1;
    int waited_ms     = 0;

    pr_info("tenebrion: watcher started, polling every %dms\n",
            POLL_INTERVAL_MS);

    while (waited_ms < BOOT_DELAY_MS) {
        if (kthread_should_stop())
            return 0;
        msleep(BOOT_DELAY_STEP_MS);
        waited_ms += BOOT_DELAY_STEP_MS;
    }

    active_path = tenebrion_detect_path();
    if (active_path == PATH_UNSUPPORTED) {
        pr_info("tenebrion: path unsupported - watcher exiting\n");
        return 0;
    }

    tenebrion_qos_init();
    tenebrion_iosched_detect();

    while (!kthread_should_stop()) {
        if (active_path == PATH_NONE) {
            active_path = tenebrion_detect_path();
            if (active_path == PATH_UNSUPPORTED) {
                pr_info("tenebrion: retry failed - watcher exiting\n");
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

    pr_info("tenebrion: active - detection deferred to watcher, poll=%dms possible_cpus=%u\n",
            POLL_INTERVAL_MS, num_possible_cpus());
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

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kanagawa Yamada");
MODULE_DESCRIPTION("Tenebrion: Screen state based CPU frequency throttler + cpuset limiter + ZRAM Enforcer + I/O scheduler switch");