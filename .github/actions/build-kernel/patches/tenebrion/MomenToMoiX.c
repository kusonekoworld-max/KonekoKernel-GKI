#define pr_fmt(fmt) "momx: " fmt

#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/fs.h>
#include <linux/pm_qos.h>
#include <linux/slab.h>
#include <linux/cpumask.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/cpufreq.h>
#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/suspend.h>
#include <linux/notifier.h>
#include <linux/moduleparam.h>

MODULE_IMPORT_NS(VFS_internal_I_am_really_a_filesystem_and_am_NOT_a_driver);

static int poll_interval_ms = 1000; 
module_param(poll_interval_ms, int, 0644);

static int boot_delay_ms = 40000;
module_param(boot_delay_ms, int, 0644);

static int thermal_threshold_mc = 65000; 
module_param(thermal_threshold_mc, int, 0644);

static int thermal_hysteresis_mc = 5000;
module_param(thermal_hysteresis_mc, int, 0644);

static int thermal_hold_ms = 3000; 
module_param(thermal_hold_ms, int, 0644);

static int charging_freq_bias_percent = 20;
module_param(charging_freq_bias_percent, int, 0644);

static int doze_active_freq_bias_percent = 10;
module_param(doze_active_freq_bias_percent, int, 0644);

#define DPMS_PATH           "/sys/class/drm/card0-DSI-1/dpms"
#define BACKLIGHT_PATH      "/sys/class/backlight/panel0-backlight/brightness"
#define THERMAL_ZONE_MAX    15

#define CPUSET_SYSBG_PATH   "/dev/cpuset/system-background/cpus"
#define CPUSET_BG_PATH      "/dev/cpuset/background/cpus"

#define IOSCHED_TARGET       "none"

static const char *iosched_candidates[] = {
    "/sys/block/sda/queue/scheduler",
    "/sys/block/mmcblk0/queue/scheduler",
    "/sys/block/nvme0n1/queue/scheduler",
    NULL,
};

static const char *power_status_candidates[] = {
    "/sys/class/power_supply/battery/status",
    "/sys/class/power_supply/bms/status",
    "/sys/class/power_supply/BAT0/status",
    NULL,
};

static struct freq_qos_request momx_min_req[NR_CPUS];
static struct freq_qos_request momx_max_req[NR_CPUS];
static bool qos_initialized[NR_CPUS];
static struct task_struct *watcher_thread;
static DEFINE_MUTEX(momx_lock);

static bool is_screen_off = false;
static bool in_deep_sleep = false;
static bool thermal_hold_active = false;
static unsigned long thermal_hold_expire = 0;
static int last_temp_mc = -273000;
static char power_status_path[64] = "";
static bool power_status_available = false;

static char iosched_path[64] = "";
static char default_iosched[32] = "none";
static bool iosched_available = false;

static int momx_read_file(const char *path, char *buf, size_t size) {
    struct file *f;
    loff_t pos = 0;
    int ret;
    f = filp_open(path, O_RDONLY, 0);
    if (IS_ERR(f)) return -1;
    ret = kernel_read(f, buf, size - 1, &pos);
    filp_close(f, NULL);
    if (ret > 0) {
        if (buf[ret - 1] == '\n') buf[ret - 1] = '\0';
        else buf[ret] = '\0';
    } else ret = -1;
    return ret;
}

static int momx_write_file(const char *path, const char *buf) {
    struct file *f;
    loff_t pos = 0;
    int ret;
    f = filp_open(path, O_WRONLY, 0);
    if (IS_ERR(f)) return -1;
    ret = kernel_write(f, buf, strlen(buf), &pos);
    filp_close(f, NULL);
    return ret > 0 ? 0 : -1;
}

static int momx_read_max_temp(void) {
    char path[64], buf[32];
    int i, temp, max_temp = -273000;
    long val;
    for (i = 0; i < THERMAL_ZONE_MAX; i++) {
        snprintf(path, sizeof(path), "/sys/class/thermal/thermal_zone%d/temp", i);
        if (momx_read_file(path, buf, sizeof(buf)) <= 0) continue;
        if (kstrtol(buf, 10, &val)) continue;
        temp = (int)val;
        if (temp > max_temp) max_temp = temp;
    }
    return max_temp;
}

static void momx_charge_detect(void) {
    char buf[32];
    int i;
    for (i = 0; power_status_candidates[i] != NULL; i++) {
        if (momx_read_file(power_status_candidates[i], buf, sizeof(buf)) > 0) {
            strscpy(power_status_path, power_status_candidates[i], sizeof(power_status_path));
            power_status_available = true;
            pr_info("Battery status node detected at: %s\n", power_status_path);
            return;
        }
    }
    pr_warn("No battery status node detected from candidates.\n");
}

static bool momx_is_charging(void) {
    char buf[32];
    if (!power_status_available || momx_read_file(power_status_path, buf, sizeof(buf)) <= 0)
        return false;
    return strstr(buf, "Charging") != NULL || strstr(buf, "Full") != NULL;
}

static void momx_cpuset_restrict(void) {
    momx_write_file(CPUSET_SYSBG_PATH, "0\n");
    momx_write_file(CPUSET_BG_PATH, "0\n");
    pr_info("Cpuset: system-background & background isolated to CPU 0\n");
}

static void momx_cpuset_restore(void) {
    char fallback[32];
    snprintf(fallback, sizeof(fallback), "0-%d\n", num_possible_cpus() - 1);
    momx_write_file(CPUSET_SYSBG_PATH, fallback);
    momx_write_file(CPUSET_BG_PATH, fallback);
    pr_info("Cpuset: Restored system-background & background to all CPUs (%s)\n", fallback);
}

static void momx_iosched_detect(void) {
    char buf[128]; 
    int i;
    char *start;
    char *end;
    size_t len;

    for (i = 0; iosched_candidates[i] != NULL; i++) {
        if (momx_read_file(iosched_candidates[i], buf, sizeof(buf)) > 0) {
            strscpy(iosched_path, iosched_candidates[i], sizeof(iosched_path));
            iosched_available = true;
            pr_info("I/O Scheduler node detected at: %s\n", iosched_path);

            start = strchr(buf, '[');
            end = strchr(buf, ']');
            if (start && end && (end > start)) {
                len = end - start - 1;
                if (len < sizeof(default_iosched)) {
                    memcpy(default_iosched, start + 1, len);
                    default_iosched[len] = '\0';
                    pr_info("Detected stock scheduler: %s\n", default_iosched);
                }
            }
            return;
        }
    }
    pr_warn("I/O Scheduler: No compatible block storage node found.\n");
}

static void momx_iosched_restrict(void) {
    if (iosched_available) {
        momx_write_file(iosched_path, IOSCHED_TARGET);
        pr_info("I/O Scheduler: Switched to %s (Screen Off optimized)\n", IOSCHED_TARGET);
    }
}

static void momx_iosched_restore(void) {
    if (iosched_available) {
        momx_write_file(iosched_path, default_iosched);
        pr_info("I/O Scheduler: Restored to stock (%s)\n", default_iosched);
    }
}

static void momx_qos_init(void) {
    unsigned int cpu;
    struct cpufreq_policy *policy;
    int count = 0;

    for_each_online_cpu(cpu) {
        policy = cpufreq_cpu_get(cpu);
        if (!policy) continue;
        if (policy->cpu == cpu && !qos_initialized[cpu]) {
            freq_qos_add_request(&policy->constraints, &momx_min_req[cpu], FREQ_QOS_MIN, policy->cpuinfo.min_freq);
            freq_qos_add_request(&policy->constraints, &momx_max_req[cpu], FREQ_QOS_MAX, policy->cpuinfo.max_freq);
            qos_initialized[cpu] = true;
            count++;
        }
        cpufreq_cpu_put(policy);
    }
    pr_info("Initialized Freq QoS requests for %d CPUs\n", count);
}

static void momx_apply_freq_bias(int percent) {
    unsigned int cpu;
    struct cpufreq_policy *policy;
    unsigned int target;
    for_each_online_cpu(cpu) {
        policy = cpufreq_cpu_get(cpu);
        if (!policy) continue;
        if (policy->cpu == cpu && qos_initialized[cpu]) {
            target = policy->cpuinfo.min_freq + ((policy->cpuinfo.max_freq - policy->cpuinfo.min_freq) * percent) / 100;
            freq_qos_update_request(&momx_min_req[cpu], target);
            freq_qos_update_request(&momx_max_req[cpu], target);
        }
        cpufreq_cpu_put(policy);
    }
    pr_info("Applied frequency bias: %d%%\n", percent);
}

static void momx_restore_freq(void) {
    unsigned int cpu;
    struct cpufreq_policy *policy;
    for_each_online_cpu(cpu) {
        policy = cpufreq_cpu_get(cpu);
        if (!policy) continue;
        if (policy->cpu == cpu && qos_initialized[cpu]) {
            freq_qos_update_request(&momx_max_req[cpu], policy->cpuinfo.max_freq);
            freq_qos_update_request(&momx_min_req[cpu], policy->cpuinfo.min_freq);
        }
        cpufreq_cpu_put(policy);
    }
    pr_info("Restored stock CPU frequencies (QoS restrictions cleared)\n");
}

static void momx_on_screen_off(void) {
    bool charging;
    int bias;
    int temp;

    charging = momx_is_charging();
    bias = charging ? charging_freq_bias_percent : doze_active_freq_bias_percent;
    temp = momx_read_max_temp();

    last_temp_mc = temp;
    is_screen_off = true;

    pr_info("Display Mode: OFF | Charging: %s | Max Temp: %d mC\n", charging ? "Yes" : "No", temp);

    momx_apply_freq_bias(bias);
    momx_cpuset_restrict();
    momx_iosched_restrict();

    if (temp >= thermal_threshold_mc) {
        thermal_hold_active = true;
        pr_info("Thermal threshold exceeded (%d >= %d mC). Thermal hold armed.\n", temp, thermal_threshold_mc);
    }
}

static void momx_on_screen_on(void) {
    is_screen_off = false;
    pr_info("Display Mode: ON\n");

    momx_cpuset_restore();
    momx_iosched_restore();

    if (thermal_hold_active) {
        thermal_hold_expire = jiffies + msecs_to_jiffies(thermal_hold_ms);
        pr_info("Thermal hold active! Maintaining CPU freq bias restriction for %d ms.\n", thermal_hold_ms);
    } else {
        momx_restore_freq();
    }
}

static void momx_thermal_hold_tick(void) {
    int temp;
    if (!thermal_hold_active || is_screen_off) return;

    if (time_before(jiffies, thermal_hold_expire)) return;

    temp = momx_read_max_temp();
    if (temp < (thermal_threshold_mc - thermal_hysteresis_mc)) {
        pr_info("Thermal hold expired and temp cooled down (%d mC). Releasing CPU frequencies.\n", temp);
        momx_restore_freq();
        thermal_hold_active = false;
    } else {
        thermal_hold_expire = jiffies + msecs_to_jiffies(poll_interval_ms);
        pr_info("Thermal hold timer expired but temp still high (%d mC). Extending CPU hold.\n", temp);
    }
}

static int momx_pm_notifier(struct notifier_block *nb, unsigned long action, void *data) {
    if (action == PM_SUSPEND_PREPARE) {
        in_deep_sleep = true;
    } else if (action == PM_POST_SUSPEND) {
        in_deep_sleep = false;
    }
    return NOTIFY_OK;
}

static struct notifier_block momx_pm_nb = { .notifier_call = momx_pm_notifier };

static int momx_watcher(void *data) {
    int last_state = -1;

    pr_info("Watcher thread spawned. Sleeping for %d ms boot delay...\n", boot_delay_ms);
    msleep(boot_delay_ms);

    momx_qos_init();
    momx_charge_detect();
    momx_iosched_detect(); 

    pr_info("Watcher core loop active (Sysfs Polling mode).\n");

    while (!kthread_should_stop()) { 
        int current_state = -1;
        char buf[64];

        if (momx_read_file(DPMS_PATH, buf, sizeof(buf)) > 0) {
            current_state = strstr(buf, "On") ? 1 : 0;
        } 
        else if (momx_read_file(BACKLIGHT_PATH, buf, sizeof(buf)) > 0) {
            long bl_val;
            if (!kstrtol(buf, 10, &bl_val)) {
                current_state = (bl_val > 0) ? 1 : 0;
            }
        }

        if (current_state != -1 && current_state != last_state) {
            mutex_lock(&momx_lock);
            if (current_state == 0 && !is_screen_off) momx_on_screen_off();
            else if (current_state == 1 && is_screen_off) momx_on_screen_on();
            mutex_unlock(&momx_lock);
            last_state = current_state;
        }

        mutex_lock(&momx_lock);
        momx_thermal_hold_tick();
        mutex_unlock(&momx_lock);

        msleep_interruptible(poll_interval_ms);
    }

    pr_info("Watcher thread exiting.\n");
    return 0;
}

static int __init momx_init(void) {
    pr_info("Loading MomenToMoiX (Hybrid Engine)...\n");

    register_pm_notifier(&momx_pm_nb);

    watcher_thread = kthread_run(momx_watcher, NULL, "momx_watch");
    if (IS_ERR(watcher_thread)) {
        pr_err("Failed to run watcher kthread!\n");
        unregister_pm_notifier(&momx_pm_nb);
        return PTR_ERR(watcher_thread);
    }

    return 0;
}

static void __exit momx_exit(void) {
    if (watcher_thread) {
        kthread_stop(watcher_thread);
    }

    unregister_pm_notifier(&momx_pm_nb);

    momx_restore_freq();
    momx_cpuset_restore();
    momx_iosched_restore();

    pr_info("MomenToMoiX Clean unloaded\n");
}

module_init(momx_init);
module_exit(momx_exit);
MODULE_LICENSE("GPL");
