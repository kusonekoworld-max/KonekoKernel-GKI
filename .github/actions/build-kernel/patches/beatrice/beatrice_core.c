#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/init.h>
#include <linux/kdev_t.h>

static struct kobject *beatrice_kobj;
static int lite_mode = 0;
static int thermal_state = 0;
static int battery_saver_delay = 30;
static int cache_clean_trigger = 0;

static void beatrice_notify(const char *event)
{
    char event_string[64];
    char *envp[2];

    snprintf(event_string, sizeof(event_string), "BEATRICE_EVENT=%s", event);
    envp[0] = event_string;
    envp[1] = NULL;

    kobject_uevent_env(beatrice_kobj, KOBJ_CHANGE, envp);
}

static ssize_t lite_mode_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", lite_mode);
}
static ssize_t lite_mode_store(struct kobject *kobj, struct kobj_attribute *attr,
                                const char *buf, size_t count)
{
    int val;
    if (kstrtoint(buf, 10, &val) < 0)
        return -EINVAL;
    lite_mode = val ? 1 : 0;
    beatrice_notify(lite_mode ? "LITE_MODE_ON" : "LITE_MODE_OFF");
    return count;
}

static ssize_t thermal_state_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", thermal_state);
}
static ssize_t thermal_state_store(struct kobject *kobj, struct kobj_attribute *attr,
                                    const char *buf, size_t count)
{
    int val;
    if (kstrtoint(buf, 10, &val) < 0)
        return -EINVAL;
    thermal_state = val;
    beatrice_notify("THERMAL_CHANGED");
    return count;
}

static ssize_t battery_saver_delay_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", battery_saver_delay);
}
static ssize_t battery_saver_delay_store(struct kobject *kobj, struct kobj_attribute *attr,
                                          const char *buf, size_t count)
{
    int val;
    if (kstrtoint(buf, 10, &val) < 0)
        return -EINVAL;
    battery_saver_delay = val;
    return count;
}

static ssize_t cache_clean_trigger_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", cache_clean_trigger);
}
static ssize_t cache_clean_trigger_store(struct kobject *kobj, struct kobj_attribute *attr,
                                          const char *buf, size_t count)
{
    int val;
    if (kstrtoint(buf, 10, &val) < 0)
        return -EINVAL;
    if (val == 1) {
        cache_clean_trigger = 1;
        beatrice_notify("CACHE_CLEAN_REQUEST");
    } else {
        cache_clean_trigger = 0;
    }
    return count;
}

static struct kobj_attribute lite_mode_attr = __ATTR(lite_mode, 0664, lite_mode_show, lite_mode_store);
static struct kobj_attribute thermal_state_attr = __ATTR(thermal_state, 0664, thermal_state_show, thermal_state_store);
static struct kobj_attribute battery_saver_delay_attr = __ATTR(battery_saver_delay, 0664, battery_saver_delay_show, battery_saver_delay_store);
static struct kobj_attribute cache_clean_trigger_attr = __ATTR(cache_clean_trigger, 0664, cache_clean_trigger_show, cache_clean_trigger_store);

static struct attribute *beatrice_attrs[] = {
    &lite_mode_attr.attr,
    &thermal_state_attr.attr,
    &battery_saver_delay_attr.attr,
    &cache_clean_trigger_attr.attr,
    NULL,
};

static struct attribute_group beatrice_attr_group = {
    .attrs = beatrice_attrs,
};

static int __init beatrice_init(void)
{
    int ret;

    beatrice_kobj = kobject_create_and_add("beatrice", kernel_kobj);
    if (!beatrice_kobj)
        return -ENOMEM;

    ret = sysfs_create_group(beatrice_kobj, &beatrice_attr_group);
    if (ret) {
        kobject_put(beatrice_kobj);
        return ret;
    }

    pr_info("beatrice_core: initialized at /sys/kernel/beatrice/\n");
    return 0;
}

static void __exit beatrice_exit(void)
{
    kobject_put(beatrice_kobj);
}

module_init(beatrice_init);
module_exit(beatrice_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Koneko_dev");
MODULE_DESCRIPTION("Beatrice sysfs + uevent interface for Betty's Tweaks daemon");