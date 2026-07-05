/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_DYNAMIC_FSYNC_H
#define _LINUX_DYNAMIC_FSYNC_H

#ifdef CONFIG_DYNAMIC_FSYNC

/* Screen state: 0 = unset, 1 = off, 2 = on */
extern int dynamic_fsync_state __read_mostly;
/* Master enable */
extern bool dynamic_fsync_enabled __read_mostly;

static inline bool dynamic_fsync_should_skip(void)
{
	return READ_ONCE(dynamic_fsync_enabled) &&
	       READ_ONCE(dynamic_fsync_state) == 2;
}

#else
static inline bool dynamic_fsync_should_skip(void) { return false; }
#endif /* CONFIG_DYNAMIC_FSYNC */

#endif /* _LINUX_DYNAMIC_FSYNC_H */
