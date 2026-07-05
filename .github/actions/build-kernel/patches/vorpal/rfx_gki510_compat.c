// SPDX-License-Identifier: GPL-2.0
/*
 * rfx_gki510_compat.c - shared GKI 5.10-style compat symbols consumed
 * by cpufreq_vorpal.c.
 *
 * Vorpal declares three externs it expects to find already exported
 * somewhere in the tree:
 *
 *   extern int  sched_gaming_active;
 *   extern void rfx_get_util_gki510(int cpu, unsigned long boost,
 *                                   unsigned long *util, unsigned long *bwmin);
 *   extern bool rfx_dl_bw_exceeded_gki510(int cpu, unsigned long bwmin);
 *
 * Neither cpufreq_reflex.c nor fair.c actually define these in this
 * tree (checked -- reflex has its own *static*, differently-shaped
 * rfx_get_util() and never touches sched_gaming_active at all), so
 * vorpal.o would fail to link with "undefined reference" otherwise.
 *
 * This file defines and exports all three. NOTE: despite the tree
 * claiming a 5.15 base, schedutil_cpu_util() is not actually declared
 * here -- only the simpler public wrapper sched_cpu_util(cpu, max) is
 * available (confirmed via build failure + compiler suggestion), so
 * this uses that instead of the assumption in the original comment.
 * cpu_bw_dl() / arch_scale_cpu_capacity() are unaffected.
 *
 * Lives in kernel/sched/ (not drivers/cpufreq/) because it needs the
 * internal kernel/sched/sched.h declarations (struct rq, cpu_rq(),
 * cpu_util_cfs(), cpu_bw_dl()) that aren't exposed outside this
 * directory.
 */

#include "sched.h"
#include <linux/export.h>
#include <linux/sched/cpufreq.h>

/*
 * Scheduler coupling flag. Written by vorpal's gaming_mode_store()
 * sysfs handler; intended to be read by fair.c for gaming-specific
 * scheduling biases (BORE/CFS) in a future patch. Plain int, no
 * struct-layout changes -> KMI safe.
 */
int sched_gaming_active;
EXPORT_SYMBOL_GPL(sched_gaming_active);

/*
 * Core-sched util getter, GKI-510-style signature (cpu-indexed, with
 * a separate bwmin out-param) expected by vorpal.c.
 */
void rfx_get_util_gki510(int cpu, unsigned long boost,
			  unsigned long *util, unsigned long *bwmin)
{
	struct rq *rq = cpu_rq(cpu);
	unsigned long max_cap = arch_scale_cpu_capacity(cpu);
	unsigned long u;

	*bwmin = cpu_bw_dl(rq);

	/*
	 * This tree does not declare schedutil_cpu_util() (cpu, util_cfs,
	 * max, type, sg_cpu) despite claiming a 5.15 base -- it exposes the
	 * simpler public wrapper sched_cpu_util(cpu, max) instead, which
	 * derives cfs util internally. No separate cpu_util_cfs()/raw_util
	 * or FREQUENCY_UTIL enum needed here anymore.
	 */
	u = sched_cpu_util(cpu, max_cap);
	u = max(u, boost);
	if (u > max_cap)
		u = max_cap;

	*util = u;
}
EXPORT_SYMBOL_GPL(rfx_get_util_gki510);

/*
 * Deadline-bandwidth bypass check, GKI-510-style signature expected
 * by vorpal.c. Returns true when the CPU's current DL running
 * bandwidth exceeds the caller-supplied floor, meaning the caller
 * should skip its own frequency floor/boost logic (a DL task already
 * has the frequency it needs).
 */
bool rfx_dl_bw_exceeded_gki510(int cpu, unsigned long bwmin)
{
	struct rq *rq = cpu_rq(cpu);

	return cpu_bw_dl(rq) > bwmin;
}
EXPORT_SYMBOL_GPL(rfx_dl_bw_exceeded_gki510);
