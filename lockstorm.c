/*
 * Copyright 2017-2020 Anton Blanchard, IBM Corporation <anton@linux.ibm.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#define pr_fmt(fmt) "lockstorm: " fmt

#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/delay.h>

static long timeout = 30;
module_param(timeout, long, S_IRUGO);
MODULE_PARM_DESC(timeout, "Timeout in seconds (default = 30)");

static bool wait = true;
module_param(wait, bool, S_IRUGO);
MODULE_PARM_DESC(wait, "Wait for IPI to finish? (default true)");

static bool single = false;
module_param(single, bool, S_IRUGO);
MODULE_PARM_DESC(single, "IPI to a single destination? (default false, IPI to all)");

static unsigned long offset = 1;
module_param(offset, ulong, S_IRUGO);
MODULE_PARM_DESC(offset, "Offset from current CPU for single destination IPI (default 1)");

static unsigned long mask = 0;
module_param(mask, ulong, S_IRUGO);
MODULE_PARM_DESC(mask, "Mask for single destination IPI (default no mask)");

static unsigned long delay = 0;
module_param(delay, ulong, S_IRUGO);
MODULE_PARM_DESC(delay, "Delay between calls in us (default 0)");

static char cpulist_str[256];
module_param_string(cpulist, cpulist_str, sizeof(cpulist_str), 0644);
MODULE_PARM_DESC(cpulist, "List of CPUs (default all)");

static unsigned int num_cpus;
static atomic_t running;

static DECLARE_COMPLETION(lockstorm_done);

struct obj {
	spinlock_t spinlock;
	int taken;
};

static __cacheline_aligned struct obj obj;

static int lockstorm_thread(void *data)
{
//	unsigned long mycpu = (unsigned long)data;
	unsigned long t;
	u64 iters = 0;
	ktime_t start, end;
	int prev_taken = -1;
	int max_taken = 0;
	int min_taken = INT_MAX;
	int max_sequential = 0;
	int cur_sequential = 0;

	atomic_inc(&running);

	while (atomic_read(&running) < num_cpus) {
		if (need_resched())
			schedule();
	}

	t = jiffies + timeout*HZ;

	start = ktime_get();

	iters = 0;
	while (time_before(jiffies, t)) {
		int taken = obj.taken;

		spin_lock(&obj.spinlock);
		if (obj.taken == prev_taken) {
			cur_sequential++;
		} else if (cur_sequential > max_sequential) {
			max_sequential = cur_sequential;
			cur_sequential = 0;
		}
		prev_taken = obj.taken;
		if (obj.taken - taken < min_taken)
			min_taken = obj.taken - taken;
		if (obj.taken - taken < max_taken)
			max_taken = obj.taken - taken;
		obj.taken++;
		spin_unlock(&obj.spinlock);

		iters++;

		if (atomic_read(&running) < num_cpus)
			break;
	}
	end = ktime_get();

	printk("%llu locks in %lluns\n", iters, ktime_sub_ns(end, start));

	if (atomic_dec_and_test(&running))
		complete(&lockstorm_done);

	return 0;
}

static int __init lockstorm_init(void)
{
	unsigned long cpu;
	cpumask_var_t mask;
	int ret = 0;

	spin_lock_init(&obj.spinlock);
	obj.taken = 0;

	init_completion(&lockstorm_done);

	if (!zalloc_cpumask_var(&mask, GFP_KERNEL))
		return -ENOMEM;

	if (cpulist_str[0]) {
		ret = cpulist_parse(cpulist_str, mask);
		if (ret)
			goto out_free;

		if (!cpumask_subset(mask, cpu_online_mask)) {
			pr_err("Invalid CPU list: %s\n", cpulist_str);
			ret = -EINVAL;
			goto out_free;
		}
	} else {
		cpumask_copy(mask, cpu_online_mask);
	}

	num_cpus = cpumask_weight(mask);

	for_each_cpu(cpu, mask) {
		struct task_struct *p;
		p = kthread_create(lockstorm_thread, (void *)cpu,
				   "lockstorm/%lu", cpu);
		if (IS_ERR(p)) {
			pr_err("kthread_create on CPU %lu failed\n", cpu);
			atomic_inc(&running);
		} else {
			kthread_bind(p, cpu);
			wake_up_process(p);
		}
	}

out_free:
	free_cpumask_var(mask);
	return ret;
}

static void __exit lockstorm_exit(void)
{
	wait_for_completion(&lockstorm_done);
}

module_init(lockstorm_init)
module_exit(lockstorm_exit)

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Anton Blanchard");
MODULE_DESCRIPTION("Lock testing");
