/*
 * Copyright 2017-2020 Anton Blanchard, IBM Corporation <anton@linux.ibm.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#define pr_fmt(fmt) "lockstorm: " fmt

#include <linux/limits.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/slab.h>

static long timeout = 10;
module_param(timeout, long, S_IRUGO);
MODULE_PARM_DESC(timeout, "Timeout in seconds (default = 10)");

static bool use_atomics = false;
module_param(use_atomics , bool, S_IRUGO);
MODULE_PARM_DESC(use_atomics, "Use atomic ops rather than spinlock (default false)");

static char cpulist_str[256];
module_param_string(cpulist, cpulist_str, sizeof(cpulist_str), 0644);
MODULE_PARM_DESC(cpulist, "List of CPUs (default all)");

static unsigned int num_cpus;
static atomic_t running;

static DECLARE_COMPLETION(lockstorm_done);

struct obj {
	spinlock_t spinlock;
	unsigned int lock;
	unsigned int taken;
};

// static __cacheline_aligned struct obj obj;

static int gmax_starve[NR_CPUS];
static int gmin_starve[NR_CPUS];
static int gmax_sequential[NR_CPUS];
static u64 gmin_wait[NR_CPUS];
static u64 gmax_wait[NR_CPUS];
static u64 gnr_locks[NR_CPUS];

static __inline__ unsigned int incret(unsigned int *v, int eh)
{
	unsigned int t;

	__asm__ __volatile__(
"1:     lwarx   %0,0,%2,%3	 # incret\n"
"       addic   %0,%0,1\n"
"       stwcx.  %0,0,%2\n"
"       bne-    1b"
	: "=&r" (t), "+m" (*v)
	: "r" (v), "i" (eh)
	: "cc", "xer");

	return t;
}

static __inline__ bool tas_lock(unsigned int *v, int eh)
{
	unsigned int t;

	__asm__ __volatile__(
"1:     lwarx   %0,0,%2,%4	 # incret\n"
"       cmpw	%0,0\n"
"       bne-	2f\n"
"       stwcx.  %3,0,%2\n"
"       bne-    1b\n"
"	lwsync\n"
"2:"
	: "=&r" (t), "+m" (*v)
	: "r" (v), "r"(1), "i" (eh)
	: "cc", "xer");

	return t;
}

static __inline__ void clr_unlock(unsigned int *v)
{
	smp_store_release(v, 0);
}

static void spin_lock2(struct obj *obj)
{
	for (;;) {
		if (tas_lock(&obj->lock, 1) == 0)
			break;
		while (obj->lock != 0)
			cpu_relax();
	}
}

static void spin_unlock2(struct obj *obj)
{
	clr_unlock(&obj->lock);
}

static int lockstorm_thread(void *data)
{
	struct obj *obj = data;
	unsigned long t;
	u64 iters = 0;
	ktime_t start, end;
	ktime_t lock, granted;
	u64 max_wait = 0;
	u64 min_wait = U64_MAX;
	int prev_taken = -1;
	int max_taken = 0;
	int min_taken = INT_MAX;
	int max_sequential = 0;
	int cur_sequential = 0;
	int taken, tdelta;

	atomic_inc(&running);

	while (atomic_read(&running) < num_cpus) {
		if (need_resched())
			schedule();
	}

	t = jiffies + timeout*HZ;

	start = ktime_get();

	iters = 0;
	prev_taken = 0;
	while (time_before(jiffies, t)) {
		lock = ktime_get();
//		spin_lock(&obj->spinlock);
		spin_lock2(obj);
		taken = obj->taken++;
		if (iters == 0 || atomic_read(&running) < num_cpus)
			goto next;
		granted = ktime_get();
		if (ktime_sub_ns(granted, lock) < min_wait)
			min_wait = ktime_sub_ns(granted, lock);
		if (ktime_sub_ns(granted, lock) > max_wait)
			max_wait = ktime_sub_ns(granted, lock);

		if (taken == prev_taken + 1)
			cur_sequential++;
		else
			cur_sequential = 0;

		if (cur_sequential > max_sequential)
			max_sequential = cur_sequential;

		tdelta = taken - prev_taken;
		if (tdelta < min_taken)
			min_taken = tdelta;
		if (tdelta > max_taken)
			max_taken = tdelta;
next:
//		spin_unlock(&obj->spinlock);
		spin_unlock2(obj);

		prev_taken = taken;

		iters++;

		if (atomic_read(&running) < num_cpus)
			break;
	}
	end = ktime_get();

	gmax_starve[smp_processor_id()] = max_taken;
	gmin_starve[smp_processor_id()] = min_taken;
	gmax_sequential[smp_processor_id()] = max_sequential;
	gmax_wait[smp_processor_id()] = max_wait;
	gmin_wait[smp_processor_id()] = min_wait;
	gnr_locks[smp_processor_id()] = iters;
	if (atomic_dec_and_test(&running))
		complete(&lockstorm_done);

	return 0;
}

static int atomicstorm_thread(void *data)
{
	struct obj *obj = data;
	unsigned long t;
	u64 iters = 0;
	ktime_t start, end;
	ktime_t lock, granted;
	u64 max_wait = 0;
	u64 min_wait = U64_MAX;
	int prev_taken = -1;
	int max_taken = 0;
	int min_taken = INT_MAX;
	int max_sequential = 0;
	int cur_sequential = 0;
	int taken, tdelta;

	atomic_inc(&running);

	while (atomic_read(&running) < num_cpus) {
		if (need_resched())
			schedule();
	}

	t = jiffies + timeout*HZ;

	start = ktime_get();

	iters = 0;
	prev_taken = 0;
	while (time_before(jiffies, t)) {
		lock = ktime_get();
		taken = incret(&obj->taken, 0);
		if (iters == 0 || atomic_read(&running) < num_cpus)
			goto next;
		granted = ktime_get();
		if (ktime_sub_ns(granted, lock) < min_wait)
			min_wait = ktime_sub_ns(granted, lock);
		if (ktime_sub_ns(granted, lock) > max_wait)
			max_wait = ktime_sub_ns(granted, lock);

		if (taken == prev_taken + 1)
			cur_sequential++;
		else
			cur_sequential = 0;

		if (cur_sequential > max_sequential)
			max_sequential = cur_sequential;

		tdelta = taken - prev_taken;
		if (tdelta < min_taken)
			min_taken = tdelta;
		if (tdelta > max_taken)
			max_taken = tdelta;
next:
		prev_taken = taken;

		iters++;

		if (atomic_read(&running) < num_cpus)
			break;
	}
	end = ktime_get();

	gmax_starve[smp_processor_id()] = max_taken;
	gmin_starve[smp_processor_id()] = min_taken;
	gmax_sequential[smp_processor_id()] = max_sequential;
	gmax_wait[smp_processor_id()] = max_wait;
	gmin_wait[smp_processor_id()] = min_wait;
	gnr_locks[smp_processor_id()] = iters;
	if (atomic_dec_and_test(&running))
		complete(&lockstorm_done);

	return 0;
}

static int __init lockstorm_init(void)
{
	struct obj *obj;
	unsigned long cpu;
	cpumask_var_t mask;
	int ret = 0;

	obj = kmalloc_node(sizeof(struct obj), GFP_KERNEL, 0);
	if (!obj)
		return -ENOMEM;

	spin_lock_init(&obj->spinlock);
	obj->lock = 0;
	obj->taken = 0;

	init_completion(&lockstorm_done);

	if (!zalloc_cpumask_var(&mask, GFP_KERNEL)) {
		kfree(obj);
		return -ENOMEM;
	}

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

		if (use_atomics)
			p = kthread_create(lockstorm_thread, obj,
						"lockstorm/%lu", cpu);
		else
			p = kthread_create(atomicstorm_thread, obj,
						"lockstorm/%lu", cpu);
		if (IS_ERR(p)) {
			pr_err("kthread_create on CPU %lu failed\n", cpu);
			atomic_inc(&running);
		} else {
			kthread_bind(p, cpu);
			wake_up_process(p);
		}
	}
	wait_for_completion(&lockstorm_done);

	if (1) {
		int max_max_starve = 0;
		int min_max_starve = INT_MAX;
		int max_min_starve = 0;
		int min_min_starve = INT_MAX;
		int max_max_sequential = 0;
		int min_max_sequential = INT_MAX;
		u64 max_max_wait = 0;
		u64 min_max_wait = U64_MAX;
		u64 max_min_wait = 0;
		u64 min_min_wait = U64_MAX;
		u64 max_nr_locks = 0;
		u64 min_nr_locks = U64_MAX;
		u64 nr_locks = 0;

		for_each_cpu(cpu, mask) {
			if (gmax_starve[cpu] > max_max_starve)
				max_max_starve = gmax_starve[cpu];
			if (gmax_starve[cpu] < min_max_starve)
				min_max_starve = gmax_starve[cpu];

			if (gmin_starve[cpu] > max_min_starve)
				max_min_starve = gmin_starve[cpu];
			if (gmin_starve[cpu] < min_min_starve)
				min_min_starve = gmin_starve[cpu];

			if (gmax_sequential[cpu] > max_max_sequential)
				max_max_sequential = gmax_sequential[cpu];
			if (gmax_sequential[cpu] < min_max_sequential)
				min_max_sequential = gmax_sequential[cpu];

			if (gmax_wait[cpu] > max_max_wait)
				max_max_wait = gmax_wait[cpu];
			if (gmax_wait[cpu] < min_max_wait)
				min_max_wait = gmax_wait[cpu];

			if (gmin_wait[cpu] > max_min_wait)
				max_min_wait = gmin_wait[cpu];
			if (gmin_wait[cpu] < min_min_wait)
				min_min_wait = gmin_wait[cpu];

			nr_locks += gnr_locks[cpu];
			if (gnr_locks[cpu] > max_nr_locks)
				max_nr_locks = gnr_locks[cpu];
			if (gnr_locks[cpu] < min_nr_locks)
				min_nr_locks = gnr_locks[cpu];
		}

		pr_notice("%s iterations:%llu (max:%llu min:%llu) max wait:%llu (min:%llu) min wait:%llu (max:%llu) max starve:%d (min:%d) min starve:%d (max:%d) max sequential:%d (min:%d)\n", use_atomics ? "  atomic" : "spinlock", nr_locks, max_nr_locks, min_nr_locks, max_max_wait, min_max_wait, min_min_wait, max_min_wait, max_max_starve, min_max_starve, min_min_starve, max_min_starve, max_max_sequential, min_max_sequential);
	}

out_free:
	kfree(obj);
	free_cpumask_var(mask);
	return ret ? ret : -EAGAIN;
}

static void __exit lockstorm_exit(void)
{
}

module_init(lockstorm_init)
module_exit(lockstorm_exit)

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Anton Blanchard");
MODULE_DESCRIPTION("Lock testing");
