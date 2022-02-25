/*
 *  linux/drivers/cpufreq/cpufreq_endurance.c
 *
 *  Endurance CpuFreq Governor
 *
 *  Author: Tejas Udupa <tejasudupa1285@gmail>
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; version 2
 *  of the License.
 *
 *
 *  Endurance V3:-
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/cpufreq.h>
#include <linux/thermal.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/suspend.h>
#include "endurance.h"

static unsigned int nap_time_ms = 1800;			// Governor sleep Timeout in millisecond
static bool run_state = 0;
static DEFINE_PER_CPU(struct cluster_prop *, cluster_nr);
static DEFINE_PER_CPU(struct cluster_tunables *, tunable_nr);
BLOCKING_NOTIFIER_HEAD(limits_notifier_head);
static struct work_struct therm_work;
struct workqueue_struct *therm_wq;

static struct sensor_monitor *thermal_monitor;
/* realtime thread handles thermal monitoring */
static struct task_struct *thermal_mon_task;
static struct mutex rw_lock;
static struct kobject *endurance_kobj;

/*		
 * get_cpufreq_table() initialises little and big core frequency tables.
 * does all reset/init process and any failure during this process leads
 * to governor disabling and not working for the failed cluster. 
 */
int get_cpufreq_table(struct cpufreq_policy *policy){
	
	struct cluster_prop *cluster;
	int ret = 0;
	
	if(!policy)
		return ret = -1;
	/* If structure already initialized exit out */
	cluster = per_cpu(cluster_nr,policy->cpu);
	if(cluster)
		goto setup_done;
	/* Start cpu frequency table initialization */
	ret = init_cpufreq_table(policy);
	if(ret < 0)
		goto failed_inittbl;
setup_done:
	/* initilse the cluster tunables */
	ret = init_tunables(policy);
	if(ret)
		goto failed_inittbl;
		
	return ret;

failed_inittbl:
	pr_err(KERN_WARNING "%s: Failed to setup tunable attrs:%d\terr=%d", __func__,policy->cpu,ret);
	return ret;
}

/*
 * init_cpufreq_table() initilizeses the the cluster_nr structure 
 * and assigns it to all cores of that cluster and initlises few structure members.
 * 
 * @nr_levels: total number of levels or highest index of frequency array
 * @freq_table: holds pointer to frequency table data 
 * @index: get the number of frequencies the respective cluster has
 * @ppol: holds the pointer to policy cluster of the respective cluster
 */
int init_cpufreq_table(struct cpufreq_policy *policy)
{
	struct cluster_prop *cluster = per_cpu(cluster_nr,policy->cpu);
	struct cpufreq_frequency_table *freq_table;
	int i;

	/* Get Highest Frequency Index in array */
	freq_table = cpufreq_frequency_get_table(policy->cpu);

	/* Initialise the cluster_prop structure. */
	if(!cluster){
		cluster = kzalloc(sizeof(struct cluster_prop), 
						GFP_KERNEL);
		if(!cluster)
			return -ENOMEM;

		memset(cluster, 0, sizeof(struct cluster_prop));
	}
	if(!thermal_monitor){
		thermal_monitor = kzalloc(sizeof(struct sensor_monitor), 
						GFP_KERNEL);
		if(!thermal_monitor)
			return -ENOMEM;

		memset(thermal_monitor, 0, sizeof(struct sensor_monitor));
	}
	
	/* assign cluster -> cluster_nr for each avilable core in that cluster */
	for_each_cpu(i, policy->related_cpus)
		per_cpu(cluster_nr,i) = cluster;
		
	cluster->freq_table = freq_table;

	return 0;
}

/*
 * init_tunables() initlisies the governor tunables to their initial
 * default values. sets up throttle temprature and temprature diffrence to be
 * kept for each cluster. The function is re-called everytime governor is
 * restarted as the tunable structure is re-inited with default values.
 */
int init_tunables(struct cpufreq_policy *policy)
{
	struct cluster_tunables *tunable = per_cpu(tunable_nr,policy->cpu);
	int i;

	if(!tunable){
		tunable = kzalloc(sizeof(struct cluster_tunables), 
						GFP_KERNEL);
		if(!tunable)
			return -ENOMEM;

		memset(tunable, 0, sizeof(struct cluster_tunables));
	}
	else
		return 0;

	/* Initialise throttle temperature of big and little cluster */
	if(policy->cpu <= NR_LITTLE){
		tunable->throttle_temperature = THROTTLE_TEMP_LITTLE;
		tunable->temperature_diff = TEMP_DIFF_LITTLE;
		tunable->max_throttle_step = MAX_STEP_LITTLE;
	}
	else if(policy->cpu <= NR_BIG){
		tunable->throttle_temperature = THROTTLE_TEMP_BIG;
		tunable->temperature_diff = TEMP_DIFF_BIG;
		tunable->max_throttle_step = MAX_STEP_BIG;
	}
	/* assign cluster -> cluster_nr for each avilable core in that cluster */
	for_each_cpu(i, policy->related_cpus)
		per_cpu(tunable_nr,i) = tunable;
		
	return 0;
}

/*
 * cfe_reset_params() resets the cluster max_freq and nr_levels
 * this helps us to do correct thermal mitigation and dynamically switch between
 * frequency and allows users to have a certain degree of control on the max frequency
 * which has direct impact on how the thermal mitigation proceeds.
 */
void cfe_reset_params(struct cpufreq_policy *policy)
{
	struct cluster_prop *cluster = per_cpu(cluster_nr, policy->cpu);
	int index = 0;

	if(!cluster)
		return;
 	
 	mutex_lock(&rw_lock);
	PDEBUG("Prev freq:%u Cur Freq:%u New freq:%u",cluster->max_freq,
					policy->cur, policy->max);
	/* using index navigate to the equavalent frequency as that of
	 * policy->max in our frequency table.
	 */
	while(cluster->freq_table[index].frequency < policy->max)
		index++;
	PDEBUG("Reseting Index to:%d",index);

	cluster->throt_level = cluster->nr_levels = index;
	cluster->max_freq = policy->max;
	mutex_unlock(&rw_lock);
}
/*
 * update_sensor_data() get temperature reading from the desired sensor.
 * in case of failure to get current temperature from sensor it sets cur_temps to 0.
 */
static inline int update_sensor_data(void)
{
	int ret = 0;

	ret = sensor_get_temp(SENSOR_ID,&thermal_monitor->cur_temps);
	if(ret){
		pr_err(KERN_WARNING"%s: Failed to get sensor: %d temprature data.\n",
								__func__, SENSOR_ID);
		thermal_monitor->cur_temps = 0;
		return 1;
	}
	
	return 0;	
}

/*	
 * govern_cpu() calls for cpufreq mitigation based on temperature inputs from the respective sensor.
 * determines which frequency to throttle down/up to depending of the temperature of sensor.
 * precaution must be taken while passing cluster_prop as uninitilized variable ppol can
 * cause much pain(kernel panic!).
 */
static void govern_cpu(struct cluster_prop *cluster, unsigned int cpu)
{
	struct cpufreq_policy *policy = cpufreq_cpu_get(cpu);
	struct cluster_tunables *tunables = per_cpu(tunable_nr, cpu);
	int th_temp_diff = 0;
	int calc_temps = 0;

	th_temp_diff = thermal_monitor->cur_temps - tunables->throttle_temperature;
	PDEBUG("throt diff:%d cur_temp:%ld cpuid:%d", th_temp_diff, 
					thermal_monitor->cur_temps, policy->cpu);
					
	/* either we have not yet reached our cluster throttle temps or we dropped below
	 * throttle temps, so reset cluster levels and push max frequency of that cluster
	 */
	if(th_temp_diff < 0) {
		PDEBUG("Currrent temps Lower than throttle temps");
		if(cluster->nr_levels == cluster->throt_level)
			return;
		cluster->throt_level = cluster->nr_levels;
		do_cpufreq_mitigation(policy, cluster);
		nap_time_ms = 1800;
	}
	else {
		PDEBUG("Currrent temps Higher than throttle temps");
		calc_temps = cluster->nr_levels - 
				(th_temp_diff / tunables->temperature_diff + 1);
		/* exit if threshold was not reached or max throttle step was
		 * reached as we dont throttle down below user defined threshold
		 * to reduce performance impact due to further throttle.
		 */
		if((cluster->throt_level == calc_temps) ||
		   (cluster->throt_level < tunables->max_throttle_step)){
			PDEBUG("No cpufreq changes required %d %d",cluster->throt_level, calc_temps);
			return;
		}
			
		cluster->throt_level = calc_temps;
		PDEBUG("cur_temps:%ld throt_temps:%d prev_temps:%ld",thermal_monitor->cur_temps,
			tunables->throttle_temperature, thermal_monitor->prev_temps);
		do_cpufreq_mitigation(policy, cluster);
		nap_time_ms = 1200;
	}
}

static void govern_callback_task(struct work_struct *work)
{
	struct cluster_prop *cluster;
	int i = -1;

	mutex_lock(&rw_lock);
	PDEBUG("%s:",__func__);
	while(++i < CLUSTER_NR){
		cluster = per_cpu(cluster_nr, cpu_nr[i]);
		if(cluster)
			govern_cpu(cluster, cpu_nr[i]);
	}
	mutex_unlock(&rw_lock);
}

static int therm_suspend_notifier(struct notifier_block *nb,
	unsigned long event, void *dummy)
{
	switch (event) {
	case PM_SUSPEND_PREPARE:
		/* suspend process dont run the thread as the device
		   is anyway off and let it be at the throttle level
		   it was left at. */
		run_state = 0;
		PDEBUG("%s:Suspend",__func__);
		break;
	case PM_POST_SUSPEND:
		/* resume process */
		run_state = 1;
		PDEBUG("%s:Resume",__func__);
		wake_up_process(thermal_mon_task);
		break;
	default:
		break;
	}
	return 0;
}

static struct notifier_block therm_sleep_notify = {
	.notifier_call = therm_suspend_notifier,
};

static int adjust_policy_max(struct notifier_block *nb, unsigned long val,
				void *data)
{
	struct cpufreq_policy *policy = data;
	struct cluster_prop *cluster = per_cpu(cluster_nr, policy->cpu);
	
	mutex_lock(&rw_lock);
	if(policy->max > cluster->max_freq){
			PDEBUG("%s: policy:%u max:%u", __func__, policy->max, cluster->max_freq);
			do_cpufreq_mitigation(policy, cluster);
	}
	mutex_unlock(&rw_lock);
	
	return 0;
}

static struct notifier_block limits_chg_notify = {
	.notifier_call = adjust_policy_max,
};

/*
 * do_cpufreq_mitigation() it modifies the policy max frequency to the latest max
 * updated by the calling function and sets the frequency on the cluster.
 */
static inline void do_cpufreq_mitigation(struct cpufreq_policy *policy,
			struct cluster_prop *cluster)
{
	PDEBUG("THROTTLE to %u from %u level:%d max_lvl:%d cpu:%d",
			cluster->freq_table[cluster->throt_level].frequency,
			cluster->max_freq,cluster->throt_level, cluster->nr_levels, 
			policy->cpu);
	cluster->max_freq = cluster->freq_table[cluster->throt_level].frequency;
	policy->max = cluster->max_freq;
	__cpufreq_driver_target(policy, policy->max, CPUFREQ_RELATION_C);
}

/*
 * thermal_mon_task() calls the govern_cpu() function.
 * if this function fails te governor is essentially dead, locks have been put to mitigate
 * contention issues that may arise during execution.
 */
static int thermal_poll(void *data)
{
	int i, ret = 0;
	
	PDEBUG("Endurance Running...");
	while (!kthread_should_stop()) {
		set_current_state(TASK_RUNNING);
		/* sleep thread until notifier completes its task or if govenor disabled */
		if(!run_state){
			set_current_state(TASK_INTERRUPTIBLE);
			schedule();
		}
		/* get updated thermal reading */
		ret = update_sensor_data();
		if(ret)
			goto skip;

		/* compare with updated temps to see if the current temps changed or not */
		if(thermal_monitor->cur_temps != thermal_monitor->prev_temps)
			queue_work(therm_wq, &therm_work);
		thermal_monitor->prev_temps = thermal_monitor->cur_temps;
		/* check if user explicitly changed ploci max and if higher than 
		   throttle level then take action. 
		 */
		for(i = 0;i < CLUSTER_NR;i++)
			blocking_notifier_call_chain(&limits_notifier_head, 
						0,cpufreq_cpu_get(cpu_nr[i]));
skip:
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(nap_time_ms));
	}
	
	return 0;
}


/**********************************************************************************/
/********************************* sysfs interface ********************************/

/* Create show/store routines */
static int store_fn(int temp, unsigned short *arg, short int cpu)
{
	struct cluster_prop *cluster;
	
 	if (!temp)
 		return -EINVAL;
 	*arg = temp;
 	cluster = per_cpu(cluster_nr, cpu);
	govern_cpu(cluster, cpu);
	return 0;
}

static ssize_t mod_status_show(struct kobject *kobj, struct kobj_attribute *attr,
				  char *buf)
{
	PDEBUG("%s:SHOW",__func__);
	return sprintf(buf,"%d\n", run_state);		  
}
static ssize_t mod_state_store(struct kobject *kobj, struct kobj_attribute *attr,
				  const char *buf, size_t count)
{
	int enable;
	PDEBUG("%s:STORE",__func__);
	sscanf(buf,"%d", &enable);
	/* check for state if running do nothing else call init */
	if((enable & run_state) == 0x01)
		return count;
	if(enable && !run_state)
		postinit(START);
	else
		postinit(STOP);
		
	return count;
}

static struct kobj_attribute mod_state =
	__ATTR(enable, 0644, mod_status_show, mod_state_store);
	
static ssize_t throttle_temperature_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	struct cluster_tunables *tunable;
	unsigned int temp[CLUSTER_NR], i = 0;
	
	PDEBUG("%s:SHOW",__func__);
	while(i < CLUSTER_NR) {
		tunable = per_cpu(tunable_nr, cpu_nr[i]);
		temp[i++] = tunable->throttle_temperature;
	}
	i = 0;
	return sprintf(buf, "%u:%u\n",temp[i] ,temp[i+1]);
}

static ssize_t throttle_temperature_store(struct kobject *kobj, struct kobj_attribute *attr,
			      const char *buf, size_t count)
{
	struct cluster_tunables *tunable;
	int temp[CLUSTER_NR], i = 0;
	
	sscanf(buf, "%u:%u", &temp[i], &temp[i+1]); 
	for(;i < CLUSTER_NR;i++) {
		tunable = per_cpu(tunable_nr, cpu_nr[i]);
		store_fn(temp[i], 
			&tunable->throttle_temperature, cpu_nr[i]);
	}

 	return count;
}

static ssize_t temperature_diff_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	struct cluster_tunables *tunable;
	unsigned int temp[CLUSTER_NR], i = 0;
	
	PDEBUG("%s:SHOW",__func__);
	while(i < CLUSTER_NR) {
		tunable = per_cpu(tunable_nr, cpu_nr[i]); 
		temp[i++] = tunable->temperature_diff;
	}
	i = 0;
	return sprintf(buf, "%u:%u\n",temp[i] ,temp[i+1]);
}

static ssize_t temperature_diff_store(struct kobject *kobj, struct kobj_attribute *attr,
			      const char *buf, size_t count)
{
	struct cluster_tunables *tunable;
	int temp[CLUSTER_NR], i = 0;
	
	sscanf(buf, "%u:%u", &temp[i], &temp[i+1]); 
	for(;i < CLUSTER_NR;i++) {
		tunable = per_cpu(tunable_nr, cpu_nr[i]);
		store_fn(temp[i], 
			&tunable->temperature_diff, cpu_nr[i]);
	}

 	return count;
}

static ssize_t max_throttle_step_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	struct cluster_tunables *tunable;
	unsigned int temp[CLUSTER_NR], i = 0;
	
	PDEBUG("%s:SHOW",__func__);
	while(i < CLUSTER_NR) {
		tunable = per_cpu(tunable_nr, cpu_nr[i]); 
		temp[i++] = tunable->max_throttle_step;
	}
	i = 0;
	return sprintf(buf, "%u:%u\n",temp[i] ,temp[i+1]);
}

static ssize_t max_throttle_step_store(struct kobject *kobj, struct kobj_attribute *attr,
			      const char *buf, size_t count)
{
	struct cluster_tunables *tunable;
	unsigned int temp[CLUSTER_NR], i = 0;
	
	sscanf(buf, "%u:%u", &temp[i], &temp[i+1]); 
	for(;i < CLUSTER_NR;i++) {
		tunable = per_cpu(tunable_nr, cpu_nr[i]);
		store_fn(temp[i], 
			&tunable->max_throttle_step, cpu_nr[i]);
	}

 	return count;
}

therm_attr(throttle_temperature);
therm_attr(temperature_diff);
therm_attr(max_throttle_step);

static struct attribute *tunables_attr[] = {
	&throttle_temperature_attr.attr,
	&temperature_diff_attr.attr,
	&max_throttle_step_attr.attr,
	NULL
};

static struct attribute_group tunable_grp = {
	.attrs = tunables_attr,
};


/*********************************** sysfs end ***********************************/
/**********************************************************************************/

/*
 * start_setup() setups the governor.
 * setup up the frequencytable and all related operations as its the core part, any
 * failure here the governor may behave unpredictably so redundancies have 
 * been added to prevent that.
 */
int start_setup(struct cpufreq_policy *policy)
{
	int err = 0;
	
	PDEBUG("Starting Setup for cpu:%d",policy->cpu);
	err = get_cpufreq_table(policy);
	if(err){
		pr_err(KERN_WARNING "%s: Failed to setup governor.\n", __func__);
		return 0;
	}
	/* Cluster set initial frequency mitigation settings and parameters 
	 * before handing over to speedchange_task() thread for rest of cpu governing tasks.
	 */
	cfe_reset_params(policy);
	
	return 1;
}

static void postinit(unsigned int event)
{
	struct cpufreq_policy *policy;
	int temp, i = 0;
	
	/* check for state of thread if its already running then setup
	   is already complete, so don't run this again.*/
	if((event == START) && !run_state){
		while(i < CLUSTER_NR){
				policy = cpufreq_cpu_get(cpu_nr[i++]);
				if(!policy)
					goto err;
				temp += start_setup(policy);
		}
		if(temp != CLUSTER_NR)
			goto err;
		if (sysfs_create_group(endurance_kobj, &tunable_grp)) {
			pr_err(KERN_WARNING "%s:Failed to create sysfs files\n", __func__);
			goto err;
		}
		/* Register notification interfaces */
		register_pm_notifier(&therm_sleep_notify);
		blocking_notifier_chain_register(&limits_notifier_head,
							&limits_chg_notify);
		run_state = 1;
		PDEBUG("Finished setup\n");
		wake_up_process(thermal_mon_task);
	}
	else if(run_state) {	// Run teardown only if thermal mon is running
		mutex_lock(&rw_lock);
		run_state = 0;	// Disable the running thread first before anything
		unregister_pm_notifier(&therm_sleep_notify);
		blocking_notifier_chain_unregister(&limits_notifier_head,
					&limits_chg_notify);
		sysfs_remove_group(endurance_kobj, &tunable_grp);
		mutex_unlock(&rw_lock);
	}
	return;
err:
	pr_err(KERN_WARNING "%s: Failed to setup\n",__func__);
}

static int __init endurance_init(void)
{
	int rc = 0;
	
	struct sched_param param = { .sched_priority = MAX_RT_PRIO-2 };
	printk("Init endurnace....");
	
	mutex_init(&rw_lock);
	/* Create sysyfs folder under/kernel */
	endurance_kobj = kobject_create_and_add("endurance", kernel_kobj) ;
	if (!endurance_kobj) {
		pr_err(KERN_WARNING "%s: endurance create_and_add failed\n", __func__);
		return 0;
	}
	/* Create sysyfs file for enabling driver */
	rc = sysfs_create_file(endurance_kobj, &mod_state.attr);
	if (rc) {
		pr_err(KERN_WARNING "%s: sysfs_create_file failed for sweep2wake\n", __func__);
	}
	/* init thermal mitigtaion work queue */
	therm_wq = alloc_workqueue("therm_task", WQ_UNBOUND, 1);
	if (!therm_wq) {
		pr_err("Failed to create workqueue\n");
		return -ENOMEM;
	}
	INIT_WORK(&therm_work, govern_callback_task);
	/* init thermal poll task */
	thermal_mon_task =
			kthread_create(thermal_poll, NULL, "thermendurance");
	if (IS_ERR(thermal_mon_task))
		return PTR_ERR(thermal_mon_task);

	sched_setscheduler_nocheck(thermal_mon_task, SCHED_FIFO, &param);
	get_task_struct(thermal_mon_task);
	/* NB: wake up so the thread does not look hung to the freezer */
	wake_up_process(thermal_mon_task);

	return 0;
}

static void __exit endurance_exit(void)
{
	struct cluster_prop *cluster;
	struct cluster_tunables *tunable;
	unsigned int cpu;
	
	/* seize thermal monitor first */
	postinit(STOP);
	kthread_stop(thermal_mon_task);
	put_task_struct(thermal_mon_task);
	destroy_workqueue(therm_wq);
	/* freeup all allocated memory */
	for_each_possible_cpu(cpu){
		cluster = per_cpu(cluster_nr,cpu);
		tunable = per_cpu(tunable_nr, cpu);
		if(cluster)
			kfree(cluster);
		if(tunable)
			kfree(tunable);
	}
	kfree(thermal_monitor);
}

MODULE_AUTHOR("Tejas Udupa <tejasudupa1285@gmail>");
MODULE_DESCRIPTION("");
MODULE_LICENSE("GPL");

module_init(endurance_init);
module_exit(endurance_exit);
