/*
 *  linux/drivers/cpufreq/cpufreq_endurance.c
 *
 *  Copyright (C) 2002 - 2003 Dominik Brodowski <linux@brodo.de>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/cpufreq.h>
#include <linux/thermal.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include "cpufreq_endurance.h"

unsigned int nap_time_ms = 1500;			// Governor sleep Timeout in millisecond
static unsigned short int min_step = 5;		// Max throttle step limit
static unsigned short int temp_diff = 2;		// Temperature Diffrence
/* governor status check variables */
static bool kthread_sleep = 0;
unsigned int governor_enabled = 0;
static bool setup_complete = 0;

ATOMIC_NOTIFIER_HEAD(therm_alert_notifier_head);
static DEFINE_PER_CPU(struct cluster_prop *, cluster_nr);
static struct sensor_monitor *therm_monitor;

/* realtime thread handles frequency scaling */
static struct task_struct *speedchange_task;
static spinlock_t speedchange_cpumask_lock;
static struct mutex gov_lock;
static struct mutex speedchange_lock;

/*		
 * get_cpufreq_table() initialises little and big core frequency tables.
 * does all reset/init process and any failure during this process leads
 * to governor disabling and not working for the failed cluster. 
 */
int get_cpufreq_table(struct cpufreq_policy *policy){
	
	struct cluster_prop *cluster;
	int ret = 0,i;
	
	/* If structure already initialized exit out */
	cluster = per_cpu(cluster_nr,policy->cpu);
	if(cluster)
		goto setup_done;
	
	PDEBUG("%s: starting frequency table init of core:%d", __func__,policy->cpu);
	
	/* Start cpu frequency table initialization */
	ret = init_cpufreq_table(policy);
	if(ret < 0)
		goto failed_inittbl;
		
	cluster = per_cpu(cluster_nr, policy->cpu);
	if(!cluster)
		goto failed_inittbl;
	if(!therm_monitor)
		goto failed_inittbl;
	
	/* Initialise throttle temperature of big and little cluster */
	if(policy->cpu <= NR_LITTLE)
		cluster->throt_temps = THROTTLE_TEMP_LITTLE;
	else if(policy->cpu >= NR_BIG)
		cluster->throt_temps = THROTTLE_TEMP_BIG;
		
	/* temprature initialization */
	ret = update_sensor_data();
	if(ret)
		goto failed_gettbl;
	
	setup_complete = 1;
	
	// Debugging functions
	if(cfe_debug){
		for(i = 0; i <= cluster->nr_levels; i++)
			PDEBUG("%u ",cluster->freq_table[i].frequency);
		PDEBUG("\n");
	}
	
setup_done:
	/* reassign even if already inited as the address changes after resume/suspend */
	cluster->ppol = policy;	
	/* Cluster set initial frequency mitigation settings and parameters 
	 * before handing over to speedchange_task() thread for rest of cpu governing tasks.
	 */
	cfe_reset_params(policy);
	
	governor_enabled++;
	PDEBUG("governor state:%d",governor_enabled);
	return 0;

failed_inittbl:
	pr_err(KERN_WARNING"%s: Failed to initialise cpufreq table for core:%d\terr=%d", __func__,policy->cpu,ret);
failed_gettbl:
	return 1;
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
	if(!therm_monitor){
		therm_monitor = kzalloc(sizeof(struct sensor_monitor), 
						GFP_KERNEL);
		if(!therm_monitor)
			return -ENOMEM;

		memset(therm_monitor, 0, sizeof(struct sensor_monitor));
	}
	
	/* assign cluster -> cluster_nr for each avilable core in that cluster */
	for_each_cpu(i, policy->related_cpus)
		per_cpu(cluster_nr,i) = cluster;
		
	cluster->freq_table = freq_table;
		
	return 0;
}

/*
 * cfe_reset_params() resets the cluster max_freq ,nr_levels and prev_temps
 * this helps us to do correct thermal mitigation and dynamically switch between
 * frequency and allows users to have a certain degree of control on the max frequency
 * which has direct impact on how the thermal mitigation proceeds.
 */
int cfe_reset_params(struct cpufreq_policy *policy)
{
	struct cluster_prop *cluster = per_cpu(cluster_nr, policy->cpu);
	int i,temp,index = 0;
		
	if(!therm_monitor || !cluster)
		goto skip;
		
	//PDEBUG("cfe reset");
	
	/* policy limits change gets called multiple times eventhough change
	 * happened only once so therefore add checks to see if the reset 
	 * has already been performed. 
	 */
	if(cluster->max_freq == policy->max)
		goto skip;
		
	PDEBUG("Prev freq:%u Cur Freq:%u New freq:%u",cluster->max_freq,policy->cur,policy->max);	
	/* using index navigate to the equavalent frequency as that of
	 * policy->max in our frequency table.
	 */
	while(cluster->freq_table[index].frequency < policy->max)
		index++;
	PDEBUG("Reseting Index to:%d",index);

	cluster->cur_level = cluster->nr_levels = index;
	cluster->max_freq = cluster->prev_freq = policy->max;
	therm_monitor->prev_temps = therm_monitor->cur_temps;
	therm_monitor->updated_temps = 0;
	
	/* check if throttle down is required if required then loop until it
	 * reaches its correct level else just update the frequency and set it
	 * to new requested frequency. 
	 */
	if(therm_monitor->cur_temps >= cluster->throt_temps){
		temp = therm_monitor->cur_temps - cluster->throt_temps;
		if(temp)
			temp = temp / 2;
		PDEBUG("go down by %d levels",temp);
		for(i = 0; i <= temp; i++)
			do_cpufreq_mitigation(policy, cluster, THROTTLE_DOWN);
	}
	else
		do_cpufreq_mitigation(policy, cluster, UPDATE);

	return 0;

skip:
	do_cpufreq_mitigation(policy, cluster, UPDATE);
	
	return 0;
}

/*
 * update_sensor_data() get temperature reading from the desired sensor.
 * in case of failure to get current temperature from sensor it sets cur_temps to 0.
 */
static inline int update_sensor_data(void)
{
	int ret = 0;
	
	ret = sensor_get_temp(SENSOR_ID,&therm_monitor->cur_temps);
	if(ret){
		pr_err(KERN_WARNING"%s: Failed to get sensor: %d temprature data.\n", __func__, SENSOR_ID);
		therm_monitor->cur_temps = 0; // give zero to disable temperature based cpu governing
		return 1;
	}
	
	return 0;	
}

/*	
 * govern_cpu() calls for cpufreq mitigation based on temperature inputs from the respective sensor.
 * this function only sends signal to do_cpufreq_mitigation() and doesn't edit/modify cpufreq_prop
 * structure. precaution must be taken while passing cluster_prop as uninitilized variable ppol can 
 * cause much pain(kernel panic!). 
 */
static int govern_cpu(struct cluster_prop *cluster)
{
	struct cpufreq_policy *policy = cluster->ppol;
	int cl_temp_diff = 0;
	int th_temp_diff = 0;
	int temp = 0;
	int level_diff = 0;
	int ret = 0;

	cl_temp_diff = therm_monitor->cur_temps - therm_monitor->prev_temps;
	th_temp_diff = therm_monitor->cur_temps - cluster->throt_temps;
	PDEBUG("cluster diff:%d throt diff:%d cpuid:%d",cl_temp_diff,th_temp_diff,policy->cpu);
		
	/* either we have not yet reached our cluster throttle temps or we dropped below
	 * throttle temps, so reset cluster levels and push max frequency of that cluster 
	 */
	if(th_temp_diff < 0){
		PDEBUG("Currrent temps lower than throttle temps");
		if(therm_monitor->prev_temps < cluster->throt_temps)
			goto end;
		else
			ret = do_cpufreq_mitigation(policy, cluster, RESET);
	}
	/* we have same or higher temps as that of throttle temps for that respective cluster,
	 * so we compare for five cases :-
	 * 1.temps have just reached throttle points
	 * 2.temps have gone past throttle points
	 * 3.temps have started drop to downward slope 
	 */
	else {
		temp = cl_temp_diff * -1;
		level_diff = cluster->nr_levels - cluster->cur_level;
		PDEBUG("Current temps higher than throttle temps");
		if(!th_temp_diff && (cl_temp_diff > 0)){
			PDEBUG("temps have reached trip point");
			ret = do_cpufreq_mitigation(policy, cluster, THROTTLE_DOWN);
		}
		
		/* temps have started to drop either due to low avg load or idleing of the cluster,
	   	 * so start throttling the core up by one level as the temps drop. 
	   	 */
		else if(temp >= temp_diff){
			PDEBUG("current temps lower than previous");
			ret = do_cpufreq_mitigation(policy, cluster, THROTTLE_UP);
		}
		
		/* we have reached max throttle frequency and going lower will only make 
		 * the device sluggish so maintain the frequency. 
		 */
		else if(level_diff == min_step)
			goto end;
	
		/* temps have gone past throttle points and now frequency of respective cluster
	   	 * is being mitigated down  through multiple levels depending on how much the
	   	 * temps have gone up since last recorded. 
	   	 */
		else if(cl_temp_diff >= temp_diff){
			PDEBUG("current temps higher than previous");
			ret = do_cpufreq_mitigation(policy, cluster, THROTTLE_DOWN);
		}		
	}
		
	return ret;

end:
	PDEBUG("no cpufreq changes required");
	return 0;
}

static int thermal_change_callback(struct notifier_block *nb, unsigned long val,
				void *data)
{
	unsigned int cpu = 0;
	unsigned int per_cpu_governor = governor_enabled;
	int ret = 0;
	
	/* sleep cfe thread during this process as we dont want the cur_temps updating 
	 * inbetween the process as this could result in frequency of clusters further
	 * away being reported as to not having enough threshold to mitigate frequency.
	 */
	kthread_sleep = 1;
	/* loop through one core in each cluster */
	for_each_possible_cpu(cpu){
		if(per_cpu_governor){
			if((cpu == NR_BIG) || (cpu == NR_LITTLE)){
				struct cluster_prop *cluster = per_cpu(cluster_nr, cpu);
				if(cluster)
					ret += govern_cpu(cluster);
				per_cpu_governor--;
			}
		}
	}
	/* record current temperature */
	if(ret)
		therm_monitor->prev_temps = therm_monitor->cur_temps;
	kthread_sleep = 0;
	return 0;
}

static struct notifier_block therm_notifier_block = {
	.notifier_call = thermal_change_callback,
};

/*
 * do_cpufreq_mitigation() depending on event signals from govern_cpu() it decides
 * the throttling direction & records the current temperature of the sensor.
 * it modifies the policy max frequency to the latest max as per the event signal
 * paseed by the calling function.
 */
static int do_cpufreq_mitigation(struct cpufreq_policy *policy, 
					struct cluster_prop *cluster, state_info event){
	
	switch(event){
		case RESET:	
			cluster->cur_level = cluster->nr_levels;
			PDEBUG("RESET");
			break;
		case THROTTLE_DOWN:
			if((cluster->cur_level > (cluster->cur_level - min_step))
							&& (cluster->cur_level != 0)){
				cluster->cur_level--;
				PDEBUG("THROTTLE_DOWN");
			}else
				printk("MAX_THROTTLE");
			break;
		case THROTTLE_UP:
			if(cluster->cur_level < cluster->nr_levels){
				cluster->cur_level++;
				PDEBUG("THROTTLE_UP");
			}break;
		case UPDATE:
			PDEBUG("UPDATE");
			goto update;
		default: return 0;
	}
	
	PDEBUG("cur_temps:%ld throt_temps:%d prev_temps:%ld",therm_monitor->cur_temps,
						cluster->throt_temps,therm_monitor->prev_temps);
	PDEBUG("THROTTLE to %u from %u level:%d max_lvl:%d cpu:%d",cluster->freq_table[cluster->cur_level].frequency,
			policy->cur,cluster->cur_level,cluster->nr_levels, policy->cpu);

update:
	policy->max = cluster->freq_table[cluster->cur_level].frequency;
	cluster->prev_freq = policy->max;
	__cpufreq_driver_target(policy, policy->max,
						CPUFREQ_RELATION_H);
	return 1;
}

/*
 * cpufreq_endurance_speedchange_task() calls the govern_cpu() function.
 * if this function fails te governor is essentially dead, locks have been put to mitigate
 * contention issues that may arise during execution.
 */
static int cfe_thermal_monitor_task(void *data){

	unsigned int cpu;
	unsigned int gov_down = 0;
	struct cluster_prop *cluster;
	
	PDEBUG("CFEndurance Running");
	while (!kthread_should_stop()) {
		set_current_state(TASK_RUNNING);
		if(!governor_enabled){
			set_current_state(TASK_INTERRUPTIBLE);
			schedule();
		}
		
		if(kthread_sleep)
			goto sleep;
			
		/* get updated thermal reading */
		ret = update_sensor_data();
		if(ret)
			goto sleep;
			
		if(therm_monitor->cur_temps != therm_monitor->updated_temps)
			atomic_notifier_call_chain(&therm_alert_notifier_head, 0,0);
			
		therm_monitor->updated_temps = therm_monitor->cur_temps;
		therm_monitor->updated_temps = therm_monitor->cur_temps;	
sleep:
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(nap_time_ms));
	}
	
	return 0;
}

/*
 * start_gov_setup() setups the governor.
 * setup up the frequencytable and all related operations as its the core part, any
 * failure here the governor may behave unpredictably so redundancies have 
 * been added to prevent that.
 */
void start_gov_setup(struct cpufreq_policy *policy)
{
	int err = 0;
	
	PDEBUG("Starting Governor for cpu:%d",policy->cpu);
	
	/* aquire lock so that we dont overwrite critical sections which could lead to wrong 
	 * data being taken in and improper working of governor
	 */
	err = get_cpufreq_table(policy);
	if(err)
		goto error;

	/* setup kthread for endurance governing skip is it has already been setup */
	if(governor_enabled == 1)
	{
		atomic_notifier_chain_register(&therm_alert_notifier_head,
								&therm_notifier_block);
		wake_up_process(speedchange_task);
		PDEBUG("Run kthread");
	}
	PDEBUG("Finished setup");
	return;
	
error:
	pr_err(KERN_INFO"%s: Failed to setup governor.\n", __func__);
}

/* 
 *cfe_cleanup() dealloctates all structures before exiting for each cluster
 */
static void cfe_cleanup(void)
{
	struct cluster_prop *cluster = NULL;
	unsigned int cpu;
	
	for_each_possible_cpu(cpu){
		cluster = per_cpu(cluster_nr,cpu);
		if(cluster)
			kfree(cluster);
	}
	kfree(therm_monitor);
}

static int cpufreq_governor_endurance(struct cpufreq_policy *policy,
					unsigned int event)
{
	struct cluster_prop *cluster;
	switch (event) {
	case CPUFREQ_GOV_START:
		mutex_lock(&gov_lock);		
		start_gov_setup(policy);
		mutex_unlock(&gov_lock);
		break;
	case CPUFREQ_GOV_LIMITS:
		mutex_lock(&speedchange_lock);
		cfe_reset_params(policy);
		mutex_unlock(&speedchange_lock);
		break;
	case CPUFREQ_GOV_STOP:
		mutex_lock(&gov_lock);
		cluster = per_cpu(cluster_nr,policy->cpu);
		governor_enabled--;
		if(!governor_enabled)
			atomic_notifier_chain_unregister(
						&therm_alert_notifier_head,
						&therm_notifier_block);
		mutex_unlock(&gov_lock);
		break;
	default:
		break;
	}
	return 0;
}

#ifdef CONFIG_CPU_FREQ_GOV_ENDURANCE_MODULE
static
#endif
struct cpufreq_governor cpufreq_gov_endurance = {
	.name		= "endurance",
	.governor	= cpufreq_governor_endurance,
	.owner		= THIS_MODULE,
};

static int __init cpufreq_gov_endurance_init(void)
{
	struct sched_param param = { .sched_priority = MAX_RT_PRIO-2 };
	
	mutex_init(&gov_lock);
	mutex_init(&speedchange_lock);
	spin_lock_init(&speedchange_cpumask_lock);
	
	speedchange_task =
			kthread_create(cfe_thermal_monitor_task, NULL,
				       "cfendurance");
	if (IS_ERR(speedchange_task))
		return PTR_ERR(speedchange_task);

	sched_setscheduler_nocheck(speedchange_task, SCHED_FIFO, &param);
	get_task_struct(speedchange_task);
	
	/* NB: wake up so the thread does not look hung to the freezer */
	wake_up_process(speedchange_task);
	
	return cpufreq_register_governor(&cpufreq_gov_endurance);
}

static void __exit cpufreq_gov_endurance_exit(void)
{
	kthread_stop(speedchange_task);
	put_task_struct(speedchange_task);
	cfe_cleanup();
	cpufreq_unregister_governor(&cpufreq_gov_endurance);
}

MODULE_AUTHOR("Tejas Udupa <tejasudupa1285@gmail>");
MODULE_DESCRIPTION("'cpufreq_endurance' - A governor that is based"
	"on 'performance' by Dominik Brodowski,"
	"pushes cpufreq to max avialable frequency now modified to"
	"push max frequency depending on temprature on the given sensor");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_ENDURANCE
fs_initcall(cpufreq_gov_endurance_init);
#else
module_init(cpufreq_gov_endurance_init);
#endif
module_exit(cpufreq_gov_endurance_exit);
