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

static bool init_failed = 0;
unsigned int nap_time_ms = 1500;			// Governor sleep Timeout in millisecond
static unsigned short int min_step = 5;		// Max throttle step limit
static unsigned short int temp_diff = 2;		// Temperature Diffrence

static DEFINE_PER_CPU(struct cluster_prop *, cluster_nr);
static struct sensor_monitor *therm_monitor;
/* realtime thread handles frequency scaling */
static struct task_struct *speedchange_task;
static spinlock_t speedchange_cpumask_lock;
static struct mutex gov_lock;
static struct mutex speedchange_lock;

/*		
 * get_cpufreq_table() initialises little and big core frequency tables.
 * @buf: a temporary buffer used to get frequncy table 
 */
int get_cpufreq_table(struct cpufreq_policy *policy){
	
	struct cluster_prop *cluster;
	int ret = 0,i;
	
	mutex_lock(&gov_lock);
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
	
	/* Cluster temprature initialization */
	ret = set_temps(cluster, policy->cpu);
	if(ret)
		goto failed_gettbl;
	
	/* Cluster set initial frequency mitigation settings and parameters 
	 * before handing over to speedchange_task() thread for rest of cpu governing tasks.
	 */
	cfe_reset_params(policy, cluster);
	
	// Debugging functions
	if(cfe_debug){
		for(i = 0; i <= cluster->nr_levels; i++)
			PDEBUG("%u ",cluster->freq_table[i].frequency);
		PDEBUG("\n");
	}

setup_done:
	/* reassign as the address changes after resume/suspend */
	cluster->ppol = policy;
	governor_enabled++;
	PDEBUG("governor state:%d",cluster->governor_enabled);
	mutex_unlock(&gov_lock);
	return 0;

failed_inittbl:
	pr_err(KERN_WARNING"%s: Failed to initialise cpufreq table for core:%d\terr=%d", __func__,policy->cpu,ret);
failed_gettbl:
	mutex_unlock(&gov_lock);
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
	__cpufreq_driver_target(policy, policy->max,
						CPUFREQ_RELATION_H);
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
	if(ret)
		goto fail;

	return 0;
fail:
	pr_err(KERN_WARNING"%s: Failed to get sensor: %d temprature data.\n", __func__, SENSOR_ID);
	therm_monitor->cur_temps = 0; // give zero to disable temperature based cpu governing
	return 1;	
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

	cl_temp_diff = therm_monitor->cur_temps - therm_monitor->prev_temps;
	th_temp_diff = cluster->throt_temps - therm_monitor->cur_temps;
	PDEBUG("cluster diff:%d throt diff:%d cpuid:%d",cl_temp_diff,th_temp_diff,policy->cpu);
	PDEBUG("cur_temps:%ld throt_temps:%d prev_temps:%ld",therm_monitor->cur_temps,cluster->throt_temps,
					therm_monitor->prev_temps);
	/* we have reached max throttle frequency and going lower will only make the device sluggish so maintain frequency */
	if(cluster->cur_level == min_step)
		goto end;

	/* Reset current/max frequency if changed explicitly by scripts, programs or if the governor was restarted */
	/*if(policy->max - cluster->max_freq){
			PDEBUG("Calling Reset cur:%d new:%d",cluster->max_freq,policy->max);
			cfe_reset_params(policy);
	}*/
		
	/* either we have not yet reached our cluster throttle temps or we dropped below throttle temps, 
	 * so reset cluster levels and push max frequency of that cluster 
	 */
	if(th_temp_diff > 0){
		PDEBUG("Currrent temps lower than throttle temps");
		if(cl_temp_diff >= 0)
			goto end;
		else
			do_cpufreq_mitigation(policy, cluster, RESET);
	}
	/* we have same or higher temps as that of throttle temps for that respective cluster, so we compare for five cases
	 * 1.temps have just reached throttle points
	 * 2.temps have gone past throttle points
	 * 3.temps have started drop to downward slope 
	 */
	else {
		PDEBUG("Current temps higher than throttle temps");
		if(!th_temp_diff && (cl_temp_diff > 0)){
			PDEBUG("temps have reached trip point");
			do_cpufreq_mitigation(policy, cluster, THROTTLE_DOWN);
		}
	
		/* temps have gone past throttle points and now frequency of respective cluster is being mitigated down 
	   	 * through multiple levels depending on how much the temps have gone up since last recorded. 
	   	 */
		else if(cl_temp_diff >= temp_diff){
			PDEBUG("current temps higher than previous");
			do_cpufreq_mitigation(policy, cluster, THROTTLE_DOWN);
		}
		
		/* temps have started to drop either due to low avg load or idleing of the cluster,
	   	 * so start throttling the core up by one level as the temps drop. 
	   	 */
		else if((cl_temp_diff * -1) >= temp_diff){
			PDEBUG("current temps lower than previous");
			do_cpufreq_mitigation(policy, cluster, THROTTLE_UP);
		}		
	}
	
	return 0;

end:
	PDEBUG("no cpufreq changes required");
	return 0;
}

/*
 * do_cpufreq_mitigation() depending on event signals from govern_cpu() it decides the throttling direction &
 * records the current temperature of the sensor. it modifies the policy max frequency to the latest max as
 * per the event signal recived by the function.
 */
int do_cpufreq_mitigation(struct cpufreq_policy *policy, 
					struct cluster_prop *cluster, state_info event){
	
	switch(event){
		case RESET:	
			cluster->cur_level = cluster->nr_levels;
			PDEBUG("RESET");
			break;
		case THROTTLE_DOWN:
			if((cluster->cur_level > (cluster->cur_level - min_step)) && (cluster->cur_level != 0)){
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
		default:
			break;
	}
	PDEBUG("therm_monitor:%ld throt_temps:%d prev_temps:%ld",therm_monitor->cur_temps,cluster->throt_temps,
					therm_monitor->prev_temps);
	therm_monitor->prev_temps = therm_monitor->cur_temps;

	PDEBUG("THROTTLE to %u from %u level:%d max_lvl:%d cpu:%d",cluster->freq_table[cluster->cur_level].frequency,
			policy->cur,cluster->cur_level,cluster->nr_levels, policy->cpu);

update:
	policy->cur = cluster->freq_table[cluster->cur_level].frequency;
	cluster->prev_freq = policy->cur;
	__cpufreq_driver_target(policy, policy->cur,
						CPUFREQ_RELATION_H);
	return 0;
}

/*
 * cpufreq_endurance_speedchange_task() calls the govern_cpu() function.
 * if this function fails te governor is essentially dead, locks have been put to mitigate
 * contention issues that may arise during execution.
 */
static int cpufreq_endurance_speedchange_task(void *data){

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
			
		for_each_possible_cpu(cpu){
			if((cpu == NR_LITTLE) || (cpu == NR_BIG)){
				cluster = per_cpu(cluster_nr, cpu);
				if(cluster){
					govern_cpu(cluster);
				}
			}
		}

		/* both clusters have disabled endurance governor */
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
int start_gov_setup(struct cpufreq_policy *policy)
{
	struct sched_param param = { .sched_priority = MAX_RT_PRIO-1 };
	int err = 0;
	
	PDEBUG("Starting Governor for cpu:%d",policy->cpu);
	
	/* aquire lock so that we dont overwrite critical sections which could lead to wrong 
		   data being taken in and improper working of governor */
	
	err = get_cpufreq_table(policy);
	if(err)
		goto error;
	
	/* setup kthread for endurance governing skip is it has already been setup */
	if(governor_enabled == 1){
		wake_up_process(speedchange_task);
		PDEBUG("Run kthread");
	}
	PDEBUG("Finished setup");
	
	return 0;
error:
	pr_err(KERN_INFO"%s: Failed to setup governor.\n", __func__);
	return 1;
}

/* 
 *cfe_cleanup() dealloctates all structures before exiting for each cluster
 */
static void cfe_cleanup(void){
	unsigned int cpu;
	struct cluster_prop *cluster = NULL;
	
	for_each_possible_cpu(cpu){
		cluster = per_cpu(cluster_nr,cpu);
		if(cluster)
			kfree(cluster);
	}
}

static int cpufreq_governor_endurance(struct cpufreq_policy *policy,
					unsigned int event)
{
	struct cluster_prop *cluster;
	switch (event) {
	case CPUFREQ_GOV_START:		
		init_failed = start_gov_setup(policy);
	case CPUFREQ_GOV_LIMITS:
		mutex_lock(&speedchange_lock);
		cfe_reset_params(policy);
		mutex_unlock(&speedchange_lock);
		break;
	case CPUFREQ_GOV_STOP:
		mutex_lock(&gov_lock);
		cluster = per_cpu(cluster_nr,policy->cpu);
		governor_enabled--;
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
