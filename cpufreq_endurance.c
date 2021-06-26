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
static bool kthread_awake = false;

static DEFINE_PER_CPU(struct cluster_prop *, cluster_nr);

/* realtime thread handles frequency scaling */
static struct task_struct *speedchange_task;
static spinlock_t speedchange_cpumask_lock;
static struct mutex gov_lock;

/*		
 * get_cpufreq_table() initialises little and big core frequency tables.
 * @buf: a temporary buffer used to get frequncy table 
 */
int get_cpufreq_table(struct cpufreq_policy *policy){
	
	struct cluster_prop *cluster;
	int ret = 0,temp,i;
	
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
	
	/* Cluster set initial frequency mitigation state before handover to
	 * speedchange_task() thread for rest of cpu governing tasks.
	 */
	do_cpufreq_mitigation(policy,cluster,RESET);
	
	if(cluster->cur_temps >= cluster->throt_temps){
		temp = cluster->cur_temps - cluster->throt_temps;
		if(temp)
			temp = temp / 2;
		PDEBUG("go down levels by:%d",temp);
		for(i = 0; i <= temp; i++)
			do_cpufreq_mitigation(policy, cluster, THROTTLE_DOWN);
	}
	
	// Debugging functions
	if(debug){
		for(i = 0; i <= cluster->nr_levels; i++)
			PDEBUG("%u ",cluster->freq_table[i].frequency);
		PDEBUG("\n");
	}

setup_done:
	cluster->max_freq = cluster->prev_freq = policy->max;
	cluster->governor_enabled = true;
	PDEBUG("governor state:%d",cluster->governor_enabled);
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
	unsigned int freqmax;
	int i,index;
	
	freqmax = UINT_MAX;

	/* Get Highest Frequency Index in array */
	freq_table = cpufreq_frequency_get_table(policy->cpu);
	cpufreq_frequency_table_target(policy,freq_table,freqmax,
				CPUFREQ_RELATION_H,&index);
	PDEBUG("Index:%d",index);

	/* Initialise the cluster_prop structure. */
	if(!cluster){
		cluster = kzalloc(sizeof(struct cluster_prop), 
						GFP_KERNEL);
		if(!cluster)
			return -ENOMEM;

		memset(cluster, 0, sizeof(struct cluster_prop));
	}
	
	cluster->freq_table = freq_table;
	cluster->nr_levels = index;
	cluster->ppol = policy;
	
	/* assign cluster -> cluster_nr for each avilable core in that cluster */
	for_each_cpu(i, policy->related_cpus)
		per_cpu(cluster_nr,i) = cluster;
		
	return 0;
}

/*
 * get_sensor_dat() get temperature reading from the desired sensor.
 * in case of failure to get current temperature from sensor it returns true value.
 */
static inline int get_sensor_dat(struct cluster_prop *cluster)
{
	int ret = 0;
	
	ret = sensor_get_temp(SENSOR_ID,&cluster->cur_temps);
	if(ret)
		goto fail;
	if((cluster->cur_temps > 100) || (cluster->cur_temps < 0))
		goto fail;

	return 0;
fail:
	pr_err(KERN_WARNING"%s: Failed to get sensor: %d temprature data.\n", __func__, SENSOR_ID);
	cluster->cur_temps = (long)NULL; // give null to disable temperature based cpu governing
	return 1;	
}

/*	
 * set_temps() records current temperature for the current cluster
 * sets the throttle temperature for available clusters and returns true value in case of
 * failure to get the sensor temperature.
 */
int set_temps(struct cluster_prop *cluster, unsigned int cpu)
{
	int ret = 0;

	if(cluster == NULL){
		pr_err(KERN_WARNING"%s: core: %d table hasn't been initialised or has failed.\n", __func__, cpu);
		goto failed_unalloc;
	}
	else{
		ret = get_sensor_dat(cluster);
		if(ret)
			goto failed_sens;

		cluster->prev_temps = cluster->cur_temps;
		
		/* Initialise throttle temperature of big and little cluster */
		if(cpu <= NR_LITTLE)
			cluster->throt_temps = THROTTLE_TEMP_LITTLE;
		else if(cpu >= NR_BIG)
			cluster->throt_temps = THROTTLE_TEMP_BIG;
	}
	return 0;

failed_sens:
	pr_err(KERN_WARNING"%s: Failed to get sensor readings aborting operation.\n", __func__);
failed_unalloc:
	return 1;
}

/*	
 * govern_cpu() calls for cpufreq mitigation based on temperature inputs from the respective sensor.
 * this function only sends signal to do_cpufreq_mitigation() and doesn't edit/modify cpufreq_prop
 * structure. incase of failure in returning temperature it returns true value.
 */
static int govern_cpu(struct cluster_prop *cluster)
{
	struct cpufreq_policy *policy = cluster->ppol;
	int ret = 0, i, temp;
	
	/* get current temperature sensor readings.	*/
	ret = get_sensor_dat(cluster);
	if(ret)
		goto failed;
	
	/* we have reached max throttle frequency and going lower will only make the device sluggish so maintain frequency */
	if((cluster->cur_level == min_step) && (cluster->cur_temps >= cluster->prev_temps))
		goto end;

	/* Update current frequency if changed explicitly by user or other programs */
	if(cluster->max_freq != policy->max){
			PDEBUG("Prev max freq:%u New max freq:%u",cluster->max_freq,policy->max);
			do_cpufreq_mitigation(policy, cluster, UPDATE);
	}
		
	/* either we have not yet reached our cluster throttle temps or we dropped below throttle temps, 
	   so reset cluster levels and push max frequency of that cluster */
	if(cluster->cur_temps < cluster->throt_temps){
		PDEBUG("Currrent temps lower than throttle temps");
		if((cluster->prev_temps < cluster->throt_temps) && (cluster->cur_level == cluster->nr_levels))
			goto end;
		else
			do_cpufreq_mitigation(policy, cluster, RESET);
	}

	/* we have same or higher temps as that of throttle temps for that respective cluster, so we compare for five cases
	   1.temps have just reached throttle points
	   2.temps have gone past throttle points
	   3.temps have started drop to downward slope 
	   4-5.temps reported are inconsistent */
	else if(cluster->cur_temps >= cluster->throt_temps){
		PDEBUG("Current temps higher than throttle temps");
		if((cluster->cur_temps == cluster->throt_temps) && (cluster->cur_temps > cluster->prev_temps))
			do_cpufreq_mitigation(policy, cluster, THROTTLE_DOWN);
	
		/* temps have gone past throttle points and now frequency of respective cluster is being mitigated down 
	   	   through multiple levels depending on how much the temps have gone up since last recorded. */
		else if(cluster->cur_temps == (cluster->prev_temps + temp_diff)){
			PDEBUG("current temps higher than previous");
			do_cpufreq_mitigation(policy, cluster, THROTTLE_DOWN);
		}
		
		/* temps have started to drop either due to low avg load or idleing of the cluster,
	   	   so start throttling the core up by one level as the temps drop. */
		else if((cluster->cur_temps + temp_diff) == cluster->prev_temps){
			PDEBUG("current temps lower than previous");
			do_cpufreq_mitigation(policy, cluster, THROTTLE_UP);
		}
			
		/* get into these loops when there are wrong sensor reports like sudden spikes in temperature reported & other 
		   undesired scenarios causing cpu-frequency to be throttled inaccurately. But we would want to avoid this
		   situation as it consumes cpu cycles and is recommended to choose a sensor that is stable.  */

		else if(cluster->cur_temps > (cluster->prev_temps + temp_diff)){
			temp = (cluster->cur_temps - cluster->prev_temps) / 2;
			PDEBUG("current temps dont match previous DOWN max_dif:%d",temp);
			for(i = 0; i<=temp ;i++)
				do_cpufreq_mitigation(policy, cluster, THROTTLE_DOWN);
		}
		
		else if(cluster->prev_temps > (cluster->cur_temps + temp_diff)){
			temp = (cluster->prev_temps - cluster->cur_temps) / 2;
			PDEBUG("current temps dont match previous UP max_dif:%d",temp);
			for(i = 0; i<=temp; i++)
				do_cpufreq_mitigation(policy, cluster, THROTTLE_UP);
		}		
			
	}
	
	return 0;

end:
	PDEBUG("no cpufreq changes required");
	return 0;

failed:
	pr_err(KERN_WARNING"%s: Failed to get sensor readings aborting operation.\n", __func__);
	return 1;
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
		case NO_CHANGE:
			PDEBUG("NO_CHANGE");
			goto end;
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
	PDEBUG("cur_temps:%ld throt_temps:%d prev_temps:%ld",cluster->cur_temps,cluster->throt_temps,
					cluster->prev_temps);
	cluster->prev_temps = cluster->cur_temps;

update:
	cluster->max_freq = policy->max;
	PDEBUG("THROTTLE to %u from %u level:%d max_lvl:%d cpu:%d",cluster->freq_table[cluster->cur_level].frequency,
			policy->max,cluster->cur_level,cluster->nr_levels, policy->cpu);
	policy->max = cluster->freq_table[cluster->cur_level].frequency;
	cluster->prev_freq = policy->max;
	__cpufreq_driver_target(policy, policy->max,
						CPUFREQ_RELATION_H);

end:
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
	kthread_awake = true;
	
	PDEBUG("CFEndurance Running wake:%d",kthread_awake);
	while (!kthread_should_stop()) {
		set_current_state(TASK_RUNNING);

		for_each_possible_cpu(cpu){
			if((cpu == NR_LITTLE) || (cpu == NR_BIG)){
				cluster = per_cpu(cluster_nr, cpu);
				if(cluster && cluster->governor_enabled){
					mutex_lock(&gov_lock);
					govern_cpu(cluster);
					mutex_unlock(&gov_lock);
				}
				else
					gov_down++;
			}
		}

		/* both clusters have disabled endurance governor */
		if(gov_down == CLUSTER_NR)
			goto done;
		gov_down = 0;
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(nap_time_ms));
	}
	
	return 0;
done:
	kthread_awake = false;
	PDEBUG("Offlining Endurance Governor wake:%d",kthread_awake);
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
	mutex_lock(&gov_lock);
	err = get_cpufreq_table(policy);
	if(err)
		goto error;
	
	mutex_unlock(&gov_lock);
	PDEBUG("Finished setup wake:%d",kthread_awake);
	
	/* setup kthread for endurance governing skip is it has already been setup */
	if(kthread_awake == false){
		speedchange_task =
			kthread_create(cpufreq_endurance_speedchange_task, NULL,
				       "cfendurance");
		if (IS_ERR(speedchange_task))
			return PTR_ERR(speedchange_task);

		sched_setscheduler_nocheck(speedchange_task, SCHED_FIFO, &param);
		get_task_struct(speedchange_task);
		/* NB: wake up so the thread does not look hung to the freezer */
		wake_up_process(speedchange_task);
		PDEBUG("Run kthread");
	}
	
	return 0;
error:
	pr_err(KERN_INFO"%s: Failed to setup governor.\n", __func__);
	return 1;
}

static int cpufreq_governor_endurance(struct cpufreq_policy *policy,
					unsigned int event)
{
	struct cluster_prop *cluster;
	switch (event) {
	case CPUFREQ_GOV_START:		
		init_failed = start_gov_setup(policy);
	case CPUFREQ_GOV_LIMITS:
		if(init_failed)
			__cpufreq_driver_target(policy, policy->max,
						CPUFREQ_RELATION_H);
		break;
	case CPUFREQ_GOV_STOP:
		mutex_lock(&gov_lock);
		cluster = per_cpu(cluster_nr,policy->cpu);
		cluster->governor_enabled = false;
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
	mutex_init(&gov_lock);
	spin_lock_init(&speedchange_cpumask_lock);
	return cpufreq_register_governor(&cpufreq_gov_endurance);
}

static void __exit cpufreq_gov_endurance_exit(void)
{
	kthread_stop(speedchange_task);
	put_task_struct(speedchange_task);
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
