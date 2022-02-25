#ifndef _ENDURANCE_H_
#define _ENDURANCE_H_

#define CFE_DEBUG

#ifdef CFE_DEBUG
#define PDEBUG(fmt, args...)	 {					\
		printk( KERN_INFO "CFE: " fmt, ##args);		\
		printk("\n");						\
}
#else
#define PDEBUG(fmt, args...)
#endif

#define therm_attr(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = __stringify(_name),	\
		.mode = 0644,			\
	},					\
	.show	= _name##_show,			\
	.store	= _name##_store,		\
}


/* Individual device default configuration */
#define START	(0)
#define STOP	(1)
#define NR_LITTLE 3			// starting little cluster cpu id
#define NR_BIG	5			// starting big cluster cpu id
#define CLUSTER_NR 2			// Number of clusters device has
#define SENSOR_ID 6			// Sensor ID in themal zone
#define THROTTLE_TEMP_LITTLE 42	// Throttle temperature of little cluster
#define THROTTLE_TEMP_BIG 46		// Throttle temperature of big cluster
#define TEMP_DIFF_LITTLE 3		// Temprature diffrence after which core must be throttled
#define TEMP_DIFF_BIG 4
#define MAX_STEP_LITTLE 4		// Max steps the core should throttle down
#define MAX_STEP_BIG 5
static int cpu_nr[] = {NR_LITTLE, NR_BIG};

struct cluster_prop {
	unsigned short	nr_levels;		// Stores number of total levels
	unsigned short	throt_level;		// Stores current level of throttle
	//unsigned int	prev_freq;		// Holds memory of previous cpufreq
	unsigned int	max_freq;		// Holds memory of max cpufreq avilable at the time
	struct cpufreq_frequency_table *freq_table;	// Holds the Frequency table for the respective cluster
};

struct sensor_monitor {
	long int cur_temps;			// Present sensor readings in Celsius
	long int prev_temps;			// Last updated sensor readings in Celsius
};

struct cluster_tunables {
	unsigned short throttle_temperature;	// Throttle temperature of the respective cluster
	unsigned short temperature_diff;	// Temperature Diffrence
	unsigned short max_throttle_step;	// Max Steps to be throttled
};

/* Function prototypes */
int init_cpufreq_table(struct cpufreq_policy *);
void cfe_reset_params(struct cpufreq_policy *);
int init_tunables(struct cpufreq_policy *);
static inline int update_sensor_data(void);
static inline void do_cpufreq_mitigation(struct cpufreq_policy *, 
			struct cluster_prop *);
static void postinit(unsigned int);
static void govern_callback_task(struct work_struct *);
#endif
