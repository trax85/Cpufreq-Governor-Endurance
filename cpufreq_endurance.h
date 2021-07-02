#ifndef _CPU_FREQ_ENDURANCE_H_
#define _CPU_FREQ_ENDURANCE_H_

static unsigned int cfe_debug = 1;

#define PDEBUG(fmt, args...)	 {					\
	if(cfe_debug != 0){						\
		printk( KERN_INFO "CFE: " fmt, ##args);	\
		printk("\n");						\
	}								\
}

/* Individual device configuration */
#define NR_LITTLE 3			// starting little cluster cpu id
#define NR_BIG	4			// starting big cluster cpu id
#define SENSOR_ID 5			// Sensor ID in themal zone
#define THROTTLE_TEMP_LITTLE 46	// Throttle temperature of little cluster
#define THROTTLE_TEMP_BIG 44		// Throttle temperature of big cluster
#define CLUSTER_NR 2

struct cluster_prop {
	unsigned short int throt_temps;	// Throttle temperature of the respective cluster
	unsigned short int nr_levels;		// Stores number of total levels
	unsigned short int cur_level;		// Stores current level of throttle
	unsigned int prev_freq;		// Holds memory of previous cpufreq
	unsigned int max_freq;			// Holds memory of max cpufreq avilable at the time
	struct cpufreq_policy *ppol;		// Points to the policy struct of the respective cluster
	struct cpufreq_frequency_table *freq_table;	// Holds the Frequency table for the respective cluster
};

struct sensor_monitor {
	long int cur_temps;			// Present sensor temperature in Celsius
	long int prev_temps;			// Previous sensor temperature in Celsius
};

typedef enum {
	NO_CHANGE = 0,
	THROTTLE_UP,
	THROTTLE_DOWN,
	RESET,
	UPDATE
}state_info;

/* Main Function prototypes */
int get_cpufreq_table(struct cpufreq_policy *);
int init_cpufreq_table(struct cpufreq_policy *);
void cfe_reset_params(struct cpufreq_policy *,struct cluster_prop *);
int do_cpufreq_mitigation(struct cpufreq_policy *, 
static inline int update_sensor_data(void);
int do_cpufreq_mitigation(struct cpufreq_policy *,
					struct cluster_prop *, state_info);
int start_gov_setup(struct cpufreq_policy *);

#endif
