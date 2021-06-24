#ifndef _CPU_FREQ_ENDURANCE_H_
#define _CPU_FREQ_ENDURANCE_H_

static unsigned int debug = 1;

#define PDEBUG(fmt, args...)	 {					\
	if(debug != 0){						\
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

/* Main Function prototypes */
int get_cpufreq_table(struct cpufreq_policy *);
int init_cpufreq_table(struct cpufreq_policy *);
struct cluster_prop *get_cluster(unsigned short int);
static inline int get_sensor_dat(struct cluster_prop *);
int set_temps(struct cluster_prop *);
int do_cpufreq_mitigation(struct cpufreq_policy *, 
					struct cluster_prop *, state_info);
int start_gov_setup(struct cpufreq_policy *);

#endif
