#ifndef _CPU_FREQ_ENDURANCE_H_
#define _CPU_FREQ_ENDURANCE_H_

static unsigned int debug = 1;

#define PDEBUG(fmt, args...)	 {					\
	if(debug != 0){						\
		printk( KERN_INFO "CFE: " fmt, ##args);	\
		printk("\n");						\
	}								\
}

#endif
