# Cpufreq Governor Endurance

- Based on performance governor.
- This governor aims at solving the slow HAL reaction to throttle or completely replace it in case its broken.
- Governor mitigates the frequency based on the temperature of the current sensor, once the throttle points have been reached
  the frequency steps down by 1 level for every specified degree rise in sesnsor temperature.
- This a experimental work in progess project and a fun way of learning basics of cpufreq governor :)
