# Cpufreq Governor Endurance

- This Governor is based on Performance and Interactive Governors.
- It aims to solve Broken HALs(unfixable) and gives more flexiblity in throttling the CPU cores.
- This is still work in progess(beta) governor and more features are still on the pipeline.
- See [link](https://github.com/trax85/Citrus-EAS/commit/773916b58b89400beb546751c5db6dfd68f76285) for integrating governor into kernel.
 
## Tunable Parameters
- `throttle_temperature` - This defines after what point the CPU must be throttled down (indicating the device is heating up).
- `temprature_diff` - This parameter defines at what temprature the governor must throttle up/down.
- `max_throttle_step` - This parameter defines the maximum steps of frequency for the respective cluster can be throttled down.
- `idle_frequency` - This paraemter defines the frequency to be used when the CPU is 'idle'.
- `idle_threshold` - This parameter defines 'idle' for the respective cluster. The threshold is average load of all cores in that cluster and if average is lower than defined threshold `idle_threshold` then the `idle_frequency` is applied to the CPU.

```
 Steps/Levels  |     Frequency(Mhz)           Governor Parameters
 --------------------------------------------------------------------
 0             |      400000            -----> idle_frequency (default)
 1             |      691200
 2             |      806400
 3             |      960000                                                __
 4             |      1017600           -----> tunable->max_throttle_step     |
 5             |      1190400                                                 |
 6             |      1305600                                                 |-> cluster>cur_level can move up/down withing this
 7             |      1382400           -----> cluster>cur_level              | 
 8             |      1401600           -----> cluster->nr_levels           __|
```
## Working
Using the above representation we will see how the governor works. The `cluster->nr_levels` is a fixed variable that doesn't change once governor is initilized.
`cluster>cur_level` holds the current frequency step the CPU is at, initially it's at highest step in the unthrottled state. The function `update_sensor_data()` gets the 
latest temperature reading for the defined sensor. This temprature(in Celsius) is used to throttle the cores. The temprature reading when it passes the Trip points 
defined by `throttle_temperature` the function `govern_cpu()` checks if the cpu must be throttled down. After the trip points are crossed the governor throttles down the 
frequency by 'one step' each time the temprature goes up by value defined by `temprature_diff`. `thermal_monitor->cur_temps` holds the current temprature for the defined 
sensor. The poll time for `govern_cpu` is 1500ms.

To explain this let us take an example using the above representation. `cluster->nr_level` = 8  ,`tunable->max_throttle_step` = 4 ,`throttle_temperature` = 40, 
`temprature_diff` = 2,`thermal_monitor->cur_temps` = 40. Using this, Initially the `cluster>cur_level` is equal to `cluster->nr_levels`. Now the 
`thermal_monitor->cur_temps` have increased to 40 which is also the Trip point(`throttle_temperature`) for the governor so the  `govern_cpu()` does the first throttle by 
decrementing `cluster>cur_level` by one, changing `cluster>cur_level` from 8 to 7 and the `cluster>cur_level` is applied to CPU, and then from here on out the governor 
polls for every rise equal to `temprature_diff` the governor throttle's 'down' CPU by decrementing `cluster>cur_level` by one, until it reaches `max_throttle_step` after 
which any temprature rise will have no affect on frequency of the CPU. Same is true if the CPU is already throttled and temperature is falling, it thottle's 'up' CPU for 
every drop equal to `temprature_diff`, until it reaches max level `cluster->nr_levels` at which point the CPU is no more Hot and temprature has dropped below 
`throttle_temperature`.

### Governor Idling
If the CPU idles during this time the `govern_cpu()` function is skipped until the CPU goes out of idle (only if all clusters of the CPU are in idle state). The polling 
is reduced to 500ms and only average load is checked until the CPU average load rises above `idle_threshold`. The `idle_frequency` is appled to the idling cluster and 
`idle_frequency` is lower than frequency of corresponding value of `max_throttle_step`. There is a bias implemented that checks if any core load is above 60% then the 
idling is skipped even if all other cores in the cluster are idle.

So was this worth my time and all the frustration? maybe not, it's a very niche problem but I had that and maybe could have done some band-aiding to fix that but this was 
fun and it's a fun way to learn about working of linux cpufreq governors :D
