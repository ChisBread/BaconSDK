#!/usr/bin/sudo /bin/bash
echo performance > /sys/class/devfreq/fde60000.gpu/governor
echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor
echo performance > /sys/class/devfreq/dmc/governor