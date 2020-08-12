#!/bin/bash
set -x
set -e

rundir_prefix=./behzad_trial_zones
for i in 1 2 3 4
do
    rundir=$rundir_prefix/$i
    commondir=$rundir_prefix/common
    mkdir -p $rundir/zone_boxplots
    mkdir -p $rundir/time_progression
    python3 zone_analysis_dynamic_static.py -d $rundir/stats.json -s $commondir/static_stats.json -e $commondir/EnvGenStats.json -r $rundir/zone_boxplots
    python3 plot_mission_metrics.py -d $rundir/stats.json -s $commondir/static_stats.json -r $rundir/
    python3 ../end_to_end_data/time_progression_dynamic_static.py -d $rundir/stats.json -s $commondir/static_stats.json -e $commondir/EnvGenStats.json -r $rundir/time_progression
done
