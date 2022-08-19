#!/bin/bash

#only run these benchmarks
#1 3 4 5 6 7 9 10 11 12

# declare -a spec_bench_names=([1]="perl"    
# 							 [2]="gcc"       
# 							 [3]="mcf"       
# 							 [4]="namd"   
# 							 [5]="xalan" 
# 							 [6]="exchange"      
# 							 [7]="deepsjeng"  
# 							 [8]="leela"      
# 							 [9]="xz"         
# 							 [10]="lbm"       
# 							 [11]="nab"
# 							 [12]="wrf"
# 							 [13]="povray"
# 							 [14]="fotonik"     
# 							)

#if [ -z "$1" ]
#then
#    echo "Argument expected (unique identifier)"
#    echo "Using 1 as unique identifier"
#    unique_id=1
#else
#    unique_id=$1
#fi

if [ "$#" -eq 1 ]; then
    base_type=$1
else
    echo "Usage: ./check_all_simpoints.sh <type>"
    echo "Ex:    ./check_all_simpoints.sh logan_raw_reg"
    exit
fi

unique_id=$$
#compare_type="logan_super_skylake_f59_use_stale"
#base_type="logan_raw_skylake_f59"
find_str="system.switch_cpus.commit.op_class_0::MemWrite *"
run_dir=$(pwd)

#compare the diff of SuperOptSanityCheck output and mark as not divergent iff the only differences are at the end of the superoptomized run and there are less than 10 extra lines.

run_search () {
    for (( i=1; i<=$1; i++ )); do
        base_filename=$(echo m5out_sim_*_"$base_type"_"$i"/stats.txt)
        if [[ ! -f "$base_filename" ]]; then
            ls $PWD/*${i}_${base_type}.slurm
 #           echo failed
            continue
        fi

        base_out=$(grep -oE "$find_str"[0-9]+ m5out_sim_*_"$base_type"_"$i"/stats.txt)
        base_out=$(echo "${base_out/system.switch_cpus.commit.op_class_0::MemWrite/}")
        if [ -z "$base_out" ]; then
            ls $PWD/*${i}_${base_type}.slurm
 #           echo failed
 #       else
 #           echo passed
        fi
        #echo $base_out
    done
}

echo "The following scripts need to be rerun:"

#perl
#echo "Processing perlbench"
cd "/p/csd/SPEC2017/benchspec/CPU/600.perlbench_s/run/perl_run_ref/" ;
run_search 7

#mcf
#echo "Processing mcf"
cd "/p/csd/SPEC2017/benchspec/CPU/605.mcf_s/run/mcf_ren_ref/" ;
run_search 7

#xalan
#echo "Processing xalan"
cd "/p/csd/SPEC2017/benchspec/CPU/623.xalancbmk_s/run/xalan_run_ref/" ;
run_search 6

#exchange
#echo "Processing exchange"
cd "/p/csd/SPEC2017/benchspec/CPU/648.exchange2_s/run/exchange_run_ref/" ;
run_search 5

#deepsjeng
#echo "Processing deepsjeng"
cd "/p/csd/SPEC2017/benchspec/CPU/631.deepsjeng_s/run/deepsjeng_run_ref/" ;
run_search 3

#xz
#echo "Processing xz"
cd "/p/csd/SPEC2017/benchspec/CPU/657.xz_s/run/xz_run_ref/" ;
run_search 6

#lbm
#echo "Processing lbm"
cd "/p/csd/SPEC2017/benchspec/CPU/619.lbm_s/run/lbm_run_ref/" ;
run_search 5

#nab
#echo "Processing nab"
cd "/p/csd/SPEC2017/benchspec/CPU/644.nab_s/run/nab_run_ref/" ;
run_search 4

#wrf
#echo "Processing wrf"
cd "/p/csd/SPEC2017/benchspec/CPU/621.wrf_s/run/wrf_run_ref/" ;
run_search 6

#leela
#echo "Processing leela"
cd "/p/csd/SPEC2017/benchspec/CPU/641.leela_s/run/leela_run_ref/" ;
run_search 8

#gcc 
#echo "Processing gcc"
cd "/p/csd/SPEC2017/benchspec/CPU/602.gcc_s/run/gcc_run_ref/" ;
run_search 7

