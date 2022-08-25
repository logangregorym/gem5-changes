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


if [ "$#" -eq 1 ]; then
    base_type=$1
else
    echo "Usage: ./check_all_parsec.sh <type>"
    echo "Ex:    ./check_all_parsec.sh logan_raw_reg"
    exit
fi

unique_id=$$
#compare_type="logan_super_skylake_f59_use_stale"
#base_type="logan_raw_skylake_f59"
find_str="system.cpu.commit.op_class_0::MemWrite *"
run_dir=$(pwd)

run_search () {
        base_filename=$(echo m5out_sim_*_"$base_type"/stats.txt)
        if [[ ! -f "$base_filename" ]]; then
            ls $PWD/*_${base_type}.slurm
 #           echo failed
            return
        fi

        base_out=$(grep -oE "$find_str"[0-9]+ m5out_sim_*_"$base_type"/stats.txt)
        base_out=$(echo "${base_out/system.switch_cpus.commit.op_class_0::MemWrite/}")
        if [ -z "$base_out" ]; then
            ls $PWD/*_${base_type}.slurm
 #           echo failed
 #       else
 #           echo passed
        fi
        #echo $base_out
}

echo "The following scripts need to be rerun:"



cd "/p/csd/parsec-port/pkgs/apps/vips/run/" ;
run_search

cd "/p/csd/parsec-port/pkgs/apps/freqmine/run/" ;
run_search

cd "/p/csd/parsec-port/pkgs/apps/x264/run/" ;
run_search

cd "/p/csd/parsec-port/pkgs/apps/swaptions/run/" ;
run_search


cd "/p/csd/parsec-port/pkgs/kernels/dedup/run/" ;
run_search

cd "/p/csd/parsec-port/pkgs/apps/blackscholes/run/" ;
run_search


cd "/p/csd/parsec-port/pkgs/apps/ferret/run/" ;
run_search

cd "/p/csd/parsec-port/pkgs/kernels/streamcluster/run/" ;
run_search


