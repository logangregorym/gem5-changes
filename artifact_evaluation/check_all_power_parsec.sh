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




find_str="system.switch_cpus.numCycles *"
if [ "$#" -eq 2 ]; then
    compare_type=$2
    base_type=$1
else
    echo "Usage: ./check_all_power.sh <base_type> <compare_type>"
    echo "Ex:    ./check_all_power.sh logan_raw_skylake logan_super_skylake"
    exit
fi

run_search () {
        rm -rf m5out
        mkdir m5out

        cp m5out_sim_*_"$compare_type"/config.ini m5out
        cp m5out_sim_*_"$compare_type"/stats.txt m5out
        sed -i 's/switch_cpus/cpu/g' m5out/stats.txt
        /p/csd/mcpat/mcpat-parse-se-parsec.py
        /p/csd/mcpat/mcpat -infile power.xml -print_level 5 >power.txt

        compare_power=$(grep -m 1 "Runtime Dynamic" power.txt)
        compare_power=$(echo " ${compare_power/Runtime Dynamic = /}" |xargs)
        compare_power=$(echo " ${compare_power/W/}" |xargs)

        compare_intRF_power=$(grep -A 5 "Integer RF" power.txt |tail -n 1)
        compare_intRF_power=$(echo " ${compare_intRF_power/Runtime Dynamic = /}" |xargs)
        compare_intRF_power=$(echo " ${compare_intRF_power/W/}" |xargs)

        compare_rename_power=$(grep -A 5 "Renaming Unit" power.txt |tail -n 1)
        compare_rename_power=$(echo " ${compare_rename_power/Runtime Dynamic = /}" |xargs)
        compare_rename_power=$(echo " ${compare_rename_power/W/}" |xargs)

        #mv power.txt compare_power.txt

        rm power.txt power.xml
        rm -rf m5out
        mkdir m5out

        cp m5out_sim_*_"$base_type"/config.ini m5out
        cp m5out_sim_*_"$base_type"/stats.txt m5out
        sed -i 's/switch_cpus/cpu/g' m5out/stats.txt
        /p/csd/mcpat/mcpat-parse-se-parsec.py
        /p/csd/mcpat/mcpat -infile power.xml -print_level 5 >power.txt

        base_power=$(grep -m 1 "Runtime Dynamic" power.txt)
        base_power=$(echo " ${base_power/Runtime Dynamic = /}" |xargs)
        base_power=$(echo " ${base_power/W/}" |xargs)

        base_intRF_power=$(grep -A 5 "Integer RF" power.txt |tail -n 1)
        base_intRF_power=$(echo " ${base_intRF_power/Runtime Dynamic = /}" |xargs)
        base_intRF_power=$(echo " ${base_intRF_power/W/}" |xargs)

        base_rename_power=$(grep -A 5 "Renaming Unit" power.txt |tail -n 1)
        base_rename_power=$(echo " ${base_rename_power/Runtime Dynamic = /}" |xargs)
        base_rename_power=$(echo " ${base_rename_power/W/}" |xargs)

        echo "${compare_power} ${compare_intRF_power} ${compare_rename_power} ${base_power} ${base_intRF_power} ${base_rename_power}"

        rm power.txt power.xml
        rm -rf m5out
}

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


