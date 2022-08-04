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
    echo "Usage: ./check_all_cycles.sh <base_type> <compare_type>"
    echo "Ex:    ./check_all_cycles.sh logan_raw_skylake logan_super_skylake"
    exit
fi

run_search () {
    for (( i=1; i<=$1; i++ )); do
	>&2 echo "${i} super"
        rm -rf m5out
        mkdir m5out

        cp m5out_sim_*_"$compare_type"_"$i"/config.ini m5out
        cp m5out_sim_*_"$compare_type"_"$i"/stats.txt m5out
        if test -f "m5out/stats.txt"; then
	
            /p/csd/mcpat/mcpat-parse-se.py
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
        else
            compare_power=0
            compare_intRF_power=0
            compare_rename_power=0
        fi
        #mv power.txt compare_power.txt

        rm power.txt power.xml
        rm -rf m5out
        mkdir m5out

	>&2 echo "${i} raw"
        cp m5out_sim_*_"$base_type"_"$i"/config.ini m5out
        cp m5out_sim_*_"$base_type"_"$i"/stats.txt m5out
        if test -f "m5out/stats.txt"; then
            /p/csd/mcpat/mcpat-parse-se.py
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
        else
            base_power=0
            base_intRF_power=0
            base_rename_power=0
        fi
        echo "${compare_power} ${compare_intRF_power} ${compare_rename_power} ${base_power} ${base_intRF_power} ${base_rename_power}"

        rm power.txt power.xml
        rm -rf m5out
    done
}

#perl
>&2 echo "perl"
cd "/p/csd/SPEC2017/benchspec/CPU/600.perlbench_s/run/perl_run_ref/" ;
run_search 7

#mcf
>&2 echo "mcf"
cd "/p/csd/SPEC2017/benchspec/CPU/605.mcf_s/run/mcf_ren_ref/" ;
run_search 7

#xalan
>&2 echo "xalan"
cd "/p/csd/SPEC2017/benchspec/CPU/623.xalancbmk_s/run/xalan_run_ref/" ;
run_search 6

#exchange
>&2 echo "exchange"
cd "/p/csd/SPEC2017/benchspec/CPU/648.exchange2_s/run/exchange_run_ref/" ;
run_search 5

#deepsjeng
>&2 echo "deepsjeng"
cd "/p/csd/SPEC2017/benchspec/CPU/631.deepsjeng_s/run/deepsjeng_run_ref/" ;
run_search 3

#xz
>&2 echo "xz"
cd "/p/csd/SPEC2017/benchspec/CPU/657.xz_s/run/xz_run_ref/" ;
run_search 6

#lbm
>&2 echo "lbm"
cd "/p/csd/SPEC2017/benchspec/CPU/619.lbm_s/run/lbm_run_ref/" ;
run_search 5

#nab
>&2 echo "nab"
cd "/p/csd/SPEC2017/benchspec/CPU/644.nab_s/run/nab_run_ref/" ;
run_search 4

#wrf
>&2 echo "wrf"
cd "/p/csd/SPEC2017/benchspec/CPU/621.wrf_s/run/wrf_run_ref/" ;
run_search 6

#leela
>&2 echo "leela"
cd "/p/csd/SPEC2017/benchspec/CPU/641.leela_s/run/leela_run_ref/" ;
run_search 8

#gcc 
>&2 echo "gcc"
cd "/p/csd/SPEC2017/benchspec/CPU/602.gcc_s/run/gcc_run_ref/" ;
run_search 7

#omnetpp
#cd "/p/csd/SPEC2017/benchspec/CPU/620.omnetpp_s/run/omnetpp_run_ref/";
#run_search 1

