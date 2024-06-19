#!/bin/bash

#only run these benchmarks
#1 3 4 5 6 7 9 10 11 12

# declare -a spec_bench_names=([1]="perl"    
# 							 [2]="gcc"       
# 							 [3]="mcf"       
# 							 [4]="xalan" 
# 							 [5]="exchange"      
# 							 [6]="deepsjeng"  
# 							 [7]="leela"      
# 							 [8]="xz"         
# 							 [9]="lbm"       
# 							 [10]="nab"
# 							 [11]="wrf"
# 							)
#perl
cd "/p/csd/SPEC2017/benchspec/CPU/600.perlbench_s/run/perl_run_ref/" ;
./gem5_sim_run_all_perl_artifact_evaluation_super_yilong.sh

#exchange
cd "/p/csd/SPEC2017/benchspec/CPU/648.exchange2_s/run/exchange_run_ref/" ;
./gem5_sim_run_all_exchange_artifact_evaluation_super_yilong.sh

#sleep 60m

#mcf
cd "/p/csd/SPEC2017/benchspec/CPU/605.mcf_s/run/mcf_ren_ref/" ;
./gem5_sim_run_all_mcf_artifact_evaluation_super_yilong.sh
#sleep 60m
#deepsjeng
cd "/p/csd/SPEC2017/benchspec/CPU/631.deepsjeng_s/run/deepsjeng_run_ref/" ;
./gem5_sim_run_all_deepsjeng_artifact_evaluation_super_yilong.sh

#sleep 120m

#xalan
cd "/p/csd/SPEC2017/benchspec/CPU/623.xalancbmk_s/run/xalan_run_ref/" ;
./gem5_sim_run_all_xalan_artifact_evaluation_super_yilong.sh

#sleep 90m

#xz
cd "/p/csd/SPEC2017/benchspec/CPU/657.xz_s/run/xz_run_ref/" ;
./gem5_sim_run_all_xz_artifact_evaluation_super_yilong.sh

#lbm
cd "/p/csd/SPEC2017/benchspec/CPU/619.lbm_s/run/lbm_run_ref/" ;
./gem5_sim_run_all_lbm_artifact_evaluation_super_yilong.sh

#nab
cd "/p/csd/SPEC2017/benchspec/CPU/644.nab_s/run/nab_run_ref/" ;
./gem5_sim_run_all_nab_artifact_evaluation_super_yilong.sh

#wrf
cd "/p/csd/SPEC2017/benchspec/CPU/621.wrf_s/run/wrf_run_ref/" ;
./gem5_sim_run_all_wrf_artifact_evaluation_super_yilong.sh

#sleep 60m

#leela
cd "/p/csd/SPEC2017/benchspec/CPU/641.leela_s/run/leela_run_ref/" ;
./gem5_sim_run_all_leela_artifact_evaluation_super_yilong.sh

#gcc 
cd "/p/csd/SPEC2017/benchspec/CPU/602.gcc_s/run/gcc_run_ref/" ;
./gem5_sim_run_all_gcc_artifact_evaluation_super_yilong.sh


