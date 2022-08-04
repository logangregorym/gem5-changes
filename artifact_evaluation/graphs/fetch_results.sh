#!/bin/bash


spec_root="/p/csd/SPEC2017"
spec_run_root="${spec_root}/benchspec/CPU"




#this will change the scripts name and output folders
#REMEBER TO CHANGE ALL THE PARAMETERS THAT WE WANT TO CHNAGE FOR THIS SIMULATION
 


declare -a spec_num_chkpoints=([1]=7
							    [2]=7
							 	[3]=7
							 	[4]=6
							 	[5]=5
							 	[6]=3
							 	[7]=8
							 	[8]=6
							 	[9]=5
							 	[10]=4
								[11]=6
							  )

declare -a spec_bench_names=([1]="perl"    
							 [2]="gcc"       
							 [3]="mcf"       
							 [4]="xalan" 
							 [5]="exchange"      
							 [6]="deepsjeng"  
							 [7]="leela"      
							 [8]="xz"         
							 [9]="lbm"       
							 [10]="nab"
							 [11]="wrf"
							)

declare -a spec_bench_nums=( [1]=600
							 [2]=602
							 [3]=605
							 [4]=623
							 [5]=648
							 [6]=631
							 [7]=641
							 [8]=657
							 [9]=619
							 [10]=644
							 [11]=621
							)

declare -a spec_bench_dir=(  [1]="600.perlbench_s/run/perl_run_ref"
							 [2]="602.gcc_s/run/gcc_run_ref"
							 [3]="605.mcf_s/run/mcf_ren_ref"
							 [4]="623.xalancbmk_s/run/xalan_run_ref"
							 [5]="648.exchange2_s/run/exchange_run_ref"
							 [6]="631.deepsjeng_s/run/deepsjeng_run_ref"
							 [7]="641.leela_s/run/leela_run_ref"
							 [8]="657.xz_s/run/xz_run_ref"
							 [9]="619.lbm_s/run/lbm_run_ref"
							 [10]="644.nab_s/run/nab_run_ref"
							 [11]="621.wrf_s/run/wrf_run_ref"
							)

declare -a spec_bench_commands=([1]="perlbench_s_base.mytest-m64"
							 	[2]="sgcc_base.mytest-m64"
							 	[3]="mcf_s_base.mytest-m64"
							 	[4]="xalancbmk_s_base.mytest-m64"
							 	[5]="exchange2_s_peak.mytest-m64"
							 	[6]="deepsjeng_s_base.mytest-m64"
							 	[7]="leela_s_peak.mytest-m64"
							 	[8]="xz_s_base.mytest-m64"
							 	[9]="lbm_s_base.mytest-m64"
							 	[10]="nab_s_base.mytest-m64"
								[11]="wrf_s_base.mytest-m64"
							)

declare -a type_tags=(	
						"artifact_evaluation_lvpraw"
                        "artifact_evaluation_super"
					)

for type in "${type_tags[@]}"
do 
	echo "Fetching: $type"
	#continue
	#(  sbatch "gem5_${spec_bench_names[$i]}.slurm" ; cd "${home_root}/${bbv_gen_slurm_script_dir}" )
	spec_bnech_names_length=${#spec_bench_names[@]}
	#spec_bnech_names_length=2
	#
	
	for i in 1 2 3 4 5 6 7 8 9 10 11;
	do

	#echo before comment
	#: <<'END'
		result_dir="./${spec_bench_names[$i]}_${type}"
		if [ ! -e ${result_dir} ]; then
			mkdir ${result_dir}
		fi

		for (( n=1; n<=${spec_num_chkpoints[$i]}; n=n+1 ));
		do	
		#m5out_dante_sim_perl_ct_en_1_super
			mkdir -p "./${result_dir}/m5out_sim_${spec_bench_names[$i]}_${type}_${n}/"
			#echo "${spec_run_root}/${spec_bench_dir[$i]}/m5out_sim_${spec_bench_names[$i]}_${type}_${n}/config.ini" "./${result_dir}/m5out_sim_${spec_bench_names[$i]}_${type}_${n}/."
			cp -r "${spec_run_root}/${spec_bench_dir[$i]}/m5out_sim_${spec_bench_names[$i]}_${type}_${n}/config.ini" "./${result_dir}/m5out_sim_${spec_bench_names[$i]}_${type}_${n}/."
			cp -r "${spec_run_root}/${spec_bench_dir[$i]}/m5out_sim_${spec_bench_names[$i]}_${type}_${n}/config.json" "./${result_dir}/m5out_sim_${spec_bench_names[$i]}_${type}_${n}/."
			cp -r "${spec_run_root}/${spec_bench_dir[$i]}/m5out_sim_${spec_bench_names[$i]}_${type}_${n}/stats.txt" "./${result_dir}/m5out_sim_${spec_bench_names[$i]}_${type}_${n}/."

		done

	done
done


