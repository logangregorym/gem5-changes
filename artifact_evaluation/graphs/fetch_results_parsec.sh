#!/bin/bash


parsec_run_root="/p/csd/parsec-port/pkgs"




#this will change the scripts name and output folders
#REMEBER TO CHANGE ALL THE PARAMETERS THAT WE WANT TO CHNAGE FOR THIS SIMULATION

declare -a parsec_bench_names=(
							 [1]="freqmine"
                             [2]="swaptions" 
                             [3]="streamcluster" 
                             [4]="blackscholes" 
                             [5]="canneal" 
                             [6]="x264" 
                             [7]="vips" 
                             [8]="dedup" 
                             [9]="ferret" 
							)

declare -a parsec_bench_dir=(  [1]="apps"
							 [2]="apps"
							 [3]="kernels"
							 [4]="apps"
							 [5]="kernels"
							 [6]="apps"
							 [7]="apps"
							 [8]="kernels"
							 [9]="apps"
							)

#(  sbatch "gem5_${parsec_bench_names[$i]}.slurm" ; cd "${home_root}/${bbv_gen_slurm_script_dir}" )
parsec_bnech_names_length=${#parsec_bench_names[@]}
#parsec_bnech_names_length=2
#
#declare -a desired_type_tags=(	
#						"logan_raw_dynamic_thresh_fa3p_noTSO_lvp_reg_0629"
#                        "logan_raw_dynamic_thresh_eves_noTSO_lvp_48sets_0629"
#						"logan_raw_dynamic_thresh_eves_noTSO_lvp_36sets_0629"
#						"logan_super_dynamic_thresh_fa3p_noTSO_SCC_assoc_12spec_36uop_0629"
#                        "logan_super_dynamic_thresh_eves_noTSO_SCC_assoc_12spec_36uop_0629"
#						"logan_super_dynamic_thresh_eves_noTSO_SCC_assoc_12spec_36uop_noControl_0629"
#						"logan_super_dynamic_thresh_eves_noTSO_SCC_assoc_12spec_36uop_noCC_0629"
#						"logan_super_dynamic_thresh_eves_noTSO_SCC_assoc_12spec_36uop_noProp_0629"
#                        "logan_super_dynamic_thresh_eves_noTSO_SCC_assoc_12spec_36uop_cWidth32_0630"
#                        "logan_super_dynamic_thresh_eves_noTSO_SCC_assoc_12spec_36uop_cWidth16_0630"
#                        "logan_super_dynamic_thresh_eves_noTSO_SCC_assoc_12spec_36uop_cWidth8_0630"
#						"logan_super_dynamic_thresh_eves_noTSO_SCC_assoc_partition_0629"
#						"logan_super_dynamic_thresh_eves_noTSO_SCC_assoc_36spec_12uop_0629"
#					)


declare -a type_tags=(	
						"parsec_artifact_evaluation_lvpraw"
						"parsec_artifact_evaluation_super"
					)


num_types=${#type_tags[@]}
for ((type_num=0;type_num<num_types;type_num++)); do
	parsec_type=${type_tags[$type_num]}
	#desired_type=${desired_type_tags[$type_num]}
	desired_type=$parsec_type

	echo "Fetching: $parsec_type -> $desired_type"
	
	#for i in 1 2 3 4 5 6 7 8 9;
	for i in 8;
	do

	#echo before comment
	#: <<'END'

		result_dir="./${parsec_bench_names[$i]}_${desired_type}"
		if [ ! -e ${result_dir} ]; then
			mkdir ${result_dir}
		fi

		#m5out_dante_sim_perl_ct_en_1_super
		mkdir -p "./${result_dir}/m5out_sim_${parsec_bench_names[$i]}_${desired_type}_1/"
		#echo "${parsec_run_root}/${parsec_bench_dir[$i]}/m5out_sim_${parsec_bench_names[$i]}_${type}_${n}/config.ini" "./${result_dir}/m5out_sim_${parsec_bench_names[$i]}_${type}_${n}/."
		cp -r "${parsec_run_root}/${parsec_bench_dir[$i]}/${parsec_bench_names[$i]}/run/m5out_sim_${parsec_bench_names[$i]}_${parsec_type}/config.ini" "./${result_dir}/m5out_sim_${parsec_bench_names[$i]}_${desired_type}_1/."
		cp -r "${parsec_run_root}/${parsec_bench_dir[$i]}/${parsec_bench_names[$i]}/run/m5out_sim_${parsec_bench_names[$i]}_${parsec_type}/config.json" "./${result_dir}/m5out_sim_${parsec_bench_names[$i]}_${desired_type}_1/."
		cp -r "${parsec_run_root}/${parsec_bench_dir[$i]}/${parsec_bench_names[$i]}/run/m5out_sim_${parsec_bench_names[$i]}_${parsec_type}/stats.txt" "./${result_dir}/m5out_sim_${parsec_bench_names[$i]}_${desired_type}_1/."

	done

done
