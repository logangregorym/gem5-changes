#!/bin/bash

#This should point to where "gem5-changes" is
home_root="/u/lgm4xn"

script_dir="${PWD}"
spec_root="/p/csd/SPEC2017"
spec_run_root="${spec_root}/benchspec/CPU"


gem5="gem5-changes/build/X86/gem5.opt"
gem5_config="gem5-changes/configs/example/se.py"


#this will change the scripts name and output folders
#REMEBER TO CHANGE ALL THE PARAMETERS THAT WE WANT TO CHNAGE FOR THIS SIMULATION
#type="logan_raw_reg"
#type="logan_lvpraw_reg"
#type="logan_lvpraw_double_front_0612"
#type="logan_lvpraw_double"
#type="logan_super_partition"
type="artifact_evaluation_lvpraw"
#type="logan_super_partition_dynamic"
#type="logan_super_double"
#type="logan_super_double_dynamic"
#type="logan_lvpraw_double_dynamic"


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

number_of_nodes=19

declare -a nodelist_pool=(
		[1]="cheetah03"
		[2]="adriatic02"
		[3]="adriatic03"
		[4]="adriatic04"
		[5]="adriatic05"
		[6]="adriatic06"
		[7]="adriatic01"
		[8]="cortado06"
		[9]="cortado07"
		[10]="cortado08"
		[11]="cortado09"
		[12]="cortado10"
		[13]="cheetah03"
		[14]="adriatic02"
		[15]="adriatic03"
		[16]="adriatic04"
		[17]="adriatic05"
		[18]="adriatic06"
		[19]="adriatic01"
)

declare -a partition_pool=(
		[1]="gpu"
		[2]="gpu"
		[3]="gpu"
		[4]="gpu"
		[5]="gpu"
		[6]="gpu"
		[7]="gpu"
		[8]="main"
		[9]="main"
		[10]="main"
		[11]="main"
		[12]="main"
		[13]="gpu"
		[14]="gpu"
		[15]="gpu"
		[16]="gpu"
		[17]="gpu"
		[18]="gpu"
		[19]="gpu"
)



get_bench_input() {
	local  spec_bench_inputs=([1]="-I./lib checkspam.pl 2500 5 25 11 150 1 1 1 1"
							  [2]="gcc-pp.c -O5 -finline-limit=24000 -fgcse -fgcse-las -fgcse-lm -fgcse-sm -o gcc-pp.opts-O5_-finline-limit_24000_-fgcse_-fgcse-las_-fgcse-lm_-fgcse-sm.s"
							  [3]="inp.in"
							  [4]="-v t5.xml xalanc.xsl"
							  [5]="6"
							  [6]="ref.txt"
							  [7]="ref.sgf"
							  [8]="cpu2006docs.tar.xz 6643 055ce243071129412e9dd0b3b69a21654033a9b723d874b2015c774fac1553d9713be561ca86f74e4f16f22e664fc17a79f30caa5ad2c04fbc447549c2810fae 1036078272 1111795472 4"
							  [9]="2000 reference.dat 0 0 200_200_260_ldc.of"
							  [10]="3j1n 20140317 220"
							  [11]=""
							)

	echo "${spec_bench_inputs[$1]}"
}


declare -A checkpoints_at
declare -A exec_counts
num_rows=11
num_columns=8

for ((i=1;i<=num_rows;i++)) do
    for ((j=1;j<=num_columns;j++)) do
        checkpoints_at[$i,$j]=0
		exec_counts[$i,$j]=0
    done
done

#perl 
#gcc 
#mcf
#xalan

checkpoints_at[1,1]=161916430
checkpoints_at[1,2]=173790553
checkpoints_at[1,3]=171152144
checkpoints_at[1,4]=174705426
checkpoints_at[1,5]=163408281
checkpoints_at[1,6]=162342019
checkpoints_at[1,7]=163544232

checkpoints_at[2,1]=191500866
checkpoints_at[2,2]=205573108
checkpoints_at[2,3]=188644158
checkpoints_at[2,4]=192774781
checkpoints_at[2,5]=192795070
checkpoints_at[2,6]=188195003
checkpoints_at[2,7]=187952995

checkpoints_at[3,1]=169195487
checkpoints_at[3,2]=177340641
checkpoints_at[3,3]=169506928
checkpoints_at[3,4]=188162491
checkpoints_at[3,5]=144976081
checkpoints_at[3,6]=148288187
checkpoints_at[3,7]=161632380

checkpoints_at[4,1]=197228713
checkpoints_at[4,2]=211340732
checkpoints_at[4,3]=207442292
checkpoints_at[4,4]=207458250
checkpoints_at[4,5]=210966440
checkpoints_at[4,6]=208250551

#exchange
#deepsjeng
#leela
#xz
#lbm
#nab
#wrf
checkpoints_at[5,1]=198575302
checkpoints_at[5,2]=227867878
checkpoints_at[5,3]=228630886
checkpoints_at[5,4]=224990510
checkpoints_at[5,5]=214808764

checkpoints_at[6,1]=170439102
checkpoints_at[6,2]=170971342
checkpoints_at[6,3]=170747145

checkpoints_at[7,1]=199683521
checkpoints_at[7,2]=193710924
checkpoints_at[7,3]=203720138
checkpoints_at[7,4]=186385355
checkpoints_at[7,5]=186407629
checkpoints_at[7,6]=188659245
checkpoints_at[7,7]=190182299
checkpoints_at[7,8]=189845901

checkpoints_at[8,1]=133129189
checkpoints_at[8,2]=158869737
checkpoints_at[8,3]=161886284
checkpoints_at[8,4]=157141774
checkpoints_at[8,5]=158828332
checkpoints_at[8,6]=158738317

checkpoints_at[9,1]=122348469
checkpoints_at[9,2]=122683958
checkpoints_at[9,3]=122735074
checkpoints_at[9,4]=121874742
checkpoints_at[9,5]=122015526

checkpoints_at[10,1]=172296445
checkpoints_at[10,2]=157997105
checkpoints_at[10,3]=171373973
checkpoints_at[10,4]=171372530

checkpoints_at[11,1]=189812062
checkpoints_at[11,2]=187065979
checkpoints_at[11,3]=188108724
checkpoints_at[11,4]=181306069
checkpoints_at[11,5]=191069086
checkpoints_at[11,6]=190900673

#job_num=19
job_num=19
#job_num=14
#job_num=21
#job_num=28
#job_num=35
#job_num=42
#job_num=49
#job_num=56
#SBATCH --nodelist=${nodelist_pool[$node_number]}
create_gem5_run_chk_slurm_batch_file (){

	local bench_input="$(get_bench_input $1)"
	local gem5_slurm_file="${spec_run_root}/${spec_bench_dir[$1]}/gem5_sim_run_${spec_bench_names[$1]}_$2_${type}.slurm"
	if [ -e ${gem5_slurm_file} ]; then
  		rm ${gem5_slurm_file}
	fi

    let node_number=$((job_num % number_of_nodes + 1))
    (( job_num++ ))
	# if you want to work multiline easily
cat <<EOF > ${gem5_slurm_file}
#!/bin/bash
# Submission script for Hercules
#SBATCH --job-name=${spec_bench_nums[$1]}-${2}-r
#SBATCH -N 1
#SBATCH -n 1
#SBATCH -c 1
#
#SBATCH --partition=${partition_pool[$node_number]}
#SBATCH --nodelist=${nodelist_pool[$node_number]}
#
#SBATCH --output="slurm-%j-${spec_bench_names[$1]}-${type}-${2}.out"
#
#SBATCH --comment=raw

source /etc/profile.d/modules.sh
module load gcc-6.3.0

${home_root}/${gem5} --outdir=m5out_sim_${spec_bench_names[$1]}_${type}_$2 ${home_root}/${gem5_config}  -r $2 -I 100000000 --checkpoint-dir ${spec_bench_names[$1]}.sim.64G -c ./${spec_bench_commands[$1]} -o '${bench_input}' --caches --l2cache --cpu-type=O3_X86_icelake_1   --mem-type=DDR4_2400_16x4 --mem-size=64GB --mem-channels=2  --enable-microop-cache --uopCacheNumSets=48 --uopCacheNumWays=8 --uopCacheNumUops=6 --enable-micro-fusion --l3cache --lvpredType=eves --dynamicThreshold=5 --constantThreshold=3 --predictionConfidenceThreshold=15 --enableValuePredForwarding --predictingArithmetic=1 --enableDynamicThreshold --forceNoTSO  --uopCacheNumTicks=28


EOF

}

create_gem5_run_slurm_script (){

	local bench_input="$(get_bench_input $1)"
	local gem5_slurm_file="${spec_run_root}/${spec_bench_dir[$1]}/gem5_sim_run_all_${spec_bench_names[$1]}_${type}.sh"
	if [ -e ${gem5_slurm_file} ]; then
  		rm ${gem5_slurm_file}
	fi


	# if you want to work multiline easily
cat <<EOF > ${gem5_slurm_file}
#!/bin/bash

for (( n=1; n<=${spec_num_chkpoints[$1]}; n=n+1 ));
do	
		sbatch "gem5_sim_run_${spec_bench_names[$1]}_\${n}_$2.slurm"  

done

EOF

}

create_gem5_run_them_all_shell_script (){


	local shell_file="${script_dir}/gem5_sim_run_all_${type}.sh"
	if [ -e ${shell_file} ]; then
  		rm ${shell_file}
	fi

cat <<EOF > ${shell_file}
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
cd "${spec_run_root}/${spec_bench_dir[1]}/" ;
./gem5_sim_run_all_${spec_bench_names[1]}_${type}.sh

#exchange
cd "${spec_run_root}/${spec_bench_dir[5]}/" ;
./gem5_sim_run_all_${spec_bench_names[5]}_${type}.sh

#sleep 60m

#mcf
cd "${spec_run_root}/${spec_bench_dir[3]}/" ;
./gem5_sim_run_all_${spec_bench_names[3]}_${type}.sh

#deepsjeng
cd "${spec_run_root}/${spec_bench_dir[6]}/" ;
./gem5_sim_run_all_${spec_bench_names[6]}_${type}.sh

sleep 90m

#xalan
cd "${spec_run_root}/${spec_bench_dir[4]}/" ;
./gem5_sim_run_all_${spec_bench_names[4]}_${type}.sh
#sleep 30m
#xz
cd "${spec_run_root}/${spec_bench_dir[8]}/" ;
./gem5_sim_run_all_${spec_bench_names[8]}_${type}.sh

#lbm
cd "${spec_run_root}/${spec_bench_dir[9]}/" ;
./gem5_sim_run_all_${spec_bench_names[9]}_${type}.sh

#nab
cd "${spec_run_root}/${spec_bench_dir[10]}/" ;
./gem5_sim_run_all_${spec_bench_names[10]}_${type}.sh

#wrf
cd "${spec_run_root}/${spec_bench_dir[11]}/" ;
./gem5_sim_run_all_${spec_bench_names[11]}_${type}.sh

#sleep 90m

#leela
cd "${spec_run_root}/${spec_bench_dir[7]}/" ;
./gem5_sim_run_all_${spec_bench_names[7]}_${type}.sh

#gcc 
cd "${spec_run_root}/${spec_bench_dir[2]}/" ;
./gem5_sim_run_all_${spec_bench_names[2]}_${type}.sh


EOF

}

spec_bnech_names_length=${#spec_bench_names[@]}

#i=1
#for (( n=1; n<=${spec_num_chkpoints[$i]}; n=n+1 ));
#do
#    create_gem5_run_chk_slurm_batch_file $i $n
#done

for (( i=1; i<=${spec_bnech_names_length}; i++ ));
do
	for (( n=1; n<=${spec_num_chkpoints[$i]}; n=n+1 ));
	do	
  		create_gem5_run_chk_slurm_batch_file $i $n
  	done
done

for (( i=1; i<=${spec_bnech_names_length}; i++ ));
do
	create_gem5_run_slurm_script $i $type
	chmod +x "${spec_run_root}/${spec_bench_dir[$i]}/gem5_sim_run_all_${spec_bench_names[$i]}_${type}.sh"

done

create_gem5_run_them_all_shell_script
chmod +x "${script_dir}/gem5_sim_run_all_${type}.sh"
