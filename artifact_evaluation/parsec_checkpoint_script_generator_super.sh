#!/bin/bash

#This should point to where "gem5-changes" is
home_root="/u/lgm4xn"

script_dir="${PWD}"

parsec_root="/p/csd/parsec-port"
parsec_run_root="${parsec_root}/pkgs"


gem5="gem5-changes/build/X86/gem5.opt"
gem5_config="gem5-changes/configs/example/se.py"


#this will change the scripts name and output folders
#REMEBER TO CHANGE ALL THE PARAMETERS THAT WE WANT TO CHNAGE FOR THIS SIMULATION
type="parsec_artifact_evaluation_super"

declare -a parsec_bench_names=([1]="blackscholes"
							 [2]="bodytrack"
                             [3]="canneal"
							 [4]="dedup"
                             [5]="facesim"
							 [6]="ferret"
                             [7]="fluidanimate"
                             [8]="freqmine"
                             [9]="streamcluster"
							 [10]="swaptions"
							 [11]="vips"
                             [12]="x264"
							)

declare -a parsec_run_dir=(  [1]="apps/blackscholes/run"
							 [2]="apps/bodytrack/run"
                             [3]="kernels/canneal/run"
							 [4]="kernels/dedup/run"
							 [5]="apps/facesim/run"
							 [6]="apps/ferret/run"
							 [7]="apps/fluidanimate/run"
							 [8]="apps/freqmine/run"
							 [9]="kernels/streamcluster/run"
							 [10]="apps/swaptions/run"
							 [11]="apps/vips/run"
							 [12]="apps/x264/run"
							)

declare -a parsec_bench_commands=([1]="/p/csd/parsec-port/pkgs/apps/blackscholes/inst/amd64-linux.gcc-serial/bin/blackscholes"
							 	[2]="/p/csd/parsec-port/pkgs/apps/bodytrack/inst/amd64-linux.gcc-serial/bin/bodytrack"
							 	[3]="/p/csd/parsec-port/pkgs/kernels/canneal/inst/amd64-linux.gcc-serial/bin/canneal"
							 	[4]="/p/csd/parsec-port/pkgs/kernels/dedup/inst/amd64-linux.gcc-serial/bin/dedup"
							 	[5]="/p/csd/parsec-port/pkgs/apps/facesim/inst/amd64-linux.gcc-serial/bin/facesim"
							 	[6]="/p/csd/parsec-port/pkgs/apps/ferret/inst/amd64-linux.gcc-serial/bin/ferret"
							 	[7]="/p/csd/parsec-port/pkgs/apps/fluidanimate/inst/amd64-linux.gcc-serial/bin/fluidanimate"
							 	[8]="/p/csd/parsec-port/pkgs/apps/freqmine/inst/amd64-linux.gcc-serial/bin/freqmine"
							 	[9]="/p/csd/parsec-port/pkgs/kernels/streamcluster/inst/amd64-linux.gcc-serial/bin/streamcluster"
							 	[10]="/p/csd/parsec-port/pkgs/apps/swaptions/inst/amd64-linux.gcc-serial/bin/swaptions"
							 	[11]="/p/csd/parsec-port/pkgs/apps/vips/inst/amd64-linux.gcc-serial/bin/vips"
							 	[12]="/p/csd/parsec-port/pkgs/apps/x264/inst/amd64-linux.gcc-serial/bin/x264"
							)
#checkpoints_at[1]=193939810
#checkpoints_at[6]=152414425
#checkpoints_at[11]=177528607
#checkpoints_at[12]=176212621
#checkpoints_at[9]=159340379


#checkpoints_at[8]=500000000
#checkpoints_at[10]=500000000

number_of_nodes=24

declare -a nodelist_pool=(
		[1]="cheetah03"
		[2]="cheetah03"
		[3]="cortado02"
		[4]="cortado03"
		[5]="cortado04"
		[6]="adriatic01"
		[7]="adriatic01"
		[8]="adriatic02"
		[9]="cheetah03"
		[10]="cheetah03"
		[11]="cortado05"
		[12]="cortado06"
		[13]="cortado07"
		[14]="adriatic02"
		[15]="adriatic03"
		[16]="adriatic03"
		[17]="optane01"
		[18]="optane01"
		[19]="cortado08"
		[20]="cortado09"
		[21]="cortado10"
		[22]="adriatic04"
		[23]="adriatic04"
		[24]="adriatic05"
)

declare -a partition_pool=(
		[1]="gpu"
		[2]="gpu"
		[3]="main"
		[4]="main"
		[5]="main"
		[6]="gpu"
		[7]="gpu"
		[8]="gpu"
		[9]="gpu"
		[10]="gpu"
		[11]="main"
		[12]="main"
		[13]="main"
		[14]="gpu"
		[15]="gpu"
		[16]="gpu"
		[17]="main"
		[18]="main"
		[19]="main"
		[20]="main"
		[21]="main"
		[22]="gpu"
		[23]="gpu"
		[24]="gpu"
)




get_bench_input() {
	local  parsec_bench_inputs=([1]="1 in_4K.txt prices.txt"
							  [2]="sequenceB_1 4 1 100 3 0 1"
							  [3]="1 10000 2000 100.nets 64"
							  [4]="-c -p -v -t 1 -i hamlet.dat -o output.dat.ddp"
							  [5]="-timing -threads 1"
							  [6]="corel lsh queries 5 5 1 output.txt"
							  [7]="1 3 in_15K.fluid out.fluid"
							  [8]="T10I4D100K_1k.dat 3"
							  [9]="6 20 6 1024 1024 10 none output.txt 1"
							  [10]="ns 8 -sm 1000 -nt 1"
							  [11]="im_benchmark barbados_256x288.v output.v"
							  [12]="--quiet --qp 20 --partitions b8x8,i4x4 --ref 5 --direct auto --b-pyramid --weightb --mixed-refs --no-fast-pskip --me umh --subme 7 --analyse b8x8,i4x4 --threads 1 -o eledream.264 eledream_640x360_8.y4m"
							)

	echo "${parsec_bench_inputs[$1]}"
}

job_num=0
#SBATCH --nodelist=${nodelist_pool[$node_number]}
create_gem5_run_chk_slurm_batch_file (){

	local bench_input="$(get_bench_input $1)"
	local gem5_slurm_file="${parsec_run_root}/${parsec_run_dir[$1]}/gem5_sim_run_${parsec_bench_names[$1]}_${type}.slurm"
	if [ -e ${gem5_slurm_file} ]; then
  		rm ${gem5_slurm_file}
	fi
    let node_number=$((job_num % number_of_nodes + 1))
    (( job_num++ ))
	# if you want to work multiline easily
cat <<EOF > ${gem5_slurm_file}
#!/bin/bash
# Submission script for Hercules
#SBATCH --job-name=${parsec_bench_names[$1]}-s
#SBATCH -N 1
#SBATCH -n 1
#SBATCH -c 1
#
#SBATCH --partition=${partition_pool[$node_number]}
#SBATCH --nodelist=${nodelist_pool[$node_number]}
#
#SBATCH --output="slurm-%j-${parsec_bench_names[$1]}-${type}.out"
#
#SBATCH --comment=raw

source /etc/profile.d/modules.sh
module load gcc-6.3.0

#time ${parsec_bench_commands[$1]} ${bench_input}

${home_root}/${gem5} --outdir=m5out_sim_${parsec_bench_names[$1]}_${type} ${home_root}/${gem5_config} -c ${parsec_bench_commands[$1]} -o '${bench_input}' --caches --l2cache --cpu-type=O3_X86_icelake_1   --mem-type=DDR4_2400_16x4 --mem-size=64GB --mem-channels=2  --enable-microop-cache --enable-micro-fusion --enable-superoptimization --lvpredType=eves --dynamicThreshold=5 --constantThreshold=3 --predictingArithmetic=1 --usingControlTracking=1 --maxRecursiveDepth=1 --usingCCTracking=1  --predictionConfidenceThreshold=5 --uopCacheNumSets=36 --uopCacheNumWays=8 --uopCacheNumUops=6 --specCacheNumSets=12 --specCacheNumWays=8 --specCacheNumUops=6 --l3cache --lvpLookupAtFetch --enableDynamicThreshold --forceNoTSO --uopCacheNumTicks=28 --specCacheNumTicks=3
#--disableSuperProp
#--disableSuperSimple
EOF

}

create_gem5_run_them_all_shell_script (){


	local shell_file="${script_dir}/gem5_sim_run_all_${type}.sh"
	if [ -e ${shell_file} ]; then
  		rm ${shell_file}
	fi

cat <<EOF > ${shell_file}
#!/bin/bash

cd "${parsec_run_root}/${parsec_run_dir[4]}/" ;
sbatch gem5_sim_run_${parsec_bench_names[4]}_${type}.slurm

cd "${parsec_run_root}/${parsec_run_dir[6]}/" ;
sbatch gem5_sim_run_${parsec_bench_names[6]}_${type}.slurm

cd "${parsec_run_root}/${parsec_run_dir[11]}/" ;
sbatch gem5_sim_run_${parsec_bench_names[11]}_${type}.slurm

cd "${parsec_run_root}/${parsec_run_dir[8]}/" ;
sbatch gem5_sim_run_${parsec_bench_names[8]}_${type}.slurm

cd "${parsec_run_root}/${parsec_run_dir[10]}/" ;
sbatch gem5_sim_run_${parsec_bench_names[10]}_${type}.slurm

cd "${parsec_run_root}/${parsec_run_dir[9]}/" ;
sbatch gem5_sim_run_${parsec_bench_names[9]}_${type}.slurm

cd "${parsec_run_root}/${parsec_run_dir[1]}/" ;
sbatch gem5_sim_run_${parsec_bench_names[1]}_${type}.slurm

cd "${parsec_run_root}/${parsec_run_dir[12]}/" ;
sbatch gem5_sim_run_${parsec_bench_names[12]}_${type}.slurm

#cd "${parsec_run_root}/${parsec_run_dir[3]}/" ;
#sbatch gem5_sim_run_${parsec_bench_names[3]}_${type}.slurm

#cd "${parsec_run_root}/${parsec_run_dir[2]}/" ;
#sbatch gem5_sim_run_${parsec_bench_names[2]}_${type}.slurm

#cd "${parsec_run_root}/${parsec_run_dir[5]}/" ;
#sbatch gem5_sim_run_${parsec_bench_names[5]}_${type}.slurm

#cd "${parsec_run_root}/${parsec_run_dir[7]}/" ;
#sbatch gem5_sim_run_${parsec_bench_names[7]}_${type}.slurm

EOF

}

spec_bnech_names_length=${#parsec_bench_names[@]}

declare -a parsec_bench_names=([1]="blackscholes"
							 [2]="bodytrack"
                             [3]="canneal"
							 [4]="dedup"
                             [5]="facesim"
							 [6]="ferret"
                             [7]="fluidanimate"
                             [8]="freqmine"
                             [9]="streamcluster"
							 [10]="swaptions"
							 [11]="vips"
                             [12]="x264"
							)


create_gem5_run_chk_slurm_batch_file 1
create_gem5_run_chk_slurm_batch_file 12 #maybe
create_gem5_run_chk_slurm_batch_file 6
create_gem5_run_chk_slurm_batch_file 4
create_gem5_run_chk_slurm_batch_file 11
create_gem5_run_chk_slurm_batch_file 9
create_gem5_run_chk_slurm_batch_file 8
create_gem5_run_chk_slurm_batch_file 10
#create_gem5_run_chk_slurm_batch_file 3
#create_gem5_run_chk_slurm_batch_file 2
#create_gem5_run_chk_slurm_batch_file 5
#create_gem5_run_chk_slurm_batch_file 7
#for (( i=1; i<=${spec_bnech_names_length}; i++ ));
#do
#  	create_gem5_run_chk_slurm_batch_file $i
#done

create_gem5_run_them_all_shell_script
chmod +x "${script_dir}/gem5_sim_run_all_${type}.sh"
