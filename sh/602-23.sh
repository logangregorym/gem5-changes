# test 602 with 2b conf, threshold 3

cd /u/jer5ae/gem5/gem5-changes;
scons build/X86/gem5.opt -j9;
cd /p/csd/SPEC2017;
source shrc;
go run 602;
cd run_base_refspeed_mytest-m64.0015;
~/gem5/gem5-changes/build/X86/gem5.opt --outdir=/u/jer5ae/gem5/gem5-changes/m5out602-23-Half0 ~/gem5/gem5-changes/configs/example/se.py -c sgcc_base.mytest-m64 -o 'gcc-pp.c -O5 -fipa-pta -o gcc-pp.opts-O5_-fipa-pta.s' --cpu-type=O3_X86_skylake_1 --caches --l2cache --mem-size=1024MB --enable-microop-cache --enable-micro-fusion --enable-superoptimization --lvpredType=fa3p --predictingArithmetic=0 --tableEntries=4096 --predictStage=3 --mem-type=DDR4_2400_16x4 --mem-size=32GB --mem-channels=2 --constantThreshold=5 --dynamicThreshold=0 -I 500000000 -m 100000000000 --branchConfidenceCounterSize=2 --branchConfidenceThreshold=3;
