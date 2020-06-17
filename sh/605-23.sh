cd /u/jer5ae/gem5/gem5-changes;
scons build/X86/gem5.opt -j9;
cd /p/csd/SPEC2017;
source shrc;
go run 605;
cd run_base_refspeed_mytest-m64.0008;
~/gem5/gem5-changes/build/X86/gem5.opt --outdir=/u/jer5ae/gem5/gem5-changes/m5out605-23-lim-Half0 ~/gem5/gem5-changes/configs/example/se.py -c mcf_s_base.mytest-m64 -o "inp.in" --cpu-type=O3_X86_skylake_1 --caches --l2cache --mem-size=1024MB --enable-microop-cache --enable-micro-fusion --enable-superoptimization --lvpredType=fa3p --predictingArithmetic=0 --tableEntries=4096 --predictStage=3 --mem-type=DDR4_2400_16x4 --mem-size=32GB --mem-channels=2 --constantThreshold=5 --dynamicThreshold=0 -I 500000000 -m 100000000000 --branchConfidenceCounterSize=2 --branchConfidenceThreshold=3;
