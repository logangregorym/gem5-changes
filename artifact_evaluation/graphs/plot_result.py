import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import palettable
from collections import defaultdict
from scipy.stats.mstats import gmean 
from matplotlib import gridspec
import warnings
import os

type1 = "artifact_evaluation_lvpraw"
type2 = "artifact_evaluation_super"


file_type = "results"


#matplotlib.rcParams['ps.useafm'] = True
#matplotlib.rcParams['pdf.use14corefonts'] = True   
#matplotlib.rcParams['text.usetex'] = True             
#matplotlib.rcParams["figure.figsize"] = [6.4, 3]     
#matplotlib.rcParams['text.latex.preamble'] = [r'\usepackage{sfmath} \boldmath']
#matplotlib.rc('text', usetex=True)
# matplotlib.rc('axes', linewidth=2)
matplotlib.rc('font', weight='bold', size=8)

def measure(results_index, contents):

        data_dict = defaultdict(list)
        
        data_list = [lines.split() for lines in contents.split("\n")]
        for line in data_list:
                if len(line) != 0 and line[0] in results_index:
                        data_dict[line[0]].append(line[1])


        for line in results_index:
                if line in data_dict:
                        pass
                else:
                        data_dict[line].append('0')

        return data_dict





NUM_OF_TYPES = 1

COLUMNS = ['benchmark', 'Raw',  'Super']
x_axe = ['Raw', 'Super']


#COLUMNS_ACC_COV = ['benchmark', 'Super',  'Super']
#x_axe_label_acc_cov = ['Super', 'Super']
#x_axe_acc_cov = ['Super','Super']

# #'benchmark': ['perlbench', 'mcf', 'xalancbmk', 'deepsjeng',  'namd', 'lbm', 'nab', 'exchange', 'xz', 'wrf',  'leela']
# final_results_performance = {       'benchmark': ['perlbench', 'xalancbmk', 'exchange',  'leela', 'gcc', 'mcf', 'wrf', 'nab', 'lbm', 'xz', 'deepsjeng', 'freqmine', 'swaptions', 'streamcluster', 'blackscholes', 'x264', 'vips', 'dedup', 'ferret'],
#                                 'Raw':[],
#                                 'Super':[]
#                                 }

# final_results_from = {       'benchmark': ['perlbench', 'xalancbmk', 'exchange',  'leela', 'gcc', 'mcf', 'wrf', 'nab', 'lbm', 'xz', 'deepsjeng'],
#                                 'Spec $':[],
#                                 'Nonspec $':[]
#                                 }    
# inst_lables = ['AND', 'ANDI', 'OR', 'ORI', 'XOR', 'XORI', 'ADD', 'ADDI', 'SUB', 'SUBI', 'SLLI', 'SRLI', 'LEA', 'SEXTI', 'ZEXTI', 'LIMM', 'RDIP', 'MOV', 'MOVI', 'NOP', 'PredSource']                            
# iew_cause_lables = ['stalledDueToCommit', 'stalledDueToIQ', 'IQFull', 'LSQFull', 'bandwidthFull']
# commit_detail_lables = ['FromSpecLVP', 'FromSpecNotLVP', 'FromUopLVP', 'FromUopNotLVP', 'FromICacheLVP', 'FromICacheNotLVP']
# branch_detail_lables = ['FromBranchPred', 'FromTraceGen', 'FromUopCache', 'FromICache']

# #commit_detail_lables = ['FromSpecLVP', 'FromSpecNotLVP', 'FromUopNotLVP', 'FromICacheNotLVP']

# final_results_uop_expansion = {       'benchmark': ['perlbench', 'xalancbmk', 'exchange',  'leela', 'gcc', 'mcf', 'wrf', 'nab', 'lbm', 'xz', 'deepsjeng'],
#                                 'Raw':[],
#                                 'Super':[]
#                                 }


# final_results_squashed_insts = {       'benchmark': ['perlbench', 'xalancbmk', 'exchange',  'leela', 'gcc', 'mcf', 'wrf', 'nab', 'lbm', 'xz', 'deepsjeng'],
#                                 'Raw':[],
#                                 'Super':[]
#                                 }

# final_results_acc = {       'benchmark': ['perlbench', 'xalancbmk', 'exchange',  'leela', 'gcc', 'mcf', 'wrf', 'nab', 'lbm', 'xz', 'deepsjeng'],
#                                 'Super':[],
#                                 'Super':[]
#                                 }
# final_results_cov = {       'benchmark': ['perlbench', 'xalancbmk', 'exchange',  'leela', 'gcc', 'mcf', 'wrf', 'nab', 'lbm', 'xz', 'deepsjeng'],
#                                 'Super':[],
#                                 'Super':[]
#                                 }
# final_results_branch_detail= {       'benchmark': ['perlbench', 'xalancbmk', 'exchange',  'leela', 'gcc', 'mcf', 'wrf', 'nab', 'lbm', 'xz', 'deepsjeng'],
#                                 'Taken':[],
#                                 'NotTaken':[],
#                                 'Indirect':[]
#                                 }
# indexes = ['Super']

# indexes_acc_cov  = ['Super', 'Super']

#final_results_performance = {    'benchmark':  ['perlbench',  'exchange', 'xalancbmk', 'gcc',  'leela',  'wrf', 'deepsjeng', 'nab', 'mcf', 'xz', 'lbm',   'vips', 'x264', 'freqmine',  'swaptions', 'dedup', 'blackscholes',  'ferret', 'streamcluster', "", "SPEC Geomean", "PARSEC Geomean"],
final_results_performance = {    'benchmark':  ['perlbench',  'exchange', 'xalancbmk', 'gcc',  'leela',  'wrf', 'deepsjeng', 'nab', 'mcf', 'xz', 'lbm', "", "SPEC Geomean"],
                                'Raw':[],
                                 'Super':[]
                                 }
             


benchs  = {                     
                                'perl': [1,2,3,4,5,6,7],
                                'exchange': [1,2,3,4,5],
                                'xalan': [1,2,3,4,5,6],
                                'gcc': [1,2,3,4,5],
                                'leela': [1,2,3,4,5,6,7,8],
                                'wrf': [1,2,3,4,5,6],
                                'deepsjeng': [1,2,3],
                                'nab': [1,2,3,4],
                                'mcf': [1,2,3,4,5,6,7],
                                'xz': [1,2,3,4,5,6],
                                'lbm': [1,2,3,4,5]#,
                                #'vips': [1], 
                                #'x264': [1], 
                                #'freqmine': [1], 
                                #'swaptions': [1], 
                                #'dedup': [1], 
                                #'blackscholes': [1], 
                                #'ferret': [1],
                                #'streamcluster': [1]
                                #'canneal': [1]
                                
                                #'povray'
                                }

weights  = {                    
                                'xalan':[0.177687 , 0.0680793 , 0.0917369 , 0.201515 , 0.416645 , 0.0432304],
                                
                                'perl': [0.0153096, 0.133925 , 0.0781059 , 0.169421 , 0.189676 , 0.156957 ,0.256605 ],
                               
                                'leela':[ 0.186113 ,0.130977 ,0.210597 ,0.09057 ,0.0311783 ,0.143171 ,0.0251052 ,0.181857],
                               
                                'wrf':[ 0.230124 , 0.154126 , 0.181549 , 0.173412 , 0.18028 ,0.0805083 ],                                
                                
                                'namd': [0.115099 ,0.0888132 ,0.0793817 ,0.0585538 ,0.292638 ,0.198542 ,0.166972 ],
                                
                                'nab':[0.309196 , 0.164877 , 0.249017 ,0.183404 ],
                                
                                'mcf': [0.0879996 ,0.0297703 ,0.0873683 , 0.0665339 ,0.0483706 ,0.160701 ,0.519256],
                              
                                'lbm':[0.0342543 ,0.0460752 , 0.717145 ,0.0387249 ,0.158122 ],
        
                                'exchange':[0.359977 ,0.058249 , 0.0322749 ,0.128164 ,0.421335 ],

                                'gcc':[ 0.0154094, 0.015947, 0.634832, 0.0326106, 0.00304605],
                           
                                'xz':[0.0850169 , 0.152142 , 0.099203 ,0.292489 , 0.202172 ,0.112572],

                                'deepsjeng': [0.383868 , 0.138951 , 0.472347 ]#,
                              
                                #'povray':[0.192913 , 0.235802 , 0.082035 , 0.0750824 , 0.336042 , 0.0781259 ]
                                
                                #'vips': [1.0], 

                                #'freqmine': [1.0], 
                                
                                #'x264': [1.0], 
                                
                                #'swaptions': [1.0], 
                                
                                #'dedup': [1.0], 
                                
                                #'blackscholes': [1], 
                                
                                #'ferret': [1.0],
                                
                                #'streamcluster': [1.0]
                                
                                #'canneal': [1.0], 

                                }


def get_main_results():
            
        # Plotting the bars
        fig = plt.figure(figsize=(20,12))
        gs = gridspec.GridSpec(2, 1, figure=fig)#height_ratios=[1,1]) 
        # print('AVG', final_results_performance['CHEx86ContextSensitive'])
        # print('GEOMEAN', 1 - gmean(final_results_performance['CHEx86ContextSensitive']))
        #fig, ((ax0, ax1), (ax2, ax3)) = plt.subplots(2, 2)
        ax0 = plt.subplot(gs[0])
        ax1 = plt.subplot(gs[1])#,sharex = ax0)
        #ax2 = plt.subplot(gs[1,0])
        #ax3 = plt.subplot(gs[1,1])
        get_inst_results(ax0, False)
        get_performance_results(ax1)
        #get_branch_pred_results(ax2)
        #get_commitSquashed_simple_results(ax3)

        handles, labels = plt.gca().get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        fig.legend(by_label.values(), by_label.keys(), bbox_to_anchor=(0,1.02,1,0.2), loc="lower left", mode="expand", borderaxespad=0, edgecolor='black', ncol=5, prop={"size":25, "weight":'bold'})

        #plt.xticks(rotation=55)
        
        plt.tight_layout()

        if not os.path.exists(file_type):
                os.makedirs(file_type)
        plt.savefig(file_type + '/mainPerformance.png', bbox_inches='tight')
        plt.show()

def get_uop_cache_results():
        # Plotting the bars
        fig = plt.figure(figsize=(20,16))
        gs = gridspec.GridSpec(2, 1, figure=fig)#height_ratios=[1,1]) 
        # print('AVG', final_results_performance['CHEx86ContextSensitive'])
        # print('GEOMEAN', 1 - gmean(final_results_performance['CHEx86ContextSensitive']))
        #fig, ((ax0, ax1), (ax2, ax3)) = plt.subplots(2, 2)
        ax0 = plt.subplot(gs[0])
        ax1 = plt.subplot(gs[1])
        get_performance_results(ax0)
        get_streamedfrom_results(ax1)

        #handles, labels = plt.gca().get_legend_handles_labels()
        #by_label = dict(zip(labels, handles))
        #fig.legend(by_label.values(), by_label.keys(), bbox_to_anchor=(0,1.02,1,0.2), loc="lower left", mode="expand", borderaxespad=0, edgecolor='black', ncol=3, prop={"size":16, "weight":'bold'})

        #plt.xticks(rotation=55)
        
        plt.tight_layout()

        if not os.path.exists(file_type):
                os.makedirs(file_type)
        plt.savefig(file_type + '/uopcache_results.png', bbox_inches='tight')
        plt.show()

def get_performance_results(ax, print_x_labels = True):
        
        results_index_file = open('./results_index.txt', mode='r')
        contents = results_index_file.read()
        results_index = contents.split("\n")

        final_results_performance["Raw"] = []
        final_results_performance["Super"] = []

        #for bench in benchs:
        SpecSuperTotal = 1.0
        SpecRawTotal = 1.0
        NumOfSpec = 0
        ParsecSuperTotal = 1.0
        ParsecRawTotal = 1.0
        NumOfParsec = 0
        for i, bench in enumerate(benchs):

                PerformanceOverheadCap = 0.0 #should be zero
                total_weight = 0.0

                for chk_num in range(len(benchs[bench])):

                        if weights[bench][chk_num] == 0.0:
                                continue
                        
                        cap_stat_file_dir = './' + bench + '_' + type2 + '/m5out_sim_' + bench + '_' + type2 + "_" + str(chk_num+1) + '/stats.txt'
                        Super_stat_file_dir = './' + bench + '_' + type1 + '/m5out_sim_' + bench + '_' + type1 + '_' + str(chk_num+1) + '/stats.txt'
                        #print(cap_stat_file_dir)
                        #print(Super_stat_file_dir)
                        cap_stat_file = open(cap_stat_file_dir, mode='r')
                        Super_stat_file = open(Super_stat_file_dir, mode='r')
                        cap_stat_file_content = cap_stat_file.read()
                        Super_stat_file_content = Super_stat_file.read()

                        cap_results = measure(results_index, cap_stat_file_content)
                        Super_results = measure(results_index, Super_stat_file_content)

                        if i <= 10:
                            if int(cap_results['system.switch_cpus.numCycles'][-1]) == 0 or int(Super_results['system.switch_cpus.numCycles'][-1]) == 0:
                                warnings.warn(bench + " simpoint " + str(chk_num+1) + " has not stat!")
                                cap_stat_file.close()
                                Super_stat_file.close()
                                continue
                        
                            total_weight = total_weight + weights[bench][chk_num]
                            print("Super: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", int(Super_results['system.switch_cpus.numCycles'][-1]))
                            print("Raw: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", int(cap_results['system.switch_cpus.numCycles'][-1]))
                        
                            #PerformanceOverheadCap = PerformanceOverheadCap + weights[bench][chk_num] * ((int(cap_results['system.switch_cpus.numCycles'][-1]))/int(Super_results['system.switch_cpus.numCycles'][-1]))
                            PerformanceOverheadCap = PerformanceOverheadCap + weights[bench][chk_num] * (int(Super_results['system.switch_cpus.numCycles'][-1])/int(cap_results['system.switch_cpus.numCycles'][-1]))
                            print("PerformanceOverheadCap: ", PerformanceOverheadCap)
                            print('PerformanceOverheadSuper: ', total_weight) 
                            cap_stat_file.close()
                            Super_stat_file.close()
                        else:
                            if int(cap_results['system.cpu.numCycles'][-1]) == 0 or int(Super_results['system.cpu.numCycles'][-1]) == 0:
                                warnings.warn(bench + " simpoint " + str(chk_num+1) + " has not stat!")
                                cap_stat_file.close()
                                Super_stat_file.close()
                                continue
                        
                            total_weight = total_weight + weights[bench][chk_num]
                            print("Super: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", int(Super_results['system.cpu.numCycles'][-1]))
                            print("Raw: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", int(cap_results['system.cpu.numCycles'][-1]))
                        
                            #PerformanceOverheadCap = PerformanceOverheadCap + weights[bench][chk_num] * ((int(cap_results['system.switch_cpus.numCycles'][-1]))/int(Super_results['system.switch_cpus.numCycles'][-1]))
                            PerformanceOverheadCap = PerformanceOverheadCap + weights[bench][chk_num] * (int(Super_results['system.cpu.numCycles'][-1])/int(cap_results['system.cpu.numCycles'][-1]))
                            print("PerformanceOverheadCap: ", PerformanceOverheadCap)
                            print('PerformanceOverheadSuper: ', total_weight) 
                            cap_stat_file.close()
                            Super_stat_file.close()

                        

                print("-------------------------------------------DONE----------------------------")
                if total_weight == 0:
                    final_results_performance["Super"].append(0)
                    final_results_performance["Raw"].append(0)
                    warnings.warn(bench + " Has no working simpoints!!")
                    continue

                print('PerformanceOverheadCap: ', PerformanceOverheadCap/total_weight)
                print('PerformanceOverheadSuper: ', total_weight,total_weight)   

                final_results_performance["Super"].append((PerformanceOverheadCap/total_weight))
                final_results_performance["Raw"].append(total_weight/total_weight)
                
                if i <= 10:
                    SpecSuperTotal *= PerformanceOverheadCap/total_weight
                    SpecRawTotal *= total_weight/total_weight
                    NumOfSpec += 1
                else:
                    ParsecSuperTotal *= PerformanceOverheadCap/total_weight
                    ParsecRawTotal *= total_weight/total_weight
                    NumOfParsec += 1


        final_results_performance["Super"].append(SpecSuperTotal**(1/NumOfSpec))
        final_results_performance["Raw"].append(SpecRawTotal**(1//NumOfSpec))
        # final_results_performance["Super"].append(ParsecSuperTotal/NumOfParsec)
        # final_results_performance["Raw"].append(ParsecRawTotal/NumOfParsec)
                



        df = pd.DataFrame(final_results_performance, columns = COLUMNS)
        # Setting the positions and width for the bars
        pos = list(range(len(df['Raw']))) 
        #print(pos)
        width = 0.25
            
        # Plotting the bars
        # fig = plt.figure(figsize=(10,6))
        # gs = gridspec.GridSpec(1, 1, height_ratios=[1]) 
        # print('AVG', final_results_performance['CHEx86ContextSensitive'])
        # print('GEOMEAN', 1 - gmean(final_results_performance['CHEx86ContextSensitive']))
        # ax0 = plt.subplot(gs[0])
        patterns = [ "*" , "+" , "x" , "o" , "\\" , "/", "o", "O", ".", "*" ]
        #colors = palettable.colorbrewer.diverging.Spectral_4.mpl_colors
        colors = palettable.scientific.sequential.Davos_6.mpl_colors


        for i in range(0,NUM_OF_TYPES+1):
            ax.bar([p + width*i for p in pos], 
                        df[x_axe[i]], 
                        #hatch=3*patterns[i],
                        # of width
                        width=width,
                        # with color
                        #edgecolor=colors[i], 
                        color=colors[i],
                        # with label the first value in benchmark
                        label=x_axe[i],
                        zorder=3) 
            ax.bar([p + width*i for p in pos], 
                        df[x_axe[i]], 
                        # of width
                        width=width, 
                        # with color
                        color='none', 
                        edgecolor='black',
                        zorder=4) 

        if (print_x_labels):
            ax.set_xticks([p + 2.5 * width for p in pos])                
            ax.set_xticklabels(df['benchmark'], rotation=55)
            for tick in ax.xaxis.get_majorticklabels():
                tick.set_horizontalalignment("right")
            ax.tick_params(axis="x", labelsize=30)
        else:
            ax.set_xticks([])

        ax.tick_params(axis="y", labelsize=25)
        ax.set_ylabel('$Speedup$', fontsize = 25.0)
        ax.grid( linestyle='-.', linewidth=0.5, alpha=.5, axis='y', zorder=0)
        
        ax.legend(["Raw","Super"], bbox_to_anchor=(0,1.05,1,0.2), loc="lower left", mode="expand", borderaxespad=0, edgecolor='black', ncol=2, prop={"size":25, "weight":'bold'})
        ax.grid( linestyle='-.', linewidth=0.5, alpha=.5, axis='y', zorder=0)
        """   
        # Set the labels for the x ticks
        #ax.set_xticklabels(df['benchmark'])
        ax.tick_params(axis="y", labelsize=15)
        # Setting the x-axis and y-axis limits
        #plt.xlim(min(pos)-width, max(pos)+width*4)
        #ax0.set(ylabel='$Normalized$ \n $Performance$')
        ax.set_ylabel('$Speedup$', fontsize = 16.0)
        ax.grid( linestyle='-.', linewidth=0.5, alpha=.5, axis='y', zorder=0)
        #ax0.ylim([0,1.1])
        
        #plt.yscale('symlog', linthreshy=1)
        # Adding the legend and showing the plot

        # Set the position of the x ticks
        ax0.set_xticks([p + 2.5 * width for p in pos])
        ax0.set_yticks([0,.1,.2,.3,.4,.5,.6,.7,.8,.9,1,1.1,1.2])

        # Set the labels for the x ticks
        ax0.set_xticklabels(df['benchmark'])
        ax0.tick_params(axis="x", labelsize=20)
        ax0.tick_params(axis="y", labelsize=15)
        # Setting the x-axis and y-axis limits
        #plt.xlim(min(pos)-width, max(pos)+width*4)
        #ax0.set(ylabel='$Normalized$ \n $Performance$')
        ax0.set_ylabel('$Speedup$', fontsize = 16.0)
        #ax0.ylim([0,1.1])
        plt.xticks(rotation=55)
        #plt.yscale('symlog', linthreshy=1)
        # Adding the legend and showing the plot

        for tick in ax0.xaxis.get_majorticklabels():
            tick.set_horizontalalignment("right")

        plt.legend(bbox_to_anchor=(0,1.02,1,0.2), loc="lower left", mode="expand", borderaxespad=0, edgecolor='black', ncol=2, prop={"size":16, "weight":'bold'})
        ax0.grid( linestyle='-.', linewidth=0.5, alpha=.5, axis='y', zorder=0)
        plt.tight_layout()

        if not os.path.exists(file_type):
                os.makedirs(file_type)
        plt.savefig(file_type + '/NumCycles.png', bbox_inches='tight')
        plt.show()             
        """

def get_inst_results(ax, print_x_labels = True):
        
        results_index_file = open('./results_index.txt', mode='r')
        contents = results_index_file.read()
        results_index = contents.split("\n")

        final_results_performance["Raw"] = []
        final_results_performance["Super"] = []

        idx = 0
        #for bench in benchs:
        SpecSuperTotal = 1.0
        SpecRawTotal = 1.0
        NumOfSpec = 0
        ParsecSuperTotal = 1.0
        ParsecRawTotal = 1.0
        NumOfParsec = 0
        for i, bench in enumerate(benchs):

                SimOpsOverhead = 0.0 #should be zero
                total_weight = 0.0

                for chk_num in range(len(benchs[bench])):

                        if weights[bench][chk_num] == 0.0:
                                continue
                        
                        cap_stat_file_dir = './' + bench + '_' + type2 + '/m5out_sim_' + bench + '_' + type2 + "_" + str(chk_num+1) + '/stats.txt'
                        Super_stat_file_dir = './' + bench + '_' + type1 + '/m5out_sim_' + bench + '_' + type1 + '_' + str(chk_num+1) + '/stats.txt'
                        # print(cap_stat_file_dir)
                        # print(Super_stat_file_dir)
                        cap_stat_file = open(cap_stat_file_dir, mode='r')
                        Super_stat_file = open(Super_stat_file_dir, mode='r')
                        cap_stat_file_content = cap_stat_file.read()
                        Super_stat_file_content = Super_stat_file.read()

                        cap_results = measure(results_index, cap_stat_file_content)
                        Super_results = measure(results_index, Super_stat_file_content)
                        
                        if int(cap_results['sim_ops'][-1]) == 0 or int(Super_results['sim_ops'][-1]) == 0:
                                warnings.warn(bench + " simpoint " + str(chk_num+1) + " has not stat!")
                                cap_stat_file.close()
                                Super_stat_file.close()
                                continue
                        
                        total_weight = total_weight + weights[bench][chk_num]
                        print("SimOps Super: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", int(Super_results['sim_ops'][-1]))
                        print("SimOps Raw: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", int(cap_results['sim_ops'][-1]))
                        
                        
                        SimOpsOverhead = SimOpsOverhead + weights[bench][chk_num] * ((int(cap_results['sim_ops'][-1]))/int(Super_results['sim_ops'][-1]))
                        #print("SimOpsRaw: ", SimOpsOverhead)
                        #print('SimOpsSuper: ', total_weight) 
                        cap_stat_file.close()
                        Super_stat_file.close()

                        
                print("-------------------------------------------DONE----------------------------")
                if total_weight == 0:
                    final_results_performance["Super"].append(0)
                    final_results_performance["Raw"].append(0)
                    warnings.warn(bench + " Has no working simpoints!!")
                    continue
                
                print('SimOps Raw: ', SimOpsOverhead/total_weight)
                print('SimOps Super: ', total_weight,total_weight)   

                final_results_performance["Super"].append((SimOpsOverhead/total_weight))
                final_results_performance["Raw"].append(total_weight/total_weight)
                
                
                if i <= 10:
                    SpecSuperTotal *= SimOpsOverhead/total_weight
                    SpecRawTotal *= total_weight/total_weight
                    NumOfSpec += 1
                else:
                    ParsecSuperTotal *= SimOpsOverhead/total_weight
                    ParsecRawTotal *= total_weight/total_weight
                    NumOfParsec += 1


        final_results_performance["Super"].append(SpecSuperTotal**(1/NumOfSpec))
        final_results_performance["Raw"].append(SpecRawTotal**(1/NumOfSpec))
        # final_results_performance["Super"].append(ParsecSuperTotal/NumOfParsec)
        # final_results_performance["Raw"].append(ParsecRawTotal/NumOfParsec)
                

        


        df = pd.DataFrame(final_results_performance, columns = COLUMNS)
        # Setting the positions and width for the bars
        pos = list(range(len(df['Raw']))) 
        #print(pos)
        width = 0.25
            
        # Plotting the bars
        #fig = plt.figure(figsize=(10,6))
        #gs = gridspec.GridSpec(1, 1, height_ratios=[1]) 
        # print('AVG', final_results_performance['CHEx86ContextSensitive'])
        # print('GEOMEAN', 1 - gmean(final_results_performance['CHEx86ContextSensitive']))
        #ax0 = plt.subplot(gs[0])
        patterns = [ "*" , "+" , "x" , "o" , "\\" , "/", "o", "O", ".", "*" ]
        #colors = palettable.colorbrewer.diverging.Spectral_4.mpl_colors
        colors = palettable.scientific.sequential.Davos_6.mpl_colors


        for i in range(0,NUM_OF_TYPES+1):
                ax.bar([p + width*i for p in pos], 
                        df[x_axe[i]], 
                        #hatch=3*patterns[i],
                        # of width
                        width=width,
                        # with color
                        #edgecolor=colors[i], 
                        color=colors[i],
                        # with label the first value in benchmark
                        label=x_axe[i],
                        zorder=3) 
                ax.bar([p + width*i for p in pos], 
                        df[x_axe[i]], 
                        # of width
                        width=width, 
                        # with color
                        color='none', 
                        edgecolor='black',
                        zorder=4) 

        if (print_x_labels):
            ax.set_xticks([p + 2.5 * width for p in pos])                
            ax.set_xticklabels(df['benchmark'], rotation=55)
            for tick in ax.xaxis.get_majorticklabels():
                tick.set_horizontalalignment("right")
            ax.tick_params(axis="x", labelsize=25)
        else:
            ax.set_xticks([])

        ax.tick_params(axis="y", labelsize=25)
        ax.set_ylabel('$Normalized$ \n $Executed$ $Microops$', fontsize = 25.0)
        ax.grid( linestyle='-.', linewidth=0.5, alpha=.5, axis='y', zorder=0)
        
        """
        # Set the position of the x ticks
        ax0.set_xticks([p + 2.5 * width for p in pos])
        ax0.set_yticks([0,.1,.2,.3,.4,.5,.6,.7,.8,.9,1])


        # Set the labels for the x ticks
        ax0.set_xticklabels(df['benchmark'])
        ax0.tick_params(axis="x", labelsize=20)
        ax0.tick_params(axis="y", labelsize=15)
        # Setting the x-axis and y-axis limits
        #plt.xlim(min(pos)-width, max(pos)+width*4)
        #ax0.set(ylabel='$Normalized$ \n $Performance$')
        ax0.set_ylabel('$Normalized$ \n $Executed$ $Microops$', fontsize = 16.0)
        #ax0.ylim([0,1.1])
        plt.xticks(rotation=55)
        #plt.yscale('symlog', linthreshy=1)
        # Adding the legend and showing the plot

        for tick in ax0.xaxis.get_majorticklabels():
            tick.set_horizontalalignment("right")

        plt.legend(bbox_to_anchor=(0,1.02,1,0.2), loc="lower left", mode="expand", borderaxespad=0, edgecolor='black', ncol=2, prop={"size":16, "weight":'bold'})
        ax0.grid( linestyle='-.', linewidth=0.5, alpha=.5, axis='y', zorder=0)
        plt.tight_layout()

        if not os.path.exists(file_type):
                os.makedirs(file_type)
        plt.savefig(file_type + '/sim_ops.png', bbox_inches='tight')
        plt.show()  
        """           
        

def get_squash_results():
        
        results_index_file = open('./results_index.txt', mode='r')
        contents = results_index_file.read()
        results_index = contents.split("\n")


        final_results_performance["Raw"] = []
        final_results_performance["Super"] = []

        for bench in benchs:
        #for index, bench in enumerate(benchs):

                SquashedInstsCapCycles = 0.0 #should be zero
                SquashedInstsSuperCycles = 0.0 #should be zero
                total_weight = 0.0

                for chk_num in range(len(benchs[bench])):

                        if weights[bench][chk_num] == 0.0:
                                continue
                        
                        cap_stat_file_dir = './' + bench + '_' + type2 + '/m5out_sim_' + bench + '_' + type2 + "_" + str(chk_num+1) + '/stats.txt'
                        Super_stat_file_dir = './' + bench + '_' + type1 + '/m5out_sim_' + bench + '_' + type1 + '_' + str(chk_num+1) + '/stats.txt'
                        # print(cap_stat_file_dir)
                        # print(Super_stat_file_dir)
                        cap_stat_file = open(cap_stat_file_dir, mode='r')
                        Super_stat_file = open(Super_stat_file_dir, mode='r')
                        cap_stat_file_content = cap_stat_file.read()
                        Super_stat_file_content = Super_stat_file.read()

                        cap_results = measure(results_index, cap_stat_file_content)
                        Super_results = measure(results_index, Super_stat_file_content)

                        if int(cap_results['system.switch_cpus.decode.RunCycles'][-1]) == 0:
                                warnings.warn(bench + " simpoint " + str(chk_num+1) + " has not stat!")
                                cap_stat_file.close()
                                Super_stat_file.close()
                                continue
                        
                        total_weight = total_weight + weights[bench][chk_num]
                        print("Super: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", int(Super_results['system.switch_cpus.decode.RunCycles'][-1]))
                        print("Raw: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", int(cap_results['system.switch_cpus.decode.RunCycles'][-1]))
                        
                        cap_sq = (int(cap_results['system.switch_cpus.decode.SquashCycles'][-1]))/int(cap_results['system.switch_cpus.decode.RunCycles'][-1])
                        Super_sq = (int(Super_results['system.switch_cpus.decode.SquashCycles'][-1]))/int(Super_results['system.switch_cpus.decode.RunCycles'][-1])
                        SquashedInstsCapCycles = SquashedInstsCapCycles + weights[bench][chk_num] * (cap_sq)
                        SquashedInstsSuperCycles = SquashedInstsSuperCycles + weights[bench][chk_num] * (Super_sq)

                        

                        cap_stat_file.close()
                        Super_stat_file.close()

                if total_weight == 0:
                    final_results_squashed_insts["Super"].append(0)
                    final_results_squashed_insts["Raw"].append(0)
                    warnings.warn(bench + " Has no working simpoints!!")
                    continue

                print('SquashedInstsCap: ', SquashedInstsCapCycles/total_weight)
                print('SquashedInstsSuper: ', SquashedInstsSuperCycles/total_weight)   


                final_results_squashed_insts["Super"].append((SquashedInstsCapCycles/total_weight) * 100.0)
                final_results_squashed_insts["Raw"].append((SquashedInstsSuperCycles/total_weight) * 100.0)




        df = pd.DataFrame(final_results_squashed_insts, columns = COLUMNS)
        # Setting the positions and width for the bars
        pos = list(range(len(df['Raw']))) 
        #print(pos)
        width = 0.25
            
        # Plotting the bars
        fig = plt.figure(figsize=(10,6))
        gs = gridspec.GridSpec(2, 1, height_ratios=[1, 1]) 
        # print('AVG', final_results_performance['CHEx86ContextSensitive'])
        # print('GEOMEAN', 1 - gmean(final_results_performance['CHEx86ContextSensitive']))
        ax0 = plt.subplot(gs[0])
        patterns = [ "*" , "+" , "x" , "o" , "\\" , "/", "o", "O", ".", "*" ]
        #colors = palettable.colorbrewer.diverging.Spectral_4.mpl_colors
        colors = palettable.cartocolors.diverging.Earth_3.mpl_colors


        for i in range(0,NUM_OF_TYPES+1):
                ax0.bar([p + width*i for p in pos], 
                        df[x_axe[i]], 
                        #hatch=3*patterns[i],
                        # of width
                        width=width,
                        # with color
                        #edgecolor=colors[i], 
                        color=colors[i],
                        # with label the first value in benchmark
                        label=x_axe[i],
                        zorder=3) 
                ax0.bar([p + width*i for p in pos], 
                        df[x_axe[i]], 
                        # of width
                        width=width, 
                        # with color
                        color='none', 
                        edgecolor='black',
                        zorder=4) 


        # Set the position of the x ticks
        ax0.set_xticks([p + 0.5 * width for p in pos])

        # Set the labels for the x ticks
        ax0.set_xticklabels(df['benchmark'])
        ax0.tick_params(axis="x", labelsize=20)
        ax0.tick_params(axis="y", labelsize=15)
        ax0.set_ylabel('$Percentage$ $of$ $Time$\n $Spent$ $Squashing$', fontsize=16)
        plt.xticks(rotation=55)
        # Setting the x-axis and y-axis limits
        #plt.xlim(min(pos)-width, max(pos)+width*4)
        #plt.ylim([0,1.05])
        #plt.yscale('symlog', linthreshy=1)
        # Adding the legend and showing the plot
        for tick in ax0.xaxis.get_majorticklabels():
            tick.set_horizontalalignment("right")
        ax0.set_yticklabels(['{:.0f}\%'.format(x) for x in ax0.get_yticks()])
        plt.legend(bbox_to_anchor=(0,1.02,1,0.2), loc="lower left", mode="expand", borderaxespad=0, edgecolor='black', ncol=2, prop={"size":16, "weight":'bold'})
        ax0.grid( linestyle='-.', linewidth=0.5, alpha=.5, axis='y', zorder=0)
        plt.tight_layout()
        plt.subplots_adjust(hspace=.04)
        
        if not os.path.exists(file_type):
                os.makedirs(file_type)
        plt.savefig(file_type + '/Squash.png', bbox_inches='tight')
        plt.show() 


def get_accuracy_results():
        results_index_file = open('./results_index.txt', mode='r')
        contents = results_index_file.read()
        results_index = contents.split("\n")

        for bench in benchs:
        #for index, bench in enumerate(benchs):

                RawAccuracy = 0.0 #should be zero
                SuperAccuracy = 0.0
                total_weight = 0.0

                for chk_num in range(len(benchs[bench])):

                        if weights[bench][chk_num] == 0.0:
                                continue
                        
                        cap_stat_file_dir = './' + bench + '_' + type2 + '/m5out_sim_' + bench + '_' + type2 + "_" + str(chk_num+1) + '/stats.txt'
                        Super_stat_file_dir = './' + bench + '_' + type1 + '/m5out_sim_' + bench + '_' + type1 + '_' + str(chk_num+1) + '/stats.txt'
                        # print(cap_stat_file_dir)
                        # print(Super_stat_file_dir)
                        cap_stat_file = open(cap_stat_file_dir, mode='r')
                        Super_stat_file = open(Super_stat_file_dir, mode='r')
                        cap_stat_file_content = cap_stat_file.read()
                        Super_stat_file_content = Super_stat_file.read()

                        cap_results = measure(results_index, cap_stat_file_content)
                        Super_results = measure(results_index, Super_stat_file_content)

                        if float(cap_results['system.switch_cpus.loadPred.accuracy'][-1]) == 0.0 or float(Super_results['system.switch_cpus.loadPred.accuracy'][-1]) == 0.0:
                                warnings.warn(bench + " simpoint " + str(chk_num+1) + " has not stat!")
                                cap_stat_file.close()
                                Super_stat_file.close()
                                continue
                        
                        total_weight = total_weight + weights[bench][chk_num]
                        print("Super: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", float(Super_results['system.switch_cpus.loadPred.accuracy'][-1]))
                        print("Super: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", float(cap_results['system.switch_cpus.loadPred.accuracy'][-1]))
                        
                        cap_sq = (float(cap_results['system.switch_cpus.loadPred.accuracy'][-1]))
                        Super_sq = (float(Super_results['system.switch_cpus.loadPred.accuracy'][-1]))
                        RawAccuracy = RawAccuracy + weights[bench][chk_num] * (cap_sq)
                        SuperAccuracy = SuperAccuracy + weights[bench][chk_num] * (Super_sq)
                        

                        cap_stat_file.close()
                        Super_stat_file.close()

                if total_weight == 0:
                    final_results_acc["Super"].append(0)
                    final_results_acc["Super"].append(0)
                    warnings.warn(bench + " Has no working simpoints!!")
                    continue

                print('RawAccuracy: ', RawAccuracy/total_weight)
                print('SuperAccuracy: ', SuperAccuracy/total_weight)   


                final_results_acc["Super"].append((RawAccuracy/total_weight))
                final_results_acc["Super"].append((SuperAccuracy/total_weight))




        df = pd.DataFrame(final_results_acc, columns = COLUMNS_ACC_COV)
        # Setting the positions and width for the bars
        pos = list(range(len(df['Super']))) 
        #print(pos)
        width = 0.255
            
        # Plotting the bars
        fig = plt.figure(figsize=(10,6))
        gs = gridspec.GridSpec(2, 1, height_ratios=[1, 1]) 
        # print('AVG', final_results_performance['CHEx86ContextSensitive'])
        # print('GEOMEAN', 1 - gmean(final_results_performance['CHEx86ContextSensitive']))
        ax0 = plt.subplot(gs[0])


        patterns = [ "*" , "+" , "x" , "o" , "\\" , "/", "o", "O", ".", "*" ]
        #colors = palettable.colorbrewer.diverging.Spectral_4.mpl_colors
        colors = palettable.cartocolors.diverging.Earth_3.mpl_colors


        for i in range(0,2):
                ax0.bar([p + width*i for p in pos], 
                        df[x_axe_acc_cov[i]], 
                        #hatch=3*patterns[i],
                        # of width
                        width=width,
                        # with color
                        #edgecolor=colors[i], 
                        color=colors[i],
                        # with label the first value in benchmark
                        label=x_axe_label_acc_cov[i],
                        zorder=3) 
                ax0.bar([p + width*i for p in pos], 
                        df[x_axe_acc_cov[i]], 
                        # of width
                        width=width, 
                        # with color
                        color='none', 
                        edgecolor='black',
                        zorder=4) 

        # Set the position of the x ticks
        ax0.set_xticks([p + 0.5 * width for p in pos])

        # Set the labels for the x ticks
        #ax.set_xticklabels(df['benchmark'])
        #ax.tick_params(axis="x", labelsize=20)
        ax0.tick_params(axis="y", labelsize=15)
        # Setting the x-axis and y-axis limits
        #plt.xlim(min(pos)-width, max(pos)+width*4)
        ax0.set_ylabel('$Accuracy$', fontsize=16)
        #plt.xticks(rotation=55)
        #plt.ylim([0,1.05])
        #plt.yscale('symlog', linthreshy=1)
        # Adding the legend and showing the plot
        # for tick in ax.xaxis.get_majorticklabels():
        #     tick.set_horizontalalignment("right")
        #ax0.set_yticklabels(['{:.0f}\%'.format(x) for x in ax0.get_yticks()])
        ax0.legend( loc='upper right', bbox_to_anchor=(0.9, 1), edgecolor='black', fontsize=20,prop={"size":12, "weight":'bold'})
        ax0.grid( linestyle='-.', linewidth=0.5, alpha=.5, axis='y', zorder=0)
        #plt.tight_layout()

        #plt.savefig('LVPTMissRate.png', bbox_inches='tight')
        #plt.show()
        
        results_index_file = open('./results_index.txt', mode='r')
        contents = results_index_file.read()
        results_index = contents.split("\n")


        for bench in benchs:
        #for index, bench in enumerate(benchs):

                SuperCoverage = 0.0 #should be zero
                SuperCoverage = 0.0
                total_weight = 0.0

                for chk_num in range(len(benchs[bench])):

                        if weights[bench][chk_num] == 0.0:
                                continue
                        
                        cap_stat_file_dir = './' + bench + '_' + type2 + '/m5out_sim_' + bench + '_' + type2 + "_" + str(chk_num+1) + '/stats.txt'
                        Super_stat_file_dir = './' + bench + '_' + type1 + '/m5out_sim_' + bench + '_' + type1 + '_' + str(chk_num+1) + '/stats.txt'
                        # print(cap_stat_file_dir)
                        # print(Super_stat_file_dir)
                        cap_stat_file = open(cap_stat_file_dir, mode='r')
                        Super_stat_file = open(Super_stat_file_dir, mode='r')
                        cap_stat_file_content = cap_stat_file.read()
                        Super_stat_file_content = Super_stat_file.read()

                        cap_results = measure(results_index, cap_stat_file_content)
                        Super_results = measure(results_index, Super_stat_file_content)

                        if float(cap_results['system.switch_cpus.loadPred.coverage'][-1]) == 0.0 or float(Super_results['system.switch_cpus.loadPred.coverage'][-1]) == 0.0:
                                warnings.warn(bench + " simpoint " + str(chk_num+1) + " has not stat!")
                                cap_stat_file.close()
                                Super_stat_file.close()
                                continue
                        
                        total_weight = total_weight + weights[bench][chk_num]
                        print("Super: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", float(Super_results['system.switch_cpus.loadPred.coverage'][-1]))
                        print("Super: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", float(cap_results['system.switch_cpus.loadPred.coverage'][-1]))
                        
                        cap_sq = (float(cap_results['system.switch_cpus.loadPred.coverage'][-1]))
                        Super_sq = (float(Super_results['system.switch_cpus.loadPred.coverage'][-1]))
                        SuperCoverage = SuperCoverage + weights[bench][chk_num] * (cap_sq)
                        SuperCoverage = SuperCoverage + weights[bench][chk_num] * (Super_sq)

                        

                        cap_stat_file.close()
                        Super_stat_file.close()

                if total_weight == 0:
                    final_results_cov["Super"].append(0)
                    final_results_cov["Super"].append(0)
                    warnings.warn(bench + " Has no working simpoints!!")
                    continue

                print('SuperCoverage: ', SuperCoverage/total_weight)
                print('SuperCoverage: ', SuperCoverage/total_weight)   


                final_results_cov["Super"].append((SuperCoverage/total_weight))
                final_results_cov["Super"].append((SuperCoverage/total_weight))



        
        df = pd.DataFrame(final_results_cov, columns = COLUMNS_ACC_COV)
        #df_squash_cycles = pd.DataFrame(final_results_squashed_cycles, columns = COLUMNS_SQUASHED)
        # Setting the positions and width for the bars
        pos = list(range(len(df['Super']))) 
        #print(pos)
        width = 0.255
            
        # Plotting the bars
        ax1 = plt.subplot(gs[1], sharex = ax0)


        patterns = [ "*" , "+" , "x" , "o" , "\\" , "/", "o", "O", ".", "*" ]
        #colors = palettable.colorbrewer.diverging.Spectral_4.mpl_colors
        colors = palettable.cartocolors.diverging.Earth_3.mpl_colors


        for i in range(0,2):
                ax1.bar([p + width*i for p in pos], 
                        df[x_axe_acc_cov[i]], 
                        #hatch=3*patterns[i],
                        # of width
                        width=width,
                        # with color
                        #edgecolor=colors[i], 
                        color=colors[i],
                        # with label the first value in benchmark
                        label=x_axe_label_acc_cov[i],
                        zorder=3) 
                ax1.bar([p + width*i for p in pos], 
                        df[x_axe_acc_cov[i]], 
                        # of width
                        width=width, 
                        # with color
                        color='none', 
                        edgecolor='black',
                        zorder=4) 


        # Set the position of the x ticks
        ax1.set_xticks([p + 0.5 * width for p in pos])

        # Set the labels for the x ticks
        ax1.set_xticklabels(df['benchmark'])
        ax1.tick_params(axis="x", labelsize=20)
        ax1.tick_params(axis="y", labelsize=15)
        ax1.set_ylabel('$Coverage$', fontsize=16)
        plt.xticks(rotation=55)
        # Setting the x-axis and y-axis limits
        #plt.xlim(min(pos)-width, max(pos)+width*4)
        #plt.ylim([0,1.05])
        #plt.yscale('symlog', linthreshy=1)
        # Adding the legend and showing the plot
        for tick in ax1.xaxis.get_majorticklabels():
            tick.set_horizontalalignment("right")
        #ax1.set_yticklabels(['{:.0f}\%'.format(x) for x in ax1.get_yticks()])
        ax1.legend( loc='upper right', bbox_to_anchor=(0.9, 1), edgecolor='black', fontsize=20, prop={"size":12, "weight":'bold'})
        ax1.grid( linestyle='-.', linewidth=0.5, alpha=.5, axis='y', zorder=0)
        plt.tight_layout()
        plt.subplots_adjust(hspace=.04)

        if not os.path.exists(file_type):
                os.makedirs(file_type)
        plt.savefig(file_type + '/AccuracyCoverage.png', bbox_inches='tight')
        plt.show()                                       


def get_streamedfrom_results(ax0):
        
        results_index_file = open('./results_index.txt', mode='r')
        contents = results_index_file.read()
        results_index = contents.split("\n")

        final_results_performance["Raw"] = []
        final_results_performance["Super"] = []

        #for bench in benchs:
        SpecSuperTotal = [0.0, 0.0, 0.0]
        SpecRawTotal = [0.0, 0.0, 0.0]
        NumOfSpec = 0
        ParsecSuperTotal = [0.0, 0.0, 0.0]
        ParsecRawTotal = [0.0, 0.0, 0.0]
        NumOfParsec = 0
        for i, bench in enumerate(benchs):

                # spec, uop, icache
                Super_full = [0,0,0]
                Raw_full = [0,0,0]
                total_weight = 0.0

                for chk_num in range(len(benchs[bench])):

                        if weights[bench][chk_num] == 0.0:
                                continue
                        
                        cap_stat_file_dir = './' + bench + '_' + type2 + '/m5out_sim_' + bench + '_' + type2 + "_" + str(chk_num+1) + '/stats.txt'
                        Super_stat_file_dir = './' + bench + '_' + type1 + '/m5out_sim_' + bench + '_' + type1 + "_" + str(chk_num+1) + '/stats.txt'
                        # print(cap_stat_file_dir)
                        # print(Super_stat_file_dir)
                        cap_stat_file = open(cap_stat_file_dir, mode='r')
                        Super_stat_file = open(Super_stat_file_dir, mode='r')
                        cap_stat_file_content = cap_stat_file.read()
                        Super_stat_file_content = Super_stat_file.read()

                        cap_results = measure(results_index, cap_stat_file_content)
                        Super_results = measure(results_index, Super_stat_file_content)
                        
                        Raw_ops = 0
                        Raw_Uops = 0
                        Raw_Specops = 0
                        Super_ops = 0
                        Super_Uops = 0
                            

                        if i <= 10:
                            Raw_ops = int(cap_results['system.switch_cpus.fetch.Ops'][-1])
                            Raw_Uops = int(cap_results['system.switch_cpus.fetch.uopCacheHitOps'][-1])
                            Raw_Specops = int(cap_results['system.switch_cpus.fetch.specCacheHitOps'][-1])
    
                            Super_ops = int(Super_results['system.switch_cpus.fetch.Ops'][-1])
                            Super_Uops = int(Super_results['system.switch_cpus.fetch.uopCacheHitOps'][-1])
                            #Super_Specops = 0
                        else:
                            Raw_ops = int(cap_results['system.cpu.fetch.Ops'][-1])
                            Raw_Uops = int(cap_results['system.cpu.fetch.uopCacheHitOps'][-1])
                            Raw_Specops = int(cap_results['system.cpu.fetch.specCacheHitOps'][-1])
    
                            Super_ops = int(Super_results['system.cpu.fetch.Ops'][-1])
                            Super_Uops = int(Super_results['system.cpu.fetch.uopCacheHitOps'][-1])
                            


                        if Raw_ops == 0 or Super_ops == 0:
                                warnings.warn(bench + " simpoint " + str(chk_num+1) + " has no stat!")
                                cap_stat_file.close()
                                Super_stat_file.close()
                                continue
                        
                        total_weight = total_weight + weights[bench][chk_num]
                        #print("Raw: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", int(cap_results['system.switch_cpus.numCycles'][-1]))
                        
                        Super_full[1] += weights[bench][chk_num] * Super_Uops#/Super_ops
                        Super_full[2] += weights[bench][chk_num] * (Super_ops - Super_Uops)#/Super_ops

                        Raw_full[0] += weights[bench][chk_num] * Raw_Specops#/Raw_ops
                        Raw_full[1] += weights[bench][chk_num] * Raw_Uops#/Raw_ops
                        Raw_full[2] += weights[bench][chk_num] * (Raw_ops - Raw_Uops - Raw_Specops)#/Raw_ops
                        
                        #PerformanceOverheadCap = PerformanceOverheadCap + weights[bench][chk_num] * ((int(cap_results['system.switch_cpus.numCycles'][-1]))/int(Super_results['system.switch_cpus.numCycles'][-1]))
                        #if int(Super_results['system.switch_cpus.rename.IQFullEvents'][-1]) != 0:
                        print("Super_full: ", Super_full)
                        print("Raw_full: ", Raw_full)
                        cap_stat_file.close()

                        

                print("-------------------------------------------DONE----------------------------")
                if total_weight == 0:
                    final_results_performance["Raw"].append([0,0,0])
                    final_results_performance["Super"].append([0,0,0])
                    warnings.warn(bench + " Has no working simpoints!!")
                    continue

                #print('PerformanceOverheadCap: ', PerformanceOverheadCap/total_weight)
                #print('PerformanceOverheadSuper: ', total_weight,total_weight)   

                #final_results_performance["Super"].append((PerformanceOverheadCap/total_weight))
                for j in range(3):
                        Super_full[j] /= total_weight
                        Raw_full[j] /= total_weight

                final_results_performance["Raw"].append(Super_full)
                final_results_performance["Super"].append(Raw_full)

                if i <= 10:
                    for j in range(3):
                        SpecSuperTotal[j] += Raw_full[j]
                        SpecRawTotal[j] += Super_full[j]
                    NumOfSpec += 1
                else:
                    for j in range(3):
                        ParsecSuperTotal[j] += Raw_full[j]
                        ParsecRawTotal[j] += Super_full[j]
                    NumOfParsec += 1

        #for j in range(3):
        #        SpecSuperTotal[j] /= NumOfSpec
        #        SpecRawTotal[j] /= NumOfSpec
        #        ParsecSuperTotal[j] /= NumOfParsec
        #        ParsecRawTotal[j] /= NumOfParsec

        # final_results_performance["Super"].append(SpecSuperTotal)
        # final_results_performance["Raw"].append(SpecRawTotal)
        # final_results_performance["Super"].append(ParsecSuperTotal)
        # final_results_performance["Raw"].append(ParsecRawTotal)

                
        Raw = np.array(final_results_performance["Raw"]).T.tolist()
        RawOpt = np.array(final_results_performance["Super"]).T.tolist()

        df = pd.DataFrame(final_results_performance, columns = COLUMNS)
        
        #print(df)
        # Setting the positions and width for the bars
        pos = list(range(len(df["benchmark"]))) 
        #print(pos)
        width = 0.25

            
        # Plotting the bars
        #fig = plt.figure(figsize=(10,6))
        #gs = gridspec.GridSpec(1, 1, height_ratios=[1]) 
        # print('AVG', final_results_performance['CHEx86ContextSensitive'])
        # print('GEOMEAN', 1 - gmean(final_results_performance['CHEx86ContextSensitive']))
        #ax0 = plt.subplot(gs[0])
        patterns = [ "*" , "+" , "x" , "o" , "\\" , "/", "o", "O", ".", "*" ]
        #colors = palettable.colorbrewer.diverging.Spectral_4.mpl_colors
        colors = palettable.cartocolors.diverging.Earth_4.mpl_colors

        bar1 = list(range(len(df["benchmark"]))) 
        bar2 = [i + (width*1) for i in bar1]

        #i_bot = list(np.add(Raw[0], Raw[1]))
        #ax0.bar(bar1, Raw[0], width, label="Optimized Partition", color=colors[0])#, hatch=3*patterns[0], edgecolor=colors[0])
        #ax0.bar(bar1, Raw[1], width, bottom=Raw[0], label="Unoptimized Partition", color=colors[1])#, hatch=3*patterns[1], edgecolor=colors[1])
        #ax0.bar(bar1, Raw[2], width, bottom=i_bot, label="I Cache", color=colors[2])#, hatch=3*patterns[2], edgecolor=colors[2])

        #i_bot = list(np.add(RawOpt[0], RawOpt[1]))
        #ax0.bar(bar2, RawOpt[0], width, color=colors[0])#hatch=3*patterns[0], edgecolor=colors[0])
        #ax0.bar(bar2, RawOpt[1], width, bottom=RawOpt[0], color=colors[1])#, hatch=3*patterns[1], edgecolor=colors[1])
        #ax0.bar(bar2, RawOpt[2], width, bottom=i_bot, color=colors[2])#, hatch=3*patterns[2], edgecolor=colors[2])

        ax0.bar(bar1, Raw[1], width, label="Unoptimized Partition", color=colors[2])#, hatch=3*patterns[1], edgecolor=colors[1])
        ax0.bar(bar1, Raw[2], width, bottom=Raw[1], label="I Cache", color=colors[3])#, hatch=3*patterns[2], edgecolor=colors[2])

        ax0.bar(bar2, RawOpt[1], width, color=colors[2])#, hatch=3*patterns[1], edgecolor=colors[1])
        ax0.bar(bar2, RawOpt[2], width, bottom=RawOpt[1], color=colors[3])#, hatch=3*patterns[2], edgecolor=colors[2])


        # Set the position of the x ticks
        ax0.set_xticks([p + 2 * width for p in bar1])

        # Set the labels for the x ticks
        ax0.set_xticklabels(df['benchmark'])
        ax0.tick_params(axis="x", labelsize=30)
        ax0.tick_params(axis="y", labelsize=25)
        # Setting the x-axis and y-axis limits
        #plt.xlim(min(pos)-width, max(pos)+width*4)
        #ax0.set(ylabel='$Normalized$ \n $Performance$')
        #ax0.set_ylabel('$Normalized$ \n $Execution$ $Time$', fontsize = 16.0)
        ax0.set_ylabel('$Number$ $of$\n$\u03BCops$ $Streamed$', fontsize = 25.0)
        #ax0.ylim([0,1.1])
        plt.xticks(rotation=55)
        for tick in ax0.xaxis.get_majorticklabels():
                tick.set_horizontalalignment("right")
        #plt.yscale('symlog', linthreshy=1)
        #plt.yscale('linear')
        # Adding the legend and showing the plot

        #for tick in ax0.xaxis.get_majorticklabels():
        #    tick.set_horizontalalignment("right")

        #plt.legend(bbox_to_anchor=(0,1.05,1,0.2), loc="lower left", mode="expand", borderaxespad=0, edgecolor='black', ncol=3, prop={"size":12, "weight":'bold'})
        #ax0.grid( linestyle='-.', linewidth=0.5, alpha=.5, axis='y', zorder=0)
        #plt.tight_layout()

        #if not os.path.exists(file_type):
        #        os.makedirs(file_type)
        #plt.savefig(file_type + '/Streaming.png', bbox_inches='tight')
        #plt.show()
        ax0.legend(["µop cache hit ops","µop cache miss ops"], bbox_to_anchor=(0,1.05,1,0.2), loc="lower left", mode="expand", borderaxespad=0, edgecolor='black', ncol=2, prop={"size":25, "weight":'bold'})
        ax0.grid( linestyle='-.', linewidth=0.5, alpha=.5, axis='y', zorder=0)
 


def get_fetch_blocked_results():
        
        results_index_file = open('./results_index.txt', mode='r')
        contents = results_index_file.read()
        results_index = contents.split("\n")

        final_results_performance["Raw"] = []
        final_results_performance["Super"] = []

        #for bench in benchs:
        for bench in benchs:

                # squash, block, other
                Super_full = [0,0,0]
                Raw_full = [0,0,0]
                total_weight = 0.0

                for chk_num in range(len(benchs[bench])):

                        if weights[bench][chk_num] == 0.0:
                                continue
                        
                        cap_stat_file_dir = './' + bench + '_' + type2 + '/m5out_sim_' + bench + '_' + type2 + "_" + str(chk_num+1) + '/stats.txt'
                        Super_stat_file_dir = './' + bench + '_' + type1 + '/m5out_sim_' + bench + '_' + type1 + "_" + str(chk_num+1) + '/stats.txt'
                        # print(cap_stat_file_dir)
                        # print(Super_stat_file_dir)
                        cap_stat_file = open(cap_stat_file_dir, mode='r')
                        Super_stat_file = open(Super_stat_file_dir, mode='r')
                        cap_stat_file_content = cap_stat_file.read()
                        Super_stat_file_content = Super_stat_file.read()

                        cap_results = measure(results_index, cap_stat_file_content)
                        Super_results = measure(results_index, Super_stat_file_content)

                        Raw_squash = int(cap_results['system.switch_cpus.fetch.SquashCycles'][-1])
                        Raw_numCycles = int(cap_results['system.switch_cpus.numCycles'][-1])
                        Raw_cycles = int(cap_results['system.switch_cpus.fetch.Cycles'][-1])
                        
                        Super_squash = int(Super_results['system.switch_cpus.fetch.SquashCycles'][-1])
                        Super_numCycles = int(Super_results['system.switch_cpus.numCycles'][-1])
                        Super_cycles = int(Super_results['system.switch_cpus.fetch.Cycles'][-1])

                        if Raw_numCycles == 0 or Super_numCycles == 0:
                                warnings.warn(bench + " simpoint " + str(chk_num+1) + " has no stat!")
                                cap_stat_file.close()
                                Super_stat_file.close()
                                continue
                        
                        total_weight = total_weight + weights[bench][chk_num]
                        #print("Raw: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", int(cap_results['system.switch_cpus.numCycles'][-1]))
                        
                        Super_full[0] += weights[bench][chk_num] * Super_squash #/ Super_numCycles
                        Super_full[1] += weights[bench][chk_num] * (Super_numCycles - Super_cycles - Super_squash) #/ Super_numCycles
                        Super_full[2] += weights[bench][chk_num] * (Super_cycles) #/ Super_numCycles

                        Raw_full[0] += weights[bench][chk_num] * Raw_squash #/ Raw_numCycles
                        Raw_full[1] += weights[bench][chk_num] * (Raw_numCycles - Raw_cycles - Raw_squash) #/ Raw_numCycles
                        Raw_full[2] += weights[bench][chk_num] * (Raw_cycles) #/ Raw_numCycles
                        
                        #PerformanceOverheadCap = PerformanceOverheadCap + weights[bench][chk_num] * ((int(cap_results['system.switch_cpus.numCycles'][-1]))/int(Super_results['system.switch_cpus.numCycles'][-1]))
                        #if int(Super_results['system.switch_cpus.rename.IQFullEvents'][-1]) != 0:
                        print("Super_full: ", Super_full)
                        print("Raw_full: ", Raw_full)
                        cap_stat_file.close()

                        

                print("-------------------------------------------DONE----------------------------")
                if total_weight == 0:
                    final_results_performance["Raw"].append([0,0,0])
                    final_results_performance["Super"].append([0,0,0])
                    warnings.warn(bench + " Has no working simpoints!!")
                    continue

                #print('PerformanceOverheadCap: ', PerformanceOverheadCap/total_weight)
                #print('PerformanceOverheadSuper: ', total_weight,total_weight)   

                #final_results_performance["Super"].append((PerformanceOverheadCap/total_weight))
                for i in range(3):
                        Super_full[i] /= total_weight
                        Raw_full[i] /= total_weight

                final_results_performance["Raw"].append(Super_full)
                final_results_performance["Super"].append(Raw_full)
                
        Raw = np.array(final_results_performance["Raw"]).T.tolist()
        RawOpt = np.array(final_results_performance["Super"]).T.tolist()

        df = pd.DataFrame(final_results_performance, columns = COLUMNS)
        
        print(df)
        # Setting the positions and width for the bars
        pos = list(range(len(df["benchmark"]))) 
        #print(pos)
        width = 0.25

            
        # Plotting the bars
        fig = plt.figure(figsize=(10,6))
        gs = gridspec.GridSpec(1, 1, height_ratios=[1]) 
        # print('AVG', final_results_performance['CHEx86ContextSensitive'])
        # print('GEOMEAN', 1 - gmean(final_results_performance['CHEx86ContextSensitive']))
        ax0 = plt.subplot(gs[0])
        patterns = [ "*" , "+" , "x" , "o" , "\\" , "/", "o", "O", ".", "*" ]
        #colors = palettable.colorbrewer.diverging.Spectral_4.mpl_colors
        colors = palettable.cartocolors.diverging.Earth_3.mpl_colors

        bar1 = list(range(len(df["benchmark"]))) 
        bar2 = [i + (width*1) for i in bar1]

        #i_bot = list(np.add(Raw[0], Raw[1]))
        #ax0.bar(bar1, Raw[0], width, label="Squashing", color='green')
        #ax0.bar(bar1, Raw[1], width, bottom=Raw[0], label="Blocking", color='blue')
        #ax0.bar(bar1, Raw[2], width, bottom=i_bot, label="Other", color='red')

        #i_bot = list(np.add(RawOpt[0], RawOpt[1]))
        #ax0.bar(bar2, RawOpt[0], width, color='green')
        #ax0.bar(bar2, RawOpt[1], width, bottom=RawOpt[0], color='blue')
        #ax0.bar(bar2, RawOpt[2], width, bottom=i_bot, color='red')

        i_bot = list(np.add(Raw[0], Raw[1]))
        ax0.bar(bar1, Raw[0], width, label="Squashing", color='none', hatch=3*patterns[0], edgecolor=colors[0])
        ax0.bar(bar1, Raw[1], width, bottom=Raw[0], label="Blocking", color='none', hatch=3*patterns[1], edgecolor=colors[1])
        ax0.bar(bar1, Raw[2], width, bottom=i_bot, label="Other", color='none', hatch=3*patterns[2], edgecolor=colors[2])

        i_bot = list(np.add(RawOpt[0], RawOpt[1]))
        ax0.bar(bar2, RawOpt[0], width, color='none', hatch=3*patterns[0], edgecolor=colors[0])
        ax0.bar(bar2, RawOpt[1], width, bottom=RawOpt[0], color='none', hatch=3*patterns[1], edgecolor=colors[1])
        ax0.bar(bar2, RawOpt[2], width, bottom=i_bot, color='none', hatch=3*patterns[2], edgecolor=colors[2])


        # Set the position of the x ticks
        ax0.set_xticks([p + 2.5 * width for p in bar1])

        # Set the labels for the x ticks
        ax0.set_xticklabels(df['benchmark'])
        ax0.tick_params(axis="x", labelsize=20)
        ax0.tick_params(axis="y", labelsize=15)
        # Setting the x-axis and y-axis limits
        #plt.xlim(min(pos)-width, max(pos)+width*4)
        #ax0.set(ylabel='$Normalized$ \n $Performance$')
        #ax0.set_ylabel('$Normalized$ \n $Execution$ $Time$', fontsize = 16.0)
        ax0.set_ylabel('Number of Cycles', fontsize = 16.0)
        #ax0.ylim([0,1.1])
        plt.xticks(rotation=55)
        #plt.yscale('symlog', linthreshy=1)
        plt.yscale('linear')
        # Adding the legend and showing the plot

        for tick in ax0.xaxis.get_majorticklabels():
            tick.set_horizontalalignment("right")

        plt.legend(bbox_to_anchor=(0,1.05,1,0.2), loc="lower left", mode="expand", borderaxespad=0, edgecolor='black', ncol=2, prop={"size":16, "weight":'bold'})
        ax0.grid( linestyle='-.', linewidth=0.5, alpha=.5, axis='y', zorder=0)
        plt.tight_layout()

        if not os.path.exists(file_type):
                os.makedirs(file_type)
        plt.savefig(file_type + '/BlockedFetch.png', bbox_inches='tight')
        plt.show()
 


def get_rename_blocked_results():
        
        results_index_file = open('./results_index.txt', mode='r')
        contents = results_index_file.read()
        results_index = contents.split("\n")

        final_results_performance["Raw"] = []
        final_results_performance["Super"] = []

        #for bench in benchs:
        for bench in benchs:

                # squash, block, other
                Super_full = [0,0,0]
                Raw_full = [0,0,0]
                total_weight = 0.0

                for chk_num in range(len(benchs[bench])):

                        if weights[bench][chk_num] == 0.0:
                                continue
                        
                        cap_stat_file_dir = './' + bench + '_' + type2 + '/m5out_sim_' + bench + '_' + type2 + "_" + str(chk_num+1) + '/stats.txt'
                        Super_stat_file_dir = './' + bench + '_' + type1 + '/m5out_sim_' + bench + '_' + type1 + "_" + str(chk_num+1) + '/stats.txt'
                        # print(cap_stat_file_dir)
                        # print(Super_stat_file_dir)
                        cap_stat_file = open(cap_stat_file_dir, mode='r')
                        Super_stat_file = open(Super_stat_file_dir, mode='r')
                        cap_stat_file_content = cap_stat_file.read()
                        Super_stat_file_content = Super_stat_file.read()

                        cap_results = measure(results_index, cap_stat_file_content)
                        Super_results = measure(results_index, Super_stat_file_content)

                        Raw_squash = int(cap_results['system.switch_cpus.rename.SquashCycles'][-1])
                        Raw_block = int(cap_results['system.switch_cpus.rename.BlockCycles'][-1])
                        Raw_numCycles = int(cap_results['system.switch_cpus.numCycles'][-1])
                        
                        Super_squash = int(Super_results['system.switch_cpus.rename.SquashCycles'][-1])
                        Super_block = int(Super_results['system.switch_cpus.rename.BlockCycles'][-1])
                        Super_numCycles = int(Super_results['system.switch_cpus.numCycles'][-1])

                        if Raw_numCycles == 0 or Super_numCycles == 0:
                                warnings.warn(bench + " simpoint " + str(chk_num+1) + " has no stat!")
                                cap_stat_file.close()
                                Super_stat_file.close()
                                continue
                        
                        total_weight = total_weight + weights[bench][chk_num]
                        #print("Raw: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", int(cap_results['system.switch_cpus.numCycles'][-1]))
                        
                        Super_full[0] += weights[bench][chk_num] * Super_squash #/ Super_numCycles
                        Super_full[1] += weights[bench][chk_num] * Super_block #/ Super_numCycles
                        Super_full[2] += weights[bench][chk_num] * (Super_numCycles - Super_squash - Super_block) #/ Super_numCycles

                        Raw_full[0] += weights[bench][chk_num] * Raw_squash #/ Raw_numCycles
                        Raw_full[1] += weights[bench][chk_num] * Raw_block #/ Raw_numCycles
                        Raw_full[2] += weights[bench][chk_num] * (Raw_numCycles - Raw_squash - Raw_block) #/ Raw_numCycles
                        
                        #PerformanceOverheadCap = PerformanceOverheadCap + weights[bench][chk_num] * ((int(cap_results['system.switch_cpus.numCycles'][-1]))/int(Super_results['system.switch_cpus.numCycles'][-1]))
                        #if int(Super_results['system.switch_cpus.rename.IQFullEvents'][-1]) != 0:
                        print("Super_full: ", Super_full)
                        print("Raw_full: ", Raw_full)
                        cap_stat_file.close()

                        

                print("-------------------------------------------DONE----------------------------")
                if total_weight == 0:
                    final_results_performance["Raw"].append([0,0,0])
                    final_results_performance["Super"].append([0,0,0])
                    warnings.warn(bench + " Has no working simpoints!!")
                    continue

                #print('PerformanceOverheadCap: ', PerformanceOverheadCap/total_weight)
                #print('PerformanceOverheadSuper: ', total_weight,total_weight)   

                #final_results_performance["Super"].append((PerformanceOverheadCap/total_weight))
                for i in range(3):
                        Super_full[i] /= total_weight
                        Raw_full[i] /= total_weight

                final_results_performance["Raw"].append(Super_full)
                final_results_performance["Super"].append(Raw_full)
                
        Raw = np.array(final_results_performance["Raw"]).T.tolist()
        RawOpt = np.array(final_results_performance["Super"]).T.tolist()

        df = pd.DataFrame(final_results_performance, columns = COLUMNS)
        
        print(df)
        # Setting the positions and width for the bars
        pos = list(range(len(df["benchmark"]))) 
        #print(pos)
        width = 0.25

            
        # Plotting the bars
        fig = plt.figure(figsize=(10,6))
        gs = gridspec.GridSpec(1, 1, height_ratios=[1]) 
        # print('AVG', final_results_performance['CHEx86ContextSensitive'])
        # print('GEOMEAN', 1 - gmean(final_results_performance['CHEx86ContextSensitive']))
        ax0 = plt.subplot(gs[0])
        patterns = [ "*" , "+" , "x" , "o" , "\\" , "/", "o", "O", ".", "*" ]
        #colors = palettable.colorbrewer.diverging.Spectral_4.mpl_colors
        colors = palettable.cartocolors.diverging.Earth_3.mpl_colors

        bar1 = list(range(len(df["benchmark"]))) 
        bar2 = [i + (width*1) for i in bar1]

        #i_bot = list(np.add(Raw[0], Raw[1]))
        #ax0.bar(bar1, Raw[0], width, label="Squashing", color='green')
        #ax0.bar(bar1, Raw[1], width, bottom=Raw[0], label="Blocking", color='blue')
        #ax0.bar(bar1, Raw[2], width, bottom=i_bot, label="Other", color='red')

        #i_bot = list(np.add(RawOpt[0], RawOpt[1]))
        #ax0.bar(bar2, RawOpt[0], width, color='green')
        #ax0.bar(bar2, RawOpt[1], width, bottom=RawOpt[0], color='blue')
        #ax0.bar(bar2, RawOpt[2], width, bottom=i_bot, color='red')

        i_bot = list(np.add(Raw[0], Raw[1]))
        ax0.bar(bar1, Raw[0], width, label="Squashing", color='none', hatch=3*patterns[0], edgecolor=colors[0])
        ax0.bar(bar1, Raw[1], width, bottom=Raw[0], label="Blocking", color='none', hatch=3*patterns[1], edgecolor=colors[1])
        ax0.bar(bar1, Raw[2], width, bottom=i_bot, label="Other", color='none', hatch=3*patterns[2], edgecolor=colors[2])

        i_bot = list(np.add(RawOpt[0], RawOpt[1]))
        ax0.bar(bar2, RawOpt[0], width, color='none', hatch=3*patterns[0], edgecolor=colors[0])
        ax0.bar(bar2, RawOpt[1], width, bottom=RawOpt[0], color='none', hatch=3*patterns[1], edgecolor=colors[1])
        ax0.bar(bar2, RawOpt[2], width, bottom=i_bot, color='none', hatch=3*patterns[2], edgecolor=colors[2])


        # Set the position of the x ticks
        ax0.set_xticks([p + 2.5 * width for p in bar1])

        # Set the labels for the x ticks
        ax0.set_xticklabels(df['benchmark'])
        ax0.tick_params(axis="x", labelsize=20)
        ax0.tick_params(axis="y", labelsize=15)
        # Setting the x-axis and y-axis limits
        #plt.xlim(min(pos)-width, max(pos)+width*4)
        #ax0.set(ylabel='$Normalized$ \n $Performance$')
        #ax0.set_ylabel('$Normalized$ \n $Execution$ $Time$', fontsize = 16.0)
        ax0.set_ylabel('Number of Cycles', fontsize = 16.0)
        #ax0.ylim([0,1.1])
        plt.xticks(rotation=55)
        #plt.yscale('symlog', linthreshy=1)
        plt.yscale('linear')
        # Adding the legend and showing the plot

        for tick in ax0.xaxis.get_majorticklabels():
            tick.set_horizontalalignment("right")

        plt.legend(bbox_to_anchor=(0,1.05,1,0.2), loc="lower left", mode="expand", borderaxespad=0, edgecolor='black', ncol=2, prop={"size":16, "weight":'bold'})
        ax0.grid( linestyle='-.', linewidth=0.5, alpha=.5, axis='y', zorder=0)
        plt.tight_layout()

        if not os.path.exists(file_type):
                os.makedirs(file_type)
        plt.savefig(file_type + '/BlockedRename.png', bbox_inches='tight')
        plt.show()
 


def get_iew_blocked_results():
        
        results_index_file = open('./results_index.txt', mode='r')
        contents = results_index_file.read()
        results_index = contents.split("\n")

        final_results_performance["Raw"] = []
        final_results_performance["Super"] = []

        #for bench in benchs:
        for bench in benchs:

                # squash, block, other
                Super_full = [0,0,0]
                Raw_full = [0,0,0]
                total_weight = 0.0

                for chk_num in range(len(benchs[bench])):

                        if weights[bench][chk_num] == 0.0:
                                continue
                        
                        cap_stat_file_dir = './' + bench + '_' + type2 + '/m5out_sim_' + bench + '_' + type2 + "_" + str(chk_num+1) + '/stats.txt'
                        Super_stat_file_dir = './' + bench + '_' + type1 + '/m5out_sim_' + bench + '_' + type1 + "_" + str(chk_num+1) + '/stats.txt'
                        # print(cap_stat_file_dir)
                        # print(Super_stat_file_dir)
                        cap_stat_file = open(cap_stat_file_dir, mode='r')
                        Super_stat_file = open(Super_stat_file_dir, mode='r')
                        cap_stat_file_content = cap_stat_file.read()
                        Super_stat_file_content = Super_stat_file.read()

                        cap_results = measure(results_index, cap_stat_file_content)
                        Super_results = measure(results_index, Super_stat_file_content)

                        #print(cap_results)
                        # print(Super_stat_file_dir)

                        Raw_squash = int(cap_results['system.switch_cpus.iew.iewSquashCycles'][-1])
                        Raw_block = int(cap_results['system.switch_cpus.iew.iewBlockCycles'][-1])
                        Raw_numCycles = int(cap_results['system.switch_cpus.numCycles'][-1])
                        
                        Super_squash = int(Super_results['system.switch_cpus.iew.iewSquashCycles'][-1])
                        Super_block = int(Super_results['system.switch_cpus.iew.iewBlockCycles'][-1])
                        Super_numCycles = int(Super_results['system.switch_cpus.numCycles'][-1])

                        if Raw_numCycles == 0 or Super_numCycles == 0:
                                warnings.warn(bench + " simpoint " + str(chk_num+1) + " has no stat!")
                                cap_stat_file.close()
                                Super_stat_file.close()
                                continue
                        
                        total_weight = total_weight + weights[bench][chk_num]
                        #print("Raw: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", int(cap_results['system.switch_cpus.numCycles'][-1]))
                        
                        Super_full[0] += weights[bench][chk_num] * Super_squash #/ Super_numCycles
                        Super_full[1] += weights[bench][chk_num] * Super_block #/ Super_numCycles
                        Super_full[2] += weights[bench][chk_num] * (Super_numCycles - Super_squash - Super_block) #/ Super_numCycles

                        Raw_full[0] += weights[bench][chk_num] * Raw_squash #/ Raw_numCycles
                        Raw_full[1] += weights[bench][chk_num] * Raw_block #/ Raw_numCycles
                        Raw_full[2] += weights[bench][chk_num] * (Raw_numCycles - Raw_squash - Raw_block) #/ Raw_numCycles
                        
                        #PerformanceOverheadCap = PerformanceOverheadCap + weights[bench][chk_num] * ((int(cap_results['system.switch_cpus.numCycles'][-1]))/int(Super_results['system.switch_cpus.numCycles'][-1]))
                        #if int(Super_results['system.switch_cpus.rename.IQFullEvents'][-1]) != 0:
                        print("Super_full: ", Super_full)
                        print("Raw_full: ", Raw_full)
                        cap_stat_file.close()

                        

                print("-------------------------------------------DONE----------------------------")
                if total_weight == 0:
                    final_results_performance["Raw"].append([0,0,0])
                    final_results_performance["Super"].append([0,0,0])
                    warnings.warn(bench + " Has no working simpoints!!")
                    continue

                #print('PerformanceOverheadCap: ', PerformanceOverheadCap/total_weight)
                #print('PerformanceOverheadSuper: ', total_weight,total_weight)   

                #final_results_performance["Super"].append((PerformanceOverheadCap/total_weight))
                for i in range(3):
                        Super_full[i] /= total_weight
                        Raw_full[i] /= total_weight

                final_results_performance["Raw"].append(Super_full)
                final_results_performance["Super"].append(Raw_full)
                
        Raw = np.array(final_results_performance["Raw"]).T.tolist()
        RawOpt = np.array(final_results_performance["Super"]).T.tolist()

        df = pd.DataFrame(final_results_performance, columns = COLUMNS)
        
        print(df)
        # Setting the positions and width for the bars
        pos = list(range(len(df["benchmark"]))) 
        #print(pos)
        width = 0.25

            
        # Plotting the bars
        fig = plt.figure(figsize=(10,6))
        gs = gridspec.GridSpec(1, 1, height_ratios=[1]) 
        # print('AVG', final_results_performance['CHEx86ContextSensitive'])
        # print('GEOMEAN', 1 - gmean(final_results_performance['CHEx86ContextSensitive']))
        ax0 = plt.subplot(gs[0])
        patterns = [ "*" , "+" , "x" , "o" , "\\" , "/", "o", "O", ".", "*" ]
        #colors = palettable.colorbrewer.diverging.Spectral_4.mpl_colors
        colors = palettable.cartocolors.diverging.Earth_3.mpl_colors

        bar1 = list(range(len(df["benchmark"]))) 
        bar2 = [i + (width*1) for i in bar1]

        #i_bot = list(np.add(Raw[0], Raw[1]))
        #ax0.bar(bar1, Raw[0], width, label="Squashing", color='green')
        #ax0.bar(bar1, Raw[1], width, bottom=Raw[0], label="Blocking", color='blue')
        #ax0.bar(bar1, Raw[2], width, bottom=i_bot, label="Other", color='red')

        #i_bot = list(np.add(RawOpt[0], RawOpt[1]))
        #ax0.bar(bar2, RawOpt[0], width, color='green')
        #ax0.bar(bar2, RawOpt[1], width, bottom=RawOpt[0], color='blue')
        #ax0.bar(bar2, RawOpt[2], width, bottom=i_bot, color='red')

        i_bot = list(np.add(Raw[0], Raw[1]))
        ax0.bar(bar1, Raw[0], width, label="Squashing", color='none', hatch=3*patterns[0], edgecolor=colors[0])
        ax0.bar(bar1, Raw[1], width, bottom=Raw[0], label="Blocking", color='none', hatch=3*patterns[1], edgecolor=colors[1])
        ax0.bar(bar1, Raw[2], width, bottom=i_bot, label="Other", color='none', hatch=3*patterns[2], edgecolor=colors[2])

        i_bot = list(np.add(RawOpt[0], RawOpt[1]))
        ax0.bar(bar2, RawOpt[0], width, color='none', hatch=3*patterns[0], edgecolor=colors[0])
        ax0.bar(bar2, RawOpt[1], width, bottom=RawOpt[0], color='none', hatch=3*patterns[1], edgecolor=colors[1])
        ax0.bar(bar2, RawOpt[2], width, bottom=i_bot, color='none', hatch=3*patterns[2], edgecolor=colors[2])


        # Set the position of the x ticks
        ax0.set_xticks([p + 2.5 * width for p in bar1])

        # Set the labels for the x ticks
        ax0.set_xticklabels(df['benchmark'])
        ax0.tick_params(axis="x", labelsize=20)
        ax0.tick_params(axis="y", labelsize=15)
        # Setting the x-axis and y-axis limits
        #plt.xlim(min(pos)-width, max(pos)+width*4)
        #ax0.set(ylabel='$Normalized$ \n $Performance$')
        #ax0.set_ylabel('$Normalized$ \n $Execution$ $Time$', fontsize = 16.0)
        ax0.set_ylabel('Number of Cycles', fontsize = 16.0)
        #ax0.ylim([0,1.1])
        plt.xticks(rotation=55)
        #plt.yscale('symlog', linthreshy=1)
        plt.yscale('linear')
        # Adding the legend and showing the plot

        for tick in ax0.xaxis.get_majorticklabels():
            tick.set_horizontalalignment("right")

        plt.legend(bbox_to_anchor=(0,1.05,1,0.2), loc="lower left", mode="expand", borderaxespad=0, edgecolor='black', ncol=2, prop={"size":16, "weight":'bold'})
        ax0.grid( linestyle='-.', linewidth=0.5, alpha=.5, axis='y', zorder=0)
        plt.tight_layout()

        if not os.path.exists(file_type):
                os.makedirs(file_type)
        plt.savefig(file_type + '/BlockedIEW.png', bbox_inches='tight')
        plt.show()
 

def get_iqueue_results():
        
        results_index_file = open('./results_index.txt', mode='r')
        contents = results_index_file.read()
        results_index = contents.split("\n")

        final_results_performance["Raw"] = []
        final_results_performance["Super"] = []

        #for bench in benchs:
        for bench in benchs:

                Super_full = 0.0
                Raw_full = 0.0
                total_weight = 0.0

                for chk_num in range(len(benchs[bench])):

                        if weights[bench][chk_num] == 0.0:
                                continue
                        
                        cap_stat_file_dir = './' + bench + '_' + type2 + '/m5out_sim_' + bench + '_' + type2 + "_" + str(chk_num+1) + '/stats.txt'
                        Super_stat_file_dir = './' + bench + '_' + type1 + '/m5out_sim_' + bench + '_' + type1 + "_" + str(chk_num+1) + '/stats.txt'
                        # print(cap_stat_file_dir)
                        # print(Super_stat_file_dir)
                        cap_stat_file = open(cap_stat_file_dir, mode='r')
                        Super_stat_file = open(Super_stat_file_dir, mode='r')
                        cap_stat_file_content = cap_stat_file.read()
                        Super_stat_file_content = Super_stat_file.read()

                        cap_results = measure(results_index, cap_stat_file_content)
                        Super_results = measure(results_index, Super_stat_file_content)

                        if int(cap_results['system.switch_cpus.rename.IQFullEvents'][-1]) == 0:
                                warnings.warn(bench + " simpoint " + str(chk_num+1) + " has no stat!")
                                cap_stat_file.close()
                                Super_stat_file.close()
                                continue
                        
                        total_weight = total_weight + weights[bench][chk_num]
                        #print("Raw: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", int(cap_results['system.switch_cpus.numCycles'][-1]))
                        
                        
                        #PerformanceOverheadCap = PerformanceOverheadCap + weights[bench][chk_num] * ((int(cap_results['system.switch_cpus.numCycles'][-1]))/int(Super_results['system.switch_cpus.numCycles'][-1]))
                        #if int(Super_results['system.switch_cpus.rename.IQFullEvents'][-1]) != 0:
                        Raw_full += weights[bench][chk_num] * int(cap_results['system.switch_cpus.rename.IQFullEvents'][-1])
                        Super_full += weights[bench][chk_num] * int(Super_results['system.switch_cpus.rename.IQFullEvents'][-1])
                        print("Super_full: ", Super_full)
                        print("Raw_full: ", Raw_full)
                        cap_stat_file.close()

                        

                print("-------------------------------------------DONE----------------------------")
                if total_weight == 0:
                    final_results_performance["Raw"].append(0)
                    final_results_performance["Super"].append(0)
                    warnings.warn(bench + " Has no working simpoints!!")
                    continue

                #print('PerformanceOverheadCap: ', PerformanceOverheadCap/total_weight)
                #print('PerformanceOverheadSuper: ', total_weight,total_weight)   

                #final_results_performance["Super"].append((PerformanceOverheadCap/total_weight))
                final_results_performance["Raw"].append(Super_full/total_weight)
                final_results_performance["Super"].append(Raw_full/total_weight)
                




        df = pd.DataFrame(final_results_performance, columns = COLUMNS)
        print(df)
        # Setting the positions and width for the bars
        pos = list(range(len(df["benchmark"]))) 
        #print(pos)
        width = 0.25
            
        # Plotting the bars
        fig = plt.figure(figsize=(10,6))
        gs = gridspec.GridSpec(1, 1, height_ratios=[1]) 
        # print('AVG', final_results_performance['CHEx86ContextSensitive'])
        # print('GEOMEAN', 1 - gmean(final_results_performance['CHEx86ContextSensitive']))
        ax0 = plt.subplot(gs[0])
        patterns = [ "*" , "+" , "x" , "o" , "\\" , "/", "o", "O", ".", "*" ]
        #colors = palettable.colorbrewer.diverging.Spectral_4.mpl_colors
        colors = palettable.cartocolors.diverging.Earth_3.mpl_colors


        for i in range(0,2):
                ax0.bar([p + width*i for p in pos], 
                        df[x_axe[i]], 
                        #hatch=3*patterns[i],
                        # of width
                        width=width,
                        # with color
                        #edgecolor=colors[i], 
                        color=colors[i],
                        # with label the first value in benchmark
                        label=x_axe[i],
                        zorder=3) 
                ax0.bar([p + width*i for p in pos], 
                        df[x_axe[i]], 
                        # of width
                        width=width, 
                        # with color
                        color='none', 
                        edgecolor='black',
                        zorder=4) 

        # Set the position of the x ticks
        ax0.set_xticks([p + 2.5 * width for p in pos])

        # Set the labels for the x ticks
        ax0.set_xticklabels(df['benchmark'])
        ax0.tick_params(axis="x", labelsize=20)
        ax0.tick_params(axis="y", labelsize=15)
        # Setting the x-axis and y-axis limits
        #plt.xlim(min(pos)-width, max(pos)+width*4)
        #ax0.set(ylabel='$Normalized$ \n $Performance$')
        #ax0.set_ylabel('$Normalized$ \n $Execution$ $Time$', fontsize = 16.0)
        ax0.set_ylabel('IQueue Full Senarios', fontsize = 16.0)
        #ax0.ylim([0,1.1])
        plt.xticks(rotation=55)
        plt.yscale('symlog', linthreshy=1)
        #plt.yscale('linear')
        # Adding the legend and showing the plot

        for tick in ax0.xaxis.get_majorticklabels():
            tick.set_horizontalalignment("right")

        plt.legend(bbox_to_anchor=(0,1.05,1,0.2), loc="lower left", mode="expand", borderaxespad=0, edgecolor='black', ncol=2, prop={"size":16, "weight":'bold'})
        ax0.grid( linestyle='-.', linewidth=0.5, alpha=.5, axis='y', zorder=0)
        plt.tight_layout()

        if not os.path.exists(file_type):
                os.makedirs(file_type)
        plt.savefig(file_type + '/IQueue.png', bbox_inches='tight')
        plt.show()
   

def get_lqueue_results():
        
        results_index_file = open('./results_index.txt', mode='r')
        contents = results_index_file.read()
        results_index = contents.split("\n")

        final_results_performance["Raw"] = []
        final_results_performance["Super"] = []

        #for bench in benchs:
        for bench in benchs:

                Super_full = 0.0
                Raw_full = 0.0
                total_weight = 0.0

                for chk_num in range(len(benchs[bench])):

                        if weights[bench][chk_num] == 0.0:
                                continue
                        
                        cap_stat_file_dir = './' + bench + '_' + type2 + '/m5out_sim_' + bench + '_' + type2 + "_" + str(chk_num+1) + '/stats.txt'
                        Super_stat_file_dir = './' + bench + '_' + type1 + '/m5out_sim_' + bench + '_' + type1 + "_" + str(chk_num+1) + '/stats.txt'
                        # print(cap_stat_file_dir)
                        # print(Super_stat_file_dir)
                        cap_stat_file = open(cap_stat_file_dir, mode='r')
                        Super_stat_file = open(Super_stat_file_dir, mode='r')
                        cap_stat_file_content = cap_stat_file.read()
                        Super_stat_file_content = Super_stat_file.read()

                        cap_results = measure(results_index, cap_stat_file_content)
                        Super_results = measure(results_index, Super_stat_file_content)

                        if int(cap_results['system.switch_cpus.rename.LQFullEvents'][-1]) == 0:
                                warnings.warn(bench + " simpoint " + str(chk_num+1) + " has no stat!")
                                cap_stat_file.close()
                                Super_stat_file.close()
                                continue
                        
                        total_weight = total_weight + weights[bench][chk_num]
                        #print("Raw: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", int(cap_results['system.switch_cpus.numCycles'][-1]))
                        
                        
                        #PerformanceOverheadCap = PerformanceOverheadCap + weights[bench][chk_num] * ((int(cap_results['system.switch_cpus.numCycles'][-1]))/int(Super_results['system.switch_cpus.numCycles'][-1]))
                        #if int(Super_results['system.switch_cpus.rename.IQFullEvents'][-1]) != 0:
                        Raw_full += weights[bench][chk_num] * int(cap_results['system.switch_cpus.rename.LQFullEvents'][-1])
                        Super_full += weights[bench][chk_num] * int(Super_results['system.switch_cpus.rename.LQFullEvents'][-1])
                        print("Super_full: ", Super_full)
                        print("Raw_full: ", Raw_full)
                        cap_stat_file.close()

                        

                print("-------------------------------------------DONE----------------------------")
                if total_weight == 0:
                    final_results_performance["Raw"].append(0)
                    final_results_performance["Super"].append(0)
                    warnings.warn(bench + " Has no working simpoints!!")
                    continue

                #print('PerformanceOverheadCap: ', PerformanceOverheadCap/total_weight)
                #print('PerformanceOverheadSuper: ', total_weight,total_weight)   

                #final_results_performance["Super"].append((PerformanceOverheadCap/total_weight))
                final_results_performance["Raw"].append(Super_full/total_weight)
                final_results_performance["Super"].append(Raw_full/total_weight)
                




        df = pd.DataFrame(final_results_performance, columns = COLUMNS)
        print(df)
        # Setting the positions and width for the bars
        pos = list(range(len(df["benchmark"]))) 
        #print(pos)
        width = 0.25
            
        # Plotting the bars
        fig = plt.figure(figsize=(10,6))
        gs = gridspec.GridSpec(1, 1, height_ratios=[1]) 
        # print('AVG', final_results_performance['CHEx86ContextSensitive'])
        # print('GEOMEAN', 1 - gmean(final_results_performance['CHEx86ContextSensitive']))
        ax0 = plt.subplot(gs[0])
        patterns = [ "*" , "+" , "x" , "o" , "\\" , "/", "o", "O", ".", "*" ]
        #colors = palettable.colorbrewer.diverging.Spectral_4.mpl_colors
        colors = palettable.cartocolors.diverging.Earth_3.mpl_colors


        for i in range(0,2):
                ax0.bar([p + width*i for p in pos], 
                        df[x_axe[i]], 
                        #hatch=3*patterns[i],
                        # of width
                        width=width,
                        # with color
                        #edgecolor=colors[i], 
                        color=colors[i],
                        # with label the first value in benchmark
                        label=x_axe[i],
                        zorder=3) 
                ax0.bar([p + width*i for p in pos], 
                        df[x_axe[i]], 
                        # of width
                        width=width, 
                        # with color
                        color='none', 
                        edgecolor='black',
                        zorder=4) 

        # Set the position of the x ticks
        ax0.set_xticks([p + 2.5 * width for p in pos])

        # Set the labels for the x ticks
        ax0.set_xticklabels(df['benchmark'])
        ax0.tick_params(axis="x", labelsize=20)
        ax0.tick_params(axis="y", labelsize=15)
        # Setting the x-axis and y-axis limits
        #plt.xlim(min(pos)-width, max(pos)+width*4)
        #ax0.set(ylabel='$Normalized$ \n $Performance$')
        #ax0.set_ylabel('$Normalized$ \n $Execution$ $Time$', fontsize = 16.0)
        ax0.set_ylabel('LQueue Full Senarios', fontsize = 16.0)
        #ax0.ylim([0,1.1])
        plt.xticks(rotation=55)
        plt.yscale('symlog', linthreshy=1)
        #plt.yscale('linear')
        # Adding the legend and showing the plot

        for tick in ax0.xaxis.get_majorticklabels():
            tick.set_horizontalalignment("right")

        plt.legend(bbox_to_anchor=(0,1.05,1,0.2), loc="lower left", mode="expand", borderaxespad=0, edgecolor='black', ncol=2, prop={"size":16, "weight":'bold'})
        ax0.grid( linestyle='-.', linewidth=0.5, alpha=.5, axis='y', zorder=0)
        plt.tight_layout()

        if not os.path.exists(file_type):
                os.makedirs(file_type)
        plt.savefig(file_type + '/LQueue.png', bbox_inches='tight')
        plt.show()

def get_squeue_results():
        
        results_index_file = open('./results_index.txt', mode='r')
        contents = results_index_file.read()
        results_index = contents.split("\n")

        final_results_performance["Raw"] = []
        final_results_performance["Super"] = []

        #for bench in benchs:
        for bench in benchs:

                Super_full = 0.0
                Raw_full = 0.0
                total_weight = 0.0

                for chk_num in range(len(benchs[bench])):

                        if weights[bench][chk_num] == 0.0:
                                continue
                        
                        cap_stat_file_dir = './' + bench + '_' + type2 + '/m5out_sim_' + bench + '_' + type2 + "_" + str(chk_num+1) + '/stats.txt'
                        Super_stat_file_dir = './' + bench + '_' + type1 + '/m5out_sim_' + bench + '_' + type1 + "_" + str(chk_num+1) + '/stats.txt'
                        # print(cap_stat_file_dir)
                        # print(Super_stat_file_dir)
                        cap_stat_file = open(cap_stat_file_dir, mode='r')
                        Super_stat_file = open(Super_stat_file_dir, mode='r')
                        cap_stat_file_content = cap_stat_file.read()
                        Super_stat_file_content = Super_stat_file.read()

                        cap_results = measure(results_index, cap_stat_file_content)
                        Super_results = measure(results_index, Super_stat_file_content)

                        if int(cap_results['system.switch_cpus.rename.SQFullEvents'][-1]) == 0:
                                warnings.warn(bench + " simpoint " + str(chk_num+1) + " has no stat!")
                                cap_stat_file.close()
                                Super_stat_file.close()
                                continue
                        
                        total_weight = total_weight + weights[bench][chk_num]
                        #print("Raw: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", int(cap_results['system.switch_cpus.numCycles'][-1]))
                        
                        
                        #PerformanceOverheadCap = PerformanceOverheadCap + weights[bench][chk_num] * ((int(cap_results['system.switch_cpus.numCycles'][-1]))/int(Super_results['system.switch_cpus.numCycles'][-1]))
                        #if int(Super_results['system.switch_cpus.rename.IQFullEvents'][-1]) != 0:
                        Raw_full += weights[bench][chk_num] * int(cap_results['system.switch_cpus.rename.SQFullEvents'][-1])
                        Super_full += weights[bench][chk_num] * int(Super_results['system.switch_cpus.rename.SQFullEvents'][-1])
                        print("Super_full: ", Super_full)
                        print("Raw_full: ", Raw_full)
                        cap_stat_file.close()

                        

                print("-------------------------------------------DONE----------------------------")
                if total_weight == 0:
                    final_results_performance["Raw"].append(0)
                    final_results_performance["Super"].append(0)
                    warnings.warn(bench + " Has no working simpoints!!")
                    continue

                #print('PerformanceOverheadCap: ', PerformanceOverheadCap/total_weight)
                #print('PerformanceOverheadSuper: ', total_weight,total_weight)   

                #final_results_performance["Super"].append((PerformanceOverheadCap/total_weight))
                final_results_performance["Raw"].append(Super_full/total_weight)
                final_results_performance["Super"].append(Raw_full/total_weight)
                




        df = pd.DataFrame(final_results_performance, columns = COLUMNS)
        print(df)
        # Setting the positions and width for the bars
        pos = list(range(len(df["benchmark"]))) 
        #print(pos)
        width = 0.25
            
        # Plotting the bars
        fig = plt.figure(figsize=(10,6))
        gs = gridspec.GridSpec(1, 1, height_ratios=[1]) 
        # print('AVG', final_results_performance['CHEx86ContextSensitive'])
        # print('GEOMEAN', 1 - gmean(final_results_performance['CHEx86ContextSensitive']))
        ax0 = plt.subplot(gs[0])
        patterns = [ "*" , "+" , "x" , "o" , "\\" , "/", "o", "O", ".", "*" ]
        #colors = palettable.colorbrewer.diverging.Spectral_4.mpl_colors
        colors = palettable.cartocolors.diverging.Earth_3.mpl_colors


        for i in range(0,2):
                ax0.bar([p + width*i for p in pos], 
                        df[x_axe[i]], 
                        #hatch=3*patterns[i],
                        # of width
                        width=width,
                        # with color
                        #edgecolor=colors[i], 
                        color=colors[i],
                        # with label the first value in benchmark
                        label=x_axe[i],
                        zorder=3) 
                ax0.bar([p + width*i for p in pos], 
                        df[x_axe[i]], 
                        # of width
                        width=width, 
                        # with color
                        color='none', 
                        edgecolor='black',
                        zorder=4) 

        # Set the position of the x ticks
        ax0.set_xticks([p + 2.5 * width for p in pos])

        # Set the labels for the x ticks
        ax0.set_xticklabels(df['benchmark'])
        ax0.tick_params(axis="x", labelsize=20)
        ax0.tick_params(axis="y", labelsize=15)
        # Setting the x-axis and y-axis limits
        #plt.xlim(min(pos)-width, max(pos)+width*4)
        #ax0.set(ylabel='$Normalized$ \n $Performance$')
        #ax0.set_ylabel('$Normalized$ \n $Execution$ $Time$', fontsize = 16.0)
        ax0.set_ylabel('SQueue Full Senarios', fontsize = 16.0)
        #ax0.ylim([0,1.1])
        plt.xticks(rotation=55)
        plt.yscale('symlog', linthreshy=1)
        #plt.yscale('linear')
        # Adding the legend and showing the plot

        for tick in ax0.xaxis.get_majorticklabels():
            tick.set_horizontalalignment("right")

        plt.legend(bbox_to_anchor=(0,1.05,1,0.2), loc="lower left", mode="expand", borderaxespad=0, edgecolor='black', ncol=2, prop={"size":16, "weight":'bold'})
        ax0.grid( linestyle='-.', linewidth=0.5, alpha=.5, axis='y', zorder=0)
        plt.tight_layout()

        if not os.path.exists(file_type):
                os.makedirs(file_type)
        plt.savefig(file_type + '/SQueue.png', bbox_inches='tight')
        plt.show()



def get_rob_results():
        
        results_index_file = open('./results_index.txt', mode='r')
        contents = results_index_file.read()
        results_index = contents.split("\n")

        final_results_performance["Raw"] = []
        final_results_performance["Super"] = []

        #for bench in benchs:
        for bench in benchs:

                Super_full = 0.0
                Raw_full = 0.0
                total_weight = 0.0

                for chk_num in range(len(benchs[bench])):

                        if weights[bench][chk_num] == 0.0:
                                continue
                        
                        cap_stat_file_dir = './' + bench + '_' + type2 + '/m5out_sim_' + bench + '_' + type2 + "_" + str(chk_num+1) + '/stats.txt'
                        Super_stat_file_dir = './' + bench + '_' + type1 + '/m5out_sim_' + bench + '_' + type1 + "_" + str(chk_num+1) + '/stats.txt'
                        # print(cap_stat_file_dir)
                        # print(Super_stat_file_dir)
                        cap_stat_file = open(cap_stat_file_dir, mode='r')
                        Super_stat_file = open(Super_stat_file_dir, mode='r')
                        cap_stat_file_content = cap_stat_file.read()
                        Super_stat_file_content = Super_stat_file.read()

                        cap_results = measure(results_index, cap_stat_file_content)
                        Super_results = measure(results_index, Super_stat_file_content)

                        if int(cap_results['system.switch_cpus.rename.ROBFullEvents'][-1]) == 0:
                                warnings.warn(bench + " simpoint " + str(chk_num+1) + " has no stat!")
                                cap_stat_file.close()
                                Super_stat_file.close()
                                continue
                        
                        total_weight = total_weight + weights[bench][chk_num]
                        #print("Raw: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", int(cap_results['system.switch_cpus.numCycles'][-1]))
                        
                        
                        #PerformanceOverheadCap = PerformanceOverheadCap + weights[bench][chk_num] * ((int(cap_results['system.switch_cpus.numCycles'][-1]))/int(Super_results['system.switch_cpus.numCycles'][-1]))
                        #if int(Super_results['system.switch_cpus.rename.IQFullEvents'][-1]) != 0:
                        Raw_full += weights[bench][chk_num] * int(cap_results['system.switch_cpus.rename.ROBFullEvents'][-1])
                        Super_full += weights[bench][chk_num] * int(Super_results['system.switch_cpus.rename.LQFullEvents'][-1])
                        print("Super_full: ", Super_full)
                        print("Raw_full: ", Raw_full)
                        cap_stat_file.close()

                        

                print("-------------------------------------------DONE----------------------------")
                if total_weight == 0:
                    final_results_performance["Raw"].append(0)
                    final_results_performance["Super"].append(0)
                    warnings.warn(bench + " Has no working simpoints!!")
                    continue

                #print('PerformanceOverheadCap: ', PerformanceOverheadCap/total_weight)
                #print('PerformanceOverheadSuper: ', total_weight,total_weight)   

                #final_results_performance["Super"].append((PerformanceOverheadCap/total_weight))
                final_results_performance["Raw"].append(Super_full/total_weight)
                final_results_performance["Super"].append(Raw_full/total_weight)
                




        df = pd.DataFrame(final_results_performance, columns = COLUMNS)
        print(df)
        # Setting the positions and width for the bars
        pos = list(range(len(df["benchmark"]))) 
        #print(pos)
        width = 0.25
            
        # Plotting the bars
        fig = plt.figure(figsize=(10,6))
        gs = gridspec.GridSpec(1, 1, height_ratios=[1]) 
        # print('AVG', final_results_performance['CHEx86ContextSensitive'])
        # print('GEOMEAN', 1 - gmean(final_results_performance['CHEx86ContextSensitive']))
        ax0 = plt.subplot(gs[0])
        patterns = [ "*" , "+" , "x" , "o" , "\\" , "/", "o", "O", ".", "*" ]
        #colors = palettable.colorbrewer.diverging.Spectral_4.mpl_colors
        colors = palettable.cartocolors.diverging.Earth_3.mpl_colors


        for i in range(0,2):
                ax0.bar([p + width*i for p in pos], 
                        df[x_axe[i]], 
                        #hatch=3*patterns[i],
                        # of width
                        width=width,
                        # with color
                        #edgecolor=colors[i], 
                        color=colors[i],
                        # with label the first value in benchmark
                        label=x_axe[i],
                        zorder=3) 
                ax0.bar([p + width*i for p in pos], 
                        df[x_axe[i]], 
                        # of width
                        width=width, 
                        # with color
                        color='none', 
                        edgecolor='black',
                        zorder=4) 

        # Set the position of the x ticks
        ax0.set_xticks([p + 2.5 * width for p in pos])

        # Set the labels for the x ticks
        ax0.set_xticklabels(df['benchmark'])
        ax0.tick_params(axis="x", labelsize=20)
        ax0.tick_params(axis="y", labelsize=15)
        # Setting the x-axis and y-axis limits
        #plt.xlim(min(pos)-width, max(pos)+width*4)
        #ax0.set(ylabel='$Normalized$ \n $Performance$')
        #ax0.set_ylabel('$Normalized$ \n $Execution$ $Time$', fontsize = 16.0)
        ax0.set_ylabel('ROB Full Senarios', fontsize = 16.0)
        #ax0.ylim([0,1.1])
        plt.xticks(rotation=55)
        plt.yscale('symlog', linthreshy=1)
        #plt.yscale('linear')
        # Adding the legend and showing the plot

        for tick in ax0.xaxis.get_majorticklabels():
            tick.set_horizontalalignment("right")

        plt.legend(bbox_to_anchor=(0,1.05,1,0.2), loc="lower left", mode="expand", borderaxespad=0, edgecolor='black', ncol=2, prop={"size":16, "weight":'bold'})
        ax0.grid( linestyle='-.', linewidth=0.5, alpha=.5, axis='y', zorder=0)
        plt.tight_layout()

        if not os.path.exists(file_type):
                os.makedirs(file_type)
        plt.savefig(file_type + '/ROB.png', bbox_inches='tight')
        plt.show()


def get_int_dependency_results():
        
        results_index_file = open('./results_index.txt', mode='r')
        contents = results_index_file.read()
        results_index = contents.split("\n")

        final_results_performance["Raw"] = []
        final_results_performance["Super"] = []

        #for bench in benchs:
        for bench in benchs:

                # nonspec_live, spec_live, nonspec_orig, spec_orig
                Super_full = [0,0,0,0]
                Raw_full = [0,0,0,0]
                total_weight = 0.0

                for chk_num in range(len(benchs[bench])):

                        if weights[bench][chk_num] == 0.0:
                                continue
                        
                        cap_stat_file_dir = './' + bench + '_' + type2 + '/m5out_sim_' + bench + '_' + type2 + "_" + str(chk_num+1) + '/stats.txt'
                        Super_stat_file_dir = './' + bench + '_' + type1 + '/m5out_sim_' + bench + '_' + type1 + "_" + str(chk_num+1) + '/stats.txt'
                        # print(cap_stat_file_dir)
                        # print(Super_stat_file_dir)
                        cap_stat_file = open(cap_stat_file_dir, mode='r')
                        Super_stat_file = open(Super_stat_file_dir, mode='r')
                        cap_stat_file_content = cap_stat_file.read()
                        Super_stat_file_content = Super_stat_file.read()

                        cap_results = measure(results_index, cap_stat_file_content)
                        #print(cap_results)
                        Super_results = measure(results_index, Super_stat_file_content)

                        Raw_spec_live = int(cap_results['system.switch_cpus.iq.speculativeInstsAddedToDependentsDueToLiveOutRegDependency::5'][-1])
                        Raw_nonspec_live = int(cap_results['system.switch_cpus.iq.nonspeculativeInstsAddedToDependentsDueToLiveOutRegDependency::5'][-1])
                        Raw_spec_orig = int(cap_results['system.switch_cpus.iq.speculativeInstsAddedToDependentsDueToOriginalRegDependency::5'][-1])
                        Raw_nonspec_orig = int(cap_results['system.switch_cpus.iq.nonspeculativeInstsAddedToDependentsDueToOriginalRegDependency::5'][-1])
                        
                        Super_spec_live = int(Super_results['system.switch_cpus.iq.speculativeInstsAddedToDependentsDueToLiveOutRegDependency::5'][-1])
                        Super_nonspec_live = int(Super_results['system.switch_cpus.iq.nonspeculativeInstsAddedToDependentsDueToLiveOutRegDependency::5'][-1])
                        Super_spec_orig = int(Super_results['system.switch_cpus.iq.speculativeInstsAddedToDependentsDueToOriginalRegDependency::5'][-1])
                        Super_nonspec_orig = int(Super_results['system.switch_cpus.iq.nonspeculativeInstsAddedToDependentsDueToOriginalRegDependency::5'][-1])

                        if (Raw_spec_live == 0 and Raw_nonspec_live == 0 and Raw_spec_orig == 0 and Raw_nonspec_orig == 0) or (Super_spec_live == 0 and Super_nonspec_live == 0 and Super_spec_orig == 0 and Super_nonspec_orig == 0):
                                warnings.warn(bench + " simpoint " + str(chk_num+1) + " has no stat!")
                                cap_stat_file.close()
                                Super_stat_file.close()
                                continue
                        
                        total_weight = total_weight + weights[bench][chk_num]
                        #print("Raw: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", int(cap_results['system.switch_cpus.numCycles'][-1]))
                        
                        Super_full[0] += weights[bench][chk_num] * Super_nonspec_live
                        Super_full[1] += weights[bench][chk_num] * Super_spec_live
                        Super_full[2] += weights[bench][chk_num] * Super_nonspec_orig
                        Super_full[3] += weights[bench][chk_num] * Super_spec_orig

                        Raw_full[0] += weights[bench][chk_num] * Raw_nonspec_live
                        Raw_full[1] += weights[bench][chk_num] * Raw_spec_live
                        Raw_full[2] += weights[bench][chk_num] * Raw_nonspec_orig
                        Raw_full[3] += weights[bench][chk_num] * Raw_spec_orig
                        
                        #PerformanceOverheadCap = PerformanceOverheadCap + weights[bench][chk_num] * ((int(cap_results['system.switch_cpus.numCycles'][-1]))/int(Super_results['system.switch_cpus.numCycles'][-1]))
                        #if int(Super_results['system.switch_cpus.rename.IQFullEvents'][-1]) != 0:
                        print("Super_full: ", Super_full)
                        print("Raw_full: ", Raw_full)
                        cap_stat_file.close()
                        Super_stat_file.close()

                        

                print("-------------------------------------------DONE----------------------------")
                if total_weight == 0:
                    final_results_performance["Raw"].append([0,0,0,0])
                    final_results_performance["Super"].append([0,0,0,0])
                    warnings.warn(bench + " Has no working simpoints!!")
                    continue

                #print('PerformanceOverheadCap: ', PerformanceOverheadCap/total_weight)
                #print('PerformanceOverheadSuper: ', total_weight,total_weight)   

                #final_results_performance["Super"].append((PerformanceOverheadCap/total_weight))
                for i in range(4):
                        Super_full[i] /= total_weight
                        Raw_full[i] /= total_weight

                final_results_performance["Raw"].append(Super_full)
                final_results_performance["Super"].append(Raw_full)
                
        Raw = np.array(final_results_performance["Raw"]).T.tolist()
        RawOpt = np.array(final_results_performance["Super"]).T.tolist()

        df = pd.DataFrame(final_results_performance, columns = COLUMNS)
        
        print(df)
        # Setting the positions and width for the bars
        pos = list(range(len(df["benchmark"]))) 
        #print(pos)
        width = 0.25

            
        # Plotting the bars
        fig = plt.figure(figsize=(10,6))
        gs = gridspec.GridSpec(1, 1, height_ratios=[1]) 
        # print('AVG', final_results_performance['CHEx86ContextSensitive'])
        # print('GEOMEAN', 1 - gmean(final_results_performance['CHEx86ContextSensitive']))
        ax0 = plt.subplot(gs[0])
        patterns = [ "*" , "+" , "x" , "o" , "\\" , "/", "o", "O", ".", "*" ]
        #colors = palettable.colorbrewer.diverging.Spectral_4.mpl_colors
        colors = palettable.cartocolors.diverging.Earth_3.mpl_colors

        bar1 = list(range(len(df["benchmark"]))) 
        bar2 = [i + (width*1) for i in bar1]

        #i_bot = list(np.add(Raw[0], Raw[1]))
        #ax0.bar(bar1, Raw[0], width, label="Squashing", color='green')
        #ax0.bar(bar1, Raw[1], width, bottom=Raw[0], label="Blocking", color='blue')
        #ax0.bar(bar1, Raw[2], width, bottom=i_bot, label="Other", color='red')

        #i_bot = list(np.add(RawOpt[0], RawOpt[1]))
        #ax0.bar(bar2, RawOpt[0], width, color='green')
        #ax0.bar(bar2, RawOpt[1], width, bottom=RawOpt[0], color='blue')
        #ax0.bar(bar2, RawOpt[2], width, bottom=i_bot, color='red')
        

        # nonspec_live, spec_live, nonspec_orig, spec_orig
        bot2 = list(np.add(Raw[0], Raw[1]))
        bot3 = list(np.add(bot2, Raw[2]))
        ax0.bar(bar1, Raw[0], width, label="Non Speculative LiveOut", color='none', hatch=3*patterns[0], edgecolor=colors[0])
        ax0.bar(bar1, Raw[1], width, bottom=Raw[0], label="Speculative LiveOut", color='none', hatch=3*patterns[1], edgecolor=colors[1])
        ax0.bar(bar1, Raw[2], width, bottom=bot2, label="Non Speculative Orig", color='none', hatch=3*patterns[2], edgecolor=colors[2])
        ax0.bar(bar1, Raw[3], width, bottom=bot3, label="Speculative Orig", color='none', hatch=3*patterns[3], edgecolor=colors[3])

        bot2 = list(np.add(RawOpt[0], RawOpt[1]))
        bot3 = list(np.add(bot2, RawOpt[2]))
        ax0.bar(bar2, RawOpt[0], width, color='none', hatch=3*patterns[0], edgecolor=colors[0])
        ax0.bar(bar2, RawOpt[1], width, bottom=RawOpt[0], color='none', hatch=3*patterns[1], edgecolor=colors[1])
        ax0.bar(bar2, RawOpt[2], width, bottom=bot2, color='none', hatch=3*patterns[2], edgecolor=colors[2])
        ax0.bar(bar2, RawOpt[3], width, bottom=bot3, color='none', hatch=3*patterns[3], edgecolor=colors[3])


        # Set the position of the x ticks
        ax0.set_xticks([p + 2.5 * width for p in bar1])

        # Set the labels for the x ticks
        ax0.set_xticklabels(df['benchmark'])
        ax0.tick_params(axis="x", labelsize=20)
        ax0.tick_params(axis="y", labelsize=15)
        # Setting the x-axis and y-axis limits
        #plt.xlim(min(pos)-width, max(pos)+width*4)
        #ax0.set(ylabel='$Normalized$ \n $Performance$')
        #ax0.set_ylabel('$Normalized$ \n $Execution$ $Time$', fontsize = 16.0)
        ax0.set_ylabel('Stalls Due to Int Dependency', fontsize = 16.0)
        #ax0.ylim([0,1.1])
        plt.xticks(rotation=55)
        #plt.yscale('symlog', linthreshy=1)
        plt.yscale('linear')
        # Adding the legend and showing the plot

        for tick in ax0.xaxis.get_majorticklabels():
            tick.set_horizontalalignment("right")

        plt.legend(bbox_to_anchor=(0,1.05,1,0.2), loc="lower left", mode="expand", borderaxespad=0, edgecolor='black', ncol=2, prop={"size":16, "weight":'bold'})
        ax0.grid( linestyle='-.', linewidth=0.5, alpha=.5, axis='y', zorder=0)
        plt.tight_layout()

        if not os.path.exists(file_type):
                os.makedirs(file_type)
        plt.savefig(file_type + '/IntDepend.png', bbox_inches='tight')
        plt.show()
 
# TODO FIX
def get_cc_dependency_results():
        
        results_index_file = open('./results_index.txt', mode='r')
        contents = results_index_file.read()
        results_index = contents.split("\n")

        final_results_performance["Raw"] = []
        final_results_performance["Super"] = []

        #for bench in benchs:
        for bench in benchs:

                # nonspec_live, spec_live, nonspec_orig, spec_orig
                Super_full = [0,0,0,0]
                Raw_full = [0,0,0,0]
                total_weight = 0.0

                for chk_num in range(len(benchs[bench])):

                        if weights[bench][chk_num] == 0.0:
                                continue
                        
                        cap_stat_file_dir = './' + bench + '_' + type2 + '/m5out_sim_' + bench + '_' + type2 + "_" + str(chk_num+1) + '/stats.txt'
                        Super_stat_file_dir = './' + bench + '_' + type1 + '/m5out_sim_' + bench + '_' + type1 + "_" + str(chk_num+1) + '/stats.txt'
                        # print(cap_stat_file_dir)
                        # print(Super_stat_file_dir)
                        cap_stat_file = open(cap_stat_file_dir, mode='r')
                        Super_stat_file = open(Super_stat_file_dir, mode='r')
                        cap_stat_file_content = cap_stat_file.read()
                        Super_stat_file_content = Super_stat_file.read()

                        cap_results = measure(results_index, cap_stat_file_content)
                        Super_results = measure(results_index, Super_stat_file_content)

                        Raw_spec_live = int(cap_results['system.switch_cpus.iq.speculativeInstsAddedToDependentsDueToLiveOutRegDependency::0'][-1])
                        Raw_spec_live += int(cap_results['system.switch_cpus.iq.speculativeInstsAddedToDependentsDueToLiveOutRegDependency::1'][-1])
                        Raw_spec_live += int(cap_results['system.switch_cpus.iq.speculativeInstsAddedToDependentsDueToLiveOutRegDependency::2'][-1])
                        Raw_spec_live += int(cap_results['system.switch_cpus.iq.speculativeInstsAddedToDependentsDueToLiveOutRegDependency::3'][-1])
                        Raw_spec_live += int(cap_results['system.switch_cpus.iq.speculativeInstsAddedToDependentsDueToLiveOutRegDependency::4'][-1])
                        Raw_nonspec_live = int(cap_results['system.switch_cpus.iq.nonspeculativeInstsAddedToDependentsDueToLiveOutRegDependency::0'][-1])
                        Raw_nonspec_live += int(cap_results['system.switch_cpus.iq.nonspeculativeInstsAddedToDependentsDueToLiveOutRegDependency::1'][-1])
                        Raw_nonspec_live += int(cap_results['system.switch_cpus.iq.nonspeculativeInstsAddedToDependentsDueToLiveOutRegDependency::2'][-1])
                        Raw_nonspec_live += int(cap_results['system.switch_cpus.iq.nonspeculativeInstsAddedToDependentsDueToLiveOutRegDependency::3'][-1])
                        Raw_nonspec_live += int(cap_results['system.switch_cpus.iq.nonspeculativeInstsAddedToDependentsDueToLiveOutRegDependency::4'][-1])
                        Raw_spec_orig = int(cap_results['system.switch_cpus.iq.speculativeInstsAddedToDependentsDueToOriginalRegDependency::0'][-1])
                        Raw_spec_orig += int(cap_results['system.switch_cpus.iq.speculativeInstsAddedToDependentsDueToOriginalRegDependency::1'][-1])
                        Raw_spec_orig += int(cap_results['system.switch_cpus.iq.speculativeInstsAddedToDependentsDueToOriginalRegDependency::2'][-1])
                        Raw_spec_orig += int(cap_results['system.switch_cpus.iq.speculativeInstsAddedToDependentsDueToOriginalRegDependency::3'][-1])
                        Raw_spec_orig += int(cap_results['system.switch_cpus.iq.speculativeInstsAddedToDependentsDueToOriginalRegDependency::4'][-1])
                        Raw_nonspec_orig = int(cap_results['system.switch_cpus.iq.nonspeculativeInstsAddedToDependentsDueToOriginalRegDependency::0'][-1])
                        Raw_nonspec_orig += int(cap_results['system.switch_cpus.iq.nonspeculativeInstsAddedToDependentsDueToOriginalRegDependency::1'][-1])
                        Raw_nonspec_orig += int(cap_results['system.switch_cpus.iq.nonspeculativeInstsAddedToDependentsDueToOriginalRegDependency::2'][-1])
                        Raw_nonspec_orig += int(cap_results['system.switch_cpus.iq.nonspeculativeInstsAddedToDependentsDueToOriginalRegDependency::3'][-1])
                        Raw_nonspec_orig += int(cap_results['system.switch_cpus.iq.nonspeculativeInstsAddedToDependentsDueToOriginalRegDependency::4'][-1])
                        
                        Super_spec_live = int(Super_results['system.switch_cpus.iq.speculativeInstsAddedToDependentsDueToLiveOutRegDependency::0'][-1])
                        Super_spec_live += int(Super_results['system.switch_cpus.iq.speculativeInstsAddedToDependentsDueToLiveOutRegDependency::1'][-1])
                        Super_spec_live += int(Super_results['system.switch_cpus.iq.speculativeInstsAddedToDependentsDueToLiveOutRegDependency::2'][-1])
                        Super_spec_live += int(Super_results['system.switch_cpus.iq.speculativeInstsAddedToDependentsDueToLiveOutRegDependency::3'][-1])
                        Super_spec_live += int(Super_results['system.switch_cpus.iq.speculativeInstsAddedToDependentsDueToLiveOutRegDependency::4'][-1])

                        Super_nonspec_live = int(Super_results['system.switch_cpus.iq.nonspeculativeInstsAddedToDependentsDueToLiveOutRegDependency::0'][-1])
                        Super_nonspec_live += int(Super_results['system.switch_cpus.iq.nonspeculativeInstsAddedToDependentsDueToLiveOutRegDependency::1'][-1])
                        Super_nonspec_live += int(Super_results['system.switch_cpus.iq.nonspeculativeInstsAddedToDependentsDueToLiveOutRegDependency::2'][-1])
                        Super_nonspec_live += int(Super_results['system.switch_cpus.iq.nonspeculativeInstsAddedToDependentsDueToLiveOutRegDependency::3'][-1])
                        Super_nonspec_live += int(Super_results['system.switch_cpus.iq.nonspeculativeInstsAddedToDependentsDueToLiveOutRegDependency::4'][-1])
                        Super_spec_orig = int(Super_results['system.switch_cpus.iq.speculativeInstsAddedToDependentsDueToOriginalRegDependency::0'][-1])
                        Super_spec_orig += int(Super_results['system.switch_cpus.iq.speculativeInstsAddedToDependentsDueToOriginalRegDependency::1'][-1])
                        Super_spec_orig += int(Super_results['system.switch_cpus.iq.speculativeInstsAddedToDependentsDueToOriginalRegDependency::2'][-1])
                        Super_spec_orig += int(Super_results['system.switch_cpus.iq.speculativeInstsAddedToDependentsDueToOriginalRegDependency::3'][-1])
                        Super_spec_orig += int(Super_results['system.switch_cpus.iq.speculativeInstsAddedToDependentsDueToOriginalRegDependency::4'][-1])
                        Super_nonspec_orig = int(Super_results['system.switch_cpus.iq.nonspeculativeInstsAddedToDependentsDueToOriginalRegDependency::0'][-1])
                        Super_nonspec_orig += int(Super_results['system.switch_cpus.iq.nonspeculativeInstsAddedToDependentsDueToOriginalRegDependency::1'][-1])
                        Super_nonspec_orig += int(Super_results['system.switch_cpus.iq.nonspeculativeInstsAddedToDependentsDueToOriginalRegDependency::2'][-1])
                        Super_nonspec_orig += int(Super_results['system.switch_cpus.iq.nonspeculativeInstsAddedToDependentsDueToOriginalRegDependency::3'][-1])
                        Super_nonspec_orig += int(Super_results['system.switch_cpus.iq.nonspeculativeInstsAddedToDependentsDueToOriginalRegDependency::4'][-1])


                        if (Raw_spec_live == 0 and Raw_nonspec_live == 0 and Raw_spec_orig == 0 and Raw_nonspec_orig == 0) or (Super_spec_live == 0 and Super_nonspec_live == 0 and Super_spec_orig == 0 and Super_nonspec_orig == 0):
                                warnings.warn(bench + " simpoint " + str(chk_num+1) + " has no stat!")
                                cap_stat_file.close()
                                Super_stat_file.close()
                                continue
                        
                        total_weight = total_weight + weights[bench][chk_num]
                        #print("Raw: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", int(cap_results['system.switch_cpus.numCycles'][-1]))

                        Super_full[0] += weights[bench][chk_num] * Super_nonspec_live
                        Super_full[1] += weights[bench][chk_num] * Super_spec_live
                        Super_full[2] += weights[bench][chk_num] * Super_nonspec_orig
                        Super_full[3] += weights[bench][chk_num] * Super_spec_orig

                        Raw_full[0] += weights[bench][chk_num] * Raw_nonspec_live
                        Raw_full[1] += weights[bench][chk_num] * Raw_spec_live
                        Raw_full[2] += weights[bench][chk_num] * Raw_nonspec_orig
                        Raw_full[3] += weights[bench][chk_num] * Raw_spec_orig
                        
                        #PerformanceOverheadCap = PerformanceOverheadCap + weights[bench][chk_num] * ((int(cap_results['system.switch_cpus.numCycles'][-1]))/int(Super_results['system.switch_cpus.numCycles'][-1]))
                        #if int(Super_results['system.switch_cpus.rename.IQFullEvents'][-1]) != 0:
                        print("Super_full: ", Super_full)
                        print("Raw_full: ", Raw_full)
                        cap_stat_file.close()
                        Super_stat_file.close()

                        

                print("-------------------------------------------DONE----------------------------")
                if total_weight == 0:
                    final_results_performance["Raw"].append([0,0,0,0])
                    final_results_performance["Super"].append([0,0,0,0])
                    warnings.warn(bench + " Has no working simpoints!!")
                    continue

                #print('PerformanceOverheadCap: ', PerformanceOverheadCap/total_weight)
                #print('PerformanceOverheadSuper: ', total_weight,total_weight)   

                #final_results_performance["Super"].append((PerformanceOverheadCap/total_weight))
                for i in range(4):
                        Super_full[i] /= total_weight
                        Raw_full[i] /= total_weight

                final_results_performance["Raw"].append(Super_full)
                final_results_performance["Super"].append(Raw_full)
                
        Raw = np.array(final_results_performance["Raw"]).T.tolist()
        RawOpt = np.array(final_results_performance["Super"]).T.tolist()

        df = pd.DataFrame(final_results_performance, columns = COLUMNS)
        
        print(df)
        # Setting the positions and width for the bars
        pos = list(range(len(df["benchmark"]))) 
        #print(pos)
        width = 0.25

            
        # Plotting the bars
        fig = plt.figure(figsize=(10,6))
        gs = gridspec.GridSpec(1, 1, height_ratios=[1]) 
        # print('AVG', final_results_performance['CHEx86ContextSensitive'])
        # print('GEOMEAN', 1 - gmean(final_results_performance['CHEx86ContextSensitive']))
        ax0 = plt.subplot(gs[0])
        patterns = [ "*" , "+" , "x" , "o" , "\\" , "/", "o", "O", ".", "*" ]
        #colors = palettable.colorbrewer.diverging.Spectral_4.mpl_colors
        colors = palettable.cartocolors.diverging.Earth_3.mpl_colors

        bar1 = list(range(len(df["benchmark"]))) 
        bar2 = [i + (width*1) for i in bar1]

        #i_bot = list(np.add(Raw[0], Raw[1]))
        #ax0.bar(bar1, Raw[0], width, label="Squashing", color='green')
        #ax0.bar(bar1, Raw[1], width, bottom=Raw[0], label="Blocking", color='blue')
        #ax0.bar(bar1, Raw[2], width, bottom=i_bot, label="Other", color='red')

        #i_bot = list(np.add(RawOpt[0], RawOpt[1]))
        #ax0.bar(bar2, RawOpt[0], width, color='green')
        #ax0.bar(bar2, RawOpt[1], width, bottom=RawOpt[0], color='blue')
        #ax0.bar(bar2, RawOpt[2], width, bottom=i_bot, color='red')

        # nonspec_live, spec_live, nonspec_orig, spec_orig
        bot2 = list(np.add(Raw[0], Raw[1]))
        bot3 = list(np.add(bot2, Raw[2]))
        ax0.bar(bar1, Raw[0], width, label="Non Speculative LiveOut", color='none', hatch=3*patterns[0], edgecolor=colors[0])
        ax0.bar(bar1, Raw[1], width, bottom=Raw[0], label="Speculative LiveOut", color='none', hatch=3*patterns[1], edgecolor=colors[1])
        ax0.bar(bar1, Raw[2], width, bottom=bot2, label="Non Speculative Orig", color='none', hatch=3*patterns[2], edgecolor=colors[2])
        ax0.bar(bar1, Raw[3], width, bottom=bot3, label="Speculative Orig", color='none', hatch=3*patterns[3], edgecolor=colors[3])

        bot2 = list(np.add(RawOpt[0], RawOpt[1]))
        bot3 = list(np.add(bot2, RawOpt[2]))
        ax0.bar(bar2, RawOpt[0], width, color='none', hatch=3*patterns[0], edgecolor=colors[0])
        ax0.bar(bar2, RawOpt[1], width, bottom=RawOpt[0], color='none', hatch=3*patterns[1], edgecolor=colors[1])
        ax0.bar(bar2, RawOpt[2], width, bottom=bot2, color='none', hatch=3*patterns[2], edgecolor=colors[2])
        ax0.bar(bar2, RawOpt[3], width, bottom=bot3, color='none', hatch=3*patterns[3], edgecolor=colors[3])


        # Set the position of the x ticks
        ax0.set_xticks([p + 2.5 * width for p in bar1])

        # Set the labels for the x ticks
        ax0.set_xticklabels(df['benchmark'])
        ax0.tick_params(axis="x", labelsize=20)
        ax0.tick_params(axis="y", labelsize=15)
        # Setting the x-axis and y-axis limits
        #plt.xlim(min(pos)-width, max(pos)+width*4)
        #ax0.set(ylabel='$Normalized$ \n $Performance$')
        #ax0.set_ylabel('$Normalized$ \n $Execution$ $Time$', fontsize = 16.0)
        ax0.set_ylabel('Stalls Due to CC Dependency', fontsize = 16.0)
        #ax0.ylim([0,1.1])
        plt.xticks(rotation=55)
        #plt.yscale('symlog', linthreshy=1)
        plt.yscale('linear')
        # Adding the legend and showing the plot

        for tick in ax0.xaxis.get_majorticklabels():
            tick.set_horizontalalignment("right")

        plt.legend(bbox_to_anchor=(0,1.05,1,0.2), loc="lower left", mode="expand", borderaxespad=0, edgecolor='black', ncol=2, prop={"size":16, "weight":'bold'})
        ax0.grid( linestyle='-.', linewidth=0.5, alpha=.5, axis='y', zorder=0)
        plt.tight_layout()

        if not os.path.exists(file_type):
                os.makedirs(file_type)
        plt.savefig(file_type + '/CCDepend.png', bbox_inches='tight')
        plt.show()


def get_int_from_results():
        
        results_index_file = open('./results_index.txt', mode='r')
        contents = results_index_file.read()
        results_index = contents.split("\n")

        final_results_from["Spec $"] = []
        final_results_from["Nonspec $"] = []

        #for bench in benchs:
        for bench in benchs:

                # nonspec_live, spec_live, nonspec_orig, spec_orig
                #Super_full = [0,0,0,0]
                spec_full = [0] * 21
                nonspec_full = [0] * 21
                total_weight = 0.0

                for chk_num in range(len(benchs[bench])):

                        if weights[bench][chk_num] == 0.0:
                                continue
                        
                        cap_stat_file_dir = './' + bench + '_' + type2 + '/m5out_sim_' + bench + '_' + type2 + "_" + str(chk_num+1) + '/stats.txt'
                        #Super_stat_file_dir = './' + bench + '_' + type1 + '/m5out_sim_' + bench + '_' + type1 + "_" + str(chk_num+1) + '/stats.txt'
                        # print(cap_stat_file_dir)
                        # print(Super_stat_file_dir)
                        cap_stat_file = open(cap_stat_file_dir, mode='r')
                        cap_stat_file_content = cap_stat_file.read()

                        cap_results = measure(results_index, cap_stat_file_content)
                        spec_int_live = [0] *21
                        nonspec_int_live = [0] *21
                        spec_int_live[0] = int(cap_results['system.switch_cpus.iq.speculativeIntLiveOutInstType_0::AND'][-1])
                        spec_int_live[1] = int(cap_results['system.switch_cpus.iq.speculativeIntLiveOutInstType_0::ANDI'][-1])
                        spec_int_live[2] = int(cap_results['system.switch_cpus.iq.speculativeIntLiveOutInstType_0::OR'][-1])
                        spec_int_live[3] = int(cap_results['system.switch_cpus.iq.speculativeIntLiveOutInstType_0::ORI'][-1])
                        spec_int_live[4] = int(cap_results['system.switch_cpus.iq.speculativeIntLiveOutInstType_0::XOR'][-1])
                        spec_int_live[5] = int(cap_results['system.switch_cpus.iq.speculativeIntLiveOutInstType_0::XORI'][-1])
                        spec_int_live[6] = int(cap_results['system.switch_cpus.iq.speculativeIntLiveOutInstType_0::ADD'][-1])
                        spec_int_live[7] = int(cap_results['system.switch_cpus.iq.speculativeIntLiveOutInstType_0::ADDI'][-1])
                        spec_int_live[8] = int(cap_results['system.switch_cpus.iq.speculativeIntLiveOutInstType_0::SUB'][-1])
                        spec_int_live[9] = int(cap_results['system.switch_cpus.iq.speculativeIntLiveOutInstType_0::SUBI'][-1])
                        spec_int_live[10] = int(cap_results['system.switch_cpus.iq.speculativeIntLiveOutInstType_0::SLLI'][-1])
                        spec_int_live[11] = int(cap_results['system.switch_cpus.iq.speculativeIntLiveOutInstType_0::SRLI'][-1])
                        spec_int_live[12] = int(cap_results['system.switch_cpus.iq.speculativeIntLiveOutInstType_0::LEA'][-1])
                        spec_int_live[13] = int(cap_results['system.switch_cpus.iq.speculativeIntLiveOutInstType_0::SEXTI'][-1])
                        spec_int_live[14] = int(cap_results['system.switch_cpus.iq.speculativeIntLiveOutInstType_0::ZEXTI'][-1])
                        spec_int_live[15] = int(cap_results['system.switch_cpus.iq.speculativeIntLiveOutInstType_0::LIMM'][-1])
                        spec_int_live[16] = int(cap_results['system.switch_cpus.iq.speculativeIntLiveOutInstType_0::RDIP'][-1])
                        spec_int_live[17] = int(cap_results['system.switch_cpus.iq.speculativeIntLiveOutInstType_0::MOV'][-1])
                        spec_int_live[18] = int(cap_results['system.switch_cpus.iq.speculativeIntLiveOutInstType_0::MOVI'][-1])
                        spec_int_live[19] = int(cap_results['system.switch_cpus.iq.speculativeIntLiveOutInstType_0::NOP'][-1])
                        spec_int_live[20] = int(cap_results['system.switch_cpus.iq.speculativeIntLiveOutInstType_0::PredSource'][-1])
                        spec_int_live_total = int(cap_results['system.switch_cpus.iq.speculativeIntLiveOutInstType_0::total'][-1])

                        nonspec_int_live[0] = int(cap_results['system.switch_cpus.iq.nonspeculativeIntLiveOutInstType_0::AND'][-1])
                        nonspec_int_live[1] = int(cap_results['system.switch_cpus.iq.nonspeculativeIntLiveOutInstType_0::ANDI'][-1])
                        nonspec_int_live[2] = int(cap_results['system.switch_cpus.iq.nonspeculativeIntLiveOutInstType_0::OR'][-1])
                        nonspec_int_live[3] = int(cap_results['system.switch_cpus.iq.nonspeculativeIntLiveOutInstType_0::ORI'][-1])
                        nonspec_int_live[4] = int(cap_results['system.switch_cpus.iq.nonspeculativeIntLiveOutInstType_0::XOR'][-1])
                        nonspec_int_live[5] = int(cap_results['system.switch_cpus.iq.nonspeculativeIntLiveOutInstType_0::XORI'][-1])
                        nonspec_int_live[6] = int(cap_results['system.switch_cpus.iq.nonspeculativeIntLiveOutInstType_0::ADD'][-1])
                        nonspec_int_live[7] = int(cap_results['system.switch_cpus.iq.nonspeculativeIntLiveOutInstType_0::ADDI'][-1])
                        nonspec_int_live[8] = int(cap_results['system.switch_cpus.iq.nonspeculativeIntLiveOutInstType_0::SUB'][-1])
                        nonspec_int_live[9] = int(cap_results['system.switch_cpus.iq.nonspeculativeIntLiveOutInstType_0::SUBI'][-1])
                        nonspec_int_live[10] = int(cap_results['system.switch_cpus.iq.nonspeculativeIntLiveOutInstType_0::SLLI'][-1])
                        nonspec_int_live[11] = int(cap_results['system.switch_cpus.iq.nonspeculativeIntLiveOutInstType_0::SRLI'][-1])
                        nonspec_int_live[12] = int(cap_results['system.switch_cpus.iq.nonspeculativeIntLiveOutInstType_0::LEA'][-1])
                        nonspec_int_live[13] = int(cap_results['system.switch_cpus.iq.nonspeculativeIntLiveOutInstType_0::SEXTI'][-1])
                        nonspec_int_live[14] = int(cap_results['system.switch_cpus.iq.nonspeculativeIntLiveOutInstType_0::ZEXTI'][-1])
                        nonspec_int_live[15] = int(cap_results['system.switch_cpus.iq.nonspeculativeIntLiveOutInstType_0::LIMM'][-1])
                        nonspec_int_live[16] = int(cap_results['system.switch_cpus.iq.nonspeculativeIntLiveOutInstType_0::RDIP'][-1])
                        nonspec_int_live[17] = int(cap_results['system.switch_cpus.iq.nonspeculativeIntLiveOutInstType_0::MOV'][-1])
                        nonspec_int_live[18] = int(cap_results['system.switch_cpus.iq.nonspeculativeIntLiveOutInstType_0::MOVI'][-1])
                        nonspec_int_live[19] = int(cap_results['system.switch_cpus.iq.nonspeculativeIntLiveOutInstType_0::NOP'][-1])
                        nonspec_int_live[20] = int(cap_results['system.switch_cpus.iq.nonspeculativeIntLiveOutInstType_0::PredSource'][-1])
                        nonspec_int_live_total = int(cap_results['system.switch_cpus.iq.nonspeculativeIntLiveOutInstType_0::total'][-1])

                        if (spec_int_live_total == 0 and nonspec_int_live_total == 0):
                                warnings.warn(bench + " simpoint " + str(chk_num+1) + " has no stat!")
                                cap_stat_file.close()
                                #Super_stat_file.close()
                                continue


                        
                        total_weight = total_weight + weights[bench][chk_num]
                        #print("Raw: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", int(cap_results['system.switch_cpus.numCycles'][-1]))

                        spec_full = list(np.add(spec_full, weights[bench][chk_num] * np.array(spec_int_live)))
                        nonspec_full = list(np.add(nonspec_full, weights[bench][chk_num] * np.array(nonspec_int_live)))
                        
                        #PerformanceOverheadCap = PerformanceOverheadCap + weights[bench][chk_num] * ((int(cap_results['system.switch_cpus.numCycles'][-1]))/int(Super_results['system.switch_cpus.numCycles'][-1]))
                        #if int(Super_results['system.switch_cpus.rename.IQFullEvents'][-1]) != 0:
                        #print("Super_full: ", Super_full)
                        #print("Raw_full: ", Raw_full)
                        cap_stat_file.close()
                        #Super_stat_file.close()

                        

                print("-------------------------------"+bench+"---DONE--------------------------------------")
                if total_weight == 0:
                    final_results_from["Spec $"].append([0] * 21)
                    final_results_from["Nonspec $"].append([0] * 21)
                    warnings.warn(bench + " Has no working simpoints!!")
                    continue

                #print('PerformanceOverheadCap: ', PerformanceOverheadCap/total_weight)
                #print('PerformanceOverheadSuper: ', total_weight,total_weight)   

                #final_results_performance["Super"].append((PerformanceOverheadCap/total_weight))
                for i in range(21):
                        spec_full[i] /= total_weight
                        nonspec_full[i] /= total_weight

                final_results_from["Spec $"].append(spec_full)
                final_results_from["Nonspec $"].append(nonspec_full)
                
        spec = np.array(final_results_from["Spec $"]).T.tolist()
        non_spec = np.array(final_results_from["Nonspec $"]).T.tolist()

        df = pd.DataFrame(final_results_from, columns = ["benchmark", "Spec $", "Nonspec $"])
        
        print(df)
        # Setting the positions and width for the bars
        pos = list(range(len(df["benchmark"]))) 
        #print(pos)
        width = 0.25

            
        # Plotting the bars
        fig = plt.figure(figsize=(10,6))
        gs = gridspec.GridSpec(1, 1, height_ratios=[1]) 
        # print('AVG', final_results_performance['CHEx86ContextSensitive'])
        # print('GEOMEAN', 1 - gmean(final_results_performance['CHEx86ContextSensitive']))
        ax0 = plt.subplot(gs[0])
        patterns = [ "", "///", "*" , "+" , "x" , "o" , "\\" , "/", "o", "O", ".", "*" ]
        colors = palettable.colorbrewer.qualitative.Set3_11.mpl_colors
        num_colors=11
        #colors = palettable.cartocolors.diverging.Earth_3.mpl_colors

        bar1 = list(range(len(df["benchmark"]))) 
        bar2 = [i+.1+ (width) for i in bar1]

        #i_bot = list(np.add(Raw[0], Raw[1]))
        #ax0.bar(bar1, Raw[0], width, label="Squashing", color='green')
        #ax0.bar(bar1, Raw[1], width, bottom=Raw[0], label="Blocking", color='blue')
        #ax0.bar(bar1, Raw[2], width, bottom=i_bot, label="Other", color='red')

        #i_bot = list(np.add(RawOpt[0], RawOpt[1]))
        #ax0.bar(bar2, RawOpt[0], width, color='green')
        #ax0.bar(bar2, RawOpt[1], width, bottom=RawOpt[0], color='blue')
        #ax0.bar(bar2, RawOpt[2], width, bottom=i_bot, color='red')

        # nonspec_live, spec_live, nonspec_orig, spec_orig
        ax0.bar(bar1, spec[0], width, label=inst_lables[0], color=colors[0], hatch=patterns[0])#, edgecolor=colors[0])
        bottom = spec[0]
        for i in range(1, 21):
                ax0.bar(bar1, spec[i], width, bottom=bottom, label=inst_lables[i], color=colors[i%num_colors], hatch=patterns[int(i/num_colors)])#, edgecolor=colors[i])
                bottom = list(np.add(bottom, spec[i]))

        ax0.bar(bar2, non_spec[0], width, color=colors[0], hatch=patterns[0])#, edgecolor=colors[0])
        bottom = non_spec[0]
        for i in range(1, 21):
                ax0.bar(bar2, non_spec[i], width, bottom=bottom, color=colors[i % num_colors], hatch=patterns[int(i/num_colors)])#, edgecolor=colors[i])
                bottom = list(np.add(bottom, non_spec[i]))


        # Set the position of the x ticks
        ax0.set_xticks([p + 2.5 * width for p in bar1])

        # Set the labels for the x ticks
        ax0.set_xticklabels(df['benchmark'])
        ax0.tick_params(axis="x", labelsize=18)
        ax0.tick_params(axis="y", labelsize=15)
        # Setting the x-axis and y-axis limits
        #plt.xlim(min(pos)-width, max(pos)+width*4)
        #ax0.set(ylabel='$Normalized$ \n $Performance$')
        #ax0.set_ylabel('$Normalized$ \n $Execution$ $Time$', fontsize = 16.0)
        ax0.set_ylabel('Stalls Due to Int Dependency', fontsize = 16.0)
        #ax0.ylim([0,1.1])
        plt.xticks(rotation=55)
        #plt.yscale('symlog', linthreshy=1)
        plt.yscale('linear')
        # Adding the legend and showing the plot

        for tick in ax0.xaxis.get_majorticklabels():
            tick.set_horizontalalignment("right")

        plt.legend(bbox_to_anchor=(0,1.05,1,0.2), loc="lower left", mode="expand", borderaxespad=0, edgecolor='black', ncol=7, prop={"size":12, "weight":'bold'})
        ax0.grid( linestyle='-.', linewidth=0.5, alpha=.5, axis='y', zorder=0)
        plt.tight_layout()

        if not os.path.exists(file_type):
                os.makedirs(file_type)
        plt.savefig(file_type + '/FromIntDepend.png', bbox_inches='tight')
        plt.show()


def get_cc_from_results():
        
        results_index_file = open('./results_index.txt', mode='r')
        contents = results_index_file.read()
        results_index = contents.split("\n")

        final_results_from["Spec $"] = []
        final_results_from["Nonspec $"] = []

        #for bench in benchs:
        for bench in benchs:

                # nonspec_live, spec_live, nonspec_orig, spec_orig
                #Super_full = [0,0,0,0]
                spec_full = [0] * 21
                nonspec_full = [0] * 21
                total_weight = 0.0

                for chk_num in range(len(benchs[bench])):

                        if weights[bench][chk_num] == 0.0:
                                continue
                        
                        cap_stat_file_dir = './' + bench + '_' + type2 + '/m5out_sim_' + bench + '_' + type2 + "_" + str(chk_num+1) + '/stats.txt'
                        #Super_stat_file_dir = './' + bench + '_' + type1 + '/m5out_sim_' + bench + '_' + type1 + "_" + str(chk_num+1) + '/stats.txt'
                        # print(cap_stat_file_dir)
                        # print(Super_stat_file_dir)
                        cap_stat_file = open(cap_stat_file_dir, mode='r')
                        cap_stat_file_content = cap_stat_file.read()

                        cap_results = measure(results_index, cap_stat_file_content)
                        spec_int_live = [0] *21
                        nonspec_int_live = [0] *21
                        spec_int_live[0] = int(cap_results['system.switch_cpus.iq.speculativeCCLiveOutInstType_0::AND'][-1])
                        spec_int_live[1] = int(cap_results['system.switch_cpus.iq.speculativeCCLiveOutInstType_0::ANDI'][-1])
                        spec_int_live[2] = int(cap_results['system.switch_cpus.iq.speculativeCCLiveOutInstType_0::OR'][-1])
                        spec_int_live[3] = int(cap_results['system.switch_cpus.iq.speculativeCCLiveOutInstType_0::ORI'][-1])
                        spec_int_live[4] = int(cap_results['system.switch_cpus.iq.speculativeCCLiveOutInstType_0::XOR'][-1])
                        spec_int_live[5] = int(cap_results['system.switch_cpus.iq.speculativeCCLiveOutInstType_0::XORI'][-1])
                        spec_int_live[6] = int(cap_results['system.switch_cpus.iq.speculativeCCLiveOutInstType_0::ADD'][-1])
                        spec_int_live[7] = int(cap_results['system.switch_cpus.iq.speculativeCCLiveOutInstType_0::ADDI'][-1])
                        spec_int_live[8] = int(cap_results['system.switch_cpus.iq.speculativeCCLiveOutInstType_0::SUB'][-1])
                        spec_int_live[9] = int(cap_results['system.switch_cpus.iq.speculativeCCLiveOutInstType_0::SUBI'][-1])
                        spec_int_live[10] = int(cap_results['system.switch_cpus.iq.speculativeCCLiveOutInstType_0::SLLI'][-1])
                        spec_int_live[11] = int(cap_results['system.switch_cpus.iq.speculativeCCLiveOutInstType_0::SRLI'][-1])
                        spec_int_live[12] = int(cap_results['system.switch_cpus.iq.speculativeCCLiveOutInstType_0::LEA'][-1])
                        spec_int_live[13] = int(cap_results['system.switch_cpus.iq.speculativeCCLiveOutInstType_0::SEXTI'][-1])
                        spec_int_live[14] = int(cap_results['system.switch_cpus.iq.speculativeCCLiveOutInstType_0::ZEXTI'][-1])
                        spec_int_live[15] = int(cap_results['system.switch_cpus.iq.speculativeCCLiveOutInstType_0::LIMM'][-1])
                        spec_int_live[16] = int(cap_results['system.switch_cpus.iq.speculativeCCLiveOutInstType_0::RDIP'][-1])
                        spec_int_live[17] = int(cap_results['system.switch_cpus.iq.speculativeCCLiveOutInstType_0::MOV'][-1])
                        spec_int_live[18] = int(cap_results['system.switch_cpus.iq.speculativeCCLiveOutInstType_0::MOVI'][-1])
                        spec_int_live[19] = int(cap_results['system.switch_cpus.iq.speculativeCCLiveOutInstType_0::NOP'][-1])
                        spec_int_live[20] = int(cap_results['system.switch_cpus.iq.speculativeCCLiveOutInstType_0::PredSource'][-1])
                        spec_int_live_total = int(cap_results['system.switch_cpus.iq.speculativeCCLiveOutInstType_0::total'][-1])

                        nonspec_int_live[0] = int(cap_results['system.switch_cpus.iq.nonspeculativeCCLiveOutInstType_0::AND'][-1])
                        nonspec_int_live[1] = int(cap_results['system.switch_cpus.iq.nonspeculativeCCLiveOutInstType_0::ANDI'][-1])
                        nonspec_int_live[2] = int(cap_results['system.switch_cpus.iq.nonspeculativeCCLiveOutInstType_0::OR'][-1])
                        nonspec_int_live[3] = int(cap_results['system.switch_cpus.iq.nonspeculativeCCLiveOutInstType_0::ORI'][-1])
                        nonspec_int_live[4] = int(cap_results['system.switch_cpus.iq.nonspeculativeCCLiveOutInstType_0::XORI'][-1])
                        nonspec_int_live[5] = int(cap_results['system.switch_cpus.iq.nonspeculativeCCLiveOutInstType_0::XOR'][-1])
                        nonspec_int_live[6] = int(cap_results['system.switch_cpus.iq.nonspeculativeCCLiveOutInstType_0::ADD'][-1])
                        nonspec_int_live[7] = int(cap_results['system.switch_cpus.iq.nonspeculativeCCLiveOutInstType_0::ADDI'][-1])
                        nonspec_int_live[8] = int(cap_results['system.switch_cpus.iq.nonspeculativeCCLiveOutInstType_0::SUB'][-1])
                        nonspec_int_live[9] = int(cap_results['system.switch_cpus.iq.nonspeculativeCCLiveOutInstType_0::SUBI'][-1])
                        nonspec_int_live[10] = int(cap_results['system.switch_cpus.iq.nonspeculativeCCLiveOutInstType_0::SLLI'][-1])
                        nonspec_int_live[11] = int(cap_results['system.switch_cpus.iq.nonspeculativeCCLiveOutInstType_0::SRLI'][-1])
                        nonspec_int_live[12] = int(cap_results['system.switch_cpus.iq.nonspeculativeCCLiveOutInstType_0::LEA'][-1])
                        nonspec_int_live[13] = int(cap_results['system.switch_cpus.iq.nonspeculativeCCLiveOutInstType_0::SEXTI'][-1])
                        nonspec_int_live[14] = int(cap_results['system.switch_cpus.iq.nonspeculativeCCLiveOutInstType_0::ZEXTI'][-1])
                        nonspec_int_live[15] = int(cap_results['system.switch_cpus.iq.nonspeculativeCCLiveOutInstType_0::LIMM'][-1])
                        nonspec_int_live[16] = int(cap_results['system.switch_cpus.iq.nonspeculativeCCLiveOutInstType_0::RDIP'][-1])
                        nonspec_int_live[17] = int(cap_results['system.switch_cpus.iq.nonspeculativeCCLiveOutInstType_0::MOV'][-1])
                        nonspec_int_live[18] = int(cap_results['system.switch_cpus.iq.nonspeculativeCCLiveOutInstType_0::MOVI'][-1])
                        nonspec_int_live[19] = int(cap_results['system.switch_cpus.iq.nonspeculativeCCLiveOutInstType_0::NOP'][-1])
                        nonspec_int_live[20] = int(cap_results['system.switch_cpus.iq.nonspeculativeCCLiveOutInstType_0::PredSource'][-1])
                        nonspec_int_live_total = int(cap_results['system.switch_cpus.iq.nonspeculativeCCLiveOutInstType_0::total'][-1])


                        if (spec_int_live_total == 0 and nonspec_int_live_total == 0):
                                warnings.warn(bench + " simpoint " + str(chk_num+1) + " has no stat!")
                                cap_stat_file.close()
                                #Super_stat_file.close()
                                continue


                        
                        total_weight = total_weight + weights[bench][chk_num]
                        #print("Raw: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", int(cap_results['system.switch_cpus.numCycles'][-1]))

                        spec_full = list(np.add(spec_full, weights[bench][chk_num] * np.array(spec_int_live)))
                        nonspec_full = list(np.add(nonspec_full, weights[bench][chk_num] * np.array(nonspec_int_live)))
                        
                        #PerformanceOverheadCap = PerformanceOverheadCap + weights[bench][chk_num] * ((int(cap_results['system.switch_cpus.numCycles'][-1]))/int(Super_results['system.switch_cpus.numCycles'][-1]))
                        #if int(Super_results['system.switch_cpus.rename.IQFullEvents'][-1]) != 0:
                        #print("Super_full: ", Super_full)
                        #print("Raw_full: ", Raw_full)
                        cap_stat_file.close()
                        #Super_stat_file.close()

                        

                print("-------------------------------"+bench+"---DONE--------------------------------------")
                if total_weight == 0:
                    final_results_from["Spec $"].append([0] * 21)
                    final_results_from["Nonspec $"].append([0] * 21)
                    warnings.warn(bench + " Has no working simpoints!!")
                    continue

                #print('PerformanceOverheadCap: ', PerformanceOverheadCap/total_weight)
                #print('PerformanceOverheadSuper: ', total_weight,total_weight)   

                #final_results_performance["Super"].append((PerformanceOverheadCap/total_weight))
                for i in range(21):
                        spec_full[i] /= total_weight
                        nonspec_full[i] /= total_weight

                final_results_from["Spec $"].append(spec_full)
                final_results_from["Nonspec $"].append(nonspec_full)
                
        spec = np.array(final_results_from["Spec $"]).T.tolist()
        non_spec = np.array(final_results_from["Nonspec $"]).T.tolist()

        df = pd.DataFrame(final_results_from, columns = ["benchmark", "Spec $", "Nonspec $"])
        
        print(df)
        # Setting the positions and width for the bars
        pos = list(range(len(df["benchmark"]))) 
        #print(pos)
        width = 0.25

            
        # Plotting the bars
        fig = plt.figure(figsize=(10,6))
        gs = gridspec.GridSpec(1, 1, height_ratios=[1]) 
        # print('AVG', final_results_performance['CHEx86ContextSensitive'])
        # print('GEOMEAN', 1 - gmean(final_results_performance['CHEx86ContextSensitive']))
        ax0 = plt.subplot(gs[0])
        patterns = [ "", "///", "*" , "+" , "x" , "o" , "\\" , "/", "o", "O", ".", "*" ]
        colors = palettable.colorbrewer.qualitative.Set3_11.mpl_colors
        num_colors = 11
        #colors = palettable.cartocolors.diverging.Earth_3.mpl_colors

        bar1 = list(range(len(df["benchmark"]))) 
        bar2 = [i+.1+ (width) for i in bar1]

        #i_bot = list(np.add(Raw[0], Raw[1]))
        #ax0.bar(bar1, Raw[0], width, label="Squashing", color='green')
        #ax0.bar(bar1, Raw[1], width, bottom=Raw[0], label="Blocking", color='blue')
        #ax0.bar(bar1, Raw[2], width, bottom=i_bot, label="Other", color='red')

        #i_bot = list(np.add(RawOpt[0], RawOpt[1]))
        #ax0.bar(bar2, RawOpt[0], width, color='green')
        #ax0.bar(bar2, RawOpt[1], width, bottom=RawOpt[0], color='blue')
        #ax0.bar(bar2, RawOpt[2], width, bottom=i_bot, color='red')

        # nonspec_live, spec_live, nonspec_orig, spec_orig
        ax0.bar(bar1, spec[0], width, label=inst_lables[0], color=colors[0], hatch=patterns[0])#, edgecolor=colors[0])
        bottom = spec[0]
        for i in range(1, 21):
                ax0.bar(bar1, spec[i], width, bottom=bottom, label=inst_lables[i], color=colors[i%num_colors], hatch=patterns[int(i/num_colors)])#, edgecolor=colors[i])
                bottom = list(np.add(bottom, spec[i]))

        ax0.bar(bar2, non_spec[0], width, color=colors[0], hatch=patterns[0])#, edgecolor=colors[0])
        bottom = non_spec[0]
        for i in range(1, 21):
                ax0.bar(bar2, non_spec[i], width, bottom=bottom, color=colors[i % num_colors], hatch=patterns[int(i/num_colors)])#, edgecolor=colors[i])
                bottom = list(np.add(bottom, non_spec[i]))


        # Set the position of the x ticks
        ax0.set_xticks([p + 2.5 * width for p in bar1])

        # Set the labels for the x ticks
        ax0.set_xticklabels(df['benchmark'])
        ax0.tick_params(axis="x", labelsize=18)
        ax0.tick_params(axis="y", labelsize=15)
        # Setting the x-axis and y-axis limits
        #plt.xlim(min(pos)-width, max(pos)+width*4)
        #ax0.set(ylabel='$Normalized$ \n $Performance$')
        #ax0.set_ylabel('$Normalized$ \n $Execution$ $Time$', fontsize = 16.0)
        ax0.set_ylabel('Stalls Due to CC Dependency', fontsize = 16.0)
        #ax0.ylim([0,1.1])
        plt.xticks(rotation=55)
        #plt.yscale('symlog', linthreshy=1)
        plt.yscale('linear')
        # Adding the legend and showing the plot

        for tick in ax0.xaxis.get_majorticklabels():
            tick.set_horizontalalignment("right")

        plt.legend(bbox_to_anchor=(0,1.05,1,0.2), loc="lower left", mode="expand", borderaxespad=0, edgecolor='black', ncol=7, prop={"size":12, "weight":'bold'})
        ax0.grid( linestyle='-.', linewidth=0.5, alpha=.5, axis='y', zorder=0)
        plt.tight_layout()

        if not os.path.exists(file_type):
                os.makedirs(file_type)
        plt.savefig(file_type + '/FromCCDepend.png', bbox_inches='tight')
        plt.show()



def get_iew_blocked_cause_results():
        
        results_index_file = open('./results_index.txt', mode='r')
        contents = results_index_file.read()
        results_index = contents.split("\n")

        final_results_performance["Raw"] = []
        final_results_performance["Super"] = []

        #for bench in benchs:
        for bench in benchs:

                # stallC, stallI, IQFull, LSQFull, bandwidth
                Super_full = [0,0,0,0,0]
                Raw_full = [0,0,0,0,0]
                total_weight = 0.0

                for chk_num in range(len(benchs[bench])):

                        if weights[bench][chk_num] == 0.0:
                                continue
                        
                        cap_stat_file_dir = './' + bench + '_' + type2 + '/m5out_sim_' + bench + '_' + type2 + "_" + str(chk_num+1) + '/stats.txt'
                        Super_stat_file_dir = './' + bench + '_' + type1 + '/m5out_sim_' + bench + '_' + type1 + "_" + str(chk_num+1) + '/stats.txt'
                        # print(cap_stat_file_dir)
                        # print(Super_stat_file_dir)
                        cap_stat_file = open(cap_stat_file_dir, mode='r')
                        Super_stat_file = open(Super_stat_file_dir, mode='r')
                        cap_stat_file_content = cap_stat_file.read()
                        Super_stat_file_content = Super_stat_file.read()

                        cap_results = measure(results_index, cap_stat_file_content)
                        Super_results = measure(results_index, Super_stat_file_content)

                        #print(cap_results)
                        # print(Super_stat_file_dir)

                        Raw_stallC = int(cap_results['system.switch_cpus.iew.stalledDueToCommit'][-1])
                        Raw_stallI = int(cap_results['system.switch_cpus.iew.stalledDueToIQFull'][-1])
                        Raw_IQFull = int(cap_results['system.switch_cpus.iew.iewIQFullEvents'][-1])
                        Raw_LSQFull = int(cap_results['system.switch_cpus.iew.iewLSQFullEvents'][-1])
                        Raw_bandwidth = int(cap_results['system.switch_cpus.iew.blockingDueToBandwidthFull'][-1])
                        
                        Super_stallC = int(Super_results['system.switch_cpus.iew.stalledDueToCommit'][-1])
                        Super_stallI = int(Super_results['system.switch_cpus.iew.stalledDueToIQFull'][-1])
                        Super_IQFull = int(Super_results['system.switch_cpus.iew.iewIQFullEvents'][-1])
                        Super_LSQFull = int(Super_results['system.switch_cpus.iew.iewLSQFullEvents'][-1])
                        Super_bandwidth = int(Super_results['system.switch_cpus.iew.blockingDueToBandwidthFull'][-1])
                        
                        if Raw_stallC == 0 or Super_stallC == 0:
                                warnings.warn(bench + " simpoint " + str(chk_num+1) + " has no stat!")
                                cap_stat_file.close()
                                Super_stat_file.close()
                                continue
                        
                        total_weight = total_weight + weights[bench][chk_num]
                        #print("Raw: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", int(cap_results['system.switch_cpus.numCycles'][-1]))
                        
                        Super_full[0] += weights[bench][chk_num] * Super_stallC
                        Super_full[1] += weights[bench][chk_num] * Super_stallI
                        Super_full[2] += weights[bench][chk_num] * Super_IQFull
                        Super_full[3] += weights[bench][chk_num] * Super_LSQFull
                        Super_full[4] += weights[bench][chk_num] * Super_bandwidth

                        Raw_full[0] += weights[bench][chk_num] * Raw_stallC
                        Raw_full[1] += weights[bench][chk_num] * Raw_stallI
                        Raw_full[2] += weights[bench][chk_num] * Raw_IQFull
                        Raw_full[3] += weights[bench][chk_num] * Raw_LSQFull
                        Raw_full[4] += weights[bench][chk_num] * Raw_bandwidth

                        #PerformanceOverheadCap = PerformanceOverheadCap + weights[bench][chk_num] * ((int(cap_results['system.switch_cpus.numCycles'][-1]))/int(Super_results['system.switch_cpus.numCycles'][-1]))
                        #if int(Super_results['system.switch_cpus.rename.IQFullEvents'][-1]) != 0:
                        print("Super_full: ", Super_full)
                        print("Raw_full: ", Raw_full)
                        cap_stat_file.close()

                        

                print("-------------------------------------------DONE----------------------------")
                if total_weight == 0:
                    final_results_performance["Raw"].append([0,0,0,0,0])
                    final_results_performance["Super"].append([0,0,0,0,0])
                    warnings.warn(bench + " Has no working simpoints!!")
                    continue

                #print('PerformanceOverheadCap: ', PerformanceOverheadCap/total_weight)
                #print('PerformanceOverheadSuper: ', total_weight,total_weight)   

                #final_results_performance["Super"].append((PerformanceOverheadCap/total_weight))
                for i in range(5):
                        Super_full[i] /= total_weight
                        Raw_full[i] /= total_weight

                final_results_performance["Raw"].append(Super_full)
                final_results_performance["Super"].append(Raw_full)
                
        Raw = np.array(final_results_performance["Raw"]).T.tolist()
        RawOpt = np.array(final_results_performance["Super"]).T.tolist()

        df = pd.DataFrame(final_results_performance, columns = COLUMNS)
        
        print(df)
        # Setting the positions and width for the bars
        pos = list(range(len(df["benchmark"]))) 
        #print(pos)
        width = 0.25

            
        # Plotting the bars
        fig = plt.figure(figsize=(10,6))
        gs = gridspec.GridSpec(1, 1, height_ratios=[1]) 
        # print('AVG', final_results_performance['CHEx86ContextSensitive'])
        # print('GEOMEAN', 1 - gmean(final_results_performance['CHEx86ContextSensitive']))
        ax0 = plt.subplot(gs[0])
        patterns = [ "", "*" , "+" , "x" , "o" , "\\" , "/", "o", "O", ".", "*" ]
        #colors = palettable.colorbrewer.diverging.Spectral_4.mpl_colors
        colors = palettable.cartocolors.diverging.Earth_3.mpl_colors
        num_colors = 6

        bar1 = list(range(len(df["benchmark"]))) 
        bar2 = [i + (width*1) for i in bar1]

        ax0.bar(bar1, Raw[0], width, label=iew_cause_lables[0], color=colors[0], hatch=patterns[0])#, edgecolor=colors[0])
        bottom = Raw[0]
        for i in range(1, 5):
                ax0.bar(bar1, Raw[i], width, bottom=bottom, label=iew_cause_lables[i], color=colors[i%num_colors], hatch=patterns[int(i/num_colors)])#, edgecolor=colors[i])
                bottom = list(np.add(bottom, Raw[i]))

        ax0.bar(bar2, RawOpt[0], width, color=colors[0], hatch=patterns[0])#, edgecolor=colors[0])
        bottom = RawOpt[0]
        for i in range(1, 5):
                ax0.bar(bar2, RawOpt[i], width, bottom=bottom, color=colors[i % num_colors], hatch=patterns[int(i/num_colors)])#, edgecolor=colors[i])
                bottom = list(np.add(bottom, RawOpt[i]))

        # Set the position of the x ticks
        ax0.set_xticks([p + 2.5 * width for p in bar1])

        # Set the labels for the x ticks
        ax0.set_xticklabels(df['benchmark'])
        ax0.tick_params(axis="x", labelsize=20)
        ax0.tick_params(axis="y", labelsize=15)
        # Setting the x-axis and y-axis limits
        #plt.xlim(min(pos)-width, max(pos)+width*4)
        #ax0.set(ylabel='$Normalized$ \n $Performance$')
        #ax0.set_ylabel('$Normalized$ \n $Execution$ $Time$', fontsize = 16.0)
        ax0.set_ylabel('Number of Cycles', fontsize = 16.0)
        #ax0.ylim([0,1.1])
        plt.xticks(rotation=55)
        #plt.yscale('symlog', linthreshy=1)
        plt.yscale('linear')
        # Adding the legend and showing the plot

        for tick in ax0.xaxis.get_majorticklabels():
            tick.set_horizontalalignment("right")

        plt.legend(bbox_to_anchor=(0,1.05,1,0.2), loc="lower left", mode="expand", borderaxespad=0, edgecolor='black', ncol=2, prop={"size":16, "weight":'bold'})
        ax0.grid( linestyle='-.', linewidth=0.5, alpha=.5, axis='y', zorder=0)
        plt.tight_layout()

        if not os.path.exists(file_type):
                os.makedirs(file_type)
        plt.savefig(file_type + '/BlockedIEWSource.png', bbox_inches='tight')
        plt.show()
 

def get_commitSquashed_results():
        
        results_index_file = open('./results_index.txt', mode='r')
        contents = results_index_file.read()
        results_index = contents.split("\n")

        final_results_performance["Raw"] = []
        final_results_performance["Super"] = []

        idx = 0
        #for bench in benchs:
        for bench in benchs:

                Raw_insts = 0.0 #should be zero
                Super_insts = 0.0 
                total_weight = 0.0

                for chk_num in range(len(benchs[bench])):

                        if weights[bench][chk_num] == 0.0:
                                continue
                        
                        cap_stat_file_dir = './' + bench + '_' + type2 + '/m5out_sim_' + bench + '_' + type2 + "_" + str(chk_num+1) + '/stats.txt'
                        Super_stat_file_dir = './' + bench + '_' + type1 + '/m5out_sim_' + bench + '_' + type1 + '_' + str(chk_num+1) + '/stats.txt'
                        # print(cap_stat_file_dir)
                        # print(Super_stat_file_dir)

                        try:
                                cap_stat_file = open(cap_stat_file_dir, mode='r')
                                Super_stat_file = open(Super_stat_file_dir, mode='r')
                        except FileNotFoundError:
                                warnings.warn(bench + " simpoint " + str(chk_num+1) + " has no file!")
                                continue

                        cap_stat_file_content = cap_stat_file.read()
                        Super_stat_file_content = Super_stat_file.read()

                        cap_results = measure(results_index, cap_stat_file_content)
                        Super_results = measure(results_index, Super_stat_file_content)

                        if int(cap_results['system.switch_cpus.numCycles'][-1]) == 0 or int(Super_results['system.switch_cpus.numCycles'][-1]) == 0:
                                warnings.warn(bench + " simpoint " + str(chk_num+1) + " has no stat!")
                                cap_stat_file.close()
                                Super_stat_file.close()
                                continue
                        
                        total_weight = total_weight + weights[bench][chk_num]
                        print("SimOps Super: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", int(Super_results['system.switch_cpus.commit.commitSquashedInsts'][-1]))
                        print("SimOps Raw: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", int(cap_results['system.switch_cpus.commit.commitSquashedInsts'][-1]))
                        
                        
                        Raw_insts = Raw_insts + weights[bench][chk_num] * int(cap_results['system.switch_cpus.commit.commitSquashedInsts'][-1])
                        Super_insts = Super_insts + weights[bench][chk_num] * int(Super_results['system.switch_cpus.commit.commitSquashedInsts'][-1])
                        #print("SimOpsRaw: ", SimOpsOverhead)
                        #print('SimOpsSuper: ', total_weight) 
                        cap_stat_file.close()
                        Super_stat_file.close()

                        
                print("-------------------------------------------DONE----------------------------")
                if total_weight == 0:
                    final_results_performance["Super"].append(0)
                    final_results_performance["Raw"].append(0)
                    warnings.warn(bench + " Has no working simpoints!!")
                    continue

                final_results_performance["Super"].append((Raw_insts/total_weight))
                final_results_performance["Raw"].append(Super_insts/total_weight)




        df = pd.DataFrame(final_results_performance, columns = COLUMNS)
        # Setting the positions and width for the bars
        pos = list(range(len(df['Raw']))) 
        #print(pos)
        width = 0.25
            
        # Plotting the bars
        fig = plt.figure(figsize=(10,6))
        gs = gridspec.GridSpec(1, 1, height_ratios=[1]) 
        # print('AVG', final_results_performance['CHEx86ContextSensitive'])
        # print('GEOMEAN', 1 - gmean(final_results_performance['CHEx86ContextSensitive']))
        ax0 = plt.subplot(gs[0])
        patterns = [ "*" , "+" , "x" , "o" , "\\" , "/", "o", "O", ".", "*" ]
        #colors = palettable.colorbrewer.diverging.Spectral_4.mpl_colors
        colors = palettable.cartocolors.diverging.Earth_3.mpl_colors


        for i in range(0,NUM_OF_TYPES+1):
                ax0.bar([p + width*i for p in pos], 
                        df[x_axe[i]], 
                        #hatch=3*patterns[i],
                        # of width
                        width=width,
                        # with color
                        #edgecolor=colors[i], 
                        color=colors[i],
                        # with label the first value in benchmark
                        label=x_axe[i],
                        zorder=3) 
                ax0.bar([p + width*i for p in pos], 
                        df[x_axe[i]], 
                        # of width
                        width=width, 
                        # with color
                        color='none', 
                        edgecolor='black',
                        zorder=4) 


        # Set the position of the x ticks
        ax0.set_xticks([p + 2.5 * width for p in pos])

        # Set the labels for the x ticks
        ax0.set_xticklabels(df['benchmark'])
        ax0.tick_params(axis="x", labelsize=20)
        ax0.tick_params(axis="y", labelsize=15)
        # Setting the x-axis and y-axis limits
        #plt.xlim(min(pos)-width, max(pos)+width*4)
        #ax0.set(ylabel='$Normalized$ \n $Performance$')
        ax0.set_ylabel('$Squashed$ $Insts$', fontsize = 16.0)
        #ax0.ylim([0,1.1])
        plt.xticks(rotation=55)
        #plt.yscale('symlog', linthreshy=1)
        # Adding the legend and showing the plot

        for tick in ax0.xaxis.get_majorticklabels():
            tick.set_horizontalalignment("right")

        plt.legend(bbox_to_anchor=(0,1.02,1,0.2), loc="lower left", mode="expand", borderaxespad=0, edgecolor='black', ncol=2, prop={"size":16, "weight":'bold'})
        ax0.grid( linestyle='-.', linewidth=0.5, alpha=.5, axis='y', zorder=0)
        plt.tight_layout()

        if not os.path.exists(file_type):
                os.makedirs(file_type)
        plt.savefig(file_type + '/commit_squash_simple.png', bbox_inches='tight')
        plt.show()             
      
 
def get_commitSquashed_detailed_results():
        
        results_index_file = open('./results_index.txt', mode='r')
        contents = results_index_file.read()
        results_index = contents.split("\n")

        final_results_performance["Raw"] = []
        final_results_performance["Super"] = []

        #for bench in benchs:
        for bench in benchs:

                # stallC, stallI, IQFull, LSQFull, bandwidth
                Super_full = [0,0,0,0,0,0]
                Raw_full = [0,0,0,0,0,0]
                total_weight = 0.0

                for chk_num in range(len(benchs[bench])):

                        if weights[bench][chk_num] == 0.0:
                                continue
                        
                        cap_stat_file_dir = './' + bench + '_' + type2 + '/m5out_sim_' + bench + '_' + type2 + "_" + str(chk_num+1) + '/stats.txt'
                        Super_stat_file_dir = './' + bench + '_' + type1 + '/m5out_sim_' + bench + '_' + type1 + "_" + str(chk_num+1) + '/stats.txt'
                        # print(cap_stat_file_dir)
                        # print(Super_stat_file_dir)
                        cap_stat_file = open(cap_stat_file_dir, mode='r')
                        Super_stat_file = open(Super_stat_file_dir, mode='r')
                        cap_stat_file_content = cap_stat_file.read()
                        Super_stat_file_content = Super_stat_file.read()

                        cap_results = measure(results_index, cap_stat_file_content)
                        Super_results = measure(results_index, Super_stat_file_content)

                        #print(cap_results)
                        # print(Super_stat_file_dir)

                        Raw_total = int(cap_results['system.switch_cpus.commit.commitSquashedInsts'][-1])
                        Raw_spec_lvp = int(cap_results['system.switch_cpus.commit.commitSquashedInstFromSpecLVP'][-1])
                        Raw_spec_nolvp = int(cap_results['system.switch_cpus.commit.commitSquashedInstsFromSpecNotLVP'][-1])
                        Raw_uop_lvp = int(cap_results['system.switch_cpus.commit.commitSquashedInstFromUopLVP'][-1])
                        Raw_uop_nolvp = int(cap_results['system.switch_cpus.commit.commitSquashedInstFromUopNotLVP'][-1])
                        Raw_icache_lvp = int(cap_results['system.switch_cpus.commit.commitSquashedInstFromICacheLVP'][-1])
                        Raw_icache_nolvp = int(cap_results['system.switch_cpus.commit.commitSquashedInstFromICacheNotLVP'][-1])

                        Super_total = int(Super_results['system.switch_cpus.commit.commitSquashedInsts'][-1])
                        Super_spec_lvp = int(Super_results['system.switch_cpus.commit.commitSquashedInstFromSpecLVP'][-1])
                        Super_spec_nolvp = int(Super_results['system.switch_cpus.commit.commitSquashedInstsFromSpecNotLVP'][-1])
                        Super_uop_lvp = int(Super_results['system.switch_cpus.commit.commitSquashedInstFromUopLVP'][-1])
                        Super_uop_nolvp = int(Super_results['system.switch_cpus.commit.commitSquashedInstFromUopNotLVP'][-1])
                        Super_icache_lvp = int(Super_results['system.switch_cpus.commit.commitSquashedInstFromICacheLVP'][-1])
                        Super_icache_nolvp = int(Super_results['system.switch_cpus.commit.commitSquashedInstFromICacheNotLVP'][-1])

                        if Raw_total == 0 or Super_total == 0:
                                warnings.warn(bench + " simpoint " + str(chk_num+1) + " has no stat!")
                                cap_stat_file.close()
                                Super_stat_file.close()
                                continue
                        
                        total_weight = total_weight + weights[bench][chk_num]
                        #print("Raw: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", int(cap_results['system.switch_cpus.numCycles'][-1]))
                        
                        Super_full[0] += weights[bench][chk_num] * Super_spec_lvp
                        Super_full[1] += weights[bench][chk_num] * Super_spec_nolvp
                        Super_full[2] += weights[bench][chk_num] * Super_uop_lvp
                        Super_full[3] += weights[bench][chk_num] * Super_uop_nolvp
                        Super_full[4] += weights[bench][chk_num] * Super_icache_lvp
                        Super_full[5] += weights[bench][chk_num] * Super_icache_nolvp

                        Raw_full[0] += weights[bench][chk_num] * Raw_spec_lvp
                        Raw_full[1] += weights[bench][chk_num] * Raw_spec_nolvp
                        Raw_full[2] += weights[bench][chk_num] * Raw_uop_lvp
                        Raw_full[3] += weights[bench][chk_num] * Raw_uop_nolvp
                        Raw_full[4] += weights[bench][chk_num] * Raw_icache_lvp
                        Raw_full[5] += weights[bench][chk_num] * Raw_icache_nolvp

                        #PerformanceOverheadCap = PerformanceOverheadCap + weights[bench][chk_num] * ((int(cap_results['system.switch_cpus.numCycles'][-1]))/int(Super_results['system.switch_cpus.numCycles'][-1]))
                        #if int(Super_results['system.switch_cpus.rename.IQFullEvents'][-1]) != 0:
                        print("Super_full: ", Super_full)
                        print("Raw_full: ", Raw_full)
                        cap_stat_file.close()

                        

                print("-------------------------------------------DONE----------------------------")
                if total_weight == 0:
                    final_results_performance["Raw"].append([0,0,0,0,0,0])
                    final_results_performance["Super"].append([0,0,0,0,0,0])
                    warnings.warn(bench + " Has no working simpoints!!")
                    continue

                #print('PerformanceOverheadCap: ', PerformanceOverheadCap/total_weight)
                #print('PerformanceOverheadSuper: ', total_weight,total_weight)   

                #final_results_performance["Super"].append((PerformanceOverheadCap/total_weight))
                for i in range(6):
                        Super_full[i] /= total_weight
                        Raw_full[i] /= total_weight

                final_results_performance["Raw"].append(Super_full)
                final_results_performance["Super"].append(Raw_full)
                
        Raw = np.array(final_results_performance["Raw"]).T.tolist()
        RawOpt = np.array(final_results_performance["Super"]).T.tolist()

        df = pd.DataFrame(final_results_performance, columns = COLUMNS)
        
        print(df)
        # Setting the positions and width for the bars
        pos = list(range(len(df["benchmark"]))) 
        #print(pos)
        width = 0.25

            
        # Plotting the bars
        fig = plt.figure(figsize=(10,6))
        gs = gridspec.GridSpec(1, 1, height_ratios=[1]) 
        # print('AVG', final_results_performance['CHEx86ContextSensitive'])
        # print('GEOMEAN', 1 - gmean(final_results_performance['CHEx86ContextSensitive']))
        ax0 = plt.subplot(gs[0])
        patterns = [ "", "*" , "+" , "x" , "o" , "\\" , "/", "o", "O", ".", "*" ]
        #colors = palettable.colorbrewer.diverging.Spectral_4.mpl_colors
        colors = palettable.cartocolors.diverging.Earth_3.mpl_colors
        num_colors = 3

        bar1 = list(range(len(df["benchmark"]))) 
        bar2 = [i + (width*1) for i in bar1]

        ax0.bar(bar1, Raw[0], width, label=commit_detail_lables[0], color=colors[0], hatch=patterns[1])#, edgecolor=colors[0])
        bottom = Raw[0]
        for i in range(1, 6):
                ax0.bar(bar1, Raw[i], width, bottom=bottom, label=commit_detail_lables[i], color=colors[int(i/2)], hatch=patterns[int((i+1)%2)])#, edgecolor=colors[i])
                bottom = list(np.add(bottom, Raw[i]))

        ax0.bar(bar2, RawOpt[0], width, color=colors[0], hatch=patterns[1])#, edgecolor=colors[0])
        bottom = RawOpt[0]
        for i in range(1, 6):
                ax0.bar(bar2, RawOpt[i], width, bottom=bottom, color=colors[int(i / 2)], hatch=patterns[int((i+1)%2)])#, edgecolor=colors[i])
                bottom = list(np.add(bottom, RawOpt[i]))

        # Set the position of the x ticks
        ax0.set_xticks([p + 2.5 * width for p in bar1])

        # Set the labels for the x ticks
        ax0.set_xticklabels(df['benchmark'])
        ax0.tick_params(axis="x", labelsize=20)
        ax0.tick_params(axis="y", labelsize=15)
        # Setting the x-axis and y-axis limits
        #plt.xlim(min(pos)-width, max(pos)+width*4)
        #ax0.set(ylabel='$Normalized$ \n $Performance$')
        #ax0.set_ylabel('$Normalized$ \n $Execution$ $Time$', fontsize = 16.0)
        ax0.set_ylabel('Number of Instructions', fontsize = 16.0)
        #ax0.ylim([0,1.1])
        plt.xticks(rotation=55)
        #plt.yscale('symlog', linthreshy=1)
        plt.yscale('linear')
        # Adding the legend and showing the plot

        for tick in ax0.xaxis.get_majorticklabels():
            tick.set_horizontalalignment("right")

        plt.legend(bbox_to_anchor=(0,1.05,1,0.2), loc="lower left", mode="expand", borderaxespad=0, edgecolor='black', ncol=2, prop={"size":16, "weight":'bold'})
        ax0.grid( linestyle='-.', linewidth=0.5, alpha=.5, axis='y', zorder=0)
        plt.tight_layout()

        if not os.path.exists(file_type):
                os.makedirs(file_type)
        plt.savefig(file_type + '/CommitSquashedDetail.png', bbox_inches='tight')
        plt.show()
 
  
def get_branch_pred_results_percent():
        
        results_index_file = open('./results_index.txt', mode='r')
        contents = results_index_file.read()
        results_index = contents.split("\n")

        final_results_performance["Raw"] = []
        final_results_performance["Super"] = []

        idx = 0
        #for bench in benchs:
        for bench in benchs:

                TotalRaw = 0.0 #should be zero
                TotalSuper = 0.0
                total_weight = 0.0

                for chk_num in range(len(benchs[bench])):
                        
                        if weights[bench][chk_num] == 0.0:
                                continue
                        
                        Raw_stat_file_dir = './' + bench + '_' + type2 + '/m5out_sim_' + bench + '_' + type2 + "_" + str(chk_num+1) + '/stats.txt'
                        Super_stat_file_dir = './' + bench + '_' + type1 + '/m5out_sim_' + bench + '_' + type1 + '_' + str(chk_num+1) + '/stats.txt'
                        # print(cap_stat_file_dir)
                        # print(Super_stat_file_dir)

                        Raw_stat_file = open(Raw_stat_file_dir, mode='r')
                        Super_stat_file = open(Super_stat_file_dir, mode='r')

                        Raw_stat_file_content = Raw_stat_file.read()
                        Super_stat_file_content = Super_stat_file.read()

                        Raw_results = measure(results_index, Raw_stat_file_content)
                        Super_results = measure(results_index, Super_stat_file_content)

                        Super_cycles = int(Super_results['system.switch_cpus.numCycles'][-1])
                        Raw_cycles = int(Raw_results['system.switch_cpus.numCycles'][-1])


                        if Raw_cycles == 0 or Super_cycles == 0:
                                warnings.warn(bench + " simpoint " + str(chk_num+1) + " has not stat!")
                                Raw_stat_file.close()
                                Super_stat_file.close()
                                continue

                        Super_branches = int(Super_results['system.switch_cpus.iew.exec_branches'][-1])
                        Raw_branches = int(Raw_results['system.switch_cpus.iew.exec_branches'][-1])

                        Super_incorrect = int(Super_results['system.switch_cpus.iew.predictedNotTakenIncorrect'][-1])
                        Raw_incorrect = int(Raw_results['system.switch_cpus.iew.predictedNotTakenIncorrect'][-1])

                        Super_incorrect += int(Super_results['system.switch_cpus.iew.predictedTakenIncorrect'][-1])
                        Raw_incorrect += int(Raw_results['system.switch_cpus.iew.predictedTakenIncorrect'][-1])

                        total_weight = total_weight + weights[bench][chk_num]
                        print("Branches Super: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", Super_branches)
                        print("Branches Raw: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", Raw_branches)
                        
                        
                        TotalRaw = TotalRaw + weights[bench][chk_num] * (Raw_incorrect/Raw_branches)
                        TotalSuper = TotalSuper + weights[bench][chk_num] * (Super_incorrect/Super_branches)
                        #print("SimOpsRaw: ", SimOpsOverhead)
                        #print('SimOpsSuper: ', total_weight) 
                        Raw_stat_file.close()
                        Super_stat_file.close()


                        
                print("-------------------------------------------DONE----------------------------")
                if total_weight == 0:
                    final_results_performance["Raw"].append(0)
                    final_results_performance["Super"].append(0)
                    warnings.warn(bench + " Has no working simpoints!!")
                    continue
                
                print('TotalRaw: ', TotalRaw/total_weight)
                print('TotalSuper: ', TotalSuper/total_weight)   

                final_results_performance["Raw"].append(TotalSuper/total_weight)
                final_results_performance["Super"].append(TotalRaw/total_weight)




        df = pd.DataFrame(final_results_performance, columns = COLUMNS)
        # Setting the positions and width for the bars
        pos = list(range(len(df['Raw']))) 
        #print(pos)
        width = 0.25
            
        # Plotting the bars
        fig = plt.figure(figsize=(10,6))
        gs = gridspec.GridSpec(1, 1, height_ratios=[1]) 
        # print('AVG', final_results_performance['CHEx86ContextSensitive'])
        # print('GEOMEAN', 1 - gmean(final_results_performance['CHEx86ContextSensitive']))
        ax0 = plt.subplot(gs[0])
        patterns = [ "*" , "+" , "x" , "o" , "\\" , "/", "o", "O", ".", "*" ]
        #colors = palettable.colorbrewer.diverging.Spectral_4.mpl_colors
        colors = palettable.cartocolors.diverging.Earth_7.mpl_colors


        for i in range(0,NUM_OF_TYPES+1):
                ax0.bar([p + width*i for p in pos], 
                        df[x_axe[i]], 
                        #hatch=3*patterns[i],
                        # of width
                        width=width,
                        # with color
                        #edgecolor=colors[i], 
                        color=colors[i],
                        # with label the first value in benchmark
                        label=x_axe[i],
                        zorder=3) 
                ax0.bar([p + width*i for p in pos], 
                        df[x_axe[i]], 
                        # of width
                        width=width, 
                        # with color
                        color='none', 
                        edgecolor='black',
                        zorder=4) 


        # Set the position of the x ticks
        ax0.set_xticks([p + 2.5 * width for p in pos])

        # Set the labels for the x ticks
        ax0.set_xticklabels(df['benchmark'])
        ax0.tick_params(axis="x", labelsize=20)
        ax0.tick_params(axis="y", labelsize=15)
        # Setting the x-axis and y-axis limits
        #plt.xlim(min(pos)-width, max(pos)+width*4)
        #ax0.set(ylabel='$Normalized$ \n $Performance$')
        ax0.set_ylabel('$Percentage$ $Incorrect$', fontsize = 16.0)
        #ax0.ylim([0,1.1])
        plt.xticks(rotation=55)
        #plt.yscale('symlog', linthreshy=1)
        # Adding the legend and showing the plot

        for tick in ax0.xaxis.get_majorticklabels():
            tick.set_horizontalalignment("right")

        plt.legend(bbox_to_anchor=(0,1.02,1,0.2), loc="lower left", mode="expand", borderaxespad=0, edgecolor='black', ncol=2, prop={"size":16, "weight":'bold'})
        ax0.grid( linestyle='-.', linewidth=0.5, alpha=.5, axis='y', zorder=0)
        plt.tight_layout()

        if not os.path.exists(file_type):
                os.makedirs(file_type)
        plt.savefig(file_type + '/percent_branches_incorrect.png', bbox_inches='tight')
        plt.show()        


        
def get_branch_pred_results_num():
        
        results_index_file = open('./results_index.txt', mode='r')
        contents = results_index_file.read()
        results_index = contents.split("\n")

        final_results_performance["Raw"] = []
        final_results_performance["Super"] = []

        idx = 0
        #for bench in benchs:
        for bench in benchs:

                TotalRaw = 0.0 #should be zero
                TotalSuper = 0.0
                total_weight = 0.0

                for chk_num in range(len(benchs[bench])):
                        
                        if weights[bench][chk_num] == 0.0:
                                continue
                        
                        Raw_stat_file_dir = './' + bench + '_' + type2 + '/m5out_sim_' + bench + '_' + type2 + "_" + str(chk_num+1) + '/stats.txt'
                        Super_stat_file_dir = './' + bench + '_' + type1 + '/m5out_sim_' + bench + '_' + type1 + '_' + str(chk_num+1) + '/stats.txt'
                        # print(cap_stat_file_dir)
                        # print(Super_stat_file_dir)

                        Raw_stat_file = open(Raw_stat_file_dir, mode='r')
                        Super_stat_file = open(Super_stat_file_dir, mode='r')

                        Raw_stat_file_content = Raw_stat_file.read()
                        Super_stat_file_content = Super_stat_file.read()

                        Raw_results = measure(results_index, Raw_stat_file_content)
                        Super_results = measure(results_index, Super_stat_file_content)

                        Super_cycles = int(Super_results['system.switch_cpus.numCycles'][-1])
                        Raw_cycles = int(Raw_results['system.switch_cpus.numCycles'][-1])


                        if Raw_cycles == 0 or Super_cycles == 0:
                                warnings.warn(bench + " simpoint " + str(chk_num+1) + " has not stat!")
                                Raw_stat_file.close()
                                Super_stat_file.close()
                                continue

                        Super_branches = int(Super_results['system.switch_cpus.iew.exec_branches'][-1])
                        Raw_branches = int(Raw_results['system.switch_cpus.iew.exec_branches'][-1])

                        Super_incorrect = int(Super_results['system.switch_cpus.iew.predictedNotTakenIncorrect'][-1])
                        Raw_incorrect = int(Raw_results['system.switch_cpus.iew.predictedNotTakenIncorrect'][-1])

                        Super_incorrect += int(Super_results['system.switch_cpus.iew.predictedTakenIncorrect'][-1])
                        Raw_incorrect += int(Raw_results['system.switch_cpus.iew.predictedTakenIncorrect'][-1])

                        total_weight = total_weight + weights[bench][chk_num]
                        print("Branches Super: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", Super_branches)
                        print("Branches Raw: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", Raw_branches)
                        
                        
                        TotalRaw = TotalRaw + weights[bench][chk_num] * Raw_incorrect
                        TotalSuper = TotalSuper + weights[bench][chk_num] * Super_incorrect
                        #print("SimOpsRaw: ", SimOpsOverhead)
                        #print('SimOpsSuper: ', total_weight) 
                        Raw_stat_file.close()
                        Super_stat_file.close()


                        
                print("-------------------------------------------DONE----------------------------")
                if total_weight == 0:
                    final_results_performance["Raw"].append(0)
                    final_results_performance["Super"].append(0)
                    warnings.warn(bench + " Has no working simpoints!!")
                    continue
                
                print('TotalRaw: ', TotalRaw/total_weight)
                print('TotalSuper: ', TotalSuper/total_weight)   

                final_results_performance["Raw"].append(TotalSuper/total_weight)
                final_results_performance["Super"].append(TotalRaw/total_weight)




        df = pd.DataFrame(final_results_performance, columns = COLUMNS)
        # Setting the positions and width for the bars
        pos = list(range(len(df['Raw']))) 
        #print(pos)
        width = 0.25
            
        # Plotting the bars
        fig = plt.figure(figsize=(10,6))
        gs = gridspec.GridSpec(1, 1, height_ratios=[1]) 
        # print('AVG', final_results_performance['CHEx86ContextSensitive'])
        # print('GEOMEAN', 1 - gmean(final_results_performance['CHEx86ContextSensitive']))
        ax0 = plt.subplot(gs[0])
        patterns = [ "*" , "+" , "x" , "o" , "\\" , "/", "o", "O", ".", "*" ]
        #colors = palettable.colorbrewer.diverging.Spectral_4.mpl_colors
        colors = palettable.cartocolors.diverging.Earth_7.mpl_colors


        for i in range(0,NUM_OF_TYPES+1):
                ax0.bar([p + width*i for p in pos], 
                        df[x_axe[i]], 
                        #hatch=3*patterns[i],
                        # of width
                        width=width,
                        # with color
                        #edgecolor=colors[i], 
                        color=colors[i],
                        # with label the first value in benchmark
                        label=x_axe[i],
                        zorder=3) 
                ax0.bar([p + width*i for p in pos], 
                        df[x_axe[i]], 
                        # of width
                        width=width, 
                        # with color
                        color='none', 
                        edgecolor='black',
                        zorder=4) 


        # Set the position of the x ticks
        ax0.set_xticks([p + 2.5 * width for p in pos])

        # Set the labels for the x ticks
        ax0.set_xticklabels(df['benchmark'])
        ax0.tick_params(axis="x", labelsize=20)
        ax0.tick_params(axis="y", labelsize=15)
        # Setting the x-axis and y-axis limits
        #plt.xlim(min(pos)-width, max(pos)+width*4)
        #ax0.set(ylabel='$Normalized$ \n $Performance$')
        ax0.set_ylabel('$Number$ $Incorrect$', fontsize = 16.0)
        #ax0.ylim([0,1.1])
        plt.xticks(rotation=55)
        #plt.yscale('symlog', linthreshy=1)
        # Adding the legend and showing the plot

        for tick in ax0.xaxis.get_majorticklabels():
            tick.set_horizontalalignment("right")

        plt.legend(bbox_to_anchor=(0,1.02,1,0.2), loc="lower left", mode="expand", borderaxespad=0, edgecolor='black', ncol=2, prop={"size":16, "weight":'bold'})
        ax0.grid( linestyle='-.', linewidth=0.5, alpha=.5, axis='y', zorder=0)
        plt.tight_layout()

        if not os.path.exists(file_type):
                os.makedirs(file_type)
        plt.savefig(file_type + '/total_branches_incorrect.png', bbox_inches='tight')
        plt.show()             

       
def get_branch_results():
        
        results_index_file = open('./results_index.txt', mode='r')
        contents = results_index_file.read()
        results_index = contents.split("\n")

        final_results_performance["Raw"] = []
        final_results_performance["Super"] = []

        idx = 0
        #for bench in benchs:
        for bench in benchs:

                TotalRaw = 0.0 #should be zero
                TotalSuper = 0.0
                total_weight = 0.0

                for chk_num in range(len(benchs[bench])):
                        
                        if weights[bench][chk_num] == 0.0:
                                continue
                        
                        Raw_stat_file_dir = './' + bench + '_' + type2 + '/m5out_sim_' + bench + '_' + type2 + "_" + str(chk_num+1) + '/stats.txt'
                        Super_stat_file_dir = './' + bench + '_' + type1 + '/m5out_sim_' + bench + '_' + type1 + '_' + str(chk_num+1) + '/stats.txt'
                        # print(cap_stat_file_dir)
                        # print(Super_stat_file_dir)

                        Raw_stat_file = open(Raw_stat_file_dir, mode='r')
                        Super_stat_file = open(Super_stat_file_dir, mode='r')

                        Raw_stat_file_content = Raw_stat_file.read()
                        Super_stat_file_content = Super_stat_file.read()

                        Raw_results = measure(results_index, Raw_stat_file_content)
                        Super_results = measure(results_index, Super_stat_file_content)

                        Super_cycles = int(Super_results['system.switch_cpus.numCycles'][-1])
                        Raw_cycles = int(Raw_results['system.switch_cpus.numCycles'][-1])


                        if Raw_cycles == 0 or Super_cycles == 0:
                                warnings.warn(bench + " simpoint " + str(chk_num+1) + " has not stat!")
                                Raw_stat_file.close()
                                Super_stat_file.close()
                                continue

                        Super_branches = int(Super_results['system.switch_cpus.iew.exec_branches'][-1])
                        Raw_branches = int(Raw_results['system.switch_cpus.iew.exec_branches'][-1])

                        total_weight = total_weight + weights[bench][chk_num]
                        print("Branches Super: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", Super_branches)
                        print("Branches Raw: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", Raw_branches)
                        
                        
                        TotalRaw = TotalRaw + weights[bench][chk_num] * Raw_branches
                        TotalSuper = TotalSuper + weights[bench][chk_num] * Super_branches
                        #print("SimOpsRaw: ", SimOpsOverhead)
                        #print('SimOpsSuper: ', total_weight) 
                        Raw_stat_file.close()
                        Super_stat_file.close()


                        
                print("-------------------------------------------DONE----------------------------")
                if total_weight == 0:
                    final_results_performance["Raw"].append(0)
                    final_results_performance["Super"].append(0)
                    warnings.warn(bench + " Has no working simpoints!!")
                    continue
                
                print('TotalRaw: ', TotalRaw/total_weight)
                print('TotalSuper: ', TotalSuper/total_weight)   

                final_results_performance["Raw"].append(TotalSuper/total_weight)
                final_results_performance["Super"].append(TotalRaw/total_weight)




        df = pd.DataFrame(final_results_performance, columns = COLUMNS)
        # Setting the positions and width for the bars
        pos = list(range(len(df['Raw']))) 
        #print(pos)
        width = 0.25
            
        # Plotting the bars
        fig = plt.figure(figsize=(10,6))
        gs = gridspec.GridSpec(1, 1, height_ratios=[1]) 
        # print('AVG', final_results_performance['CHEx86ContextSensitive'])
        # print('GEOMEAN', 1 - gmean(final_results_performance['CHEx86ContextSensitive']))
        ax0 = plt.subplot(gs[0])
        patterns = [ "*" , "+" , "x" , "o" , "\\" , "/", "o", "O", ".", "*" ]
        #colors = palettable.colorbrewer.diverging.Spectral_4.mpl_colors
        colors = palettable.cartocolors.diverging.Earth_7.mpl_colors


        for i in range(0,NUM_OF_TYPES+1):
                ax0.bar([p + width*i for p in pos], 
                        df[x_axe[i]], 
                        #hatch=3*patterns[i],
                        # of width
                        width=width,
                        # with color
                        #edgecolor=colors[i], 
                        color=colors[i],
                        # with label the first value in benchmark
                        label=x_axe[i],
                        zorder=3) 
                ax0.bar([p + width*i for p in pos], 
                        df[x_axe[i]], 
                        # of width
                        width=width, 
                        # with color
                        color='none', 
                        edgecolor='black',
                        zorder=4) 


        # Set the position of the x ticks
        ax0.set_xticks([p + 2.5 * width for p in pos])

        # Set the labels for the x ticks
        ax0.set_xticklabels(df['benchmark'])
        ax0.tick_params(axis="x", labelsize=20)
        ax0.tick_params(axis="y", labelsize=15)
        # Setting the x-axis and y-axis limits
        #plt.xlim(min(pos)-width, max(pos)+width*4)
        #ax0.set(ylabel='$Normalized$ \n $Performance$')
        ax0.set_ylabel('$Executed$ $Branches$', fontsize = 16.0)
        #ax0.ylim([0,1.1])
        plt.xticks(rotation=55)
        #plt.yscale('symlog', linthreshy=1)
        # Adding the legend and showing the plot

        for tick in ax0.xaxis.get_majorticklabels():
            tick.set_horizontalalignment("right")

        plt.legend(bbox_to_anchor=(0,1.02,1,0.2), loc="lower left", mode="expand", borderaxespad=0, edgecolor='black', ncol=2, prop={"size":16, "weight":'bold'})
        ax0.grid( linestyle='-.', linewidth=0.5, alpha=.5, axis='y', zorder=0)
        plt.tight_layout()

        if not os.path.exists(file_type):
                os.makedirs(file_type)
        plt.savefig(file_type + '/total_branches.png', bbox_inches='tight')
        plt.show()             
  
 

def get_branch_pred_detailed_results():
        
        results_index_file = open('./results_index.txt', mode='r')
        contents = results_index_file.read()
        results_index = contents.split("\n")

        final_results_branch_detail["Taken"] = []
        final_results_branch_detail["NotTaken"] = []
        final_results_branch_detail["NotTaken"] = []

        #for bench in benchs:
        for bench in benchs:

                # stallC, stallI, IQFull, LSQFull, bandwidth
                Taken_full = [0,0,0,0]
                NotTaken_full = [0,0,0,0]
                Indirect_full = [0,0,0,0]
                total_weight = 0.0

                for chk_num in range(len(benchs[bench])):

                        if weights[bench][chk_num] == 0.0:
                                continue
                        
                        cap_stat_file_dir = './' + bench + '_' + type2 + '/m5out_sim_' + bench + '_' + type2 + "_" + str(chk_num+1) + '/stats.txt'
                        #Super_stat_file_dir = './' + bench + '_' + type1 + '/m5out_sim_' + bench + '_' + type1 + "_" + str(chk_num+1) + '/stats.txt'
                        # print(cap_stat_file_dir)
                        # print(Super_stat_file_dir)
                        cap_stat_file = open(cap_stat_file_dir, mode='r')
                        #Super_stat_file = open(Super_stat_file_dir, mode='r')
                        cap_stat_file_content = cap_stat_file.read()
                        #Super_stat_file_content = Super_stat_file.read()

                        cap_results = measure(results_index, cap_stat_file_content)
                        #Super_results = measure(results_index, Super_stat_file_content)

                        #print(cap_results)
                        # print(Super_stat_file_dir)

                        Raw_total = int(cap_results['system.switch_cpus.numCycles'][-1])

                        taken_incorrect_bpred = int(cap_results['system.switch_cpus.iew.specTracePredTakenIncorrectFromBranchPred'][-1])
                        taken_incorrect_trace = int(cap_results['system.switch_cpus.iew.specTracePredTakenIncorrectFromTrace'][-1])
                        taken_incorrect_uop = int(cap_results['system.switch_cpus.iew.specTracePredTakenIncorrectFromUopCache'][-1])
                        taken_incorrect_icache = int(cap_results['system.switch_cpus.iew.specTracePredTakenIncorrectFromICache'][-1])

                        notTaken_incorrect_bpred = int(cap_results['system.switch_cpus.iew.specTracePredNotTakenIncorrectFromBranchPred'][-1])
                        notTaken_incorrect_trace = int(cap_results['system.switch_cpus.iew.specTracePredNotTakenIncorrectFromTrace'][-1])
                        notTaken_incorrect_uop = int(cap_results['system.switch_cpus.iew.specTracePredNotTakenIncorrectFromUopCache'][-1])
                        notTaken_incorrect_icache = int(cap_results['system.switch_cpus.iew.specTracePredNotTakenIncorrectFromICache'][-1])

                        indirect_incorrect_bpred = int(cap_results['system.switch_cpus.iew.specTracePredIndirectIncorrectFromBranchPred'][-1])
                        indirect_incorrect_trace = int(cap_results['system.switch_cpus.iew.specTracePredIndirectIncorrectFromTrace'][-1])
                        indirect_incorrect_uop = int(cap_results['system.switch_cpus.iew.specTracePredIndirectIncorrectFromUopCache'][-1])
                        indirect_incorrect_icache = int(cap_results['system.switch_cpus.iew.specTracePredIndirectIncorrectFromICache'][-1])


                        if Raw_total == 0:
                                warnings.warn(bench + " simpoint " + str(chk_num+1) + " has no stat!")
                                cap_stat_file.close()
                                #Super_stat_file.close()
                                continue
                        
                        total_weight = total_weight + weights[bench][chk_num]
                        #print("Raw: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", int(cap_results['system.switch_cpus.numCycles'][-1]))
                        
                        Taken_full[0] += weights[bench][chk_num] * taken_incorrect_bpred
                        Taken_full[1] += weights[bench][chk_num] * taken_incorrect_trace
                        Taken_full[2] += weights[bench][chk_num] * taken_incorrect_uop
                        Taken_full[3] += weights[bench][chk_num] * taken_incorrect_icache

                        NotTaken_full[0] += weights[bench][chk_num] * notTaken_incorrect_bpred
                        NotTaken_full[1] += weights[bench][chk_num] * notTaken_incorrect_trace
                        NotTaken_full[2] += weights[bench][chk_num] * notTaken_incorrect_uop
                        NotTaken_full[3] += weights[bench][chk_num] * notTaken_incorrect_icache

                        Indirect_full[0] += weights[bench][chk_num] * indirect_incorrect_bpred
                        Indirect_full[1] += weights[bench][chk_num] * indirect_incorrect_trace
                        Indirect_full[2] += weights[bench][chk_num] * indirect_incorrect_uop
                        Indirect_full[3] += weights[bench][chk_num] * indirect_incorrect_icache

                        #PerformanceOverheadCap = PerformanceOverheadCap + weights[bench][chk_num] * ((int(cap_results['system.switch_cpus.numCycles'][-1]))/int(Super_results['system.switch_cpus.numCycles'][-1]))
                        #if int(Super_results['system.switch_cpus.rename.IQFullEvents'][-1]) != 0:
                        print("Taken_full: ", Taken_full)
                        print("NotTaken_full: ", NotTaken_full)
                        print("Indirect_full: ", Indirect_full)
                        print("Unknown: ", int(cap_results['system.switch_cpus.iew.predictedUnknownBranchIncorrect'][-1]))
                        cap_stat_file.close()

                        

                print("-------------------------------------------DONE----------------------------")
                if total_weight == 0:
                    final_results_branch_detail["Taken"].append([0,0,0,0])
                    final_results_branch_detail["NotTaken"].append([0,0,0,0])
                    final_results_branch_detail["Indirect"].append([0,0,0,0])
                    warnings.warn(bench + " Has no working simpoints!!")
                    continue

                #print('PerformanceOverheadCap: ', PerformanceOverheadCap/total_weight)
                #print('PerformanceOverheadSuper: ', total_weight,total_weight)   

                #final_results_branch_detail["NotTaken"].append((PerformanceOverheadCap/total_weight))
                for i in range(4):
                        Taken_full[i] /= total_weight
                        NotTaken_full[i] /= total_weight
                        Indirect_full[i] /= total_weight

                final_results_branch_detail["Taken"].append(Taken_full)
                final_results_branch_detail["NotTaken"].append(NotTaken_full)
                final_results_branch_detail["Indirect"].append(Indirect_full)
                
        Taken = np.array(final_results_branch_detail["Taken"]).T.tolist()
        NotTaken = np.array(final_results_branch_detail["NotTaken"]).T.tolist()
        Indirect = np.array(final_results_branch_detail["Indirect"]).T.tolist()

        df = pd.DataFrame(final_results_branch_detail, columns = COLUMNS)
        
        print(df)
        # Setting the positions and width for the bars
        pos = list(range(len(df["benchmark"]))) 
        #print(pos)
        width = 0.25

            
        # Plotting the bars
        fig = plt.figure(figsize=(10,6))
        gs = gridspec.GridSpec(1, 1, height_ratios=[1]) 
        # print('AVG', final_results_branch_detail['CHEx86ContextSensitive'])
        # print('GEOMEAN', 1 - gmean(final_results_branch_detail['CHEx86ContextSensitive']))
        ax0 = plt.subplot(gs[0])
        patterns = [ "", "*" , "+" , "x" , "o" , "\\" , "/", "o", "O", ".", "*" ]
        #colors = palettable.colorbrewer.diverging.Spectral_4.mpl_colors
        colors = palettable.cartocolors.diverging.Earth_4.mpl_colors
        num_colors = 4

        bar1 = list(range(len(df["benchmark"]))) 
        bar2 = [i + (width*1) for i in bar1]
        bar3 = [i + (width*1) for i in bar2]

        ax0.bar(bar1, Taken[0], width, label=branch_detail_lables[0], color=colors[0])#, edgecolor=colors[0])
        bottom = Taken[0]
        for i in range(1, 4):
                ax0.bar(bar1, Taken[i], width, bottom=bottom, label=branch_detail_lables[i], color=colors[int(i)]) #, edgecolor=colors[i])
                bottom = list(np.add(bottom, Taken[i]))

        ax0.bar(bar2, NotTaken[0], width, color=colors[0])#, edgecolor=colors[0])
        bottom = NotTaken[0]
        for i in range(1, 4):
                ax0.bar(bar2, NotTaken[i], width, bottom=bottom, color=colors[int(i)])#, edgecolor=colors[i])
                bottom = list(np.add(bottom, NotTaken[i]))

        ax0.bar(bar3, Indirect[0], width, color=colors[0])#, edgecolor=colors[0])
        bottom = Indirect[0]
        for i in range(1, 4):
                ax0.bar(bar3, Indirect[i], width, bottom=bottom, color=colors[int(i)])#, edgecolor=colors[i])
                bottom = list(np.add(bottom, Indirect[i]))

        # Set the position of the x ticks
        ax0.set_xticks([p + 2.5 * width for p in bar1])

        # Set the labels for the x ticks
        ax0.set_xticklabels(df['benchmark'])
        ax0.tick_params(axis="x", labelsize=20)
        ax0.tick_params(axis="y", labelsize=15)
        # Setting the x-axis and y-axis limits
        #plt.xlim(min(pos)-width, max(pos)+width*4)
        #ax0.set(ylabel='$Normalized$ \n $Performance$')
        #ax0.set_ylabel('$Normalized$ \n $Execution$ $Time$', fontsize = 16.0)
        ax0.set_ylabel('Number of Branches', fontsize = 16.0)
        #ax0.ylim([0,1.1])
        plt.xticks(rotation=55)
        #plt.yscale('symlog', linthreshy=1)
        plt.yscale('linear')
        # Adding the legend and showing the plot

        for tick in ax0.xaxis.get_majorticklabels():
            tick.set_horizontalalignment("right")

        plt.legend(bbox_to_anchor=(0,1.05,1,0.2), loc="lower left", mode="expand", borderaxespad=0, edgecolor='black', ncol=2, prop={"size":16, "weight":'bold'})
        ax0.grid( linestyle='-.', linewidth=0.5, alpha=.5, axis='y', zorder=0)
        plt.tight_layout()

        if not os.path.exists(file_type):
                os.makedirs(file_type)
        plt.savefig(file_type + '/BranchDetail.png', bbox_inches='tight')
        plt.show()



def get_power_results():
        
        results_index_file = open('./results_index.txt', mode='r')
        contents = results_index_file.read()
        results_index = contents.split("\n")

        power_file = open('../check_power_spec.out', mode='r')
        power_content = power_file.read()
        power_results_lines = power_content.split('\n')
        #print(power_results_lines)
        power_results_split = []
        for line in power_results_lines:
                if (line != ""):
                        power_results_split.append([float(j) for j in line.split()])

        #assert(len(power_results_split) == 72)
        assert(len(power_results_split) == 64)
        
        power_results  = {                     
                'perl': [power_results_split[0],power_results_split[1],power_results_split[2],power_results_split[3],power_results_split[4],power_results_split[5],power_results_split[6]],
                'exchange': [power_results_split[20],power_results_split[21],power_results_split[22],power_results_split[23],power_results_split[24]],
                'xalan': [power_results_split[14],power_results_split[15],power_results_split[16],power_results_split[17],power_results_split[18],power_results_split[19]],
                'gcc': [power_results_split[57],power_results_split[58],power_results_split[59],power_results_split[60],power_results_split[61],power_results_split[62],power_results_split[63]],
                'leela': [power_results_split[49],power_results_split[50],power_results_split[51],power_results_split[52],power_results_split[53],power_results_split[54],power_results_split[55],power_results_split[56]],
                'wrf': [power_results_split[43],power_results_split[44],power_results_split[45],power_results_split[46],power_results_split[47],power_results_split[48]],
                'deepsjeng': [power_results_split[25],power_results_split[26],power_results_split[27]],
                'nab': [power_results_split[39],power_results_split[40],power_results_split[41],power_results_split[42]],
                'mcf': [power_results_split[7],power_results_split[8],power_results_split[9],power_results_split[10],power_results_split[11],power_results_split[12],power_results_split[13]],
                'xz': [power_results_split[28],power_results_split[29],power_results_split[30],power_results_split[31],power_results_split[32],power_results_split[33]],
                'lbm': [power_results_split[34],power_results_split[35],power_results_split[36],power_results_split[37],power_results_split[38]]#,
                #'vips': [power_results_split[64]], 
                #'x264': [power_results_split[66]], 
                #'freqmine': [power_results_split[65]], 
                #'swaptions': [power_results_split[67]], 
                #'dedup': [power_results_split[68]], 
                #'blackscholes': [power_results_split[69]], 
                #'ferret': [power_results_split[70]],
                #'streamcluster': [power_results_split[71]]
                }

        final_results_performance["Raw"] = []
        final_results_performance["Super"] = []

        #for bench in benchs:
        SpecSuperTotal = 0.0
        SpecRawTotal = 0.0
        NumOfSpec = 0
        ParsecSuperTotal = 0.0
        ParsecRawTotal = 0.0
        NumOfParsec = 0
        for i, bench in enumerate(benchs):

                TotalPowerRaw = 0.0 #should be zero
                TotalPowerSuper = 0.0
                total_weight = 0.0

                for chk_num in range(len(benchs[bench])):

                        if weights[bench][chk_num] == 0.0:
                                continue
                        
                        cap_stat_file_dir = './' + bench + '_' + type2 + '/m5out_sim_' + bench + '_' + type2 + "_" + str(chk_num+1) + '/stats.txt'
                        Super_stat_file_dir = './' + bench + '_' + type1 + '/m5out_sim_' + bench + '_' + type1 + '_' + str(chk_num+1) + '/stats.txt'
                        # print(cap_stat_file_dir)
                        # print(Super_stat_file_dir)
                        cap_stat_file = open(cap_stat_file_dir, mode='r')
                        Super_stat_file = open(Super_stat_file_dir, mode='r')
                        cap_stat_file_content = cap_stat_file.read()
                        Super_stat_file_content = Super_stat_file.read()

                        cap_results = measure(results_index, cap_stat_file_content)
                        Super_results = measure(results_index, Super_stat_file_content)
                        simpoint_power_results = power_results[bench][chk_num]
                        #print (bench, simpoint_power_results)

                        Raw_power = simpoint_power_results[0]
                        Raw_intRF_power = simpoint_power_results[1]
                        Raw_rename_power = simpoint_power_results[2]

                        Super_power = simpoint_power_results[3]
                        Super_intRF_power = simpoint_power_results[4]
                        Super_rename_power = simpoint_power_results[5]

                        if Raw_intRF_power > Super_intRF_power:
                                Raw_power -= (Raw_intRF_power - Super_intRF_power)
                        if Raw_rename_power > Super_rename_power:
                                Raw_power -= (Raw_rename_power - Super_rename_power)

                        
                        if i <= 10:
                            if int(cap_results['system.switch_cpus.numCycles'][-1]) == 0 or int(Super_results['system.switch_cpus.numCycles'][-1]) == 0:
                                warnings.warn(bench + " simpoint " + str(chk_num+1) + " has not stat!")
                                cap_stat_file.close()
                                Super_stat_file.close()
                                continue
                        else:
                            if int(cap_results['system.cpu.numCycles'][-1]) == 0 or int(Super_results['system.cpu.numCycles'][-1]) == 0:
                                warnings.warn(bench + " simpoint " + str(chk_num+1) + " has not stat!")
                                cap_stat_file.close()
                                Super_stat_file.close()
                                continue
                        
                        if i <= 10:
                            Raw_power *= int(cap_results['system.switch_cpus.numCycles'][-1])
                            Super_power *= int(Super_results['system.switch_cpus.numCycles'][-1])
                        else:
                            Raw_power *= int(cap_results['system.cpu.numCycles'][-1])
                            Super_power *= int(Super_results['system.cpu.numCycles'][-1])
                                                    
                            
                        total_weight = total_weight + weights[bench][chk_num]

                        print("Super: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", Super_power)
                        print("Raw: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", Raw_power)
                        
                        
                        
                        TotalPowerRaw = TotalPowerRaw + weights[bench][chk_num] * Raw_power
                        TotalPowerSuper = TotalPowerSuper + weights[bench][chk_num] * Super_power
                        print("TotalPowerRaw: ", TotalPowerRaw)
                        print('TotalPowerSuper: ', TotalPowerSuper) 
                        cap_stat_file.close()
                        Super_stat_file.close()

                        

                print("-------------------------------------------DONE----------------------------")
                if total_weight == 0:
                    final_results_performance["Super"].append(0)
                    final_results_performance["Raw"].append(0)
                    warnings.warn(bench + " Has no working simpoints!!")
                    continue
                
                TotalPowerRaw /= total_weight
                TotalPowerSuper /= total_weight
                #NormPowerRaw = (TotalPowerRaw - TotalPowerSuper) / TotalPowerSuper
                NormPowerRaw = (TotalPowerRaw) / TotalPowerSuper
                print('NormPowerRaw: ', NormPowerRaw)
                print('TotalPowerSuper: ', total_weight/total_weight)

                final_results_performance["Super"].append((NormPowerRaw))
                final_results_performance["Raw"].append(total_weight/total_weight)

                if i <= 10:
                    SpecSuperTotal += NormPowerRaw
                    SpecRawTotal += total_weight/total_weight
                    NumOfSpec += 1
                else:
                    ParsecSuperTotal += NormPowerRaw
                    ParsecRawTotal += total_weight/total_weight
                    NumOfParsec += 1
                
        final_results_performance["Super"].append(0)
        final_results_performance["Raw"].append(0)
        final_results_performance["Super"].append(SpecSuperTotal/NumOfSpec)
        final_results_performance["Raw"].append(SpecRawTotal/NumOfSpec)
        final_results_performance["Super"].append(ParsecSuperTotal/NumOfParsec)
        final_results_performance["Raw"].append(ParsecRawTotal/NumOfParsec)

        df = pd.DataFrame(final_results_performance, columns = COLUMNS)
        # Setting the positions and width for the bars
        pos = list(range(len(df['Raw']))) 
        #print(pos)
        width = 0.25
            
        # Plotting the bars
        fig = plt.figure(figsize=(10,6))
        gs = gridspec.GridSpec(1, 1, height_ratios=[1]) 
        # print('AVG', final_results_performance['CHEx86ContextSensitive'])
        # print('GEOMEAN', 1 - gmean(final_results_performance['CHEx86ContextSensitive']))
        ax0 = plt.subplot(gs[0])
        patterns = [ "*" , "+" , "x" , "o" , "\\" , "/", "o", "O", ".", "*" ]
        #colors = palettable.colorbrewer.diverging.Spectral_4.mpl_colors
        colors = palettable.scientific.sequential.Davos_4.mpl_colors



        for i in range(0,NUM_OF_TYPES+1):
                ax0.bar([p + width*i for p in pos], 
                        df[x_axe[i]], 
                        #hatch=3*patterns[i],
                        # of width
                        width=width,
                        # with color
                        #edgecolor=colors[i], 
                        color=colors[i+1],
                        # with label the first value in benchmark
                        label=x_axe[i],
                        zorder=3)


        # Set the position of the x ticks
        ax0.set_xticks([p + 2.5 * width for p in pos])

        # Set the labels for the x ticks
        ax0.set_xticklabels(df['benchmark'])
        ax0.tick_params(axis="x", labelsize=20)
        ax0.tick_params(axis="y", labelsize=20)
        # Setting the x-axis and y-axis limits
        #plt.xlim(min(pos)-width, max(pos)+width*4)
        #ax0.set(ylabel='$Normalized$ \n $Performance$')
        ax0.set_ylabel('$Normalized$ \n $Energy$ $Consumption$', fontsize = 20.0)
        #ax0.ylim([0,1.1])
        plt.xticks(rotation=55)
        #plt.yscale('symlog', linthreshy=1)
        # Adding the legend and showing the plot

        for tick in ax0.xaxis.get_majorticklabels():
            tick.set_horizontalalignment("right")

        plt.legend(bbox_to_anchor=(0,1.02,1,0.2), loc="lower left", mode="expand", borderaxespad=0, edgecolor='black', ncol=2, prop={"size":20, "weight":'bold'})
        ax0.grid( linestyle='-.', linewidth=0.5, alpha=.5, axis='y', zorder=0)
        plt.tight_layout()

        if not os.path.exists(file_type):
                os.makedirs(file_type)
        plt.savefig(file_type + '/Power_new.png', bbox_inches='tight')
        plt.show()             
        
def get_edp_results():
        
        results_index_file = open('./results_index.txt', mode='r')
        contents = results_index_file.read()
        results_index = contents.split("\n")

        power_file = open('check_power_spec.out', mode='r')
        power_content = power_file.read()
        power_results_lines = power_content.split('\n')
        #print(power_results_lines)
        power_results_split = []
        for line in power_results_lines:
                if (line != ""):
                        power_results_split.append([float(j) for j in line.split()])

        assert(len(power_results_split) == 64)
        
        power_results  = {                     
                'perl': [power_results_split[0],power_results_split[1],power_results_split[2],power_results_split[3],power_results_split[4],power_results_split[5],power_results_split[6]],
                'mcf': [power_results_split[7],power_results_split[8],power_results_split[9],power_results_split[10],power_results_split[11],power_results_split[12],power_results_split[13]],
                'xalan': [power_results_split[14],power_results_split[15],power_results_split[16],power_results_split[17],power_results_split[18],power_results_split[19]],
                'deepsjeng': [power_results_split[25],power_results_split[26],power_results_split[27]],
                'lbm': [power_results_split[34],power_results_split[35],power_results_split[36],power_results_split[37],power_results_split[38]],
                'nab': [power_results_split[39],power_results_split[40],power_results_split[41],power_results_split[42]],
                'exchange': [power_results_split[20],power_results_split[21],power_results_split[22],power_results_split[23],power_results_split[24]],
                'xz': [power_results_split[28],power_results_split[29],power_results_split[30],power_results_split[31],power_results_split[32],power_results_split[33]],
                'wrf': [power_results_split[43],power_results_split[44],power_results_split[45],power_results_split[46],power_results_split[47],power_results_split[48]],
                'leela': [power_results_split[49],power_results_split[50],power_results_split[51],power_results_split[52],power_results_split[53],power_results_split[54],power_results_split[55],power_results_split[56]],
                'gcc': [power_results_split[57],power_results_split[58],power_results_split[59],power_results_split[60],power_results_split[61],power_results_split[62],power_results_split[63]]
                }

        final_results_performance["Raw"] = []
        final_results_performance["Super"] = []

        #for bench in benchs:
        for bench in benchs:

                TotalPowerRaw = 0.0 #should be zero
                TotalPowerSuper = 0.0
                total_weight = 0.0

                for chk_num in range(len(benchs[bench])):

                        if weights[bench][chk_num] == 0.0:
                                continue
                        
                        cap_stat_file_dir = './' + bench + '_' + type2 + '/m5out_sim_' + bench + '_' + type2 + "_" + str(chk_num+1) + '/stats.txt'
                        Super_stat_file_dir = './' + bench + '_' + type1 + '/m5out_sim_' + bench + '_' + type1 + '_' + str(chk_num+1) + '/stats.txt'
                        # print(cap_stat_file_dir)
                        # print(Super_stat_file_dir)
                        cap_stat_file = open(cap_stat_file_dir, mode='r')
                        Super_stat_file = open(Super_stat_file_dir, mode='r')
                        cap_stat_file_content = cap_stat_file.read()
                        Super_stat_file_content = Super_stat_file.read()

                        cap_results = measure(results_index, cap_stat_file_content)
                        Super_results = measure(results_index, Super_stat_file_content)
                        simpoint_power_results = power_results[bench][chk_num]
                        #print (bench, simpoint_power_results)

                        Raw_power = simpoint_power_results[0]
                        Raw_intRF_power = simpoint_power_results[1]
                        Raw_rename_power = simpoint_power_results[2]

                        Super_power = simpoint_power_results[3]
                        Super_intRF_power = simpoint_power_results[4]
                        Super_rename_power = simpoint_power_results[5]

                        if Raw_intRF_power > Super_intRF_power:
                                Raw_power -= (Raw_intRF_power - Super_intRF_power)
                        if Raw_rename_power > Super_rename_power:
                                Raw_power -= (Raw_rename_power - Super_rename_power)


                        if int(cap_results['system.switch_cpus.numCycles'][-1]) == 0 or int(Super_results['system.switch_cpus.numCycles'][-1]) == 0:
                                warnings.warn(bench + " simpoint " + str(chk_num+1) + " has not stat!")
                                cap_stat_file.close()
                                Super_stat_file.close()
                                continue

                        Raw_power *= int(cap_results['system.switch_cpus.numCycles'][-1]) * int(cap_results['system.switch_cpus.numCycles'][-1])
                        Super_power *= int(Super_results['system.switch_cpus.numCycles'][-1]) * int(Super_results['system.switch_cpus.numCycles'][-1])
                        total_weight = total_weight + weights[bench][chk_num]

                        print("Super: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", Super_power)
                        print("Raw: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", Raw_power)
                        
                        
                        
                        TotalPowerRaw = TotalPowerRaw + weights[bench][chk_num] * Raw_power
                        TotalPowerSuper = TotalPowerSuper + weights[bench][chk_num] * Super_power
                        print("TotalPowerRaw: ", TotalPowerRaw)
                        print('TotalPowerSuper: ', TotalPowerSuper) 
                        cap_stat_file.close()
                        Super_stat_file.close()

                        

                print("-------------------------------------------DONE----------------------------")
                if total_weight == 0:
                    final_results_performance["Super"].append(0)
                    final_results_performance["Raw"].append(0)
                    warnings.warn(bench + " Has no working simpoints!!")
                    continue
                
                TotalPowerRaw /= total_weight
                TotalPowerSuper /= total_weight
                #NormPowerRaw = (TotalPowerRaw - TotalPowerSuper) / TotalPowerSuper
                NormPowerRaw = (TotalPowerRaw) / TotalPowerSuper
                print('NormPowerRaw: ', NormPowerRaw)
                print('TotalPowerSuper: ', total_weight/total_weight)

                final_results_performance["Super"].append((NormPowerRaw))
                final_results_performance["Raw"].append(total_weight/total_weight)
                




        df = pd.DataFrame(final_results_performance, columns = COLUMNS)
        # Setting the positions and width for the bars
        pos = list(range(len(df['Raw']))) 
        #print(pos)
        width = 0.25
            
        # Plotting the bars
        fig = plt.figure(figsize=(10,6))
        gs = gridspec.GridSpec(1, 1, height_ratios=[1]) 
        # print('AVG', final_results_performance['CHEx86ContextSensitive'])
        # print('GEOMEAN', 1 - gmean(final_results_performance['CHEx86ContextSensitive']))
        ax0 = plt.subplot(gs[0])
        patterns = [ "*" , "+" , "x" , "o" , "\\" , "/", "o", "O", ".", "*" ]
        #colors = palettable.colorbrewer.diverging.Spectral_4.mpl_colors
        colors = palettable.cartocolors.diverging.Earth_3.mpl_colors


        for i in range(0,NUM_OF_TYPES+1):
                ax0.bar([p + width*i for p in pos], 
                        df[x_axe[i]], 
                        #hatch=3*patterns[i],
                        # of width
                        width=width,
                        # with color
                        #edgecolor=colors[i], 
                        color=colors[i],
                        # with label the first value in benchmark
                        label=x_axe[i],
                        zorder=3)


        # Set the position of the x ticks
        ax0.set_xticks([p + 2.5 * width for p in pos])

        # Set the labels for the x ticks
        ax0.set_xticklabels(df['benchmark'])
        ax0.tick_params(axis="x", labelsize=20)
        ax0.tick_params(axis="y", labelsize=15)
        # Setting the x-axis and y-axis limits
        #plt.xlim(min(pos)-width, max(pos)+width*4)
        #ax0.set(ylabel='$Normalized$ \n $Performance$')
        ax0.set_ylabel('$Normalized$ \n $EDP$', fontsize = 16.0)
        #ax0.ylim([0,1.1])
        plt.xticks(rotation=55)
        #plt.yscale('symlog', linthreshy=1)
        # Adding the legend and showing the plot

        for tick in ax0.xaxis.get_majorticklabels():
            tick.set_horizontalalignment("right")

        plt.legend(bbox_to_anchor=(0,1.02,1,0.2), loc="lower left", mode="expand", borderaxespad=0, edgecolor='black', ncol=2, prop={"size":16, "weight":'bold'})
        ax0.grid( linestyle='-.', linewidth=0.5, alpha=.5, axis='y', zorder=0)
        plt.tight_layout()

        if not os.path.exists(file_type):
                os.makedirs(file_type)
        plt.savefig(file_type + '/EDP.png', bbox_inches='tight')
        plt.show()     
 
def get_traceLen_results():
        
        results_index_file = open('./results_index.txt', mode='r')
        contents = results_index_file.read()
        results_index = contents.split("\n")

        final_results_performance["Raw"] = []
        final_results_performance["Super"] = []

        #for bench in benchs:
        SpecSuperTotal = 0.0
        SpecRawTotal = 0.0
        NumOfSpec = 0
        ParsecSuperTotal = 0.0
        ParsecRawTotal = 0.0
        NumOfParsec = 0
        for i, bench in enumerate(benchs):

                TotalPowerRaw = 0.0 #should be zero
                TotalPowerSuper = 0.0
                total_weight = 0.0

                for chk_num in range(len(benchs[bench])):

                        if weights[bench][chk_num] == 0.0:
                                continue
                        
                        cap_stat_file_dir = './' + bench + '_' + type2 + '/m5out_sim_' + bench + '_' + type2 + "_" + str(chk_num+1) + '/stats.txt'
                        Super_stat_file_dir = './' + bench + '_' + type1 + '/m5out_sim_' + bench + '_' + type1 + '_' + str(chk_num+1) + '/stats.txt'
                        # print(cap_stat_file_dir)
                        # print(Super_stat_file_dir)
                        cap_stat_file = open(cap_stat_file_dir, mode='r')
                        Super_stat_file = open(Super_stat_file_dir, mode='r')
                        cap_stat_file_content = cap_stat_file.read()
                        Super_stat_file_content = Super_stat_file.read()

                        cap_results = measure(results_index, cap_stat_file_content)
                        Super_results = measure(results_index, Super_stat_file_content)
                        #print (bench, simpoint_power_results)

                        Raw_power = 0.0

                        if i <= 10:
                            Raw_power = float(cap_results['system.switch_cpus.commit.committedoptmized_insts_per_trace_size::mean'][-1])
                            if int(cap_results['system.switch_cpus.numCycles'][-1]) == 0 or int(Super_results['system.switch_cpus.numCycles'][-1]) == 0:
                                warnings.warn(bench + " simpoint " + str(chk_num+1) + " has not stat!")
                                cap_stat_file.close()
                                Super_stat_file.close()
                                continue
                        else:
                            Raw_power = float(cap_results['system.cpu.commit.committedoptmized_insts_per_trace_size::mean'][-1])
                            if int(cap_results['system.cpu.numCycles'][-1]) == 0 or int(Super_results['system.cpu.numCycles'][-1]) == 0:
                                warnings.warn(bench + " simpoint " + str(chk_num+1) + " has not stat!")
                                cap_stat_file.close()
                                Super_stat_file.close()
                                continue

                        #Raw_power *= int(cap_results['system.switch_cpus.numCycles'][-1]) * int(cap_results['system.switch_cpus.numCycles'][-1])
                        total_weight = total_weight + weights[bench][chk_num]

                        print("Raw: " + bench + " checkpoints_at[""," + str(chk_num+1) + "]=", Raw_power)
                        
                        
                        
                        TotalPowerRaw = TotalPowerRaw + weights[bench][chk_num] * Raw_power
                        print("TotalPowerRaw: ", TotalPowerRaw)
                        print('TotalPowerSuper: ', TotalPowerSuper) 
                        cap_stat_file.close()
                        Super_stat_file.close()

                        

                print("-------------------------------------------DONE----------------------------")
                if total_weight == 0:
                    final_results_performance["Super"].append(0)
                    final_results_performance["Raw"].append(0)
                    warnings.warn(bench + " Has no working simpoints!!")
                    continue
                
                TotalPowerRaw /= total_weight
                TotalPowerSuper /= total_weight
                #NormPowerRaw = (TotalPowerRaw - TotalPowerSuper) / TotalPowerSuper
                NormPowerRaw = (TotalPowerRaw)
                print('NormPowerRaw: ', NormPowerRaw)
                print('TotalPowerSuper: ', total_weight/total_weight)

                final_results_performance["Super"].append((NormPowerRaw))
                final_results_performance["Raw"].append(0)

                if i <= 10:
                    SpecSuperTotal += NormPowerRaw
                    SpecRawTotal += 0
                    NumOfSpec += 1
                else:
                    ParsecSuperTotal += NormPowerRaw
                    ParsecRawTotal += 0
                    NumOfParsec += 1


        # final_results_performance["Super"].append(SpecSuperTotal/NumOfSpec)
        # final_results_performance["Raw"].append(SpecRawTotal/NumOfSpec)
        # final_results_performance["Super"].append(ParsecSuperTotal/NumOfParsec)
        # final_results_performance["Raw"].append(ParsecRawTotal/NumOfParsec)
                




        df = pd.DataFrame(final_results_performance, columns = COLUMNS)
        # Setting the positions and width for the bars
        pos = list(range(len(df['Raw']))) 
        #print(pos)
        width = 0.25
            
        # Plotting the bars
        fig = plt.figure(figsize=(10,6))
        gs = gridspec.GridSpec(1, 1, height_ratios=[1]) 
        # print('AVG', final_results_performance['CHEx86ContextSensitive'])
        # print('GEOMEAN', 1 - gmean(final_results_performance['CHEx86ContextSensitive']))
        ax0 = plt.subplot(gs[0])
        patterns = [ "*" , "+" , "x" , "o" , "\\" , "/", "o", "O", ".", "*" ]
        #colors = palettable.colorbrewer.diverging.Spectral_4.mpl_colors
        colors = palettable.cartocolors.diverging.Earth_3.mpl_colors


        for i in range(0,NUM_OF_TYPES+1):
                ax0.bar([p + width*i for p in pos], 
                        df[x_axe[i]], 
                        #hatch=3*patterns[i],
                        # of width
                        width=width,
                        # with color
                        #edgecolor=colors[i], 
                        color=colors[0],
                        # with label the first value in benchmark
                        label=x_axe[i],
                        zorder=3)


        # Set the position of the x ticks
        ax0.set_xticks([p + 2.5 * width for p in pos])

        # Set the labels for the x ticks
        ax0.set_xticklabels(df['benchmark'])
        ax0.tick_params(axis="x", labelsize=20)
        ax0.tick_params(axis="y", labelsize=20)
        # Setting the x-axis and y-axis limits
        #plt.xlim(min(pos)-width, max(pos)+width*4)
        #ax0.set(ylabel='$Normalized$ \n $Performance$')
        ax0.set_ylabel('$Average$ $Sequence$ $Length$', fontsize = 20.0)
        #ax0.ylim([0,1.1])
        plt.xticks(rotation=55)
        #plt.yscale('symlog', linthreshy=1)
        # Adding the legend and showing the plot

        for tick in ax0.xaxis.get_majorticklabels():
            tick.set_horizontalalignment("right")

        #plt.legend(bbox_to_anchor=(0,1.02,1,0.2), loc="lower left", mode="expand", borderaxespad=0, edgecolor='black', ncol=2, prop={"size":16, "weight":'bold'})
        ax0.grid( linestyle='-.', linewidth=0.5, alpha=.5, axis='y', zorder=0)
        plt.tight_layout()

        if not os.path.exists(file_type):
                os.makedirs(file_type)
        plt.savefig(file_type + '/avgLen.png', bbox_inches='tight')
        plt.show()     
 
if __name__ == "__main__":
        get_main_results()
        #get_uop_cache_results()
        #get_performance_results()
        #get_inst_results()
        #get_squash_results()
        #get_accuracy_results()
        #get_streamedfrom_results()
        #get_fetch_blocked_results()
        #get_rename_blocked_results()
        #get_iew_blocked_results()
        #get_iqueue_results()
        #get_lqueue_results()
        #get_squeue_results()
        #get_rob_results()
        #get_int_dependency_results()
        #get_cc_dependency_results()
        #get_int_from_results()
        #get_cc_from_results()
        #get_iew_blocked_cause_results()
        #get_commitSquashed_results()
        #get_commitSquashed_detailed_results()
        #get_branch_pred_results_percent()
        #get_branch_pred_results_num()
        #get_branch_pred_detailed_results()
        #get_branch_results()
        get_power_results()
        #get_traceLen_results()

