import os
import sys

#[nodename,core_number,priority]
node_name = [
    ["op_behavior_selector",0,1],
    ["pure_pursuit",0,1],
    ["twist_filter",0,1]
]

if __name__ == "__main__":
    core_num = 6
    core_iteration=0
    core_designate = 0
    for node in node_name:

        ps_cmd = "ps -eLf | grep __name:=" + node[0]
        # ps_cmd = "ps -e | grep " + node[0]
        ps_res = os.popen(ps_cmd).readline()
        #print(ps_res)
        #print(ps_res.split())
        main_pid = int(ps_res.split()[1])
        #tid_list = []
        print("main pid is " + str(main_pid))

        core_designate = core_iteration % core_num
        core_iteration = core_iteration + 1

        chrt_cmd = "sudo chrt -f -a -p "+str(node[2])+" "+ str(main_pid)
        # taskset_cmd = "taskset -pc " + str(node[1]) + " "+ str(main_pid)

        os.system(chrt_cmd)
        # os.system(taskset_cmd)

"""
    
        for line in ps_res:
            if int(line.split()[1]) == main_pid:
                tid_list.append(int(line.split()[3]))
        
        for tid in tid_list:
            #renice_cmd = "sudo renice -n -2 -p " + str(tid)
            chrt_cmd="sudo chrt -r -p " + str(main_pid)
            os.system(renice_cmd)
"""