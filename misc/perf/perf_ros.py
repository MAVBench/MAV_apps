import psutil
import os
import time

#collect ros processes 
def collect_procs():
    #procObjList = [procObj for procObj in psutil.process_iter()]
    proc_list = []
    for proc in psutil.process_iter():
        pinfo = proc.as_dict(attrs=['pid', 'name', 'cmdline'])
        for arg in pinfo['cmdline']:
            if ".ros" in arg.split("/"):
                proc_list.append(pinfo)
    return proc_list

# set process affinity
def set_affinity(proc_id, core_number):
    os.system("taskset -pc " + str(core_number) + " " + str(proc_id))

def get_num_of_threads(pid):
    process = psutil.Process(pid)
    return process.num_threads()


def print_profiled_data(psutil_proc):
    print(str(psutil_proc.name()) + " cpu_perc:" + str(psutil_proc.cpu_percent()) +
          " mem_perc:" + str(psutil_proc.memory_percent()))

def main():
    # collect all the proc ids
    proc_list = collect_procs()

    _ = [set_affinity(proc['pid'], 0) for proc in proc_list]
    for 
    #print proc['name']+ str(get_num_of_threads(proc['pid']))

    psutil_proc_list = [psutil.Process(pid=proc['pid'] )for proc in proc_list]
    while True:
        time.sleep(5)
        _ = [print_profiled_data(psutil_proc) for psutil_proc in psutil_proc_list]


    """
    #@print proc_name + ":" + str(proc_id)

    """
main()
#         proc_name_pid[
#                 print pinfo['pid']
#
#./perf stat -e cycles 
