

#planning_failure_rate, planning_piecewise_failure_rate, planning_smoothening_failure_rate  
#        "S_A_latency", S_A_response_time_calculated_from_imgPublisher
"""
def get_value_easy_metrics(line):
    value_raw = line.split()[1]
    print line.split()
    return float(value_raw.split(',')[0])
"""

# parse and collect data
def parse_file(filepath, metric_dic, metrics_to_collect_easy, metrics_to_collect_hard):
    metrics_to_collect_this_round = metrics_to_collect_hard + metrics_to_collect_easy;
    with open(filepath) as fp:
       lines = fp.readlines()
       idx = 0
       for line in lines:
           # for data not comming out of the profiler  
           for metric in metrics_to_collect_easy:
               line_split_1 = line.split()
               line_split_2 = line.split(":")
               word_to_look_for_1 = '"' + metric + '":'
               word_to_look_for_2 = '"' + metric + '"'
               if  word_to_look_for_1 in line_split_1:
                   value  = line_split_1[1].split(",")[0]
                   metric_dic[metric].append(float(value))
                   metrics_to_collect_this_round.remove(metric)
               elif   word_to_look_for_2 in line_split_2:
                   if (len(line_split_2[1].split(",")[0].split('"')) < 2):
                       value =  line_split_2[1].split(",")[0].split('"')[0].split()[0] # last element with no comma

                   else:
                       value = line_split_2[1].split(",")[0].split('"')[1]
                   if not len(metric_dic[metric]) > min(list(map(lambda val: len(val), list(metric_dic.values())))): # avoid double counting
                       metric_dic[metric].append(float(value))
                       metrics_to_collect_this_round.remove(metric)
                       if metric == "experiment_number":
                           if not(len(metrics_to_collect_this_round) == 0):
                               for metric_to_collect in metrics_to_collect_this_round:
                                   metric_dic[metric_to_collect].append(-999)
                           metrics_to_collect_this_round = metrics_to_collect_easy + metrics_to_collect_hard



           # for data comming out of the profiler  
           for metric in metrics_to_collect_hard:
               line_split_1 = line.split()
               line_split_2 = line.split(":")
               word_to_look_for_1 = '"' + metric
               word_to_look_for_2 = '"' + metric + '"'
               if  word_to_look_for_1 in line_split_1:
                   next_line = lines[idx+1]
                   value  = next_line.split(":")[1].split(",")[0]
                   if not len(metric_dic[metric]) > min(list(map(lambda val: len(val), list(metric_dic.values())))): # avoid double counting
                       metric_dic[metric].append(float(value))
                       metrics_to_collect_this_round.remove(metric)
               """
               elif   word_to_look_for_2 in line_split_2:
                   if (len(line_split_2[1].split(",")[0].split('"')) < 2):
                       value =  line_split_2[1].split(",")[0].split('"')[0].split()[0] # last element with no comma
                   else:
                       value = line_split_2[1].split(",")[0].split('"')[1]
                   blah =    min(list(map(lambda val: len(val), list(metric_dic.values()))))
                   if not len(metric_dic[metric]) > min(list(map(lambda val: len(val), list(metric_dic.values())))):
                       metric_dic[metric].append(float(value))
               """
           idx +=1
    return metric_dic

# write results to to a cst file
def write_results(metric_dic, file_path):
    fp = open(file_path, "w")
    for key in metric_dic.keys():
        fp.write(key + ",")

    fp.write("\n")
    gen_length = len(list(metric_dic.values())[0])
    for idx in range(gen_length):
        for val in metric_dic.values():
            fp.write(str(val[idx]) +",")
        fp.write("\n")
    fp.close()


def main():
    input_filepath = "./../../data/package_delivery/stats.json"
    metrics_to_collect_easy = ["distance_travelled", 
        "flight_time", "piecewise_planning_budget", "perception_lower_resolution",
        "smoothening_budget", "experiment_number"]
    metrics_to_collect_hard = ["S_A_latency", "S_A_response_time_calculated_from_imgPublisher", "planning_piecewise_failure_rate", "planning_smoothening_failure_rate"]

    # initialize the metric dictionary
    metric_dic = {}
    for metric in metrics_to_collect_easy + metrics_to_collect_hard:
        metric_dic[metric] = []
        
    # parse  
    metric_dic = parse_file(input_filepath, metric_dic, metrics_to_collect_easy, metrics_to_collect_hard)
    
    # sanity check by making sure all the data lists have the same size,
    metric_data_length_dic = {}
    for key,val in metric_dic.items():
        metric_data_length_dic[key] = len(list(val))
    print metric_data_length_dic

    gen_length  = list(metric_data_length_dic.values())[0]
    for key, val in metric_data_length_dic.items():
        if not (val == gen_length):
            print ("key:" + str(key) + " length:" + str(val) + " is not equal the generic length:" + str(gen_length))

    # write results 
    write_results(metric_dic, "results.txt")


main()
