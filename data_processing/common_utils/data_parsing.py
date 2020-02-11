
from collections import OrderedDict
from copy import *
import math
#planning_failure_rate, planning_piecewise_failure_rate, planning_smoothening_failure_rate
#        "S_A_latency", S_A_response_time_calculated_from_imgPublisher
"""
def get_value_easy_metrics(line):
    value_raw = line.split()[1]
    print line.split()
    return float(value_raw.split(',')[0])
"""


# parse and collect data
def parse_stat_file_flattened(filepath, metrics_to_collect_easy, metrics_to_collect_hard):

    # initialize the metric dictionary
    result_dic = OrderedDict()
    for metric in metrics_to_collect_easy + metrics_to_collect_hard:
        result_dic[metric] = []

    #  keep track of the metrics collected to avoid double counting and also
    # data padding
    metrics_to_collect_this_round = metrics_to_collect_hard + metrics_to_collect_easy;

    # parse the file and fill out metric dict
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
                   result_dic[metric].append(float(value))
                   metrics_to_collect_this_round.remove(metric)
               elif   word_to_look_for_2 in line_split_2:
                   if (len(line_split_2[1].split(",")[0].split('"')) < 2):
                       value =  line_split_2[1].split(",")[0].split('"')[0].split()[0] # last element with no comma

                   else:
                       value = line_split_2[1].split(",")[0].split('"')[1]
                   if not len(result_dic[metric]) > min(list(map(lambda val: len(val), list(result_dic.values())))): # avoid double counting
                       result_dic[metric].append(float(value))
                       metrics_to_collect_this_round.remove(metric)
                       if metric == "experiment_number":
                           if not(len(metrics_to_collect_this_round) == 0):
                               for metric_to_collect in metrics_to_collect_this_round:
                                   result_dic[metric_to_collect].append(float("inf"))
                           metrics_to_collect_this_round = metrics_to_collect_easy + metrics_to_collect_hard

           # for data comming out of the profiler
           for metric in metrics_to_collect_hard:
               line_split_1 = line.split()
               line_split_2 = line.split(":")
               word_to_look_for_1 = '"' + metric
               word_to_look_for_2 = '"' + metric + '"'
               if  word_to_look_for_1 in line_split_1:
                   next_line = lines[idx+1]
                   values =[]
                   for el in next_line.split(":")[1].split(","):
                       if "std" in el:
                           break
                       else:
                           values.append(float(el))
                   if not len(result_dic[metric]) > min(list(map(lambda val: len(val), list(result_dic.values())))): # avoid double counting
                       result_dic[metric]+=values
           idx +=1

    # sanity check by making sure all the data lists have the same size,
    result_data_length_dic = {}
    for key, val in result_dic.items():
        result_data_length_dic[key] = len(list(val))
    #print metric_data_length_dic

    gen_length = list(result_data_length_dic.values())[0]
    for key, val in result_data_length_dic.items():
        if not (val == gen_length):
            print ("key:" + str(key) + " length:" + str(val) + " is not equal the generic length:" + str(gen_length))

    return result_dic




# parse and collect data
def parse_stat_file(filepath, metrics_to_collect_easy, metrics_to_collect_hard):

    # initialize the metric dictionary
    result_dic = OrderedDict()
    for metric in metrics_to_collect_easy + metrics_to_collect_hard:
        result_dic[metric] = []

    #  keep track of the metrics collected to avoid double counting and also
    # data padding
    metrics_to_collect_this_round = metrics_to_collect_hard + metrics_to_collect_easy;

    # parse the file and fill out metric dict
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
                   result_dic[metric].append(float(value))
                   metrics_to_collect_this_round.remove(metric)
               elif   word_to_look_for_2 in line_split_2:
                   if (len(line_split_2[1].split(",")[0].split('"')) < 2):
                       value =  line_split_2[1].split(",")[0].split('"')[0].split()[0] # last element with no comma

                   else:
                       value = line_split_2[1].split(",")[0].split('"')[1]
                   if not len(result_dic[metric]) > min(list(map(lambda val: len(val), list(result_dic.values())))): # avoid double counting
                       result_dic[metric].append(float(value))
                       metrics_to_collect_this_round.remove(metric)
                       if metric == "experiment_number":
                           if not(len(metrics_to_collect_this_round) == 0):
                               for metric_to_collect in metrics_to_collect_this_round:
                                   result_dic[metric_to_collect].append(float("inf"))
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
                   if not len(result_dic[metric]) > min(list(map(lambda val: len(val), list(result_dic.values())))): # avoid double counting
                       result_dic[metric].append(float(value))
                       metrics_to_collect_this_round.remove(metric)
               """
               elif   word_to_look_for_2 in line_split_2:
                   if (len(line_split_2[1].split(",")[0].split('"')) < 2):
                       value =  line_split_2[1].split(",")[0].split('"')[0].split()[0] # last element with no comma
                   else:
                       value = line_split_2[1].split(",")[0].split('"')[1]
                   blah =    min(list(map(lambda val: len(val), list(result_dic.values()))))
                   if not len(result_dic[metric]) > min(list(map(lambda val: len(val), list(result_dic.values())))):
                       result_dic[metric].append(float(value))
               """
           idx +=1

    # sanity check by making sure all the data lists have the same size,
    result_data_length_dic = {}
    for key, val in result_dic.items():
        result_data_length_dic[key] = len(list(val))
    #print metric_data_length_dic

    gen_length = list(result_data_length_dic.values())[0]
    for key, val in result_data_length_dic.items():
        if not (val == gen_length):
            print ("key:" + str(key) + " length:" + str(val) + " is not equal the generic length:" + str(gen_length))

    return result_dic


# write results to to a cst file
def write_results_to_csv(result_dic, file_path):
    fp = open(file_path, "w")
    for key in result_dic.keys():
        fp.write(key + ",")

    fp.write("\n")
    gen_length = len(list(result_dic.values())[0])
    for idx in range(gen_length):
        for val in result_dic.values():
            fp.write(str(val[idx]) +",")
        fp.write("\n")
    fp.close()




def filter_based_on_keys(result_dic, keys):
    gen_length = len(list(result_dic.values())[0])
    filtered_result_dic = OrderedDict()
    for key in result_dic.keys():
        if key in keys:
            filtered_result_dic[key] = result_dic[key]
    return filtered_result_dic


def filter_based_on_value(result_dict, value, mode):
    gen_length = len(list(result_dic.values())[0])
    for idx in range(gen_length):
        for val in result_dic.values():
            if float(round(val[idx], 4)) == float(round(value_, 4)):
                remove_idx = True
                break
        if (remove_idx):
            for val in result_dic.values():
                del vel[idx]
        remove_idx = False





def filter_based_on_key_value(result_dic, key_, value_, mode):
    gen_length = len(list(result_dic.values())[0])
    filtered_result_dic = OrderedDict()

    idx_to_pay_attention_to = []
    for idx in range(gen_length):
        for key, val in result_dic.items():
            if round(val[idx], 4) == round(value_, 4) and key_ == key:
                idx_to_pay_attention_to.append(idx)
                break

    if mode == "in":
        # initilaize with the keys
        for key in result_dic.keys() :
            filtered_result_dic[key] = []
        for idx in idx_to_pay_attention_to:
            for key, val in result_dic.items():
                filtered_result_dic[key].append(val[idx]);
    elif mode == "out":
        filtered_result_dic = copy.deepcopy(result_dic)
        cnt = 0
        for idx in idx_to_pay_attention_to:
            for val in filtered_result_dic.values():
                del val[idx-cnt]
            cnt +=1

    return filtered_result_dic


def filter_results(result_dic, metric_value_to_filter):
    if len(metric_value_to_filter.keys()) == 1 and list(metric_value_to_filter.keys())[0] == "any":
        filter_based_on_value((result_dic, metric_value_to_filter.values())[0])
    else:
        for key, value in metric_value_to_filter.items():
            filter_based_on_key_value(key, value)


# take the average of contegous data
def avg_over_sequence(result_dic, seq_length):
    gen_length = len(list(result_dic.values())[0])
    assert (int(gen_length/seq_length) == gen_length/seq_length), "should be able to split evenly"

    new_result_dict = OrderedDict()

    for key in list(result_dic.keys()):
        new_result_dict[key] = []

    for idx in range(gen_length/seq_length):
        for key, val in result_dic.items():
                sum = 0
                cnt = 0
                for el in val[idx*seq_length: idx*seq_length + seq_length]:
                    if not (el == float("inf")): # if inifity skip it
                        sum += el
                        cnt +=1;
                if not (cnt == 0): # if there was at least one value that wasn't inifinity
                    #new_result_dict[key].append(sum(val[idx*seq_length: idx*seq_length+ seq_length])/seq_length)
                    new_result_dict[key].append(sum/cnt)
                else:
                    new_result_dict[key].append(float("inf"))

    return new_result_dict


def std_over_sequence(result_dic, seq_length, easy_metrics):
    gen_length = len(list(result_dic.values())[0])
    assert (int(gen_length/seq_length) == gen_length/seq_length), "should be able to split evenly"

    new_result_dict = OrderedDict()

    for key in list(result_dic.keys()):
        new_result_dict[key] = []

    avg_results = avg_over_sequence(result_dic, seq_length)

    for idx in range(gen_length/seq_length):
        for key, val in result_dic.items():
                sum = 0
                cnt = 0
                for el in val[idx*seq_length: idx*seq_length + seq_length]:
                    if not (el == float("inf")): # if inifity skip it
                        sum += (el - avg_results[key][idx])**2
                        cnt +=1;


                if (key in easy_metrics):
                    new_result_dict[key].append(avg_results[key][idx])
                elif not (cnt == 0) and not(cnt ==1): # if there was at least one value that wasn't inifinity
                    #new_result_dict[key].append(sum(val[idx*seq_length: idx*seq_length+ seq_length])/seq_length)
                    new_result_dict[key].append(math.sqrt(sum/(cnt-1)))
                else:
                    new_result_dict[key].append(float("inf"))

    return new_result_dict



def template():
    result_folder = "../lower_res_time_budget"
    input_file_name = "stats.json"
    input_filepath = result_folder + "/" + input_file_name
    output_file_name = "results.csv"
    output_filepath = result_folder + "/" + output_file_name
    metrics_to_collect_easy = ["distance_travelled",
        "flight_time", "piecewise_planning_budget", "perception_lower_resolution",
        "smoothening_budget", "experiment_number"]
    metrics_to_collect_hard = ["S_A_latency", "S_A_response_time_calculated_from_imgPublisher", "planning_piecewise_failure_rate", "planning_smoothening_failure_rate"]

    # parse  data
    result_dic = parse_stat_file(input_filepath,  metrics_to_collect_easy, metrics_to_collect_hard)

    # write results
    write_results_to_csv(result_dic, output_filepath + "raw")

    result_dic = avg_over_sequence(result_dic, 3)
    write_results_to_csv(result_dic, output_filepath+"avg")


#template()
