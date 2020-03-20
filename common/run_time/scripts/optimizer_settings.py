import numpy as np


pl_to_ppl_ratio = 2  # this is there because we set the smoothening time equal to planning for the moment
# since we don't have knobs for smoothening (so we haven't included it as a stage)

# latency of the stages we didn't include in the optimizer formulation
pc_latency = .15
om_to_pc_oh = .15
misc_latency = pc_latency + om_to_pc_oh  # time of the stages not included in the controller

# constraints
pc_res_min = .15
om_to_pl_res_min = .6
r_steps = 4
r_min = .9 * pc_res_min  # minimal octomap res
r_max = (2 ** r_steps) * r_min
v_min = 1000
#v_max = np.inf
v_max = 2000000
#rt_max = 30


