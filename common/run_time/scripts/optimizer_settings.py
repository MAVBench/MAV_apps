import numpy as np


pl_to_ppl_ratio = 2  # this is there because we set the smoothening time equal to planning for the moment
# since we don't have knobs for smoothening (so we haven't included it as a stage)

# latency of the stages we didn't include in the optimizer formulation
run_diagnostics = .05
filtering = .200
depth_img_latency = .1
front_end_latency = run_diagnostics + filtering + depth_img_latency # this is all for runtime
#front_end_latency = 0 

pc_to_om_oh = .02
runtime_latency = .05 # this does include point cloud intro (i.e, generating point cloud and running diagnostics)
pc_outro = .01 # filtering and such
follow_trajectory_latency = .1
back_end_latency = runtime_latency + pc_outro +  pc_to_om_oh + follow_trajectory_latency 
#misc_latency = front_end_latency + back_end_latency # time of the stages not included in the controller

# constraints
#pc_res_min = .3
r_min_static = .3
om_to_pl_res_min = r_min_static
r_steps = 4
r_max_static = (2 ** r_steps) * r_min_static
v_min = 3000
#v_max = np.inf
v_max = 2000000
#rt_max = 30


