import numpy as np
import json

class EnvGenParser:
    def __init__(self, filename):
        self.env_gen_stats_file = filename

        self.env_gen_stats = {}
        with open(self.env_gen_stats_file, "r") as env_gen_json_file:
            self.env_gen_stats = json.load(env_gen_json_file)

        self.obstacle_coords = self.env_gen_stats["ObstacleList"]
        self.spread_of_obstacles = self.env_gen_stats["SpreadOfObstacles"]

        self.zones = self.extract_zones()


    def extract_zones(self):
        obstacle_coords = self.obstacle_coords
        zone_separation_dist = 3 * self.spread_of_obstacles

        # ordering obstacle coords in increasing order of x of top right corner of obstacle
        obstacle_coords = sorted(obstacle_coords, key=lambda x: x[0][0])

        obstacle_zones = [obstacle_coords[0]]

        for obstacle_bounds in obstacle_coords:
            obs_top_right = obstacle_bounds[0]
            obs_bottom_left = obstacle_bounds[1]

            found_zone = False
            for i, zone_bound in enumerate(obstacle_zones):
                zone_bound_top_right = zone_bound[0]
                zone_bound_bottom_left = zone_bound[1]
                
                if np.linalg.norm(np.array(obs_top_right) - np.array(zone_bound_top_right)) <= zone_separation_dist:
                    # expand current zone
                    new_zone_top_right = [max(zone_bound_top_right[0], obs_top_right[0]), max(zone_bound_top_right[1], obs_top_right[1])]
                    new_zone_bottom_left = [min(zone_bound_bottom_left[0], obs_bottom_left[0]), min(zone_bound_bottom_left[1], obs_bottom_left[1])]
                    obstacle_zones[i] = [new_zone_top_right, new_zone_bottom_left]
                    found_zone = True
                    break

            if not found_zone:
                # start new zone
                new_zone = obstacle_bounds
                obstacle_zones.append(new_zone)

        # filter out any small spurious zones
        min_zone_length = 50
        tmp = []
        for zone in obstacle_zones:
            zone_bound_top_right = zone[0]
            zone_bound_bottom_left = zone[1]

            if (zone_bound_top_right[0] - zone_bound_bottom_left[0]) > min_zone_length:
                tmp.append(zone)

        obstacle_zones = tmp

        assert len(obstacle_zones) == 2, "We don't support more than two populated zones right now"

        # we have a large amount of free space in the middle so we add it as
        # a zone between the two populated ones
        bottom_zone = obstacle_zones[0]
        bottom_zone_width = bottom_zone[0][0] - bottom_zone[1][0]
        free_space_zone_bottom_left = [bottom_zone[0][0] - bottom_zone_width, bottom_zone[0][1]]

        top_zone = obstacle_zones[1]
        top_zone_width = top_zone[0][0] - top_zone[1][0]
        free_space_zone_top_right = [top_zone[1][0] + top_zone_width, top_zone[1][1]]

        free_space_zone = [free_space_zone_top_right, free_space_zone_bottom_left]

        obstacle_zones.insert(1, free_space_zone)

        return obstacle_zones

    def split_trajectory_by_zone(self, traj_x, traj_y):
        return

    #def split_time_by_zone(self, time_cmd_received, traj_x, traj_y):
    #    zones = self.zones
    #    
    #    assert len(zones) == 3, "We don't support more than 2 populated + 1 free space zone currently"

    #    n_traj_points = len(traj_x)
    #    zone_boundary_times = []

    #    curr_zone_ind = 0
    #    # flag for checking if we're currently in a zone we've explored in the past
    #    old_zone = False
    #    for i in range(n_traj_points):
    #        curr_zone = zones[curr_zone_ind]
    #        curr_zone_top_right = curr_zone[0]
    #        curr_zone_bottom_left = curr_zone[1]
    #        if traj_x[i] < curr_zone_top_right[0] and traj_y[i] < curr_zone_top_right[1] and \
    #                curr_zone_bottom_left[0] < traj_x[i] and curr_zone_bottom_left[1] < traj_y[i]:
    #            old_zone = False
    #            continue
    #        else:
    #            # checking if it went back to an old zone
    #            past_zones = zones[:curr_zone_ind]
    #            for past_zone in past_zones:
    #                past_zone_top_right = past_zone[0]
    #                past_zone_bottom_left = past_zone[1]
    #                if traj_x[i] < past_zone_top_right[0] and traj_y[i] < past_zone_top_right[1] and \
    #                        past_zone_bottom_left[0] < traj_x[i] and past_zone_bottom_left[1] < traj_y[i]:
    #                    old_zone = True
    #                    break

    #            # making sure we haven't waded into an undefined zone
    #            if not old_zone and curr_zone_ind != len(zones) - 1:
    #                next_zone = zones[curr_zone_ind+1]
    #                next_zone_top_right = next_zone[0]
    #                next_zone_bottom_left = next_zone[1]
    #                if traj_x[i] < next_zone_top_right[0] and traj_y[i] < next_zone_top_right[1] and \
    #                        next_zone_bottom_left[0] < traj_x[i] and next_zone_bottom_left[1] < traj_y[i]:
    #                    zone_boundary_times.append(time_cmd_received[i])
    #                    curr_zone_ind += 1

    #    return zone_boundary_times

    def split_time_by_zone(self, time_cmd_received, traj_x, traj_y):
        rect_zones = self.zones
        
        assert len(rect_zones) == 3, "We don't support more than 2 populated + 1 free space zone currently"

        # we change the zone to a circle centered at the center of the populated zone
        # with radius spread of obstacles + halo

        circ_zones = {}
        circ_zones["origin"] = []
        circ_zones["radius"] = []
        for i, zone in enumerate(rect_zones):
            rect_zone = rect_zones[i]
            rect_zone_top_right = rect_zone[0]
            rect_zone_bottom_left = rect_zone[1]
            width = rect_zone_top_right[0] - rect_zone_bottom_left[0]
            height = rect_zone_top_right[1] - rect_zone_bottom_left[1]
            origin = [rect_zone_bottom_left[0] + width/2, rect_zone_bottom_left[1] + height/2]
            if i == 0:
                radius = 15 + self.spread_of_obstacles
                circ_zones["origin"].append(origin)
                circ_zones["radius"].append(radius)
            elif i == 2:
                radius = 50 + self.spread_of_obstacles
                circ_zones["origin"].append(origin)
                circ_zones["radius"].append(radius)

        n_traj_points = len(traj_x)
        zone_boundary_times = []

        curr_zone_ind = 0
        middle_zone = False
        for i in range(n_traj_points):
            curr_zone_origin = circ_zones["origin"][curr_zone_ind]
            curr_zone_radius = circ_zones["radius"][curr_zone_ind]
            traj = np.array([traj_x[i], traj_y[i]])
            origin = np.array(curr_zone_origin)
            squares = np.square(np.subtract(traj, origin))
            if squares[0] + squares[1] <= curr_zone_radius**2:
                # within circle
                if middle_zone:
                    # transitioned into populated end zone
                    zone_boundary_times.append(time_cmd_received[i])
                    middle_zone = False
                continue
            else:
                if curr_zone_ind == 0:
                    zone_boundary_times.append(time_cmd_received[i])
                    # continue checking if we are in next populated zone
                    curr_zone_ind += 1
                    middle_zone = True
                elif middle_zone:
                    # while we are in middle zone, we ignore how we're outside
                    # the bounds of the populated end zone
                    continue

        return zone_boundary_times

    def getEnvGenStats(self):
        return self.env_gen_stats
