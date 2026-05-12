import os
import json
import pathlib
import datetime
import socket
import time
import math

import numpy as np
import pandas as pd

from basic_map.graph import NetGraph

VEHICLE_WIDTH = 0.2
VEHICLE_MARGIN = 0.5

class Coordinator:
    """
    Coordinator to ensure robots do not kill each other
    """

    def __init__(self, total_schedule=None):
        self._total_schedule = total_schedule
        self._robot_ids = total_schedule['robot_id'].unique().tolist()
        self._graph = None

        self._horizon_dict = {}                # dictionary that stores horizon of all robots
        self._current_target_node_ids = {}     # dictionary that stores current targer node id of all robots

        self._remaining_nodes = {}

        self._curr_pose = {}

        self.coord_controlled_bots = {
            'stopped': set(),
            'crossing': {}
        }

        self.concerning_node = []

        for rid in self._robot_ids:
            robot_schedule = self._total_schedule[self._total_schedule["robot_id"] == rid]
            self._remaining_nodes[rid] = robot_schedule["node_id"].tolist()
            self._remaining_nodes[rid].pop(0)


    @classmethod
    def from_csv(cls, csv_path: str, csv_sep:str=','):
        """Load the total schedule from a csv file."""
        total_schedule = pd.read_csv(csv_path, sep=csv_sep, header=0)
        return cls(total_schedule)


    def load_graph_from_json(self, graph_path):
        self._graph = NetGraph.from_json(graph_path)


    def update_horizon(self, robot_id, ref_states):
        """ Update horizion dict based on new robot states """

        self._horizon_dict[robot_id] = ref_states   


    def update_target_nodes(self, robot_id, target_node):
        
        if len(self._current_target_node_ids) == len(self._robot_ids):
            # print(f"{robot_id}: {target_node}")    
            if self._current_target_node_ids[robot_id] != target_node:
                
                self._current_target_node_ids[robot_id] = target_node
                self._remaining_nodes[robot_id].pop(0)

        else:
            self._current_target_node_ids[robot_id] = target_node


    def update_curr_pose(self, robot_id, pose):

        self._curr_pose[robot_id] = pose


    def target_node_to_bot_list(self):
        """ Converts robot --> target node mapping into node --> approaching robots mapping """

        node_to_robots = {}

        for robot_id, node_id in self._current_target_node_ids.items():
            if node_id is None:
                continue

            if node_id not in node_to_robots:
                node_to_robots[node_id] = []

            node_to_robots[node_id].append(robot_id)

        conflicts = {
            node_id: robot_ids
            for node_id, robot_ids in node_to_robots.items()
            if len(robot_ids) > 1
        }

        # print(f'conflicts: {conflicts}')
        return conflicts
        

    def validate(self):
        """ Validates current path of robots to avoid deadlock scenarios """

        # check if robots are heading towards the same node [COORDINATOR SCENE 1]
        # TODO: this works for scene 1 now, but till the prioirity robot does not vacate fully, the systems keeps having issues. 
        # need to solve this, maybe by forcing the prio robot to go a safe distance ahead of the node?
        target_node_list = set(self._current_target_node_ids.values())

        return_val = {}

        if len(target_node_list) != len(self._robot_ids):
            # print('Robots moving towards same node.')
            clash_list = self.target_node_to_bot_list()
            # print(self._total_schedule)

            for node, rid_list in clash_list.items():
                clash_horizons = {}         # stores horizons of robots heading towards same node

                for rid in rid_list:
                    clash_horizons[rid] = self._horizon_dict[rid][:, :2]
                    # print(clash_horizons)

                # TODO: this should not be hard coded.
                dist_horizon_array = np.linalg.norm(clash_horizons['A1'][:, None, :] - clash_horizons['A2'][None, :, :], axis=2)
                    
                collision_risk = np.any(dist_horizon_array < ((VEHICLE_WIDTH * 2) + VEHICLE_MARGIN))    # to check if distance between robots becomes close at any point in both horizons
                # collision_risk = np.any(dist < 0.8)

                active_crossing = {}

                if collision_risk:
                    # print(dist_horizon_array)

                    crosser_rid, stop_rids = self.dist_to_node(node, clash_list[node])
                    # print(crosser_rid)

                    for rid in crosser_rid:
                        # print(rid)
                        curr_target_coord = self._graph.get_node_coord(node)
                        next_target_coord = self._graph.get_node_coord(self._remaining_nodes[rid][1])
                        # print(f'ctc: {curr_target_coord}, ntc: {next_target_coord}')

                        # assuming paths are in the same grid as demo, there will be change in coord of only one axis
                        if curr_target_coord[0] == next_target_coord[0]:

                            if next_target_coord[1] - curr_target_coord[1] > 0:
                                new_target_coord = (next_target_coord[0], (curr_target_coord[1] + 4 * VEHICLE_WIDTH))
                                # print(f'new_target_coord: {new_target_coord}')

                            else:
                                new_target_coord = (next_target_coord[0], (curr_target_coord[1] - 3.5 * VEHICLE_WIDTH))
                                # print(f'new_target_coord: {new_target_coord}')

                        else:
                            if next_target_coord[0] - curr_target_coord[0] > 0:
                                new_target_coord = ((curr_target_coord[0] + 3.5 * VEHICLE_WIDTH), new_target_coord[1])
                                # print(f'new_target_coord: {new_target_coord}')

                            else:
                                new_target_coord = ((curr_target_coord[0] - 3.5 * VEHICLE_WIDTH), new_target_coord[1])
                                # print(f'new_target_coord: {new_target_coord}')

                        # print({rid, new_target_coord})

                        # adding data into return dict.
                        return_val[rid] = {'mode': 'crossing', 'target_coord': new_target_coord}

                    # adding data into return dict
                    for rid in stop_rids:
                        return_val[rid] = {'mode': 'stopped', 'target_coord': None}

                    # return_val = self.coord_controlled_bots

        return return_val

    def dist_to_node(self, node, node_travellers):
        node_dist = {}

        for rid in node_travellers:
            dist = math.sqrt(math.pow(abs(self._curr_pose[rid][0] - self._graph.get_node_coord(node)[0]),2) 
                    + math.pow(abs(self._curr_pose[rid][1] - self._graph.get_node_coord(node)[1]),2))
            node_dist[rid] = dist

        min_dist = min(node_dist.values())

        stop_rids = [rid for rid, dist in node_dist.items() if dist != min_dist]
        crosser_rid = [rid for rid, dist in node_dist.items() if dist == min_dist]

        return crosser_rid, stop_rids
