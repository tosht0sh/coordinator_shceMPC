import os
import json
import pathlib
import datetime
import socket
import time
import math

import numpy as np
import pandas as pd
from itertools import combinations

from basic_map.graph import NetGraph

VEHICLE_WIDTH = 0.2
VEHICLE_MARGIN = 0.1

class Coordinator:
    """
    Coordinator to ensure robots do not kill each other
    """

    def __init__(self, total_schedule=None):
        self._total_schedule = total_schedule
        self._robot_ids = total_schedule['robot_id'].unique().tolist()
        self._graph = None

        self._horizon_dict = {}                # dictionary that stores horizon of all robots
        self._current_target_node_ids = {}     # dictionary that stores current target node id of all robots
        self._prev_node_ids = {}               # dictionary that stores previous node id of all robots
        self._next_node_ids = {}               # dictionary that stores next target node id of all robots

        self._remaining_nodes = {}
        self._remaining_nodes_coords = {}
        self._remaining_schedule = {}

        self._ts = 0.0

        self._curr_pose = {}

        self.coord_controlled_bots = {
            'stopped': set(),
            'crossing': {}
        }

        self.active_conflicts = {}

        for rid in self._robot_ids:
            robot_schedule = self._total_schedule[self._total_schedule["robot_id"] == rid]
            self._remaining_nodes[rid] = robot_schedule["node_id"].tolist()

            self._remaining_schedule[rid] = list(robot_schedule[["node_id", "ETA"]].itertuples(
                index=False,
                name=None
            ))

            self._prev_node_ids[rid] = self._remaining_nodes[rid][0]
            if len(self._remaining_nodes[rid]) > 2:
                self._next_node_ids[rid] = self._remaining_nodes[rid][2]
            else:
                self._next_node_ids[rid] = None

            self._remaining_nodes[rid].pop(0)
            self._remaining_schedule[rid].pop(0)

            


    @classmethod
    def from_csv(cls, csv_path: str, csv_sep:str=','):
        """Load the total schedule from a csv file."""
        total_schedule = pd.read_csv(csv_path, sep=csv_sep, header=0)
        return cls(total_schedule)


    def load_graph_from_json(self, graph_path):
        self._graph = NetGraph.from_json(graph_path)

        self.add_target_coords()

    def add_target_coords(self):
        for rid in self._remaining_nodes:
            self._remaining_nodes_coords[rid] = []
            for node in self._remaining_nodes[rid]:
                self._remaining_nodes_coords[rid].append(self._graph.get_node_coord(node))


    def update_horizon(self, robot_id, ref_states):
        """ Update horizion dict based on new robot states """

        self._horizon_dict[robot_id] = ref_states   


    def update_target_nodes(self, robot_id, target_node):
        
        if len(self._current_target_node_ids) == len(self._robot_ids):
            
            if self._current_target_node_ids[robot_id] != target_node:
                for node, conflicts in list(self.active_conflicts.items()):
                    if robot_id in conflicts['crossing'] and target_node != node:
                        self.active_conflicts.pop(node)
                        
                if self._remaining_nodes[robot_id] and self._remaining_nodes[robot_id][0] == self._current_target_node_ids[robot_id]:
                    self._remaining_nodes[robot_id].pop(0)
                    self._remaining_schedule[robot_id].pop(0)
                    self._prev_node_ids[robot_id] = self._current_target_node_ids[robot_id]
                    if self._remaining_nodes[robot_id]:
                        self._next_node_ids[robot_id] = self._remaining_nodes[robot_id][0]
                    else:
                        self._next_node_ids[robot_id] = None

                self._current_target_node_ids[robot_id] = target_node

        else:
            self._current_target_node_ids[robot_id] = target_node


    def update_curr_pose(self, robot_id, pose):
        self._curr_pose[robot_id] = pose

    def update_time(self, ts):
        self._ts = ts


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
        # print(f'\n[coord] schedule list:{self._remaining_schedule}')

        # check if robots are heading towards the same node [COORDINATOR SCENE 1]
        target_node_list = set(self._current_target_node_ids.values())

        return_val = {}

        if len(target_node_list) != len(self._robot_ids):
            clash_list = self.target_node_to_bot_list()

            for node, rid_list in clash_list.items():
                collision_risk = False
                handled_conflict2 = False

                if node in self.active_conflicts.keys():
                    # give the same command outputs as stored
                    conflict = self.active_conflicts[node]

                    for rid in conflict['crossing']:
                        return_val[rid] = {'mode': 'crossing', 'target_coord': conflict['shifted_target']}

                    for rid in conflict['stopped']:
                        return_val[rid] = {'mode': 'stopped', 'target_coord': None}

                else:
                    # next node to travel to after conflicting node [COORDINATOR SCENE 2]
                    for rid_1, rid_2 in combinations(rid_list, 2):
                        if self._prev_node_ids[rid_1] == self._next_node_ids[rid_2]:
                            print('[coord] Scene 2 conflict occuring 2-->1')
                            return_val, conflict_record = self.scene2_handle(rid_1, rid_2)
                            handled_conflict2 = True
                            self.active_conflicts[node] = conflict_record

                        elif self._prev_node_ids[rid_2] == self._next_node_ids[rid_1]:
                            print('[coord] Scene 2 conflict occuring 1-->2')
                            return_val, conflict_record = self.scene2_handle(rid_2, rid_1)
                            handled_conflict2 = True
                            self.active_conflicts[node] = conflict_record

                    if not handled_conflict2:
                        clash_horizons = {}         # stores horizons of robots heading towards same node

                        for rid in rid_list:
                            clash_horizons[rid] = self._horizon_dict[rid][:, :2]

                        for rid_1, rid_2 in combinations(rid_list, 2):
                            dist_horizon_array = np.linalg.norm(
                                clash_horizons[rid_1][:, None, :] - clash_horizons[rid_2][None, :, :],
                                axis=2
                            )

                            # to check if distance between robots becomes close at any point in both horizons
                            if np.any(dist_horizon_array < ((VEHICLE_WIDTH * 2) + VEHICLE_MARGIN)):
                                collision_risk = True
                                break

                        if collision_risk:
                            # checking for ETA delays
                            delayed_rids = []
                            on_time_etas = {}
                            for rid in rid_list:
                                schedule_row = self._total_schedule[
                                    (self._total_schedule["robot_id"] == rid)
                                    & (self._total_schedule["node_id"] == node)
                                ]

                                next_node, eta = self._remaining_schedule[rid][0]

                                if self._ts > eta:
                                    delayed_rids.append(rid)
                                else:
                                    on_time_etas[rid] = eta

                            if delayed_rids and on_time_etas:
                                crosser = min(on_time_etas, key=lambda rid: (on_time_etas[rid], rid))
                                crosser_rid = [crosser]
                                stop_rids = [rid for rid in rid_list if rid != crosser]
                            else: # if no delays or all robots delayed, then closest to node passes
                                crosser_rid, stop_rids = self.dist_to_node(node, clash_list[node])


                            for rid in crosser_rid:
                                if len(self._remaining_nodes[rid]) < 2:
                                    continue
                                new_target_coord = self.coord_shifter(rid)

                                # adding data into return dict.
                                return_val[rid] = {'mode': 'crossing', 'target_coord': new_target_coord}

                            # adding data into return dict
                            for rid in stop_rids:
                                return_val[rid] = {'mode': 'stopped', 'target_coord': None}

                            self.active_conflicts[node] = {
                                'mode': 1,
                                'crossing': crosser_rid,
                                'stopped': stop_rids,
                                'shifted_target': new_target_coord
                            }

        if return_val:
            return return_val
        else:
            return None

    def dist_to_node(self, node, node_travellers):
        node_dist = {}

        for rid in node_travellers:
            dist = math.sqrt(math.pow(abs(self._curr_pose[rid][0] - self._graph.get_node_coord(node)[0]),2) 
                    + math.pow(abs(self._curr_pose[rid][1] - self._graph.get_node_coord(node)[1]),2))
            node_dist[rid] = dist

        min_dist = min(node_dist.values())

        crosser_rid = min(node_dist, key=lambda rid: (node_dist[rid], rid))
        stop_rids = [rid for rid in node_travellers if rid != crosser_rid]

        return [crosser_rid], stop_rids
    
    def scene2_handle(self, rid_1, rid_2):
        scene2_return_dict = {}
        # stop rid_2
        scene2_return_dict[rid_2] = {'mode': 'stopped', 'target_coord': None}

        # move rid_1
        new_target_coord = self.coord_shifter(rid_1)

        scene2_return_dict[rid_1] = {'mode': 'crossing', 'target_coord': new_target_coord}

        conflict_record = {
            'mode': 2,
            'crossing': [rid_1],
            'stopped': [rid_2],
            'shifted_target': new_target_coord,
        }

        return scene2_return_dict, conflict_record
    
    def coord_shifter(self, rid):
        curr_target_coord = self._graph.get_node_coord(self._current_target_node_ids[rid])
        next_target_coord = self._graph.get_node_coord(self._next_node_ids[rid])

        # assuming paths are in the same grid as demo, there will be change in coord of only one axis
        if curr_target_coord[0] == next_target_coord[0]:
            if next_target_coord[1] - curr_target_coord[1] > 0:
                new_target_coord = (next_target_coord[0], (curr_target_coord[1] + 3 * VEHICLE_WIDTH))
            else:
                new_target_coord = (next_target_coord[0], (curr_target_coord[1] - 3 * VEHICLE_WIDTH))
        else:
            if next_target_coord[0] - curr_target_coord[0] > 0:
                new_target_coord = ((curr_target_coord[0] + 3 * VEHICLE_WIDTH), next_target_coord[1])
            else:
                new_target_coord = ((curr_target_coord[0] - 3 * VEHICLE_WIDTH), next_target_coord[1])

        return new_target_coord

        



# TODO:
# 1. [DONE] add stopping prioirity based on ETA - if a robot is delayed, delay it more than making a different robot stop
# 2. [DONE] add coord scene 2 code
# 3. [DONE] fix tesing on current scene 1 basic scenario
# 4. test scene 1 usage on the big demo
