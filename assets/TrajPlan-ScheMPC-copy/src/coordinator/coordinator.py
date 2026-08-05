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
from pkg_sche.sp_comsat.Compo_slim import Compo_slim
from coordinator.local_replanning import repair_robot_path

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
        self._pending_replans = {}          # replans for scenario 3

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

            self._routes = None
            self._jobs_list = None
            self._remaining_task_ids = None

            # task tracking
            self._current_job = {}

            self.dummy_mode = True
            self.dummy_data = None

            if self.dummy_mode:
                dummy_path = pathlib.Path(__file__).with_name(
                    "dummy_data.json"
                )

                with dummy_path.open(
                    "r",
                    encoding="utf-8",
                ) as read_file:
                    self.dummy_data = json.load(read_file)
                
                # print(self.dummy_data)


    @classmethod
    def from_csv(cls, csv_path: str, csv_sep:str=','):
        """Load the total schedule from a csv file."""
        total_schedule = pd.read_csv(csv_path, sep=csv_sep, header=0)
        return cls(total_schedule)


    def load_graph_from_json(self, graph_path):
        self._graph = NetGraph.from_json(graph_path)

        self.add_target_coords()

    def save_initial_route(self, routes):
        self._routes = routes

        if self.dummy_mode:
            self._routes = self.dummy_data["initial_routes"]

        print(f'[coord] Initial routes: {self._routes}')

        self._remaining_task_ids = {
            rid: list(route["tasks"]) for rid, route in self._routes.items()
        }
        for rid, tasks in self._remaining_task_ids.items():
            if tasks and tasks[0].startswith("start_"):
                tasks.pop(0)

            self._current_job[rid] = {
                "job_id": self._remaining_task_ids[rid][0],
                "node": self._jobs_list[rid][tasks[0]]['location']
            }

        # print(f'[coord] Cuurent job: {self._current_job}')
                
        # print(f'[coord] remaining task ids: {self._remaining_task_ids}')

    def save_jobs(self, jobs_list):
        self._jobs_list = jobs_list

        if self.dummy_mode:
            self._jobs_list = self.dummy_data["jobs_list"]

        # for rid in self._robot_ids:
        #     print(f'[coord] Jobs for {rid}: {self._jobs_list[rid]}')

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
                        
                if (self._remaining_nodes[robot_id] 
                    and self._remaining_nodes[robot_id][0] 
                        == self._current_target_node_ids[robot_id]):
                    reached_node = self._current_target_node_ids[robot_id]
                    
                    # managing stored values on target change
                    self._remaining_nodes[robot_id].pop(0)
                    self._remaining_schedule[robot_id].pop(0)
                    self._prev_node_ids[robot_id] = self._current_target_node_ids[robot_id]

                    # checking for job completetion
                    if self._prev_node_ids[robot_id] == self._current_job[robot_id]["node"]:
                        self.complete_job(robot_id)

                    pending_replan = self._pending_replans.get(robot_id)

                    if (pending_replan and pending_replan["status"] == "approaching_handoff"
                        and reached_node == pending_replan["handoff_node"]):
                        pending_replan["status"] = "at_handoff"
                        pending_replan["handoff_time"] = self._ts

                        print(f"\n\n[coord] {robot_id} reached replanning handoff "
                                f"{reached_node} at {self._ts}"
                            )

                        pending_replan["replacement"] = repair_robot_path(
                            robot_id, reached_node, pending_replan["blocked_edge"],
                            self._ts, self._jobs_list, self._remaining_task_ids
                        )

                        pending_replan["status"] = "replacement_ready"

                    # updating next node
                    if self._remaining_nodes[robot_id]:
                        self._next_node_ids[robot_id] = self._remaining_nodes[robot_id][0]
                    else:
                        self._next_node_ids[robot_id] = None

                # updating new target node
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

                    for rid in conflict["handoff"]:
                        return_val[rid] = {"mode": "handoff", "target_coord": node}

                else:
                    # next node to travel to after conflicting node [COORDINATOR SCENE 2]
                    for rid_1, rid_2 in combinations(rid_list, 2):
                        if (self._prev_node_ids[rid_1] == self._next_node_ids[rid_2] 
                            and 
                            self._prev_node_ids[rid_2] == self._next_node_ids[rid_1]):
                            # [COORDINATOR SCENE 3]
                            print('[coord] Scene 3 replanning needed')
                            # TODO: discuss what the correct policy to select which robot to replan is. 
                            # currently just deciding to stop first robot and replan the second robot.
                            replan_rid = rid_2
                            wait_rid = rid_1
                            handled_conflict2 = True

                            # lazy reporting for now 
                            # TODO: change this for better reporting later.
                            return_val[wait_rid] = {'mode': 'stopped', 'target_coord': None}
                            return_val[replan_rid] = {'mode': 'handoff', 'target_node': self._current_target_node_ids[replan_rid]}
                            conflict_record = {
                                'mode': 3,
                                'replan': [replan_rid],
                                'handoff': [replan_rid],
                                'crossing': [],
                                'stopped': [wait_rid],
                                'shifted_target': None,
                            }
                            self.active_conflicts[node] = conflict_record
                            self._pending_replans[replan_rid] = {
                                "handoff_node": self._current_target_node_ids[replan_rid],
                                "blocked_edge": (
                                    self._current_target_node_ids[replan_rid],
                                    self._next_node_ids[replan_rid],
                                ),
                                "stopped_robot": wait_rid,
                                "status": "approaching_handoff",
                                "replacement": None,
                            }

                            print(self._pending_replans)
                            # frozen_schedules = self.frozen_schedule_builder(replan_rid)
                            # print(f'[coord] Scene 3 stopped robot: {wait_rid}')
                            # print(f'[coord] Total Schedule: {self._total_schedule}')
                            # print(f'[coord] Frozen Schedules: {frozen_schedules}')

                            # fff = repair_robot_path(replan_rid, self._curr_pose[replan_rid], 
                            #                         self._prev_node_ids[replan_rid], self._current_target_node_ids[replan_rid],
                            #                         self._graph,
                            #                         node, self._ts, 
                            #                         self._jobs_list, self._routes, self._remaining_task_ids,
                            #                         frozen_schedules)

                            # exit()

                        elif self._prev_node_ids[rid_1] == self._next_node_ids[rid_2]:
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
    
    def frozen_schedule_builder(self, replan_rid):
        frozen_schedules = {}

        for rid in self._robot_ids:
            if rid == replan_rid:
                continue

            frozen_schedules[rid] = {
                'previous_node': self._prev_node_ids[rid],
                'current_target_node': self._current_target_node_ids[rid],
                'next_node': self._next_node_ids[rid],
                'remaining_nodes': list(self._remaining_nodes[rid]),
                'remaining_schedule': [{
                    'node_id': node_id,
                    'ETA': float(eta)
                } for node_id, eta in self._remaining_schedule[rid]]
            }
        
        return frozen_schedules

    def complete_job(self, rid):
        if len(self._remaining_task_ids[rid]) > 1:
           self._current_job[rid]["job_id"] = self._remaining_task_ids[rid][1]
           self._current_job[rid]["node"] = self._jobs_list[rid][self._remaining_task_ids[rid][1]]['location']
        else:
            self._current_job[rid]["job_id"] = None
            self._current_job[rid]["node"] = None

        self._remaining_task_ids[rid].pop(0)

    def handoff_reached(self, rid):
        pending_replan = self._pending_replans.get(rid)

        if(pending_replan and pending_replan["status"] == "approaching_handoff"):
            self.update_target_nodes(rid, self._next_node_ids[rid])

    def take_ready_replan(self, rid):
        pending_replan = self._pending_replans.get(rid)

        if (
            pending_replan
            and pending_replan["status"] == "replacement_ready"
        ):
            pending_replan["status"] = "installing"
            return pending_replan["replacement"]

        return None

    def apply_replan_result(self, rid, replacement):
        new_route = replacement["routes"][rid]
        new_schedule = replacement["solution"][rid]
        new_jobs = replacement["jobs_list"][rid]

        self._routes[rid] = new_route
        self._jobs_list[rid] = new_jobs

        # The first node is the handoff node already reached by the robot.
        self._prev_node_ids[rid] = new_route["nodes"][0]

        self._remaining_nodes[rid] = list(
            new_route["nodes"][1:]
        )

        self._remaining_schedule[rid] = [
            (node_id, float(eta))
            for node_id, eta in new_schedule[1:]
        ]

        self._remaining_task_ids[rid] = [
            task_id
            for task_id in new_route["tasks"]
            if not task_id.startswith("start_")
        ]

        self._current_target_node_ids[rid] = (
            self._remaining_nodes[rid][0]
        )

        if len(self._remaining_nodes[rid]) > 1:
            self._next_node_ids[rid] = (
                self._remaining_nodes[rid][1]
            )
        else:
            self._next_node_ids[rid] = None

        self._pending_replans[rid]["status"] = "clearing"


# TODO:
# 1. [DONE]coordinator scene 3 - position
# 1. coordinator scene 3 - selection policy?
# 2. coordinator scene 3 - setup: 
#       [DONE]add loading from dummy_data.json 
#       [DONE]send data to local_replanning.py 
#       [DONE]build problem and correct data imports for replanning 
#       [DONE]send problem to compo slim 
#       [DONE]stop the not replanned robot and make the replan robot earch target
#       integrate new path of robot and resume the stopped robot 
#       ensure that mpc planner is doing correct work
# 3. coordinator scene 3 - testing
# 4. test scene 1 usage on the big demo
