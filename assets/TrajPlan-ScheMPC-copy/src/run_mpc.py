import os
import json
import pathlib
import datetime
import socket
import time

import numpy as np
import pandas as pd # type: ignore

from basic_motion_model.motion_model import UnicycleModel

from pkg_motion_plan import GlobalPathCoordinator
from pkg_motion_plan import LocalTrajPlanner
from pkg_mpc_tracker import TrajectoryTracker
from pkg_robot.robot import RobotManager

from configs import MpcConfiguration
from configs import CircularRobotSpecification

from visualizer.object import CircularVehicleVisualizer
from visualizer.mpc_plot import MpcPlotInLoop # type: ignore

from coordinator.coordinator import Coordinator

def _env_bool(name, default=False):
    raw = os.getenv(name)
    if raw is None:
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on"}


def _wrap_to_pi(theta):
    return ((theta + np.pi) % (2 * np.pi)) - np.pi

def _load_robot_vehicle_map():
    raw_map = os.getenv("MPC_ROBOT_VEHICLE_MAP", "").strip()
    if not raw_map:
        return {}
    try:
        loaded = json.loads(raw_map)
    except json.JSONDecodeError:
        print("[MPC] MPC_ROBOT_VEHICLE_MAP is not valid JSON. Ignoring mapping.")
        return {}

    if not isinstance(loaded, dict):
        print("[MPC] MPC_ROBOT_VEHICLE_MAP must be a JSON object. Ignoring mapping.")
        return {}
    return {str(k): str(v) for k, v in loaded.items()}


class UdpPoseReceiver:
    """Collect latest robot poses from UDP packets produced by pose_sender_udp."""
    def __init__(self, bind_ip, bind_port, stale_timeout=0.5):
        self._stale_timeout = max(0.0, float(stale_timeout))
        self._latest_pose = {}
        self._last_vehicle = None
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((bind_ip, int(bind_port)))
        self._sock.setblocking(False)

    def poll(self, max_packets=64):
        for _ in range(max_packets):
            try:
                packet, _ = self._sock.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError as e:
                print(f"[MPC] UDP pose receive failed: {e}")
                break

            try:
                data = json.loads(packet.decode("utf-8"))
                vehicle = str(data["vehicle"])
                x = float(data["x"])
                y = float(data["y"])
                theta = _wrap_to_pi(float(data["theta"]))
            except (KeyError, TypeError, ValueError, json.JSONDecodeError):
                continue

            self._latest_pose[vehicle] = (np.asarray([x, y, theta], dtype=float), time.monotonic())
            self._last_vehicle = vehicle

    def get_state(self, vehicle=None):
        if not self._latest_pose:
            return None

        selected_vehicle = vehicle if vehicle in self._latest_pose else self._last_vehicle
        if selected_vehicle is None:
            return None

        state, stamp = self._latest_pose[selected_vehicle]
        age = time.monotonic() - stamp
        if age > self._stale_timeout:
            return None
        return state.copy()

    def close(self):
        self._sock.close()

def run_mpc(EnvFolder, naive_tracker=False, ignore_speed_ref=False, recording=False):

    DATA_NAME = "schedule_demo2_data" # "schedule_demo_data"
    CFG_FNAME = "mpc_fast_sim.yaml" # "mpc_default.yaml" or "mpc_fast.yaml"
    MAP_ONLY = True
    AUTORUN = True # if false, press key (in the plot window) to continue
    MONITOR_COST = False # if true, monitor the cost (this will slow down the simulation)
    VERBOSE = True
    TIMEOUT = 10000
    
    robot_vehicle_map = _load_robot_vehicle_map() # TODO: what is this line doing exactly?

    if recording:
        save_video_path = f'./Demo/{DATA_NAME}_{datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}.mp4'
    else:
        save_video_path = None

    root_dir = pathlib.Path(__file__).resolve().parents[1]
    data_dir = os.path.join(root_dir, "data", DATA_NAME)
    cnfg_dir = os.path.join(root_dir, "config")

    robot_ids = None # if none, read from schedule

    ### Configurations
    config_mpc_path = os.path.join(cnfg_dir, CFG_FNAME)
    config_robot_path = os.path.join(cnfg_dir, "robot_spec.yaml")

    config_mpc = MpcConfiguration.from_yaml(config_mpc_path)
    config_robot = CircularRobotSpecification.from_yaml(config_robot_path)

    ### Map, graph, and schedule paths
    map_path = os.path.join(data_dir, f"{EnvFolder}/map.json")
    graph_path = os.path.join(data_dir, f"{EnvFolder}/graph.json")

    ## Load schedule of orignal problem
    # schedule_path = os.path.join(data_dir, "schedule.csv")
    # start_path = os.path.join(data_dir, "robot_start.json")

    ## Load schedule of SingleRobot
    # schedule_path = os.path.join(data_dir, "schedule_SingleRobot.csv")
    # start_path = os.path.join(data_dir, "robot_start_SingleRobot.json")

    ## Load schedule of TwoRobots
    # schedule_path = os.path.join(data_dir, "schedule_TwoRobots.csv")
    # start_path = os.path.join(data_dir, "robot_start_TwoRobots.json")

    ## Load schedule of CoordScene1
    schedule_path = os.path.join(data_dir, "schedule_CoordScene1.csv")
    start_path = os.path.join(data_dir, "robot_start_CoordScene1.json")

    ## Open schedule
    with open(start_path, "r") as f:
        robot_starts = json.load(f)

    ### Set up the global path/schedule coordinator
    gpc = GlobalPathCoordinator.from_csv(schedule_path)
    gpc.load_graph_from_json(graph_path)
    gpc.load_map_from_json(map_path, inflation_margin=config_robot.vehicle_width+config_robot.vehicle_margin)
    robot_ids = gpc.robot_ids if robot_ids is None else robot_ids
    boundary_coords = gpc.current_map.boundary_coords
    static_obstacles = gpc.inflated_map.obstacle_coords_list

    ### Set up for coodinator
    coord = Coordinator.from_csv(schedule_path)
    coord.load_graph_from_json(graph_path)

    coord_shifted_targets = {}

    ### Set up robots
    robot_manager = RobotManager()
    for rid in robot_ids:
        robot = robot_manager.create_robot(config_robot, UnicycleModel(sampling_time=config_robot.ts), rid)
        robot.set_state(np.asarray(robot_starts[str(rid)]))
        planner = LocalTrajPlanner(config_mpc.ts, config_mpc.N_hor, config_robot.lin_vel_max, verbose=VERBOSE)
        planner.load_map(gpc.inflated_map.boundary_coords, gpc.inflated_map.obstacle_coords_list)
        controller = TrajectoryTracker(config_mpc, config_robot, robot_id=rid, verbose=VERBOSE)
        controller.load_motion_model(UnicycleModel(sampling_time=config_mpc.ts))
        controller.set_monitor(monitor_on=MONITOR_COST)
        visualizer = CircularVehicleVisualizer(config_robot.vehicle_width, indicate_angle=True)
        robot_manager.add_robot(robot, controller, planner, visualizer)

        path_coords, path_times = gpc.get_robot_schedule(rid)
        robot_manager.add_schedule(rid, np.asarray(robot_starts[str(rid)]), path_coords, path_times)

    ### Run
    map_width  = max(np.asarray(boundary_coords)[:, 0]) - min(np.asarray(boundary_coords)[:, 0])
    map_height = max(np.asarray(boundary_coords)[:, 1]) - min(np.asarray(boundary_coords)[:, 1])
    save_params = {'skip_frame': 0, 'frame_size': (1280, int(map_height/map_width * 1280)), 'dpi': 300}
    main_plotter = MpcPlotInLoop(config_robot, map_only=MAP_ONLY, fig_ratio=(map_width/map_height), save_to_path=save_video_path, save_params=save_params)
    # main_plotter.plot_in_loop_pre(gpc.current_map, gpc.inflated_map, gpc.current_graph)
    main_plotter.plot_in_loop_pre(gpc.current_map, graph_manager=gpc.current_graph)
    color_list = [
        "#0072B2", "#D55E00", "#009E73", "#F0E442", "#56B4E9",
        "#E69F00", "#CC79A7", "#0072B2", "#D55E00", "#009E73",
        "#F0E442", "#56B4E9", "#E69F00", "#CC79A7"
    ]
    for i, rid in enumerate(robot_ids):
        planner = robot_manager.get_planner(rid)
        controller = robot_manager.get_controller(rid)
        visualizer = robot_manager.get_visualizer(rid)
        main_plotter.add_object_to_pre(rid,
                                       None, #planner.ref_traj,
                                       controller.state,
                                       controller.final_goal,
                                       color=color_list[i])
        visualizer.plot(main_plotter.map_ax, *robot.state)

    actual_timetable = {rid: [] for rid in robot_ids}
    udp_state_reader = None
    live_pose_announced = set()
    missing_map_announced = set()
    idle_stop_sent = set()      # to stop idle state sending 0 speed multiple times


    for kt in range(TIMEOUT):
        if kt == 2:                         # this is added as for some reason in the first step it always adds all robots as crossing node
            coord_shifted_targets.clear()

        if udp_state_reader is not None:
            udp_state_reader.poll()

        robot_states = []
        incomplete = False

        for i, rid in enumerate(robot_ids):
            # if rid != 'A1':
            #     continue
            robot = robot_manager.get_robot(rid)
            planner = robot_manager.get_planner(rid)
            controller = robot_manager.get_controller(rid)
            visualizer = robot_manager.get_visualizer(rid)
            other_robot_states = robot_manager.get_other_robot_states(rid, config_mpc)
            using_live_state = False

            if controller.idle:
                duck_payload = json.dumps({"v": 0.0, "w": 0.0}) + "\n"
                #sock.sendall(duck_payload.encode("utf-8"))
                main_plotter.update_plot(rid, kt, 0, None, 0, None, None)
                continue
            
            ref_states, ref_speed, *_ = planner.get_local_ref(
                kt*config_mpc.ts, 
                (float(robot.state[0]), float(robot.state[1])), 
                idx_check_range=5,
                ignore_speed_ref=ignore_speed_ref
            )

            

            if kt > 2 and rid in coord_shifted_targets.keys():
                # print(f'{rid}: {planner._base_traj_target_node}')

                crossing_planner = LocalTrajPlanner(
                    config_mpc.ts,
                    config_mpc.N_hor,
                    config_robot.lin_vel_max,
                    verbose=VERBOSE,
                )
                crossing_planner.load_map(gpc.inflated_map.boundary_coords, gpc.inflated_map.obstacle_coords_list)

                current_xy = (float(robot.state[0]), float(robot.state[1]))
                shifted_xy = (float(coord_shifted_targets[rid][0]), float(coord_shifted_targets[rid][1]))
                path_coords = [current_xy, shifted_xy]

                base_path = planner._ref_path
                base_idx = planner._current_target_node_idx
                if base_path is not None and base_idx is not None and base_idx + 1 < len(base_path):
                    resume_xy = tuple(base_path[base_idx + 1])
                    if resume_xy != shifted_xy:
                        path_coords.append(resume_xy)

                crossing_planner.load_path(
                    path_coords,
                    None,
                    nomial_speed=config_robot.lin_vel_max,
                    method="linear",
                )

                print(crossing_planner._base_traj_target_node)

                robot_manager.set_planner(rid, crossing_planner)



            coord.update_horizon(rid, ref_states)
            coord.update_target_nodes(rid, gpc.get_node_id(planner._current_target_node))
            coord.update_curr_pose(rid, robot.state[:2])

            clash = coord.validate()
            # print(clash)

            if rid in clash:
                # print(clash[rid])
                if clash[rid]['mode'] == 'stopped':
                    print(f'Coordinator stopping {rid}')
                    actions = [np.array([0.0, 0.0])]
                    pred_states = np.array([robot.state.copy()] * config_mpc.N_hor)
                    current_refs = ref_states
                    debug_info = {"cost": 0.0, "step_runtime": 0.0, "monitored_cost": None}

                if clash[rid]['mode'] == 'crossing':
                    print(f'{rid} crossing node with shifterd target coords ')

                    if rid not in coord_shifted_targets.keys():
                        coord_shifted_targets.update({rid: clash[rid]['target_coord']})
                        # planner._current_target_node = coord_shifted_targets[rid]

                        print(f"(K:{kt}) Robot {rid}, ref speed: {round(ref_speed if ref_speed else -1, 4)}, next goal:{coord_shifted_targets[rid]}") # XXX
                        controller.set_current_state(robot.state)
                        controller.set_ref_states(ref_states, ref_speed=ref_speed)
                        # print(f"Robot_state {robot.state[0]}" )
                    
                        if naive_tracker:
                            (actions, pred_states, current_refs, debug_info) = controller.run_naive_step()
                        else:
                            (actions, pred_states, current_refs, debug_info) = controller.run_step(static_obstacles=static_obstacles,
                                                                        full_dyn_obstacle_list=None,
                                                                        other_robot_states=other_robot_states,
                                                                        map_updated=True, report_cost=False, ignore_speed_ref=ignore_speed_ref)

            else:
                print(f"(K:{kt}) Robot {rid}, ref speed: {round(ref_speed if ref_speed else -1, 4)}, next goal:{planner._current_target_node}") # XXX
                controller.set_current_state(robot.state)
                controller.set_ref_states(ref_states, ref_speed=ref_speed)
                
            
                if naive_tracker:
                    (actions, pred_states, current_refs, debug_info) = controller.run_naive_step()
                else:
                    (actions, pred_states, current_refs, debug_info) = controller.run_step(static_obstacles=static_obstacles,
                                                                full_dyn_obstacle_list=None,
                                                                other_robot_states=other_robot_states,
                                                                map_updated=True, report_cost=False, ignore_speed_ref=ignore_speed_ref)
        

            print(coord_shifted_targets)
            # print(f"Robot_state {robot.state[:2]}" )

            controller.report_cost(debug_info['cost'],
                                    debug_info['step_runtime'],
                                    debug_info['monitored_cost'],
                                    object_id=f"Robot {rid}")

            if not actual_timetable[rid] or actual_timetable[rid][-1][1] != gpc.get_node_id(planner._current_target_node):
                actual_timetable[rid].append((kt*config_mpc.ts, gpc.get_node_id(planner._current_target_node)))
            else: # overwrite the time
                actual_timetable[rid][-1] = (kt*config_mpc.ts, gpc.get_node_id(planner._current_target_node))

            ### Real run
            # if (np.linalg.norm(robot.state[:2] - current_refs[-1][:2]) > 0.3):
            if (not using_live_state) and (np.linalg.norm(robot.state[:2] - current_refs[-1][:2]) > 0.3):
                if controller._mode != 'safe' or (np.linalg.norm(robot.state[:2] - current_refs[-1][:2]) > 0.8) or planner.idle:
                    robot.step(actions[-1])
            robot_manager.set_pred_states(rid, np.asarray(pred_states))

            main_plotter.update_plot(rid, kt, actions[-1], None, debug_info['cost'], np.asarray(pred_states), current_refs)
            visualizer.update(*robot.state)

            if not controller.check_termination_condition(external_check=planner.idle):
                incomplete = True

                robot_states.append(robot.state)

            main_plotter.plot_in_loop(time=kt*config_mpc.ts, autorun=AUTORUN, zoom_in=None)
            if not incomplete:
                break


    main_plotter.show()
    input('Press anything to finish!')
    main_plotter.close()
    if udp_state_reader is not None:
        udp_state_reader.close()

    # Convert actual_timetable to DataFrame and save to CSV
    rows = []
    for rid, schedule in actual_timetable.items():
        for time, node_id in schedule:
            rows.append({
                'robot_id': rid,
                'node_id': node_id,
                'ETA': time
            })
    actual_df = pd.DataFrame(rows)
    actual_df = actual_df.sort_values(['robot_id', 'ETA'])
    actual_schedule_path = os.path.join(data_dir, "Actual_4Small.csv")
    actual_df.to_csv(actual_schedule_path, index=False)
    print(f"Actual schedule saved to: {actual_schedule_path}")

    if MONITOR_COST: # XXX
        import matplotlib.pyplot as plt # type: ignore
        fig, ax = plt.subplots(1, 1)
        solve_time = controller.solver_time_timelist
        ax.plot(solve_time, label="Solve time")
        ax.set_title(f"Solve time for Robot {rid}")
        ax.legend()
        plt.show()

    return None
