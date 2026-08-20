"""Shared local-planner + MPC control core.

`MpcAgent` wraps the existing TrajPlan robot, planner, and controller objects into
one reusable unit that can run on the bot and also be reused by laptop-side tools.

This is essentialy the new control logic.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple, TypedDict

import numpy as np

try:
    from ._repo_paths import ensure_trajplan_imports
except ImportError:
    from _repo_paths import ensure_trajplan_imports

ensure_trajplan_imports()

from basic_motion_model.motion_model import UnicycleModel
from configs import CircularRobotSpecification, MpcConfiguration
from pkg_motion_plan.local_traj_plan import LocalTrajPlanner
from pkg_mpc_tracker.trajectory_tracker import TrajectoryTracker
from pkg_robot.robot import Robot


PathNode = Tuple[float, float]


class StepResult(TypedDict):
    """Structured result returned from one MPC control step."""

    action: np.ndarray
    pred_states: np.ndarray
    current_refs: np.ndarray
    debug_info: Dict[str, Any]
    current_target_node: Optional[PathNode]
    controller_idle: bool
    planner_idle: bool
    solver_time: Optional[float]


class MpcAgent:
    """Single-robot control wrapper for onboard local planning and MPC.

    Responsibilities:
    - load one robot's map and schedule
    - keep the current robot state synchronized
    - compute one control action and predicted trajectory per step
    """

    def __init__(
        self,
        robot_id: str,
        config_mpc: MpcConfiguration,
        config_robot: CircularRobotSpecification,
        monitor_cost: bool = False,
        verbose: bool = False,
    ) -> None:
        self.robot_id = str(robot_id)
        self.config_mpc = config_mpc
        self.config_robot = config_robot
        self.verbose = verbose
        self.monitor_cost = monitor_cost

        self.motion_model = UnicycleModel(sampling_time=config_robot.ts)
        self.controller_motion_model = UnicycleModel(sampling_time=config_mpc.ts)
        self.robot = Robot(config_robot, self.motion_model, id_=self.robot_id, name=self.robot_id)
        self.planner = LocalTrajPlanner(
            config_mpc.ts,
            config_mpc.N_hor,
            config_robot.lin_vel_max,
            verbose=verbose,
        )
        self.controller = TrajectoryTracker(config_mpc, config_robot, robot_id=self.robot_id, verbose=verbose)
        self.controller.load_motion_model(self.controller_motion_model)
        if monitor_cost:
            self.controller.set_monitor(monitor_on=True)

        self._map_loaded = False
        self._map_refresh_needed = False
        self._schedule_loaded = False
        self._actual_timetable: List[Tuple[float, Optional[str]]] = []
        self._static_obstacles: List[List[PathNode]] = []

    @property
    def state(self) -> np.ndarray:
        return self.robot.state

    @property
    def current_target_node(self) -> Optional[PathNode]:
        if self.planner.idle:
            return None
        return self.planner.current_target_node

    @property
    def actual_timetable(self) -> List[Tuple[float, Optional[str]]]:
        return list(self._actual_timetable)

    def load_map(self, boundary_coords: List[PathNode], static_obstacles: List[List[PathNode]]) -> None:
        self.planner.load_map(boundary_coords, static_obstacles)
        self._static_obstacles = static_obstacles
        self._map_loaded = True
        self._map_refresh_needed = True

    def load_schedule(
        self,
        start_state: np.ndarray,
        path_coords: List[PathNode],
        path_times: Optional[List[float]],
    ) -> None:
        if len(path_coords) < 1:
            raise ValueError("Schedule path must contain at least one coordinate.")

        start_state = np.asarray(start_state, dtype=float)
        self.robot.set_state(start_state)
        self.planner.load_path(path_coords, path_times, nomial_speed=self.config_robot.lin_vel_max, method="linear")

        if len(path_coords) >= 2:
            goal_coord = path_coords[-1]
            goal_coord_prev = path_coords[-2]
            goal_heading = math.atan2(goal_coord[1] - goal_coord_prev[1], goal_coord[0] - goal_coord_prev[0])
        else:
            goal_coord = path_coords[-1]
            goal_heading = float(start_state[2]) if start_state.shape[0] >= 3 else 0.0
        goal_state = np.array([goal_coord[0], goal_coord[1], goal_heading], dtype=float)
        self.controller.load_init_states(start_state, goal_state)
        self._schedule_loaded = True
        self._actual_timetable.clear()

    def apply_shifted_target(self, target_coord: List[float], current_time: float) -> None:
        """Temporarily route through a coordinator-provided shifted target."""

        if not self._schedule_loaded:
            raise RuntimeError("No schedule loaded. Call load_schedule(...) first.")

        current_xy = (float(self.robot.state[0]), float(self.robot.state[1]))
        shifted_xy = (float(target_coord[0]), float(target_coord[1]))
        path_coords: List[PathNode] = [current_xy, shifted_xy]

        base_path = self.planner._ref_path
        base_times = self.planner._ref_path_time
        base_idx = self.planner._current_target_node_idx

        if base_path is not None and base_idx is not None and base_idx + 1 < len(base_path):
            resume_xy = tuple(base_path[base_idx + 1])
            if resume_xy != shifted_xy:
                path_coords.extend(tuple(point) for point in base_path[base_idx + 1:])

        path_times = None
        if base_times is not None and base_idx is not None:
            shifted_eta = max(float(base_times[base_idx]), float(current_time) + self.planner.ts)
            path_times = [float(current_time), shifted_eta]
            path_times.extend(float(time_point) for time_point in base_times[base_idx + 1:])

        self.planner.load_path(
            path_coords,
            path_times,
            nomial_speed=self.config_robot.lin_vel_max,
            method="linear",
        )
        # self._map_refresh_needed = True

    def set_state(self, state: np.ndarray) -> None:
        self.robot.set_state(np.asarray(state, dtype=float))

    def stop_action(self) -> np.ndarray:
        return np.zeros(self.config_mpc.nu, dtype=float)

    def is_idle(self) -> bool:
        return self.controller.idle

    def step(
        self,
        t_now: float,
        other_robot_states: Optional[List[float]] = None,
        ignore_speed_ref: bool = False,
        report_cost: bool = False,
    ) -> StepResult:
        if not self._schedule_loaded:
            raise RuntimeError("No schedule loaded. Call load_schedule(...) first.")

        if self.controller.idle:
            return {
                "action": self.stop_action(),
                "pred_states": np.empty((0, self.config_mpc.ns)),
                "current_refs": np.empty((0, self.config_mpc.ns)),
                "debug_info": {
                    "cost": 0.0,
                    "closest_obstacle_list": [],
                    "step_runtime": 0.0,
                    "monitored_cost": None,
                },
                "current_target_node": self.current_target_node,
                "controller_idle": True,
                "planner_idle": self.planner.idle,
                "solver_time": None,
            }

        ref_states, ref_speed, _ = self.planner.get_local_ref(
            float(t_now),
            (float(self.robot.state[0]), float(self.robot.state[1])),
            idx_check_range=5,
            ignore_speed_ref=ignore_speed_ref,
        )
        self.controller.set_current_state(self.robot.state)
        self.controller.set_ref_states(ref_states, ref_speed=ref_speed)

        actions, pred_states, current_refs, debug_info = self.controller.run_step(
            static_obstacles=self._static_obstacles,
            full_dyn_obstacle_list=None,
            other_robot_states=other_robot_states,
            map_updated=self._map_refresh_needed,
            report_cost=report_cost,
            ignore_speed_ref=ignore_speed_ref,
        )
        self._map_refresh_needed = False

        target_node = self.current_target_node
        solver_time = self.controller.solver_time_timelist[-1] if self.controller.solver_time_timelist else None
        idle_now = self.controller.check_termination_condition(external_check=self.planner.idle)

        return {
            "action": np.asarray(actions[-1], dtype=float),
            "pred_states": np.asarray(pred_states, dtype=float),
            "current_refs": np.asarray(current_refs, dtype=float),
            "debug_info": debug_info,
            "current_target_node": target_node,
            "controller_idle": idle_now,
            "planner_idle": self.planner.idle,
            "solver_time": solver_time,
        }
