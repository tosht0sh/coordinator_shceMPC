"""Shared JSON packet schema for laptop <-> bot communication.

The scheduler sends map/schedule packets to the bot over TCP.
The bot sends telemetry/status packets back to the laptop over UDP.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from typing import Any, Optional, Union


JsonDict = dict[str, Any]
PacketType = Union["MapPacket", "SchedulePacket", "TelemetryPacket", "StatusPacket"]



def _float_list(values: list[Any]) -> list[float]:
    return [float(v) for v in values]



def _float_2d(points: list[Any]) -> list[list[float]]:
    return [_float_list(list(point)) for point in points]



def _float_3d(groups: list[Any]) -> list[list[list[float]]]:
    return [_float_2d(list(group)) for group in groups]


@dataclass
class MapPacket:
    """Map payload sent from laptop to bot for onboard replanning."""

    map_id: str
    boundary_coords: list[list[float]]
    static_obstacles: list[list[list[float]]]
    kind: str = field(init=False, default="map_update")

    def to_payload(self) -> JsonDict:
        return asdict(self)

    @classmethod
    def from_payload(cls, payload: JsonDict) -> "MapPacket":
        return cls(
            map_id=str(payload.get("map_id", "default")),
            boundary_coords=_float_2d(payload.get("boundary_coords", [])),
            static_obstacles=_float_3d(payload.get("static_obstacles", [])),
        )


@dataclass
class SchedulePacket:
    """Robot-specific path and timing payload sent from laptop to bot."""

    robot_id: str
    schedule_id: str
    start_state: list[float]
    path_coords: list[list[float]]
    path_times: Optional[list[float]]
    effective_from: float = 0.0
    kind: str = field(init=False, default="schedule_update")

    def to_payload(self) -> JsonDict:
        return asdict(self)

    @classmethod
    def from_payload(cls, payload: JsonDict) -> "SchedulePacket":
        path_times = payload.get("path_times")
        return cls(
            robot_id=str(payload["robot_id"]),
            schedule_id=str(payload.get("schedule_id", "schedule-default")),
            start_state=_float_list(payload["start_state"]),
            path_coords=_float_2d(payload["path_coords"]),
            path_times=None if path_times is None else _float_list(path_times),
            effective_from=float(payload.get("effective_from", 0.0)),
        )


@dataclass
class TelemetryPacket:
    """Bot-to-laptop runtime state used for monitoring and comparison."""

    robot_id: str
    schedule_id: str
    t: float
    pose: list[float]
    action: list[float]
    pred_states: list[list[float]]
    current_refs: list[list[float]]
    current_target_node: Optional[list[float]]
    cost: Optional[float] = None
    solver_time: Optional[float] = None
    status: str = "running"
    kind: str = field(init=False, default="telemetry")

    def to_payload(self) -> JsonDict:
        return asdict(self)

    @classmethod
    def from_payload(cls, payload: JsonDict) -> "TelemetryPacket":
        target_node = payload.get("current_target_node")
        return cls(
            robot_id=str(payload["robot_id"]),
            schedule_id=str(payload.get("schedule_id", "schedule-default")),
            t=float(payload.get("t", 0.0)),
            pose=_float_list(payload.get("pose", [])),
            action=_float_list(payload.get("action", [])),
            pred_states=_float_2d(payload.get("pred_states", [])),
            current_refs=_float_2d(payload.get("current_refs", [])),
            current_target_node=None if target_node is None else _float_list(target_node),
            cost=None if payload.get("cost") is None else float(payload["cost"]),
            solver_time=None if payload.get("solver_time") is None else float(payload["solver_time"]),
            status=str(payload.get("status", "running")),
        )


@dataclass
class StatusPacket:
    """Small status/event packet sent by the bot for visibility and debugging."""

    robot_id: str
    level: str
    message: str
    t: float = 0.0
    schedule_id: str = "schedule-default"
    kind: str = field(init=False, default="status")

    def to_payload(self) -> JsonDict:
        return asdict(self)

    @classmethod
    def from_payload(cls, payload: JsonDict) -> "StatusPacket":
        return cls(
            robot_id=str(payload.get("robot_id", "unknown")),
            level=str(payload.get("level", "info")),
            message=str(payload.get("message", "")),
            t=float(payload.get("t", 0.0)),
            schedule_id=str(payload.get("schedule_id", "schedule-default")),
        )



def packet_from_payload(payload: JsonDict) -> PacketType:
    kind = str(payload.get("kind", ""))
    if kind == "map_update":
        return MapPacket.from_payload(payload)
    if kind == "schedule_update":
        return SchedulePacket.from_payload(payload)
    if kind == "telemetry":
        return TelemetryPacket.from_payload(payload)
    if kind == "status":
        return StatusPacket.from_payload(payload)
    raise ValueError(f"Unsupported packet kind: {kind}")



def packet_from_json(raw: str) -> PacketType:
    return packet_from_payload(json.loads(raw))



def packet_to_json(packet: PacketType) -> str:
    return json.dumps(packet.to_payload(), separators=(",", ":"))



def packet_to_wire(packet: PacketType) -> bytes:
    return (packet_to_json(packet) + "\n").encode("utf-8")
