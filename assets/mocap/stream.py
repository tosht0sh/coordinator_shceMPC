"""Stream Qualisys 6DoF bodies and forward planar pose to Duckiebots over UDP."""

import asyncio
import contextlib
import json
import math
import os
import socket
import xml.etree.ElementTree as ET
from importlib import resources
from typing import Dict, Iterable, Tuple

import qtm

# Tosh Laptop
TARGET_IP = "192.168.1.10"       # target on which qtm coords are sent to

# Kim Latop
# TARGET_IP = "192.168.1.10"       # target on which qtm coords are sent to





with contextlib.ExitStack() as _resource_stack:
    QTM_FILE = str(
        _resource_stack.enter_context(
            resources.as_file(resources.files("qtm").joinpath("data", "Demo.qtm"))
        )
    )


def _env_bool(name: str, default: bool = False) -> bool:
    raw = os.getenv(name)
    if raw is None:
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on"}


def create_body_index(xml_string):
    """Extract a name-to-index dictionary from 6DoF settings XML."""
    xml = ET.fromstring(xml_string)

    body_to_index = {}
    for index, body in enumerate(xml.findall("*/Body/Name")):
        body_to_index[body.text.strip()] = index

    return body_to_index


def body_enabled_count(xml_string):
    xml = ET.fromstring(xml_string)
    return sum(enabled.text == "true" for enabled in xml.findall("*/Body/Enabled"))


def _bind_targets(
    body_index: Dict[str, int], vehicles: Tuple[str, ...]
) -> Tuple[Tuple[str, int], ...]:
    bindings = []
    for vehicle in vehicles:
        wanted_index = body_index.get(vehicle)
        if wanted_index is None:
            print(f"Skipping target '{vehicle}': body not present in QTM 6DoF settings")
            continue

        bindings.append((vehicle, wanted_index))

    return tuple(bindings)


def _flatten_rotation(rotation: object) -> Iterable[float]:
    """
    converts matrix to flat format
    """
    raw = getattr(rotation, "matrix", rotation)
    if hasattr(raw, "_fields"):
        return [float(getattr(raw, field)) for field in raw._fields]

    if isinstance(raw, (list, tuple)):
        flat = []
        for item in raw:
            if isinstance(item, (list, tuple)):
                flat.extend(float(value) for value in item)
            else:
                flat.append(float(item))
        return flat

    raise TypeError(f"Unsupported rotation payload: {type(rotation)!r}")


def _yaw_from_rotation(rotation: object) -> float:
    """
    extracts yaw from given rotation matrix
    """
    values = list(_flatten_rotation(rotation))
    if len(values) != 9:
        raise ValueError(f"Expected 9 rotation-matrix values, got {len(values)}")

    # Assumes a standard 3x3 row-major rotation matrix for a Z-up world.
    return math.atan2(-values[3], values[0])


def _planar_pose(position, rotation, pos_scale: float):
    """ 
    convert received data to actual coordinates. does the following:
    1. converts x,y from mm to m for use in MPC solver
    2. extracts theta(yaw) from rotation matrix to angle
    """
    x = float(position.x) * pos_scale
    y = float(position.y) * pos_scale
    #print("Rotation matrix: ",rotation[0])
    # raw = getattr(rotation, "matrix", rotation)
    # #print("Raw data ",raw[0])
    # raw2 = math.atan2(-raw[3],raw[0])
    # print("Raw all  ", raw2)
    theta = _yaw_from_rotation(rotation)
    return x, y, theta


async def main():
    qtm_host = "192.168.1.8"
    qtm_password = "posequack"
    realtime = True
    pos_scale = 0.001
    send_hz = 30.0
    target = (TARGET_IP, 5005)
    vehicles = ("duck1", "duck2", "duck4", "duck6")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    connection = await qtm.connect(qtm_host)
    if connection is None:
        print(f"Failed to connect to QTM at {qtm_host}")
        return

    print(f"Connected to QTM at {qtm_host}")

    async with qtm.TakeControl(connection, qtm_password):
        if realtime:
            await connection.new() # Start new realtime
        else: # TODO: Maybe remove this as we only work with realtime streaming
            await connection.load(QTM_FILE) # Load tqm file
            await connection.start(rtfromfile=True) # Start rtfromfile

    
    # Get 6dof settings from qtm
    xml_string = await connection.get_parameters(parameters=["6d"])
    body_index = create_body_index(xml_string)
    bound_targets = _bind_targets(body_index, vehicles)
    latest_payload = None

    print(f"{body_enabled_count(xml_string)} of {len(body_index)} 6DoF bodies enabled")
    print(f"Streaming mocap for vehicles: {', '.join(vehicle for vehicle, _ in bound_targets)}")

    def on_packet(packet):
        nonlocal latest_payload
        info, bodies = packet.get_6d()
        packet_time = asyncio.get_event_loop().time()
        poses = {}

        for vehicle, wanted_index in bound_targets:
            if wanted_index >= info.body_count:
                continue

            position, rotation = bodies[wanted_index]
            try:
                x, y, theta = _planar_pose(position, rotation, pos_scale)  # convert matrixes to actual pose
            except (TypeError, ValueError) as exc:
                print(f"Skipping body '{vehicle}' due to rotation parse error: {exc}")
                continue

            poses[vehicle] = {
                "x": x,
                "y": y,
                "theta": theta,
            }

        latest_payload = {
            "stamp": packet_time,
            "poses": poses,
        }

    async def send_latest():
        period_s = 1.0 / send_hz
        while True:
            if latest_payload is not None:
                print(f"Sending {len(latest_payload['poses'])} poses to {target[0]}:{target[1]}")
                sock.sendto(json.dumps(latest_payload).encode("utf-8"), target)
            await asyncio.sleep(period_s)

    try:
        await connection.stream_frames(components=["6d"], on_packet=on_packet)
        await send_latest()
    finally:
        await connection.stream_frames_stop()
        sock.close()


if __name__ == "__main__":
    asyncio.get_event_loop().run_until_complete(main())
