"""Stream Qualisys 6DoF bodies and forward planar pose to Duckiebots over UDP."""

import asyncio
import json
import math
import os
import socket
import xml.etree.ElementTree as ET
from typing import Dict, Iterable, Tuple

import pkg_resources
import qtm

QTM_FILE = pkg_resources.resource_filename("qtm", "data/Demo.qtm")


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


def _load_targets() -> Dict[str, Tuple[str, int]]:
    raw = os.getenv("MOCAP_TARGETS_JSON", "").strip()
    if raw:
        loaded = json.loads(raw)
        targets = {}
        for vehicle, endpoint in loaded.items():
            if not isinstance(endpoint, dict):
                raise ValueError(f"Target for '{vehicle}' must be an object with host/port.")
            host = str(endpoint["host"])
            port = int(endpoint["port"])
            targets[str(vehicle)] = (host, port)
        return targets

    wanted_body = os.getenv("MOCAP_WANTED_BODY", "duck2").strip()
    target_host = os.getenv("MOCAP_TARGET_HOST", "192.168.1.12").strip() # Bot ip
    target_port = int(os.getenv("MOCAP_TARGET_PORT", "5010"))
    return {wanted_body: (target_host, target_port)}


def _flatten_rotation(rotation: object) -> Iterable[float]:
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
    values = list(_flatten_rotation(rotation))
    if len(values) != 9:
        raise ValueError(f"Expected 9 rotation-matrix values, got {len(values)}")

    # Assumes a standard 3x3 row-major rotation matrix for a Z-up world.
    return math.atan2(values[3], values[0])


def _planar_pose(position, rotation, pos_scale: float):
    x = float(position.x) * pos_scale
    y = float(position.y) * pos_scale
    theta = _yaw_from_rotation(rotation)
    return x, y, theta


async def main():
    qtm_host = os.getenv("QTM_HOST", "192.168.1.8")
    qtm_password = os.getenv("QTM_PASSWORD", "posequack")
    realtime = _env_bool("QTM_REALTIME", True)
    pos_scale = float(os.getenv("MOCAP_POS_SCALE", "0.001"))
    targets = _load_targets()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    connection = await qtm.connect(qtm_host)
    if connection is None:
        print(f"Failed to connect to QTM at {qtm_host}")
        return

    print(f"Connected to QTM at {qtm_host}")
    print(f"Streaming mocap for vehicles: {', '.join(sorted(targets))}")

    async with qtm.TakeControl(connection, qtm_password):
        if realtime:
            await connection.new() # Start new realtime
        else:
            await connection.load(QTM_FILE) # Load tqm file
            await connection.start(rtfromfile=True) # Start rtfromfile

    xml_string = await connection.get_parameters(parameters=["6d"])
    body_index = create_body_index(xml_string)

    print(f"{body_enabled_count(xml_string)} of {len(body_index)} 6DoF bodies enabled")

    def on_packet(packet):
        info, bodies = packet.get_6d()
        packet_time = asyncio.get_event_loop().time()

        for vehicle, endpoint in targets.items():
            if vehicle not in body_index:
                continue

            wanted_index = body_index[vehicle]
            if wanted_index >= info.body_count:
                continue

            position, rotation = bodies[wanted_index]
            try:
                x, y, theta = _planar_pose(position, rotation, pos_scale)
            except (TypeError, ValueError) as exc:
                print(f"Skipping body '{vehicle}' due to rotation parse error: {exc}")
                continue

            payload = {
                "vehicle": vehicle,
                "stamp": packet_time,
                "x": x,
                "y": y,
                "theta": theta,
            }
            print(
                f"{vehicle} -> {endpoint[0]}:{endpoint[1]} | "
                f"x={x:.3f} m, y={y:.3f} m, theta={theta:.3f} rad"
            )
            sock.sendto(json.dumps(payload).encode("utf-8"), endpoint)

    try:
        await connection.stream_frames(components=["6d"], on_packet=on_packet)
        while True:
            await asyncio.sleep(1.0)
    finally:
        await connection.stream_frames_stop()
        sock.close()


if __name__ == "__main__":
    asyncio.get_event_loop().run_until_complete(main())
