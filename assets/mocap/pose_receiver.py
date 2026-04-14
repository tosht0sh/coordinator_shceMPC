# listens to QTM and forwards pose data to the bots over UDP

import json
import math
import socket
import time

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Bind on the laptop-side data IP so the QTM stream can target this relay.
sock.bind(("192.168.1.9", 5005))
sock.setblocking(False)

# Each vehicle should only be forwarded to its own bot. Sending every vehicle to
# every bot allows one bad mocap stream (for example NaNs for an untracked body)
# to poison the wrong robot's controller.
forward_targets = {
    "duck1": ("192.168.1.11", 5005),
    "duck2": ("192.168.1.12", 5005),
    # "duck3": ("192.168.1.13", 5005),
    # "duck4": ("192.168.1.14", 5005),
}

latest_data = None
period_s = 1.0 / 30.0


def _is_finite_pose(pose: dict) -> bool:
    try:
        values = (float(pose["x"]), float(pose["y"]), float(pose["theta"]))
    except (KeyError, TypeError, ValueError):
        return False
    return all(math.isfinite(value) for value in values)


while True:
    loop_start = time.monotonic()

    while True:
        try:
            packet, sender = sock.recvfrom(4096)
        except BlockingIOError:
            break

        latest_data = json.loads(packet.decode("utf-8"))
        print(f"Received packet from {sender}: {latest_data}")

    if latest_data is not None:
        # Split the combined packet into one per vehicle and forward each robot's
        # pose only to that robot's bot-side receiver.
        for vehicle, pose in latest_data.get("poses", {}).items():
            target = forward_targets.get(vehicle)
            if target is None:
                continue

            if not _is_finite_pose(pose):
                print(f"Skipping non-finite pose for {vehicle}: {pose}")
                continue

            payload = {
                "vehicle": vehicle,
                "x": float(pose["x"]),
                "y": float(pose["y"]),
                "theta": float(pose["theta"]),
            }
            print(f"Forwarding pose for {vehicle} -> {target}: {payload}")
            encoded_payload = json.dumps(payload).encode("utf-8")
            sock.sendto(encoded_payload, target)

    elapsed = time.monotonic() - loop_start
    if elapsed < period_s:
        time.sleep(period_s - elapsed)
