# listens to QTM and forwards pose data to the bots over UDP

import json
import socket
import time

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("192.168.1.9", 5005))
sock.setblocking(False)

forward_targets = (
    ("192.168.1.11", 5005),  # duck1
    ("192.168.1.12", 5005),  # duck2
    # ("192.168.1.13", 5005),  # duck3
    # ("192.168.1.14", 5005),  # duck4
)

latest_data = None
period_s = 1.0 / 30.0

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
        # split data into separate packets for each vehicle
        for vehicle, pose in latest_data.get("poses", {}).items():
            payload = {
                "vehicle": vehicle,
                "x": pose["x"],
                "y": pose["y"],
                "theta": pose["theta"],
            }
            print(f"Forwarding pose for {vehicle} -> {payload}")
            encoded_payload = json.dumps(payload).encode("utf-8")
            for target in forward_targets:
                sock.sendto(encoded_payload, target)

    elapsed = time.monotonic() - loop_start
    if elapsed < period_s:
        time.sleep(period_s - elapsed)
