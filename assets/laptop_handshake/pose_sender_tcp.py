#!/usr/bin/env python3

"""
Purpose:
  Send target pose commands (x, y, theta) from laptop to robot over TCP.

Transport:
- TCP client to <robot-ip>:<robot-port>
- Payload format: JSON line {"x": ..., "y": ..., "theta": ...}
"""

import argparse
import json
import socket
import time


def parse_args():
    parser = argparse.ArgumentParser(description="Send x,y,theta command pose to a robot over TCP.")
    parser.add_argument("--robot-ip", required=True, help="Robot IP address")
    parser.add_argument("--robot-port", type=int, default=5006, help="Robot TCP port (default: 5006)")
    parser.add_argument("--x", type=float, required=True, help="X coordinate")
    parser.add_argument("--y", type=float, required=True, help="Y coordinate")
    parser.add_argument("--theta", type=float, required=True, help="Heading (rad)")
    parser.add_argument(
        "--rate",
        type=float,
        default=0.0,
        help="Send rate in Hz. 0 sends once and exits.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    payload = json.dumps({"x": args.x, "y": args.y, "theta": args.theta}) + "\n"
    data = payload.encode("utf-8")

    with socket.create_connection((args.robot_ip, args.robot_port), timeout=5.0) as sock:
        if args.rate <= 0.0:
            sock.sendall(data)
            print("Sent one command pose.")
            return

        period = 1.0 / args.rate
        print(f"Streaming command pose at {args.rate:.2f} Hz. Press Ctrl+C to stop.")
        while True:
            sock.sendall(data)
            time.sleep(period)


if __name__ == "__main__":
    main()
