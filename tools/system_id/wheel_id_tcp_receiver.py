#!/usr/bin/env python3
from __future__ import annotations

"""Receive wheel-ID CSV rows over TCP and write them to a local CSV file."""

import argparse
import socket
import time
from pathlib import Path


CSV_HEADER = [
    "t",
    "v_cmd",
    "omega_cmd",
    "wL_cmd",
    "wR_cmd",
    "wL_meas",
    "wR_meas",
    "wL_raw",
    "wR_raw",
    "wheel_radius",
    "axle_length",
    "enc_ready",
]
HEADER_LINE = ",".join(CSV_HEADER)


def default_output_path() -> Path:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    root = Path(__file__).resolve().parent
    return root / f"wheel_id_tcp_{stamp}.csv"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Receive wheel-ID rows over TCP and save them to CSV.")
    parser.add_argument("--bind-ip", default="0.0.0.0", help="Interface to listen on")
    parser.add_argument("--port", type=int, default=5015, help="TCP port to listen on")
    parser.add_argument(
        "--output",
        default=str(default_output_path()),
        help="Output CSV path",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Allow overwriting an existing output file",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output_path = Path(args.output).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if output_path.exists() and not args.overwrite:
        raise FileExistsError(
            f"{output_path} already exists. Use a different --output path or pass --overwrite."
        )

    with output_path.open("w", encoding="utf-8", newline="") as out_file:
        header_written = False
        sample_count = 0

        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((args.bind_ip, args.port))
        server.listen(1)

        print(f"[wheel-id-receiver] listening on {args.bind_ip}:{args.port}")
        print(f"[wheel-id-receiver] writing CSV to {output_path}")

        try:
            while True:
                conn, addr = server.accept()
                print(f"[wheel-id-receiver] client connected: {addr[0]}:{addr[1]}")
                with conn:
                    reader = conn.makefile("r", encoding="utf-8", newline="")
                    with reader:
                        for raw_line in reader:
                            line = raw_line.strip()
                            if not line:
                                continue

                            if line == HEADER_LINE:
                                if not header_written:
                                    out_file.write(HEADER_LINE + "\n")
                                    out_file.flush()
                                    header_written = True
                                    print("[wheel-id-receiver] header received")
                                continue

                            if not header_written:
                                out_file.write(HEADER_LINE + "\n")
                                out_file.flush()
                                header_written = True
                                print("[wheel-id-receiver] header missing from stream; wrote default header")

                            out_file.write(line + "\n")
                            out_file.flush()
                            sample_count += 1

                            if sample_count == 1 or sample_count % 25 == 0:
                                print(f"[wheel-id-receiver] samples={sample_count}")

                print("[wheel-id-receiver] client disconnected; waiting for reconnect")
        except KeyboardInterrupt:
            print(f"\n[wheel-id-receiver] stopped after {sample_count} samples")
        finally:
            server.close()


if __name__ == "__main__":
    main()
