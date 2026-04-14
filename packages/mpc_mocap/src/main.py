#!/usr/bin/env python3

"""Small supervisor for the merged mocap + MPC runtime package.

This script intentionally launches the pose receiver and the MPC runtime as
separate child processes. Both scripts are ROS nodes, so keeping them in
separate processes avoids `rospy`/node-name conflicts while still giving you
one command to start the full stack.
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from pathlib import Path


THIS_DIR = Path(__file__).resolve().parent
MOCAP_RECEIVER_PATH = THIS_DIR / "mocap_pose_receiver.py"
BOT_MPC_NODE_PATH = THIS_DIR / "bot_mpc_node.py"
STARTUP_DELAY = float(os.getenv("MPC_MOCAP_STARTUP_DELAY", "1.0"))
SHUTDOWN_GRACE = float(os.getenv("MPC_MOCAP_SHUTDOWN_GRACE", "5.0"))


def _start_process(label: str, script_path: Path) -> subprocess.Popen:
    if not script_path.exists():
        raise FileNotFoundError(f"{label} script not found: {script_path}")

    print(f"[main] starting {label}: {script_path}", flush=True)
    return subprocess.Popen(
        [sys.executable, str(script_path)],
        cwd=str(THIS_DIR),
        start_new_session=True,
    )


def _terminate_process(label: str, process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return

    print(f"[main] stopping {label} (pid={process.pid})", flush=True)
    try:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=SHUTDOWN_GRACE)
    except subprocess.TimeoutExpired:
        print(f"[main] {label} did not stop in time, sending SIGKILL", flush=True)
        os.killpg(process.pid, signal.SIGKILL)
        process.wait()
    except ProcessLookupError:
        pass


def main() -> int:
    mocap_proc = None
    runtime_proc = None

    def _handle_signal(signum, _frame) -> None:
        signame = signal.Signals(signum).name
        print(f"[main] received {signame}, shutting down", flush=True)
        if runtime_proc is not None:
            _terminate_process("bot_mpc_node", runtime_proc)
        if mocap_proc is not None:
            _terminate_process("mocap_pose_receiver", mocap_proc)
        raise SystemExit(0)

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    try:
        mocap_proc = _start_process("mocap_pose_receiver", MOCAP_RECEIVER_PATH)
        if STARTUP_DELAY > 0.0:
            print(f"[main] waiting {STARTUP_DELAY:.1f}s before starting bot_mpc_node", flush=True)
            time.sleep(STARTUP_DELAY)

        runtime_proc = _start_process("bot_mpc_node", BOT_MPC_NODE_PATH)

        while True:
            mocap_code = mocap_proc.poll()
            runtime_code = runtime_proc.poll()

            if mocap_code is not None:
                print(f"[main] mocap_pose_receiver exited with code {mocap_code}", flush=True)
                _terminate_process("bot_mpc_node", runtime_proc)
                return int(mocap_code)

            if runtime_code is not None:
                print(f"[main] bot_mpc_node exited with code {runtime_code}", flush=True)
                _terminate_process("mocap_pose_receiver", mocap_proc)
                return int(runtime_code)

            time.sleep(0.2)
    finally:
        if runtime_proc is not None:
            _terminate_process("bot_mpc_node", runtime_proc)
        if mocap_proc is not None:
            _terminate_process("mocap_pose_receiver", mocap_proc)


if __name__ == "__main__":
    raise SystemExit(main())
