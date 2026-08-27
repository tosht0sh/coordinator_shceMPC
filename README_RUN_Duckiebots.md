# Run Duckiebot MPC

This file is a terminal-by-terminal runbook for running the ScheMPC Duckiebot
runtime with laptop monitoring, schedule dispatch, and mocap pose streaming.

The examples use `dts`. If your setup uses `dtsduck`, replace `dts` with
`dtsduck` in the Duckiebot commands.

Keep each long-running command open in its own terminal.

In this file, "Bot Terminal" means a terminal on your laptop where you launch a
Duckiebot container with `dts devel run -H <bot> ...`.

## Before You Launch

Check these every time the setup changes:

1. `assets/TrajPlan-ScheMPC-copy/config/robot_spec.yaml`

The file has several commented robot constraint blocks. Make sure the active,
uncommented block is the one you want. The real Duckiebot constraints and the
simulation-only constraints are different.

2. `packages/mpc_runtime/src/bot_mpc_node.py`

Check `LAPTOP_TELEMETRY_IP`. The current code sends telemetry to the hard-coded
IP in that file. The old command `export MPC_TELEMETRY_IP=<LAPTOP_IP>` is not
currently read by the node.

3. Pose source

By default, `bot_mpc_node.py` subscribes to:

```text
/<vehicle>/mocap_reader
```

That means a normal mocap lab run needs the `mocap_reciever` launcher running on
each bot. If you want odometry instead, use the odometry-only section below and
set:

```bash
export MPC_POSE_TOPIC='/{vehicle}/pose_reader'
```

If your `dts` setup does not pass shell environment variables into the bot
container, put that export in `launchers/mpc_runtime.sh` before
`dt-launchfile-init`.

4. Mocap routing

Check:

- `assets/mocap/stream.py`: `TARGET_IP` is the laptop running `pose_distributor.py`.
- `assets/mocap/pose_distributor.py`: `sock.bind(...)` uses that same laptop IP.
- `assets/mocap/pose_distributor.py`: `forward_targets` maps `duck1`, `duck2`, etc. to the correct bot IPs.
- QTM body names match the bot names: `duck1`, `duck2`, `duck4`, `duck6`.

5. Build after code or launcher changes

Run once per bot:

```bash
cd /home/kim/dev/coordinator_shceMPC
dts devel build -H duck2 -f
```

6. Remove stale bot containers if needed

Use the container names you pass with `-n`.

```bash
docker -H=duck2.local rm -f duck2_mpc duck2_mocap duck2_odometry
```

## Ports

| Flow | Direction | Protocol | Port |
| --- | --- | --- | --- |
| Schedule/map packets | laptop to bot | TCP | `5007` |
| Bot telemetry | bot to laptop | UDP | `5008` |
| Mocap pose packets | laptop to bot | UDP | `5005` |
| Neighbor trajectory relay | laptop to bot | UDP | `5009` |
| Coordinator `WAIT`/`WORK`/`CROSSING` commands | laptop to bot | UDP | `5010` |

## Scenario Values

Use the same `MPC_ENV_FOLDER`, `MPC_SCHEDULE_VARIANT`, and
`MPC_BOT_ENDPOINTS` in the monitor terminal and the scheduler-dispatcher
terminal.

| Run | `MPC_ENV_FOLDER` | `MPC_SCHEDULE_VARIANT` | Schedule IDs |
| --- | --- | --- | --- |
| Single robot | `SingleRobotEnv` | `SingleRobot` | `A1` |
| Two robots | `TwoRobotEnv` | `TwoRobots` | `A1`, `A2` |
| MultiRobot Scene 1 | `MultiRobotEnv` | `MultiRobot` | `A1`, `A2` |

The live monitor and dispatcher read prepared files from:

```text
assets/TrajPlan-ScheMPC-copy/data/schedule_demo2_data
```

`data/test_cases/*.json` is used when regenerating schedules. If you regenerate
a schedule from a test case, keep `test_data.Environment` consistent with the
`MPC_ENV_FOLDER` you plan to run.

For physical coordinator runs, the monitor needs route/job context. Use
`MPC_COORD_DUMMY_DATA` to select the matching file in
`assets/TrajPlan-ScheMPC-copy/src/coordinator`.

| Run | Coordinator dummy data |
| --- | --- |
| Single robot | `dummy_data_real1.json` |
| MultiRobot Scene 1 | `dummy_data_real_cs1.json` |

If `coordinator.py` has `dummy_mode = True`, this dummy file must match the
selected schedule variant. If it does not, the monitor can fail with a missing
route error such as `Missing routes for robot IDs ['A2']`.

## Single Robot Mocap Run

This example maps schedule robot `A1` to physical robot `duck2` at
`192.168.1.12`.

### Bot Terminal 1: duck2 mocap receiver

```bash
cd /home/kim/dev/coordinator_shceMPC
dts devel run -H duck2 -L mocap_reciever -n duck2_mocap
```

The launcher name is currently spelled `mocap_reciever`, so use that exact
spelling.

### Bot Terminal 2: duck2 MPC runtime

```bash
cd /home/kim/dev/coordinator_shceMPC
dts devel run -H duck2 -L mpc_runtime -n duck2_mpc
```

The runtime waits for schedule/map packets on TCP `5007`. It will not move until
it has a schedule, fresh pose data, and coordinator `WORK` commands from the
monitor.

### Laptop Terminal 1: mocap pose distributor

```bash
cd /home/kim/dev/coordinator_shceMPC
source assets/TrajPlan-ScheMPC-copy/.venv/bin/activate
python3 assets/mocap/pose_distributor.py
```

This receives the combined mocap packet from `stream.py`, splits it by vehicle
name, and forwards each robot's pose to the correct bot.

### Laptop or Mocap Terminal 2: QTM mocap stream

```bash
cd /home/kim/dev/coordinator_shceMPC
source assets/TrajPlan-ScheMPC-copy/.venv/bin/activate
python3 assets/mocap/stream.py
```

Run this on the machine that can reach the QTM host configured in
`assets/mocap/stream.py`.

### Laptop Terminal 3: monitor

```bash
cd /home/kim/dev/coordinator_shceMPC
source assets/TrajPlan-ScheMPC-copy/.venv/bin/activate
export MPC_ENV_FOLDER=SingleRobotEnv
export MPC_SCHEDULE_VARIANT=SingleRobot
export MPC_COORD_DUMMY_DATA=dummy_data_real1.json
export MPC_TELEMETRY_PORT=5008
export MPC_BOT_ENDPOINTS='{"A1":{"host":"192.168.1.12","port":5007}}'
python3 assets/TrajPlan-ScheMPC-copy/src/monitor_node.py
```

Keep this running. It listens for telemetry, sends coordinator `WAIT`/`WORK`
commands, and relays neighbor trajectories when there is more than one robot.

### Laptop Terminal 4: send schedule

```bash
cd /home/kim/dev/coordinator_shceMPC
source assets/TrajPlan-ScheMPC-copy/.venv/bin/activate
export MPC_ENV_FOLDER=SingleRobotEnv
export MPC_SCHEDULE_VARIANT=SingleRobot
export MPC_BOT_ENDPOINTS='{"A1":{"host":"192.168.1.12","port":5007}}'
python3 assets/TrajPlan-ScheMPC-copy/src/scheduler_dispatcher.py
```

The scheduler dispatcher sends the map and schedule packets, then exits.

## Two Robot Mocap Run

This example maps:

- `A1` to `duck2` at `192.168.1.12`
- `A2` to `duck4` at `192.168.1.13`

### Bot Terminal 1: duck2 mocap receiver

```bash
cd /home/kim/dev/coordinator_shceMPC
dts devel run -H duck2 -L mocap_reciever -n duck2_mocap
```

### Bot Terminal 2: duck2 MPC runtime

```bash
cd /home/kim/dev/coordinator_shceMPC
dts devel run -H duck2 -L mpc_runtime -n duck2_mpc
```

### Bot Terminal 3: duck4 mocap receiver

```bash
cd /home/kim/dev/coordinator_shceMPC
dts devel run -H duck4 -L mocap_reciever -n duck4_mocap
```

### Bot Terminal 4: duck4 MPC runtime

```bash
cd /home/kim/dev/coordinator_shceMPC
dts devel run -H duck4 -L mpc_runtime -n duck4_mpc
```

### Laptop Terminal 1: mocap pose distributor

```bash
cd /home/kim/dev/coordinator_shceMPC
source assets/TrajPlan-ScheMPC-copy/.venv/bin/activate
python3 assets/mocap/pose_distributor.py
```

### Laptop or Mocap Terminal 2: QTM mocap stream

```bash
cd /home/kim/dev/coordinator_shceMPC
source assets/TrajPlan-ScheMPC-copy/.venv/bin/activate
python3 assets/mocap/stream.py
```

### Laptop Terminal 3: monitor

```bash
cd /home/kim/dev/coordinator_shceMPC
source assets/TrajPlan-ScheMPC-copy/.venv/bin/activate
export MPC_ENV_FOLDER=TwoRobotEnv
export MPC_SCHEDULE_VARIANT=TwoRobots
export MPC_TELEMETRY_PORT=5008
export MPC_BOT_ENDPOINTS='{"A1":{"host":"192.168.1.12","port":5007},"A2":{"host":"192.168.1.13","port":5007}}'
python3 assets/TrajPlan-ScheMPC-copy/src/monitor_node.py
```

### Laptop Terminal 4: send schedule

```bash
cd /home/kim/dev/coordinator_shceMPC
source assets/TrajPlan-ScheMPC-copy/.venv/bin/activate
export MPC_ENV_FOLDER=TwoRobotEnv
export MPC_SCHEDULE_VARIANT=TwoRobots
export MPC_BOT_ENDPOINTS='{"A1":{"host":"192.168.1.12","port":5007},"A2":{"host":"192.168.1.13","port":5007}}'
python3 assets/TrajPlan-ScheMPC-copy/src/scheduler_dispatcher.py
```

## MultiRobot Scene 1 Mocap Run

This example maps:

- `A1` to `duck2` at `192.168.1.12`
- `A2` to `duck4` at `192.168.1.13`

This run uses `MultiRobotEnv` to test Coordinator Scene 1. Both robots target
`N11`, so the monitor should send one bot `WAIT` and the other `CROSSING` with
a shifted target coordinate.

### Bot Terminal 1: duck2 mocap receiver

```bash
cd /home/kim/dev/coordinator_shceMPC
dts devel run -H duck2 -L mocap_reciever -n duck2_mocap
```

### Bot Terminal 2: duck2 MPC runtime

```bash
cd /home/kim/dev/coordinator_shceMPC
dts devel run -H duck2 -L mpc_runtime -n duck2_mpc
```

### Bot Terminal 3: duck4 mocap receiver

```bash
cd /home/kim/dev/coordinator_shceMPC
dts devel run -H duck4 -L mocap_reciever -n duck4_mocap
```

### Bot Terminal 4: duck4 MPC runtime

```bash
cd /home/kim/dev/coordinator_shceMPC
dts devel run -H duck4 -L mpc_runtime -n duck4_mpc
```

### Laptop Terminal 1: mocap pose distributor

```bash
cd /home/kim/dev/coordinator_shceMPC
source assets/TrajPlan-ScheMPC-copy/.venv/bin/activate
python3 assets/mocap/pose_distributor.py
```

### Laptop or Mocap Terminal 2: QTM mocap stream

```bash
cd /home/kim/dev/coordinator_shceMPC
source assets/TrajPlan-ScheMPC-copy/.venv/bin/activate
python3 assets/mocap/stream.py
```

### Laptop Terminal 3: monitor

```bash
cd /home/kim/dev/coordinator_shceMPC
source assets/TrajPlan-ScheMPC-copy/.venv/bin/activate
export MPC_ENV_FOLDER=MultiRobotEnv
export MPC_SCHEDULE_VARIANT=MultiRobot
export MPC_COORD_DUMMY_DATA=dummy_data_real_cs1.json
export MPC_TELEMETRY_PORT=5008
export MPC_BOT_ENDPOINTS='{"A1":{"host":"192.168.1.12","port":5007},"A2":{"host":"192.168.1.13","port":5007}}'
python3 assets/TrajPlan-ScheMPC-copy/src/monitor_node.py
```

### Laptop Terminal 4: send schedule

```bash
cd /home/kim/dev/coordinator_shceMPC
source assets/TrajPlan-ScheMPC-copy/.venv/bin/activate
export MPC_ENV_FOLDER=MultiRobotEnv
export MPC_SCHEDULE_VARIANT=MultiRobot
export MPC_BOT_ENDPOINTS='{"A1":{"host":"192.168.1.12","port":5007},"A2":{"host":"192.168.1.13","port":5007}}'
python3 assets/TrajPlan-ScheMPC-copy/src/scheduler_dispatcher.py
```

## Odometry-Only Run

Use this only when you intentionally want the MPC state from wheel odometry
instead of mocap.

This example maps schedule robot `A1` to physical robot `duck2`.

### Bot Terminal 1: duck2 odometry

```bash
cd /home/kim/dev/coordinator_shceMPC
dts devel run -H duck2 -L odometry -n duck2_odometry
```

### Bot Terminal 2: duck2 MPC runtime on `/pose_reader`

```bash
cd /home/kim/dev/coordinator_shceMPC
export MPC_POSE_TOPIC='/{vehicle}/pose_reader'
dts devel run -H duck2 -L mpc_runtime -n duck2_mpc
```

If the export does not reach the bot container, put it in
`launchers/mpc_runtime.sh` before `dt-launchfile-init`.

### Laptop Terminal 1: monitor

```bash
cd /home/kim/dev/coordinator_shceMPC
source assets/TrajPlan-ScheMPC-copy/.venv/bin/activate
export MPC_ENV_FOLDER=SingleRobotEnv
export MPC_SCHEDULE_VARIANT=SingleRobot
export MPC_TELEMETRY_PORT=5008
export MPC_BOT_ENDPOINTS='{"A1":{"host":"192.168.1.12","port":5007}}'
python3 assets/TrajPlan-ScheMPC-copy/src/monitor_node.py
```

### Laptop Terminal 2: send schedule

```bash
cd /home/kim/dev/coordinator_shceMPC
source assets/TrajPlan-ScheMPC-copy/.venv/bin/activate
export MPC_ENV_FOLDER=SingleRobotEnv
export MPC_SCHEDULE_VARIANT=SingleRobot
export MPC_BOT_ENDPOINTS='{"A1":{"host":"192.168.1.12","port":5007}}'
python3 assets/TrajPlan-ScheMPC-copy/src/scheduler_dispatcher.py
```

## Common Issues

### Dispatcher cannot connect to a bot

Check that `mpc_runtime` is already running on that bot and that
`MPC_BOT_ENDPOINTS` points to the bot IP on port `5007`.

The endpoint variable must be exported in the same terminal and must be valid
JSON. The keys must match the schedule robot IDs, such as `A1` and `A2`.

```bash
export MPC_BOT_ENDPOINTS='{"A1":{"host":"192.168.1.12","port":5007},"A2":{"host":"192.168.1.13","port":5007}}'
```

`No route to host` means the laptop cannot reach that IP address. Check the bot
IP, Wi-Fi/network, and whether the laptop can ping the bot.

### Monitor shows no telemetry

Check `LAPTOP_TELEMETRY_IP` in `packages/mpc_runtime/src/bot_mpc_node.py`, the
monitor port `MPC_TELEMETRY_PORT=5008`, and the laptop firewall.

### Bot gets schedule but does not move

The runtime publishes zero velocity until all of these are true:

- a schedule has been loaded;
- the selected pose topic is publishing fresh finite `[x, y, theta]`;
- the monitor is running and sending coordinator `WORK` commands.

For mocap runs, check `stream.py`, `pose_distributor.py`, `mocap_reciever`, and
the QTM body names. For odometry runs, check that `MPC_POSE_TOPIC` is set to
`/{vehicle}/pose_reader` and that the `odometry` launcher is running.

### Bot starts in `WAIT` forever

The monitor sends coordinator commands to the hosts in `MPC_BOT_ENDPOINTS` on
UDP `5010`. Leave the default port unless you update both sides. The bot reads
`MPC_COORDINATOR_PORT`; the monitor currently reads `COORDINATOR_PORT`.

### Container name already exists

Remove the stale container for that bot and launcher name:

```bash
docker -H=duck2.local rm -f duck2_mpc
```
