Simulation adapter (legacy)
==================================================================

This folder contains a lightweight Python simulator that speaks the same
RabbitMQ JSON formats as the scheduling algorithm. It can:

- Send map, tasks, and AGV status updates.
- Request path and trail responses and visualize them in real time.
- Move AGVs smoothly along the latest route (trail-first by default) and publish status updates.

Requirements
------------

- Python 3.8+
- RabbitMQ reachable with the same environment variables as the algorithm.
- Python deps: `pika`, `matplotlib`

Install deps:

```
pip install -r requirements.txt
```

Environment
-----------

The script uses the same env vars as the algorithm tools. A typical setup:

```
source ../config/network.env
```

By default, logs/results go under `../debug/` (override with `DEBUG_DIR` / `RESULT_DUMP_DIR`).

If RabbitMQ is on a different host, override before running:
```
AMQP_HOST=127.0.0.1 AMQP_PORT=5672 AMQP_USER=gms AMQP_PASS=123456 python sim_runner.py --bootstrap --run
```

Quick start
-----------

Run the simulator (from this `simulation/` folder):

```
python sim_runner.py --generate --agv-count 1 --task-count 2 --bind-tasks --bootstrap --run
```

One-AGV smooth demo (1s per edge, updates every 0.1s):

```
python sim_runner.py --generate --agv-count 1 --task-count 4 \
  --bind-tasks --bootstrap --run --step-interval 0.1 --edge-duration 1.0
```

Continuously run (as long as there is an idle AGV, send 1~2 new random tasks after a random delay; does NOT wait for all AGVs):

```
python sim_runner.py --generate --agv-count 1 --bootstrap --run --loop-tasks \
  --loop-mode any-idle --loop-task-min-count 1 --loop-task-max-count 2 \
  --loop-task-min-delay 1.0 --loop-task-max-delay 3.0
```

Two-AGV avoidance demo (optional; requires 2 AGVs):

```
python sim_runner.py --demo-avoidance --bootstrap --run \
  --step-interval 0.1 --edge-duration 1.0
```

Quick two-AGV run wrapper:

```
bash run_two_agv.sh
```

If you want to disable GUI visualization:

```
python sim_runner.py --bootstrap --run --no-vis
```

Visualization tips:

- Press `p`/`t`/`m`/`l`/`s` to toggle path/trail/map/labels/subtasks, `f` to fit, `r` to reset, `+`/`-` to zoom.
- Mouse wheel zooms at cursor.
- Use `--vis-path-preview 80` (0 = draw all points) to reduce clutter.
- Map nodes are hidden by default (lines-only); use `--vis-map-nodes` if you really want to draw node points.
- Markers: `G:*` = goal, `T:^` = temporary target, `N:◇` = nextDestinationPoint (from status if present), `S#:□` = sub-task points (filled □ indicates the next sub-task).
- Labels show a coarse state (`IDLE` / `WAIT` / `cur->next`) and speed.
- By default, the simulator validates that consecutive path/trail nodeIds are connected by map edges; if invalid edges appear it prints an error and truncates drawing/motion at the first invalid edge. Use `--no-snap-to-map` to draw raw points (may show off-map segments).
- If no window pops up, make sure you did not pass `--no-vis` and that your Python `matplotlib` has a GUI backend (Tk/Qt).
- The simulator auto-selects a GUI backend (prefers Qt5Agg, falls back to TkAgg); override with `MPLBACKEND=TkAgg` or `MPLBACKEND=Qt5Agg` if needed.
  (Tip: if you always want motion to follow `RobotTrailResponse`, use `--drive-mode trail` or `SIM_DRIVE_MODE=trail`.)

Recommended: use `simulation_v2/` for online visualization and smooth kinematics.

```
bash ../simulation_v2/run_all.sh
```

Common overrides:

```
python sim_runner.py \
  --map ../config/grid_20x20.json \
  --status generated_status.json \
  --tasks generated_tasks.json \
  --bootstrap --run
```

Generate payloads from the map:

```
python sim_runner.py --generate \
  --map ../config/grid_20x20.json \
  --out-dir . --agv-count 6 --task-count 20 --seed 7
```

This writes `generated_status.json` and `generated_tasks.json` in the output directory.
You can also generate and run in one step:

```
python sim_runner.py --generate --bootstrap --run
```

Notes
-----

- The simulator publishes `SendRobotStatusInfos` every `--step-interval` seconds.
- Motion is driven by `--drive-mode` (`trail` follows RobotTrailResponse only; `path` ignores trail).
- Path and trail requests are sent periodically; defaults are 1s (path) and 1s (trail). If a `RobotTrailResponse` no longer matches the latest `RobotPathResponse`, the simulator will auto-trigger a path refresh for that AGV.
- Map, status, and task routing keys follow the same rules as the C++ senders
  (`MAP_ROUTING_KEY`, `STATUS_ROUTING_KEY`, `PATH_REQUEST_ROUTING_KEY`,
  `TRAIL_REQUEST_ROUTING_KEY`, or `EXT_ROUTING_KEY` fallback).
