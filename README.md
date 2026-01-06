# P2RemoteConnection

Creating a web access interface to connect to Go2 functions.

## FastAPI backend (sit/stand via Unitree sport client)

This backend runs on the robot (or a nearby edge device) and exposes HTTP
endpoints to start/stop two whitelisted actions:

- `sit`   → `/home/unitree/unitree_ros2/example/install/unitree_ros2_example/bin/go2_sport_client 3`
- `stand` → `/home/unitree/unitree_ros2/example/install/unitree_ros2_example/bin/go2_sport_client 4`

Before running either action, the server sources the required ROS and Unitree
workspaces so the executable works even when launched via HTTP.

### How it works
- A `CommandManager` keeps an allowlist of actions → commands.
- `COMMON_PRE_COMMANDS` defines shell commands that run **before every action**
  (e.g., sourcing `setup.bash` files).
- On `POST /actions/{action}`, the server builds one shell command by joining
  pre-commands + the action command with `&&`, then spawns it via `/bin/bash`.
- On `POST /actions/{action}/stop`, the server sends SIGTERM (then SIGKILL if
  needed) to the tracked process for that action.

### Paths used (edit in `app/main.py` if your layout differs)
- ROS setup: `/opt/ros/foxy/setup.bash`
- Unitree root: `/home/unitree/unitree_ros2`
- CycloneDDS overlay: `/home/unitree/unitree_ros2/cyclonedds_ws/install/setup.bash`
- Unitree main overlay: `/home/unitree/unitree_ros2/install/setup.bash`
- Example overlay: `/home/unitree/unitree_ros2/example/install/setup.bash`
- Sport client executable: `/home/unitree/unitree_ros2/example/install/unitree_ros2_example/bin/go2_sport_client`
  - `sit`   runs with arg `3`
  - `stand` runs with arg `4`

If your paths differ, update the constants at the top of `app/main.py`.

### Run the server
```bash
pip install -r requirements.txt
uvicorn app.main:app --host 0.0.0.0 --port 8000
```

### Example calls
```bash
# Start sit action
curl -X POST http://localhost:8000/actions/sit

# Stop sit action
curl -X POST http://localhost:8000/actions/sit/stop

# Health check
curl http://localhost:8000/health
```

### Connecting to the robot
Run the server directly on the Go2 (or a nearby edge computer that has network
access to the robot). A web frontend can call these endpoints to trigger the
sit/stand actions through an allowlist, without exposing arbitrary shell access.
