# Container name
CONTAINER_NAME := "sb_dev"

# Autoware image
IMAGE := "es_auto"

# Paths (use shell backticks for maximum compatibility)
WORKSPACE := `pwd`
MAP := '/home/hcis-s05/ysws/autoware_map'
AUTOWARE_PATH := '/home/hcis-s05/ysws/autoware'


source_aw := 'source /opt/autoware/setup.bash'


exec:
	docker exec -it {{CONTAINER_NAME}} bash

up:
	docker compose up -d

down:
	docker compose down

run_aw:
	python3 -m sv.cli run ./plans/aw_test.yaml

run plan:
	python3 -m sv.cli run ./plans/{{plan}}.yaml 

test_aw:
  ros2 launch autoware_launch planning_simulator.launch.xml \
    lanelet2_map_file:=e6mini_same.osm \
    map_path:=/autoware_map/sbsvf \
    vehicle_model:=sample_vehicle \
    sensor_model:=sample_sensor_kit \
    launch_system_monitor:=true \
    

sb:
	ros2 launch autoware_launch planning_simulator.launch.xml \
		map_path:=/autoware_map/sample-map-planning \
		vehicle_model:=sample_vehicle \
		sensor_model:=sample_sensor_kit

clean:
    #!/usr/bin/env bash
    set -euo pipefail

    echo "[clean] Find autoware launch processes..."
    mapfile -t PIDS < <(pgrep -af "ros2 launch" | grep -i autoware | awk '{print $1}' || true)

    if [[ ${#PIDS[@]} -eq 0 ]]; then
      echo "[clean] No matching 'ros2 launch ... autoware ...' processes found."
      exit 0
    fi

    echo "[clean] Found PIDs: ${PIDS[*]}"
    echo "[clean] Send SIGINT (Ctrl-C) to ros2 launch..."
    for pid in "${PIDS[@]}"; do
      kill -INT "$pid" 2>/dev/null || true
    done

    echo "[clean] Wait up to 10s for graceful shutdown..."
    for _ in {1..50}; do
      alive=0
      for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then alive=1; fi
      done
      [[ "$alive" -eq 0 ]] && break
      sleep 0.2
    done

    echo "[clean] Kill remaining descendant processes (TERM)..."
    # 遞迴殺子孫：先 TERM
    kill_tree_term() {
      local parent="$1"
      local kids
      kids="$(pgrep -P "$parent" || true)"
      for k in $kids; do
        kill_tree_term "$k"
      done
      pkill -TERM -P "$parent" 2>/dev/null || true
    }

    for pid in "${PIDS[@]}"; do
      kill_tree_term "$pid"
      kill -TERM "$pid" 2>/dev/null || true
    done

    sleep 1

    echo "[clean] Force kill remaining (KILL) if any..."
    kill_tree_kill() {
      local parent="$1"
      local kids
      kids="$(pgrep -P "$parent" || true)"
      for k in $kids; do
        kill_tree_kill "$k"
      done
      pkill -KILL -P "$parent" 2>/dev/null || true
    }

    for pid in "${PIDS[@]}"; do
      kill_tree_kill "$pid"
      kill -KILL "$pid" 2>/dev/null || true
    done

    echo "[clean] Done."




# {
#     "python.analysis.extraPaths": [
#         "./opt/autoware/local/lib/python3.10/dist-packages",
#         "./lib/python3/dist-packages/pip/_vendor"
#     ]
# }

