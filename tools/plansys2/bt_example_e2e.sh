#!/bin/bash
# Copyright 2026 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# PlanSys2's plansys2_bt_example planned by Kairos and executed end to end (navigation
# simulated by nav2_sim_node): exit 0 when the executor reports the plan finished.
#
#   tools/plansys2/bt_example_e2e.sh PLANSYS2_WS KAIROS [ROS_SETUP]
#
# PLANSYS2_WS is a built workspace with PlanSys2 and its examples; ROS_SETUP defaults to
# /opt/ros/rolling/setup.bash. Logs go to $KAIROS_E2E_LOGS (default: a temporary folder).
set -uo pipefail
if [ $# -lt 2 ]; then
  echo "usage: $0 PLANSYS2_WS KAIROS [ROS_SETUP]" >&2
  exit 2
fi
ws=$(realpath "$1")
kairos=$(realpath "$2")
ros_setup=${3:-/opt/ros/rolling/setup.bash}
here=$(dirname "$(realpath "$0")")
logs=${KAIROS_E2E_LOGS:-$(mktemp -d -t kairos_e2e_XXXXXX)}
mkdir -p "$logs"

# Nothing from an earlier run may be left: it would answer instead of this one
if pgrep -f "plansys2_node|bt_action_node|assemble_action_node|nav2_sim_node" >/dev/null; then
  echo "PlanSys2 processes from an earlier run are still alive; stop them first" >&2
  exit 3
fi

overlay=$("$here/popf_overlay.sh" "$logs/overlay" "$kairos")
cleanup() {
  pkill -INT -f "plansys2_bt_example_launch.py" 2>/dev/null
  sleep 2
  pkill -f "plansys2_node|bt_action_node|assemble_action_node|nav2_sim_node" 2>/dev/null
}
trap cleanup EXIT

env -i HOME="$HOME" PATH=/usr/bin:/bin TERM=dumb bash -c "
  source '$ros_setup' && source '$ws/install/setup.bash'
  export AMENT_PREFIX_PATH='$overlay':\$AMENT_PREFIX_PATH
  export KAIROS_POPF_LOG='$logs/kairos_calls.log'
  export ROS_DOMAIN_ID=\$(( (\$\$ % 100) + 100 ))
  ros2 launch plansys2_bt_example plansys2_bt_example_launch.py > '$logs/launch.log' 2>&1 &
  ros2 run plansys2_bt_example nav2_sim_node > '$logs/nav2_sim.log' 2>&1 &
  sleep 8
  timeout 300 ros2 run plansys2_bt_example assemble_controller_node > '$logs/controller.log' 2>&1 &
  controller=\$!
  for i in \$(seq 1 300); do
    if grep -q 'Plan succesfully finished\|Plan finished with error\|Could not find plan' \
      '$logs/controller.log' 2>/dev/null; then
      break
    fi
    sleep 1
  done
  kill \$controller 2>/dev/null
  true
"
calls=$(grep -c '^; kairos' "$logs/kairos_calls.log" 2>/dev/null || echo 0)
if grep -q "Plan succesfully finished" "$logs/controller.log" && [ "$calls" -gt 0 ]; then
  echo "plansys2_bt_example: planned by Kairos ($calls calls) and executed to the end" \
    "(logs in $logs)"
  exit 0
fi
echo "plansys2_bt_example: failed (logs in $logs)" >&2
tail -20 "$logs/controller.log" >&2
exit 1
