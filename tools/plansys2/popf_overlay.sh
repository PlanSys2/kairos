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

# Kairos behind PlanSys2's current POPF plugin, until the native plugin (phase F3).
#
# The plugin runs <prefix of package popf>/lib/popf/popf [arguments] domain problem. This makes
# an ament overlay with a package "popf" whose popf runs Kairos with POPF's output format:
#
#   tools/plansys2/popf_overlay.sh OVERLAY_DIR /path/to/kairos [kairos options]
#   export AMENT_PREFIX_PATH=OVERLAY_DIR:$AMENT_PREFIX_PATH    # after sourcing ROS 2
#
# Default Kairos options: --timeout 5 (anytime: the best plan in 5 s, or sooner when no better
# one can be found). With KAIROS_POPF_LOG set, every call and its plan are appended to that file.
set -euo pipefail
if [ $# -lt 2 ]; then
  echo "usage: $0 OVERLAY_DIR KAIROS [kairos options]" >&2
  exit 2
fi
overlay=$(realpath -m "$1")
kairos=$(realpath "$2")
shift 2
options="${*:---timeout 5}"
mkdir -p "$overlay/share/ament_index/resource_index/packages" "$overlay/lib/popf"
touch "$overlay/share/ament_index/resource_index/packages/popf"
cat > "$overlay/lib/popf/popf" <<SCRIPT
#!/bin/sh
# PlanSys2's POPF plugin calls "popf [arguments] domain problem": Kairos answers in POPF's format
# (with KAIROS_POPF_LOG set, each call and its plan are appended there)
if [ -z "\$KAIROS_POPF_LOG" ]; then
  exec "$kairos" "\$@" --format popf $options
fi
out=\$("$kairos" "\$@" --format popf $options)
code=\$?
printf '%s\n%s\n' "; kairos \$*" "\$out" >> "\$KAIROS_POPF_LOG"
printf '%s\n' "\$out"
exit \$code
SCRIPT
chmod +x "$overlay/lib/popf/popf"
echo "$overlay"
