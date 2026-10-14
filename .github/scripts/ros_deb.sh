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

# As the ROS buildfarm's binarydeb job: bloom-generate rosdebian, dpkg-buildpackage with nocheck,
# then install the package and run the installed kairos
set -e
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq >/dev/null
apt-get install -y -qq python3-bloom python3-rosdep fakeroot debhelper dpkg-dev g++ cmake >/dev/null 2>&1
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || rosdep init >/dev/null 2>&1 || true
rosdep update --rosdistro $ROS_DISTRO >/dev/null 2>&1
g++ --version | head -1; cmake --version | head -1
cp -r /src /tmp/kairos && cd /tmp/kairos
bloom-generate rosdebian --ros-distro $ROS_DISTRO > /tmp/bloom.log 2>&1 || { tail -30 /tmp/bloom.log; exit 1; }
head -3 debian/changelog
export DEB_BUILD_OPTIONS=nocheck DEB_BUILD_PROFILES=nocheck
dpkg-buildpackage -b -us -uc -Pnocheck > /tmp/dpkg.log 2>&1 || { grep -E "error|Error" /tmp/dpkg.log | head -30; tail -20 /tmp/dpkg.log; exit 1; }
grep -cE "_deps/googletest|FetchContent" /tmp/dpkg.log || true
ls /tmp/*.deb
apt-get install -y -qq /tmp/*.deb >/dev/null 2>&1 || dpkg -i /tmp/*.deb
dpkg -L $(dpkg-deb -f /tmp/*.deb Package) | grep -E "bin/kairos$|libkairos|kairosConfig|completion" | head
ls /opt/ros/$ROS_DISTRO/lib/libkairos.so.1
env -i /opt/ros/$ROS_DISTRO/bin/kairos --version
env -i /opt/ros/$ROS_DISTRO/bin/kairos /tmp/kairos/docs/examples/pddls/cooking_domain.pddl /tmp/kairos/docs/examples/pddls/cooking_problem_simple.pddl --timeout 5 --no-anytime | tail -2
# A program that uses the library, as PlanSys2's plugin does (the ROS environment: lib only)
cmake -S /tmp/kairos/tests/install -B /tmp/consumer -DCMAKE_PREFIX_PATH=/opt/ros/$ROS_DISTRO > /tmp/consumer.log 2>&1 && cmake --build /tmp/consumer >> /tmp/consumer.log 2>&1 || { tail -20 /tmp/consumer.log; exit 1; }
env -i LD_LIBRARY_PATH=/opt/ros/$ROS_DISTRO/lib /tmp/consumer/kairos_consumer && echo CONSUMER OK
echo PACKAGE OK
