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

# As the ROS buildfarm's binaryrpm job: bloom-generate rosrpm, rpmbuild without tests, then install
set -e
grep -q almalinux /etc/os-release && sed -i 's/^mirrorlist/#mirrorlist/; s/^# *baseurl/baseurl/' /etc/yum.repos.d/*.repo
dnf install -y -q python3-pip rpm-build cmake gcc-c++ make git tar >/dev/null 2>&1
grep -q almalinux /etc/os-release && dnf install -y -q 'dnf-command(config-manager)' >/dev/null 2>&1 || true
pip3 install -q bloom rosdep >/dev/null 2>&1
rosdep init >/dev/null 2>&1 || true
rosdep update --rosdistro $ROS_DISTRO >/dev/null 2>&1
g++ --version | head -1; cmake --version | head -1
cp -r /src /tmp/kairos && cd /tmp/kairos
. /etc/os-release
OSN=$ID; [ "$ID" = almalinux ] && OSN=rhel
bloom-generate rosrpm --ros-distro $ROS_DISTRO --os-name $OSN --os-version ${VERSION_ID%%.*} > /tmp/bloom.log 2>&1 || { tail -30 /tmp/bloom.log; exit 1; }
SPEC=rpm/template.spec
NAME=$(rpmspec -q --qf '%{name}\n' --define "_sourcedir /tmp" $SPEC 2>/dev/null | head -1)
VER=$(rpmspec -q --qf '%{version}\n' $SPEC 2>/dev/null | head -1)
echo "$NAME $VER"
mkdir -p ~/rpmbuild/SOURCES
SRC=$(grep -E "^Source0:" $SPEC | awk '{print $2}' | sed "s/%{name}/$NAME/g; s/%{version}/$VER/g")
echo "Source0: $SRC"
cd /tmp && tar czf ~/rpmbuild/SOURCES/$(basename $SRC) --transform "s,^kairos,$NAME-$VER," kairos
rpmbuild -bb --without tests /tmp/kairos/$SPEC > /tmp/rpm.log 2>&1 || { grep -E "error|Error|Installed \(but unpackaged\)" -A5 /tmp/rpm.log | head -40; tail -15 /tmp/rpm.log; exit 1; }
RPM=$(ls ~/rpmbuild/RPMS/*/*.rpm | grep -v debug | head -1); echo $RPM
dnf install -y -q $RPM >/dev/null 2>&1 || rpm -i --nodeps $RPM
rpm -ql $(rpm -qp --qf '%{name}' $RPM) | grep -E "bin/kairos$|libkairos|kairosConfig" | head
ls /opt/ros/$ROS_DISTRO/lib/libkairos.so.1
env -i /opt/ros/$ROS_DISTRO/bin/kairos --version
env -i /opt/ros/$ROS_DISTRO/bin/kairos /tmp/kairos/docs/examples/pddls/cooking_domain.pddl /tmp/kairos/docs/examples/pddls/cooking_problem_simple.pddl --timeout 5 --no-anytime | tail -2
# A program that uses the library, as PlanSys2's plugin does (the ROS environment: lib only)
cmake -S /tmp/kairos/tests/install -B /tmp/consumer -DCMAKE_PREFIX_PATH=/opt/ros/$ROS_DISTRO > /tmp/consumer.log 2>&1 && cmake --build /tmp/consumer >> /tmp/consumer.log 2>&1 || { tail -20 /tmp/consumer.log; exit 1; }
env -i LD_LIBRARY_PATH=/opt/ros/$ROS_DISTRO/lib /tmp/consumer/kairos_consumer && echo CONSUMER OK
echo PACKAGE OK
