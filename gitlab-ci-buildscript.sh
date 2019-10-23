#!/bin/sh
set -ex

set +x # po co wyświetlać token
token=`grep 'aleph2_hardware_interface\.git' .git/config | grep -o 'https\?://gitlab-ci-token:.*/continuum/' | head -1`
sed -i "s git@.*:continuum/ $token g" $1
set -x

mkdir -p ws/src && ln -s ../.. ws/src/${PWD##*/}
wstool init ws/src $1
cd ws
wstool update -t src
for f in src/*/dependencies.rosinstall; do
  set +x
  sed -i "s git@.*:continuum/ $token g" $f
  set -x
  wstool merge -t src $f
done
wstool update -t src
apt update
rosdep update
rosdep install --from-paths src --ignore-src -y
apt install -y python-catkin-tools
catkin build --summarize --no-status --force-color --cmake-args -DCMAKE_C_FLAGS="-Wall -W -Wno-unused-parameter"
source devel/setup.bash
