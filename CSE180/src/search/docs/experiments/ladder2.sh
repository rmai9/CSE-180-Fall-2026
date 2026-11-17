source /opt/ros/jazzy/setup.bash
source /.venv/bin/activate
export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH
# person 2 at (-12, 15): view from the south, yaw ~1.361/1.431 rad
for spec in "p2_d45 -12.83 11.09 0.6288 0.7775" "p2_d60 -12.84 9.06 0.6554 0.7553"; do
  set -- $spec
  tag=$1; x=$2; y=$3; qz=$4; qw=$5
  gz service -s /world/warehouse/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
    --req req "name: \"turtlebot4\", position: {x: $x, y: $y, z: 0.01}, orientation: {z: $qz, w: $qw}" > /dev/null 2>&1
  gz service -s /world/warehouse/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
    --req "name: \"turtlebot4\", position: {x: $x, y: $y, z: 0.01}, orientation: {z: $qz, w: $qw}" > /dev/null 2>&1
  sleep 3
  echo "--- $tag requested ($x, $y) ---"
  gz model -m turtlebot4 -p 2>/dev/null | tail -2
  python /tmp/capture_frame.py /tmp/cap $tag
done
