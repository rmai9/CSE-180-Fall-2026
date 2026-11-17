
source /opt/ros/jazzy/setup.bash
source /.venv/bin/activate
export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH
for spec in "d45 1.0 3.5" "d60 1.0 5.0" "d80 1.0 7.0"; do
  set -- $spec
  tag=$1; x=$2; y=$3
  gz service -s /world/warehouse/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 3000 \
    --req "name: \"turtlebot4\", position: {x: $x, y: $y, z: 0.01}, orientation: {z: -0.7071068, w: 0.7071068}" > /tmp/tp_$tag.log 2>&1
  sleep 3
  echo "--- $tag requested ($x, $y) ---"
  gz model -m turtlebot4 -p 2>/dev/null | tail -2
  python /tmp/capture_frame.py /tmp/cap p1_$tag
done
