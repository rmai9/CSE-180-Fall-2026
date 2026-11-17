# Preserved experiment scripts (not production code)

These files are **copies of throwaway scripts** used in the inspection stage and in the
external detector experiment. They are kept so the numbers in the main README can be
traced back to the code that produced them. They are *not* part of the `search` package,
they are not installed, and nothing in the package includes or imports them.

They were written for the container session (paths `/tmp/...`, ROS Jazzy image
`ghcr.io/ucmercedrobotics/mrtp`) and assume:

* ROS 2 Jazzy sourced (`/opt/ros/jazzy/setup.bash`);
* for the YOLO scripts, the container venv `/.venv` activated and
  `PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH` exported;
* the warehouse simulation running (`ros2 launch gazeboenvs tb4_warehouse.launch.py`);
* for the YOLO scripts, the weights present in the working directory
  (`yolo26n.pt`, `yolo11n.pt`, `yolov8n.pt`, `yolo11s.pt`, `yolo26s.pt`).

## Interface / map probes

| File | What it does | Used for |
|---|---|---|
| `inspect_cam.py` | Subscribes to `/rgbd_camera/image`, `/rgbd_camera/depth_image`, `/rgbd_camera/camera_info`; prints encodings, sizes, steps, stamps, intrinsics (`K`, `D`, `R`, `P`), depth value statistics, measured rates, and TF lookups at the image stamp. | Every "verified runtime interface" fact in sections B and C of the main README. |
| `tf_probe.py` | Issues timestamped `map <- camera_optical_frame` lookups on arrival with a 50 ms timeout, plus lookups at `stamp + 0.2 s`; measures the `map -> odom` interval on `/tf`. | The "14 of 20 timestamped lookups succeeded", "0 of 20 future lookups", and "10 Hz `map -> odom`" facts. |
| `map_probe.py` | Reads `nav2_bringup`'s `warehouse.pgm`, reports size/resolution/origin, cell statistics, and where the start pose and both people land in the map. | Map extent, unknown-cell fraction, and the "people are baked into the map as obstacles" observation. |
| `reach.py` | Clearance transform + connected-component analysis with a 0.30 m clearance requirement (0.22 m robot radius + 0.08 m margin); ASCII views of reachable space. | "One connected traversable component, 1137 m², 100 % reachable from the start pose". |
| `viewpoints.py` | Enumerates candidate robot poses around a person at a given distance with a clear line of sight in the occupancy map. | Choosing the person-2 pose used to capture the 4.0 m and 6.0 m frames. |

## Detector experiment

| File | What it does | Used for |
|---|---|---|
| `capture_frame.py <outdir> <tag>` | Waits until an RGB frame and a depth frame carry the *same* stamp, then saves `<tag>_rgb.png`, `<tag>_depth.npy`, `<tag>_info.json`. | All captured frames (the exact-stamp requirement is itself the timestamp finding). |
| `ladder.sh` | Teleports the robot with `gz service /world/warehouse/set_pose` to (1.0, 3.5), (1.0, 5.0), (1.0, 7.0) with yaw −1.5708, prints the pose read back with `gz model -m turtlebot4 -p`, then captures `p1_d45`, `p1_d60`, `p1_d80`. | The 4.5 / 6.0 / 8.0 m frames of person 1 (the person is at (1, −1), so the distances are y_robot + 1). |
| `ladder2.sh` | Same for person 2 at (-12, 15): poses (-12.83, 11.09) yaw 1.3599 and (-12.84, 9.06) yaw 1.4311, capturing `p2_d45`, `p2_d60`. | The 4.0 m and 6.0 m frames of person 2. **Note:** the file tags say `d45`/`d60`, but the poses are 4.0 m and 5.95 m from the person; the tags are stale labels. |
| `yolo_test.py <images...>` | Prints installed versions, the model families shipped with the release, which weights load, detections at `imgsz=640/conf=0.25`, timing, and a file-vs-numpy box count. Writes `/tmp/yolo_result.json`. | Version table, supported weights, first detection on the 3.5 m frame. |
| `yolo_compare.py` | Channel-order check (file vs numpy RGB vs numpy BGR), model comparison, `yolo26n` confidence vs `imgsz` (320/480/640/960), and the torch thread sweep. | The BGR/RGB finding, the `imgsz` sweep, and the thread-count table. |
| `yolo_ladder.py` | Person-1 distance ladder: each model at `imgsz` 320 and 480 with `conf=0.01` in the call, person boxes above 0.05 reported, plus depth at the box centre and timing. | The person-1 detection table and the person-1 depth cross-check. |
| `yolo_p2.py` | Same script with `FRAMES` replaced by the two person-2 frames (a `sed` edit of `yolo_ladder.py`). | The person-2 detection table and depth cross-check. |
| `contact_sheet.py`, `contact_p2.py` | Draw `yolo11n` boxes (`imgsz=480`, `conf=0.05`) on zoomed frames; write `/tmp/cap/contact.png`, `/tmp/cap/contact_p2.png`, and 3× zooms of the 3.5 m and 8 m frames. | The visual check described in the README (boxes framing the person at 3.5 m; a small, partly occluded figure at 8 m). |
| `yolo_result.json` | Compact JSON record of the first run on `p1_d35`: one raw detection, one person box, conf 0.380, box `[140.4, 25.7, 173.4, 137.7]` → 33 × 112 px. | A small machine-readable result record. |

## Not preserved

* The weights (`*.pt`, 5–20 MB each) — deliberately not committed; they are downloadable
  by ultralytics and are not needed for this stage.
* The captured frames (`/tmp/cap/*_rgb.png`, `*_depth.npy`), the annotated contact sheets
  (`/tmp/cap/contact.png`, `/tmp/cap/contact_p2.png`) and the downscaled host copies
  (`/tmp/sheet_p1.png`, `/tmp/sheet_p2.png`) — treated as generated artifacts. The contact
  sheets still exist only inside the container at those `/tmp` paths, which are ephemeral.
* The inline heredoc command that ran the two `s`-model detections and their timing
  (it was typed directly into `docker exec`, never saved as a file). Its settings and
  output are quoted in the main README, but the exact command cannot be re-run from a file.
