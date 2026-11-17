# `search` — final project (CSE180)

Package that will make the TurtleBot4 search the warehouse map for people, report where
they are, and drive back home.

**Status: stage 3 — navigation-only search state machine.** The robot drives two map-frame
viewpoints, spins once at each, and drives back to the pose it saved at startup, with
per-goal deadlines and cancellation. There is still no perception and no detector ROS
package. This README is the project's development record: it carries forward the verified
environment facts, the runtime interfaces and the external detector experiment, so later
stages do not have to rediscover them.

---

## A. Current implementation status

### What the package does now

`src/search.cpp`:

1. creates the node `search`, a TF buffer/listener for `map ← base_link` and the course
   `Navigator`, linked through the exported `navigation::navigator` target;
2. waits at most 60 s for `/amcl/get_state` and `/bt_navigator/get_state` to appear, so a
   stack that is not running is reported instead of hanging in the library's unbounded wait;
3. reads the start pose from TF, logs it, and saves it as the **home pose**;
4. calls `SetInitialPose(home)` and then `WaitUntilNav2Active()`;
5. runs the state machine `SEND_GOAL → WAIT_GOAL → SEND_SPIN → WAIT_SPIN →` (next waypoint,
   or) `SEND_HOME → WAIT_HOME → DONE`, one Nav2 goal per transition;
6. gives every goal a deadline, `30 s + 3 × distance / 0.25 m/s` clamped to 120–600 s, and
   cancels it with `CancelTask()` on expiry, reporting `deadline expired` separately from
   `aborted by Nav2`, `canceled` and an unknown result code;
7. logs a summary line and exits.

The waypoints are a two-entry table in the map frame (`WAYPOINTS` in `search.cpp`); the home
pose is wherever the robot was when the node started. A failed waypoint is counted and the
search continues with the next one; a failed return home is reported as such.

### What has been tested (simulation running)

Three runs in the `tb4_warehouse` simulation. Each started with the robot teleported to the
launch pose (2.12, −21.30, yaw 1.57) and AMCL re-seeded, checked against Gazebo and TF before
the node started. All durations are wall-clock times taken from the run's log.

| Run | What it tested | Result |
|---|---|---|
| 1 — first attempt | startup, goal handling, deadline plumbing | **fails as a validation, bug found**: home pose saved as (0, 0, 0); all three goals reported `aborted by Nav2` after 0.9 s / 18.1 s / 0.6 s, with `controller_server: Resulting plan has 0 poses in it`; the robot drove ~6 m from spawn. See section E for the cause and the fix. |
| 2 — initial pose set | the full cycle, with the old deadline formula | **partial**: waypoint 1 reached in 53 s (0.04 m from it), spin done; waypoint 2 cancelled by its 138 s deadline after a ~48 m detour, 3.7 m short of the goal; home cancelled by its 189 s deadline, 16 m short; summary `1/2 waypoints reached, 1 failed`. This run is the evidence for the cancel path (section C). It also exposed that the pose was not refreshed after a failed goal: the home distance was printed as 23.9 m when the straight line was 35.6 m. |
| 3 — both fixes | the full cycle | **pass**: waypoint 1 in 57.4 s (0.10 m from the requested pose), spin 1.9 s, waypoint 2 in 140.7 s (0.05 m), spin 1.7 s, home in 184.2 s (0.13 m), total 386 s; summary `2/2 waypoints reached, 0 failed, 2 spin(s) done, home reached`, then `search finished, exiting`. |

| Check | Command | Result |
|---|---|---|
| Build (compile + link) | `colcon build --packages-select search` in `/MRTP/CSE180` | **pass**, `Finished <<< search [26.0s]`, exit 0, no warnings |
| Package entry point | `ros2 run search search --ros-args -p use_sim_time:=true` | **pass**, this is how runs 1–3 were started |
| Bounded Nav2 gate (60 s) | covered by every run | **pass** only in the positive case: services were seen after 1.4–2.2 s. The timeout/error branch was not exercised. |
| Ctrl+C during a goal | — | **not tested** in stage 3. Cancelling a goal that is still running on shutdown would come from the Navigator destructor. |
| Process exit code | — | **not captured**: runs were started detached, so the log ends with `search finished, exiting` but the process status was not collected. |

What this establishes: map-frame goals are executed and reported, `Spin` works, the home pose
is saved and returned to, and a deadline that expires cancels the goal and is reported
distinctly. What it does not establish: anything about perception, person reporting, or
repeated/reliable behaviour — each run happened once, in one fixed scene, with two
hand-picked viewpoints.

### What remains unimplemented

RGB/depth subscriptions and depth buffering; back-projection and TF localisation of a
detection; person detection, association and merging; `MarkerArray` output; the detector
node; map-derived viewpoints (reachability, 71.6° field of view, overlapping views); a
parameter- or file-driven waypoint list; observing while the robot is moving.

---

## B. Verified environment and workspace facts

| Fact | Value |
|---|---|
| Host | macOS (arm64) with Docker Desktop |
| Container | `ghcr.io/ucmercedrobotics/mrtp` (Dockerfile: `docker/Dockerfile`, `ROS_DISTRO=jazzy`), aarch64, `--net=host`, repo mounted at `/MRTP` |
| ROS | Jazzy, `/opt/ros/jazzy`; Python **3.12.3** |
| Repo root (git) | `/Users/makho/MRTP` on the host = `/MRTP` in the container, branch `main` |
| Your workspace | `/MRTP/CSE180/src/{gotogoal,patrol,safedrive,scout,search}` → install `/MRTP/CSE180/install` |
| MRTP workspace | `/MRTP/MRTP/src/{examples,gazeboenvs,multipletopics,navigation,talklisten}` → install `/MRTP/MRTP/install` |
| `navigation` package | present **only** in `/MRTP/MRTP/install`; `ros2 pkg prefix navigation` → `/MRTP/MRTP/install/navigation`; library `lib/libnavigator.a`; exported imported target **`navigation::navigator`** (`share/navigation/cmake/navigatorlibExport.cmake: add_library(navigation::navigator STATIC IMPORTED)`) |
| Stale tree (warning only, untouched) | `/MRTP/{build,install,log}` at the repo root is a broken partial install (only `first` and `gazeboenvs`, no ament index). Sourcing it registers nothing; do not use it, and it was not modified. |
| Nav2 configuration used | `MRTP/src/gazeboenvs/params/nav2_params_wh_tb4.yaml`, launched by `MRTP/src/gazeboenvs/launch/tb4_warehouse.launch.py`: `amcl.set_initial_pose: true` with `initial_pose` = spawn (2.12, −21.3, yaw 1.57), and DWB's progress checker at `required_movement_radius: 0.5` m, `movement_time_allowance: 10.0` s |

### Sourcing and build workflow (verified)

`/MRTP/CSE180/install/setup.bash` already chains `/opt/ros/jazzy` **and**
`/MRTP/MRTP/install`, so `find_package(navigation REQUIRED)` resolves from the CSE180
workspace with no extra sourcing:

```bash
# in the container
cd /MRTP/CSE180
source /opt/ros/jazzy/setup.bash
source install/setup.bash        # pulls in /MRTP/MRTP/install -> navigation
colcon build --packages-select search
source install/setup.bash
```

### Python environment (detector experiment, container venv `/.venv`, CPU only)

| Component | Version |
|---|---|
| python | 3.12.3 |
| ultralytics | **8.4.191** |
| torch / torchvision | **2.14.1+cpu / 0.29.1+cpu** (installed from `download.pytorch.org/whl/cpu`) |
| numpy | **1.26.4** (pinned `<2`) |
| opencv-python (venv) | **4.11.0.86** (pinned `<5`) |
| cv2 actually imported with the documented `PYTHONPATH` | **4.6.0** — the system `python3-opencv` from `/usr/lib/python3/dist-packages` precedes the venv on `sys.path` |
| also installed | polars 2.1.0, matplotlib 3.11.2, psutil, py-cpuinfo, pyyaml, requests |
| ROS python bits | `rclpy`, `cv_bridge 4.1.0`, `scipy 1.11.4`, PIL — available to both `/usr/bin/python3` and the venv |
| torch threads (default) | **1**; `torch.cuda.is_available()` → `False` |

Relevant nuance: the venv is *not* `--system-site-packages`; ROS Python packages are
reached with `PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH`, which the image's
`.bashrc` already appends.

---

## C. Verified runtime interfaces

All measured live in the `tb4_warehouse` simulation unless marked "read from source".

### Camera topics

| Item | Value |
|---|---|
| Color | `/rgbd_camera/image`, `sensor_msgs/msg/Image`, `rgb8`, 320×240, `step=960`, frame `oakd_rgb_camera_optical_frame`, ~10 Hz, QoS RELIABLE / VOLATILE (publisher `bridge_gz_ros_camera_image`) |
| Depth | `/rgbd_camera/depth_image`, `sensor_msgs/msg/Image`, **`32FC1`**, 320×240, `step=1280`, same frame, ~10 Hz, RELIABLE / VOLATILE |
| CameraInfo | `/rgbd_camera/camera_info`, `plumb_bob`, 320×240, `K = [221.765, 0, 160, 0, 221.765, 120, 0, 0, 1]` → `fx = fy = 221.765`, `cx = 160`, `cy = 120`; `D = 0`; `R = I`; `P = K` |
| Field of view | HFOV from `K` = **1.25 rad (71.6°)**, matching the URDF `horizontal_fov 1.25`; VFOV = 2·atan(120/221.765) = **56.8°** |
| Bridging | `tb4_bridge.yaml` bridges only `camera_info`; the two images come from `ros_gz_image image_bridge` nodes in `spawn_tb4.launch.py`. There is no `/camera/...` alternative. |

### Timestamps and registration

* **RGB and depth stamps are identical** (e.g. `87.102000000` for both in the 3.5 m capture).
  `capture_frame.py` only accepted a pair when the integer nanosecond stamps matched, and
  that succeeded on every attempt.
* **CameraInfo stamp differs** from the image stamps (`88.5` vs `88.401` in the stage-1
  sample). Intrinsics are constant, so use the latest `CameraInfo` and never its stamp as
  the observation time.
* Registration (read from source): both images come from one Gazebo
  `<sensor type="rgbd_camera">` on `oakd_rgb_camera_frame`, with the same optical frame,
  the same intrinsics and the same stamp. The empirical support for pixel correspondence is
  in section D (depth at a detected box centre versus the known distance).

### Depth units and invalid values

* Values are **meters**, metric Z along the optical axis.
* Invalid is **`+inf`**; no `NaN` and no exact zeros were observed.
* Stage-1 sample: valid range 0.447 – 46.36 m, median 4.71 m, 399 of 76800 px `+inf`.
* Per captured frame: `+inf` counts 260 (3.5 m), 542 (4.5 m), 889 (6.0 m), 1183 (8.0 m),
  2060 (person 2 at 4.0 m); smallest finite value 0.447 m in every frame, largest finite
  16.574 – 32.064 m. Sensor clip is ~0.3 m near / 100 m far (URDF).

### Mounting transform

`base_link → oakd_rgb_camera_optical_frame`: `t = (−0.0596, 0, 0.2435)`,
`q = (−0.5, 0.5, −0.5, 0.5)` (standard optical rotation, **no downward tilt**).
A standing person (head ~1.75 m) fits fully in the frame only beyond ≈2.8 m.

### TF

* AMCL publishes `map → odom` on `/tf` at a steady **10 Hz** (119 intervals: min 0.099,
  max 0.102 s).
* Timestamped `map ← oakd_rgb_camera_optical_frame` lookup issued on image arrival with a
  **50 ms** timeout: **14 of 20 succeeded**. Failures were the first seconds before AMCL's
  first update (`"map" … does not exist`) and `extrapolation into the future` when the TF
  sample at that exact stamp had not arrived yet.
* tf2 does **not** clamp: a lookup at `stamp + 0.2 s` failed **20 of 20** times.
  → use a bounded lookup timeout (~0.2 s as the measured starting tolerance), log failures,
  and never silently fall back to the latest transform.
* `map ← optical` at the latest data = `(2.12, −21.36, 0.2435)`, consistent with the spawn pose.

### Navigator (read from `MRTP/src/navigation/src/navigation.cpp`)

`class Navigator : public rclcpp::Node` (node name `navigator`),
`Navigator(bool debug = false, bool verbose = false)`.

| Call | Behaviour |
|---|---|
| `SetInitialPose(Pose::SharedPtr)` | stores the pose, publishes `/initialpose` in `map` with stamp `now + 1 s`. Non-blocking. |
| `WaitUntilNav2Active()` | **blocking**: waits for `/amcl/get_state` active → republishes `/initialpose` and spins until any `/amcl_pose` → waits for `/bt_navigator/get_state` active. **No timeout anywhere.** |
| `GoToPose(Pose::SharedPtr)` | builds a `NavigateToPose` goal (`frame_id = "map"`, stamp = now); blocks only for the goal response and returns `true` on acceptance, then runs asynchronously. |
| `Spin(double target_yaw = 1.57)` | same pattern on the `Spin` action. |
| `IsTaskComplete()` | `true` when nothing is executing; internally `spin_until_future_complete(future, 100 ms)`. Returns `true` immediately if no task is running. |
| `GetResult()` | last `rclcpp_action::ResultCode` (`SUCCEEDED` / `ABORTED` / `CANCELED` / `UNKNOWN`). |
| `CancelTask()` | **blocking**: `async_cancel_all_goals` + wait; then spins until the task ends if the cancel was accepted. Returns immediately when no task is executing (this is why the skeleton's destructor is harmless). |
| `GetFeedback()` | `shared_ptr<const void>`; cast to `NavigateToPose::Feedback` (`distance_remaining`) or `Spin::Feedback` (`angular_distance_traveled`). |
| `Backup(double = 0.15, double = 0.25)` | `BackUp` action; also blocking for acceptance only. |
| `FollowWaypoints(vector<PoseStamped>)` | exists, but poses are forwarded **verbatim** — the caller must set `header.frame_id = "map"` and stamps. |

**Executor constraints that follow.** Every blocking call runs
`spin_until_future_complete(this->get_node_base_interface(), …)` on the Navigator's *own*
node. Therefore the Navigator must not be added to an executor that is already spinning and
no second thread may spin it. The planned arrangement: Navigator created in `main()` and
driven by the state machine; perception in a separate node with its own executor on its own
thread. `use_sim_time` cannot be passed through the constructor (it uses default
`NodeOptions`), so it goes in as a process-wide argument:
`ros2 run search search --ros-args -p use_sim_time:=true`.

**Known timeout limitations.** There is no goal timeout in the library, and
`WaitUntilNav2Active()` can hang forever if `/amcl_pose` never arrives. In practice the
warehouse params set `amcl.set_initial_pose: true` with exactly the spawn pose, so
localization is initialised by the launch. The state machine still has to implement its own
deadline + `CancelTask()` and distinguish `ABORTED` from `CANCELED`.

**Other robots-side facts.** `collision_monitor` sits between `cmd_vel_smoothed` and
`cmd_vel`, which is another reason `search` must never publish `/cmd_vel`. Live action
servers include `/navigate_to_pose`, `/navigate_through_poses`, `/spin`, `/backup`,
`/follow_path`, `/follow_waypoints`, `/compute_path_to_pose`, `/smooth_path`, `/wait`,
`/assisted_teleop`, `/drive_on_heading` (plus route/dock).

### Navigator behaviour observed in stage 3 (measured)

* **The stored initial pose defaults to all zeros, and `WaitUntilNav2Active()` publishes it.**
  `wait_for_initial_pose()` calls `set_initial_pose()` unconditionally, so unless
  `SetInitialPose()` was called first it publishes pose (0, 0, 0, yaw 0) in `map`. In run 1
  AMCL accepted it: `/amcl_pose` then reported (0, 0, 0), every DWB plan came back empty
  (`Resulting plan has 0 poses in it`) and all three goals aborted. Calling
  `SetInitialPose(home)` first turns that republish into a repetition of the true pose.
  The causal step (AMCL accepting that message) is inferred from the observed (0, 0, 0) home
  pose and the AMCL warnings below, not from instrumenting AMCL.
* **`/amcl_pose` is not periodic.** `ros2 topic hz /amcl_pose` printed "does not appear to be
  published yet" while the robot stood still after a run, and `--once` returned the update
  from minutes earlier. AMCL publishes it when it handles an initial pose or updates its
  filter, not on a timer, so a consumer that needs the pose continuously must use TF.
* **`map ← base_link` is continuous.** `tf2_echo map base_link` tracked the robot through all
  three runs; it is `map → odom` (AMCL, 10 Hz) composed with `odom → base_link`.
* **`set_initial_pose()` stamps `now + 1 s`**, and AMCL logged
  `Failed to transform initial pose in time (Lookup would require extrapolation into the
  future …)` for those messages in runs 2 and 3: the library's republish is often *ignored*.
  Harmless here, because it repeated the pose AMCL already had.
* **A successful goal is logged at ERROR severity** by `print_result_diagnostic()` when debug
  is on, so `search` constructs `Navigator(false, false)` and reports results itself.
* **`CancelTask()` works as documented.** Run 2's two deadline cancels produced
  `[bt_navigator]: Client requested to cancel the goal. Cancelling.` → `Goal canceled` →
  `[controller_server]: Cancellation was successful. Stopping the robot.`, and `search`
  reported `deadline expired`, not `canceled`.
* **`Spin(1.57)` does rotate, but the yaw read immediately afterwards lags.** A separate
  `ros2 action send_goal /spin` test sampled with `tf2_echo` after the robot settled moved the
  heading 1.740 → 3.399 rad (+1.66 rad) and returned `SUCCEEDED`. In run 3 the yaw printed
  right after each spin (−1.88 from a −1.57 goal, 0.94 from a 1.36 goal) had not caught up
  with the rotation yet, so treat that number as approximate.

### Map, localization, people

* Map `warehouse.pgm` (via `nav2_bringup`, params `MRTP/src/gazeboenvs/params/nav2_wh_tb4.yaml`):
  `0.03 m`/cell, **1006 × 1674 cells = 30.2 m (x) × 50.2 m (y)**, origin `(−15.1, −25)`,
  so x ∈ [−15.1, 15.08], y ∈ [−25, 25.22], mode trinary. Pixel values: 254/255 free,
  **205 = unknown** (13.7 % of cells), 0 = occupied.
* Reachability with a 0.30 m clearance (0.22 m robot radius + 0.08 m margin;
  free = `occ_prob < 0.10`, occupied = `occ_prob > 0.65`): **one single connected traversable
  component, 1137 m², 100 % of it reachable from the start pose.**
* World people (`warehouse.sdf`): `Person 1 – Standing`, mesh `MaleVisitorOnPhone`, at
  **(1, −1)** (static include, yaw 1.57); `Person 2 – Walking`, mesh `plateau/models/Casual
  female`, at **(−12, 15)** (static include, yaw 0).
* The static map already has occupied neighbours **0.03 m** from (1, −1) and **0.12 m** from
  (−12, 15); the nearest robot-traversable cell is 0.30 m / 0.18 m away. The people (or the
  furniture next to them) are baked into the map as obstacles, and both sit next to
  reachable free space.
* Spawn pose in the stage-1 run: `(2.12, −21.3, yaw 1.57)`. Person 1 is **20.3 m** and
  person 2 **39.0 m** from it.

---

## D. Detector experiment (external experiment — no detector ROS package exists)

> **Label:** this was a standalone experiment with Python scripts and the ultralytics
> library inside the container. It is **not** a ROS node, it is not part of this package,
> and the `detector` package has **not** been created. Scripts are preserved in
> [`docs/experiments/`](docs/experiments/README.md).

### D.1 Method

**Positions and distances.** Person 1 is at map (1, −1); the robot was placed due north of
it and commanded to face −y (`yaw = −1.5708`). The 3.5 m frame was captured at the robot's
spawn pose **(1.0, 2.5)**, read back after capture with `gz model -m turtlebot4 -p` as
position (1.0, 2.5) and yaw −1.571 (the exact printed digits were not kept in the notes).
The other three were teleported with
`gz service -s /world/warehouse/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean`
and read back before capture: `(1.000000, 3.499980)`, `(1.000000, 4.999980)`,
`(1.000000, 6.999980)`, all at yaw `−1.5708`. Person 2 is at (−12, 15); the robot stood
south of it facing +y at `(−12.83, 11.09)` yaw `1.3599` and `(−12.84, 9.06)` yaw `1.4311`,
read back identically. Distances follow from these coordinates: person 1 → 3.5 / 4.5 / 6.0 /
8.0 m; person 2 → **4.0 m** (`hypot(0.83, 3.91)`) and **5.95 m** (`hypot(0.84, 5.94)`).
The script tags `p2_d45` / `p2_d60` are stale labels for those two poses.
The exact launch command of that experiment session was not saved; only the process log
`/tmp/sim_det.log`, which does not include the command line.

**Capture.** `capture_frame.py` subscribed `/rgbd_camera/image`,
`/rgbd_camera/depth_image` and `/rgbd_camera/camera_info` (QoS RELIABLE / VOLATILE,
KEEP_LAST, depth 1), set `use_sim_time`, and waited until an RGB and a depth message carried
**exactly the same stamp**; then saved the RGB as PNG (320 × 240, PNG is RGB order, matching
the sensor's `rgb8` convention), the depth as `.npy` (float32, 240 × 320) and the camera
info as JSON. One frame per condition — no averaging, no burst.

**Inference settings (as in the scripts).** Model files loaded from the working directory
(`yolo26n.pt`, `yolo11n.pt`, `yolov8n.pt`, `yolo11s.pt`, `yolo26s.pt`); `device` was never
set, i.e. CPU (no GPU in the container); predictions were requested with `conf=0.01` so that
weak boxes were visible, and **person boxes above `conf > 0.05` were then reported**;
`imgsz` 320 and 480 for the distance ladders, 640 for the channel-order and thread
experiments, 640/320/480/960 for the `imgsz` sweep; `iou` and `max_det` left at the
library defaults; no image preprocessing other than what ultralytics does internally
(resize to `imgsz`); input was a **PNG file path** for every number in the tables (the
channel check additionally passed numpy arrays).

**Timing.** Collected in the same container **while Gazebo + Nav2 + the bridges were
running**, with `time.perf_counter()` around `model.predict()`. `yolo_test.py` and
`yolo_compare.py` did 2 warm-up predictions and then 4–5 timed runs each, reporting the
median; the `s`-model run did 2 warm-ups and 3 timed runs (median). Model load, weight
download and first-call graph setup are **excluded**. Per-run spread was large (the report
kept ranges, e.g. 222–318 ms at `imgsz=480`), and only medians/ranges — not every raw
sample — were recorded.

**Depth cross-check.** For the top person box of `yolo11n` at `imgsz=480`, a **16 × 8 px**
patch centred on the box centre was taken from the saved depth array; pixels that were not
finite or `<= 0` were dropped (128/128 valid in the person-2 frames) and the **median** was
compared with the known pose distance.

**Channel order.** The same 3.5 m PNG was passed three ways: as a file path, as a numpy RGB
array, and as a numpy BGR array (`rgb[:, :, ::-1]`), with `yolo26n`, `imgsz=640`, `conf=0.01`.

### D.2 Measured results

**Person 1 — `MaleVisitorOnPhone`, map (1, −1); robot north of the person, facing −y.**
Top person confidence at `imgsz = 480`, box size in the 320 × 240 frame:

| Model | 3.5 m | 4.5 m | 6.0 m | 8.0 m |
|---|---|---|---|---|
| yolo26n | 0.656 (33×112) | 0.732 (26×87) | 0.480 (19×65) | none |
| yolo11n | 0.631 (34×113) | 0.688 (25×88) | 0.083 (17×66) | none |
| yolov8n | 0.843 (34×111) | 0.822 (25×87) | 0.791 (19×65) | none |
| yolo11s | 0.803 | *not measured* | 0.594 | 0.264 |
| yolo26s | 0.893 | *not measured* | 0.852 | **0.704** |

At `imgsz = 320` the same frames degrade sharply:

| Model | 3.5 m | 4.5 m | 6.0 m | 8.0 m |
|---|---|---|---|---|
| yolo26n | 0.374 (32×111) | 0.416 (26×87) | none | none |
| yolo11n | 0.715 (33×111) | none | none | none |
| yolov8n | 0.773 (34×112) | 0.107 (26×87) | none | none |

`imgsz` sweep on the 3.5 m frame (yolo26n, file input, `conf=0.01` requested):
`imgsz` 320 → **0.374**, 480 → **0.656**, 640 → **0.380**, 960 → **0.190**.

**Person 2 — `Casual female`, map (−12, 15); robot south of the person, facing +y.**

| Model | 4.0 m, imgsz 320 / 480 | 5.95 m, imgsz 320 / 480 |
|---|---|---|
| yolo26n | 0.822 (33×93) / 0.820 (33×93) | 0.785 (20×62, 2 boxes) / 0.809 (21×63) |
| yolo11n | 0.883 (33×93) / 0.870 (33×93) | 0.788 (21×62) / 0.872 (23×62, 2 boxes) |
| yolov8n | 0.833 (35×93) / 0.866 (34×94, 2 boxes) | 0.832 (22×61) / 0.856 (22×62) |

**Depth cross-check** (yolo11n, `imgsz=480`, 16 × 8 px patch at box centre):

| Condition | Box-centre depth (median) | Known distance |
|---|---|---|
| Person 1, 3.5 m | 3.460 m | 3.50 m |
| Person 1, 4.5 m | 4.460 m | 4.50 m |
| Person 1, 6.0 m | 5.966 m | 6.00 m |
| Person 1, 8.0 m | *no detection* | 8.00 m |
| Person 2, 4.0 m | 3.966 m | 4.00 m |
| Person 2, 5.95 m | 5.963 m | 5.95 m |

**Inference time** (ms; Gazebo + Nav2 on the same machine, CPU, median of 4–5 runs after
warm-up):

| Model | imgsz=320 | imgsz=480 | imgsz=640 |
|---|---|---|---|
| yolo26n | 147–165 | 222–318 | ~316 |
| yolo11n | 156–206 | 220–336 | *not measured* |
| yolov8n | 177–233 | ~210 | *not measured* |
| yolo11s | *not measured* | 406 | *not measured* |
| yolo26s | *not measured* | 684 | *not measured* |

Thread count (yolo26n, `imgsz=640`): 1 → 316 ms, **2 → 229 ms**, 4 → 280 ms, 8 → 574 ms.
The container's default is 1 thread.

**Channel order** (3.5 m frame, yolo26n, `imgsz=640`, `conf=0.01`):

| Input | Boxes | Top confidence |
|---|---|---|
| file path | 14 | 0.380 |
| numpy **BGR** | 14 | **0.380** |
| numpy RGB | 12 | 0.525 (different) |

**Weights.** `yolo26n.pt` (5.3 MB), `yolo11n.pt` (5.6 MB), `yolov8n.pt` (6.5 MB),
`yolo11s.pt` (19.3 MB), `yolo26s.pt` (20.4 MB) all downloaded from the v8.4.0 assets
release and loaded with 80 COCO classes. `yolo26n.pt` is fully supported, so the
assignment's suggested model works here. Model families shipped with ultralytics 8.4.191
(the `cfg/models` listing): `11, 12, 26, rt-detr, v10, v3, v5, v6, v8, v9`.

### D.3 Source-code inspection findings

* `LoadPilAndNumpy._single_check` in `ultralytics/data/loaders.py`: *"NumPy color inputs are
  assumed to use OpenCV-compatible BGR order."* This is the documented reason for the
  measured behaviour above, not an inference from the numbers alone.
* `ultralytics` 8.4.191 metadata requires `opencv-python>=4.7` and `numpy>=1.23`;
  the newest `opencv-python` (5.x) requires `numpy>=2`, which is why both are pinned.

### D.4 Derived calculations

* HFOV = 2·atan(160/221.765) = 1.25 rad; VFOV = 2·atan(120/221.765) = 0.955 rad (56.8°).
* A person at 6 m spans ≈ 1.75·221.765/6 ≈ 65 px vertically (assuming a 1.75 m person) —
  matching the measured 19×65 px box, which is why 6 m is near the limit for nano models
  at `imgsz=480`.
* 1/(0.222–0.318 s) ≈ 3.1–4.5 FPS for the nano models at `imgsz=480`, i.e. comfortable
  headroom for a 2 Hz inference timer; 1/0.684 s ≈ 1.5 FPS for `yolo26s`.

### D.5 Proposed defaults and design recommendations

`model = yolo26n.pt`, `imgsz = 480`, `conf = 0.4`, `device = cpu`, person-class filter only,
RGB → BGR conversion before inference, ~2 Hz inference timer, exact copy of the source
`header` onto the published detections. If `conf = 0.5` must be held, `yolov8n.pt` is the
strongest performer in this scene (0.79–0.87 for both people from 3.5–6 m), and
`yolo26s.pt` extends range to ~8 m at ~1.5 FPS. These are **recommendations from single
frames**, not validated settings.

### D.6 Assumptions and untested conclusions

* Assumed that both person meshes are rigid static includes (they are, from
  `warehouse.sdf`) so a "walking" person does not actually move.
* Assumed the depth at the box centre lands on the person's surface; the agreement in the
  table supports this in these frames but does not prove it for other viewpoints.
* Not tested: any second person in the same frame, added people, moving people, different
  lighting, distance beyond 8 m, detection while the robot is driving (all frames were
  captured with the robot stationary after a teleport), and the detector running as a ROS
  node with a 10 Hz stream.

---

## E. Reliability and limitations

Read section D as **single-frame point measurements**, not as a detection-performance study.
The honest claims are:

* "In the tested frame at 6 m, yolo26n/yolov8n detected person 1 with confidence 0.480 /
  0.791" — **not** "the system reliably detects people up to 6 m".
* There is **one** frame per (person, distance, model, `imgsz`) condition. There is no
  success rate, no repeated-frame test, no multi-view test, no occlusion test, and no
  additional-person test.
* A confidence value is a model score, not a calibrated probability. It says nothing about
  the probability of detection at that distance.
* The 8 m miss by the nano models must **not** be attributed to model capacity alone: the
  person is only ~14 × 49 px there, partly framed by rack structure, so apparent size,
  occlusion, viewpoint, preprocessing (resize to `imgsz`) and model capacity are all
  candidates. `yolo26s` recovering the same frame at 0.704 shows that capacity matters, but
  it does not apportion the causes.
* The depth agreement (±0.1 m in five conditions at 3.5–6 m) supports two things **in those
  conditions**: depth is metric Z along the optical axis, and matching pixel indices in the
  RGB and depth images correspond. It does **not** validate back-projection for off-centre,
  oblique or distant targets, does not test distortion (D = 0 here), and does not check
  alignment at other resolutions or after resizing.
* Timing was measured on a machine that was simultaneously running Gazebo and Nav2, with
  1–8 torch threads inside a container on an arm64 Mac; expect roughly ±30 % variation
  between runs and different absolute numbers on other hardware.
* **Corrections to earlier wording.** The previous report called the nano models' range a
  "genuine coverage constraint" of about 6 m, and described 8 m as a model-capacity limit;
  the limited evidence supports only "detected in the one tested frame at 6 m" and
  "capacity contributed at 8 m". It also quoted yolo11n reproducing 0.63 at 3.5 m as a
  duplicate-box example; that was one frame and is a single observation of two boxes on one
  person. Distances for person 2 were earlier written as 4.5/6.0 m in the script tags; the
  poses give **4.0 m and 5.95 m**, and the tables above use the pose-derived values.
* One earlier statement placed the annotated sheets at `/tmp/sheet_p1.png` on the host;
  those downscaled copies are gone. The originals live at `/tmp/cap/contact.png` and
  `/tmp/cap/contact_p2.png` **inside the container** and are ephemeral.

Claims that still need tests: repeated frames per condition (success rate over N frames);
several viewpoints per person; an occluded person; two people in one frame; added people;
a driving robot; and re-measuring any of this on the final integrated system.

### Stage-3 limitations and corrections

* **Deadlines are wall-clock, not sim time.** The stage-2 plan said "sim-time deadlines".
  Wall clock was chosen instead: simulation time is 0 before the first `/clock` and can pause
  or run slower than real time, and a deadline that never advances is a hang. The cost is
  that 300 s of deadline is 300 s of wall clock, which is more robot time when the real-time
  factor is below 1.
* **The deadline is a backstop, not the stuck detector.** Nav2's progress checker already
  aborts a goal that stops moving (0.5 m in 10 s, from the params file above); the deadline
  covers a result that never arrives.
* **Distances are straight-line and paths here are up to three times longer.** 16.2 m straight
  needed ~48 m of driving (run 2); the original `distance / 0.15 m/s` formula therefore
  cancelled a goal 3.7 m from arriving. The formula is now `30 s + 3 × distance / 0.25 m/s`,
  and run 3's legs finished at 18 %, 63 % and 40 % of their deadlines. Those margins are one
  run each, not a distribution.
* **Run 1 proves nothing about the map, localisation quality or Nav2 tuning** beyond "supply
  an initial pose before activating the stack".
* **The distance for each deadline is read from TF at the moment the goal is sent.** The TF
  buffer is only refreshed while `search` spins itself, and a Navigator call spins only its
  own node, so the code refreshes the pose at the start of `SEND_GOAL` and `SEND_HOME` rather
  than trusting the pose left over from the previous state (run 2's 23.9 m home distance was
  exactly that mistake).
* **The waypoints are hand-picked**, two of them, checked free in the static map (no occupied
  or unknown cell within 3 m of W1, 2.10 m of W2, 2.88 m of home). That check used the map
  file, not the live costmap, and the viewpoints are not derived from coverage requirements.
* **`gz model -m turtlebot4 -p` is not a dependable oracle here**: it returned nothing
  intermittently while the simulation was loading the CPU, so ground-truth checks also used
  `tf2_echo` and `/amcl_pose`.

---

## F. Reproducibility and evidence

### Stage 3 run (the numbers in section A)

```bash
# in the container, from /MRTP
source /opt/ros/jazzy/setup.bash && source MRTP/install/setup.bash
ros2 launch gazeboenvs tb4_warehouse.launch.py headless:=True > /tmp/stage3_sim.log 2>&1 &

cd CSE180 && source /opt/ros/jazzy/setup.bash && source install/setup.bash
colcon build --packages-select search

# only needed after a previous run: put the robot at the launch pose, re-seed AMCL
gz service -s /world/warehouse/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean \
  --req 'name: "turtlebot4", position: {x: 2.12, y: -21.3, z: 0.01},
         orientation: {x: 0, y: 0, z: 0.7071068, w: 0.7071068}'
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 2.12, y: -21.3, z: 0.0},
   orientation: {z: 0.7071068, w: 0.7071068}}}}"
# before starting, gz model -m turtlebot4 -p, tf2_echo map base_link and
# ros2 topic echo /amcl_pose --once must all read (2.120, -21.300, yaw 1.571)

ros2 run search search --ros-args -p use_sim_time:=true > /tmp/stage3_run.log 2>&1 &
```

Waypoint check against the static map
(`/opt/ros/jazzy/share/nav2_bringup/maps/warehouse.pgm`, 0.03 m/cell, origin (−15.1, −25)):
W1 (1.00, 2.50) free with no occupied or unknown cell within 3 m, W2 (−12.83, 11.09) free with
2.10 m, home (2.12, −21.30) free with 2.88 m. The same script confirmed the row indexing is
as expected: person 1 at (1, −1) is an occupied cell of the map itself, person 2 at (−12, 15)
is free with an occupied cell 0.12 m away.

**Evidence that is gone.** The run logs and the simulation log lived in the container's `/tmp`
(`stage3_sim.log`, `stage3_run.log`, `stage3_run2.log`, `stage3_run3.log`) and are not
committed; the tables in section A quote them. Nothing in this stage is a git commit.

* Experiment scripts are preserved verbatim in
  [`docs/experiments/`](docs/experiments/README.md), with their assumptions and their
  known gaps listed there.
* The compact result record `docs/experiments/yolo_result.json` holds the first single-frame
  result (one person box, conf 0.380, box `[140.4, 25.7, 173.4, 137.7]` → 33 × 112 px).
* Weights (`*.pt`) and captured frames (PNGs, `*.npy`, annotated contact sheets) were **not**
  committed: they are downloaded/generated artifacts and they lived under `/tmp`, which is
  ephemeral. Model weights are re-downloadable by ultralytics; the frames are not.
* The two `s`-model detections and their timing came from an inline heredoc typed into
  `docker exec` and were never saved as a file, so that part is **not** reproducible from
  this repository — only its settings and outputs are recorded.
* Nothing in this stage is a git commit: the stage deliberately stops before committing.

---

## G. Next steps

1. **Stage 3 — done** (this stage): the navigation-only state machine, two map-frame
   viewpoints, `Spin`, saved home pose, wall-clock deadlines with `CancelTask()`. Open items
   left for later: the bounded gate's timeout branch and Ctrl+C during a running goal were not
   exercised.
2. **Stage 4 — detector ROS package** (`ament_python`) publishing `Detection2DArray` from
   the latest frame at ~2 Hz, header copied from the source image, empty array when nothing
   is found. Requires the `detector` package, which does not exist yet.
3. **Stage 5 — stationary depth localisation.** Subscribe detections, depth and `CameraInfo`;
   back-project with `K`; transform to `map` with a bounded lookup at the image stamp (the TF
   buffer and the map-frame pose plumbing already exist in this stage); print the estimate
   against person 1's known world coordinates and check the ~0.5 m target while stationary and
   after a `Spin`.
4. **Stage 6 — merging and markers.** 0.6 m association, running average, distinct
   observation counting, tentative vs confirmed, `MarkerArray`.
5. **Stage 7 — integration**, perception live during navigation, return home, final report.
6. **Stage 8 — coverage and hard cases.** Waypoints derived from the map (reachability,
   71.6° HFOV, ~3–6 m observation distances, overlapping viewpoints, aisle ends), plus
   added/occluded people.

---

## Code provenance and reuse

### Reused in this stage

Paths are relative to the `CSE180` workspace root or to the course repository root, as named.

| Kind | Source | Name / block | What it does | Why appropriate | Changes |
|---|---|---|---|---|---|
| Library calls, used directly | `MRTP/src/navigation/` (package `navigation`, installed at `/MRTP/MRTP/install/navigation`) | `Navigator::SetInitialPose`, `WaitUntilNav2Active`, `GoToPose`, `Spin`, `IsTaskComplete`, `GetResult`, `CancelTask` (`include/navigation/navigation.hpp`, `src/navigation.cpp`) | Publishes the initial pose, waits for AMCL and bt_navigator to be active, sends `NavigateToPose` and `Spin` goals in `map`, polls the result and cancels the running goal. | Course-provided library; calling it instead of writing action-client plumbing is the intended reuse, and this stage exists to prove the calls work. | The calls are unchanged. They are wrapped by `wait_for_task()` (deadline + cancel) and preceded by a bounded lifecycle-service check, because the library's waits have no timeout. Constructed `Navigator(false, false)` — see section C. |
| Function, adapted | `CSE180/src/scout/src/scout.cpp` | `main()`: `tf_buffer = make_shared<tf2_ros::Buffer>(node->get_clock())`, `make_shared<tf2_ros::TransformListener>(*tf_buffer, node, true)` | TF buffer and listener for map-frame lookups. | Same requirement: read the robot pose in `map` while other work continues. | Listener built with `spin_thread = false` (the lab passes `true`): `search` spins its own node by hand, and a dedicated listener thread would put that node into a second executor. |
| Function, adapted | `scout.cpp` | `report_tf_recovery()` and the `catch (const tf2::TransformException &)` block of `timer_callback()` | Warns once when a TF lookup fails, logs recovery. | Detection cannot be localised without TF, and the "warn once, do not flood the log" behaviour is right here too. | No `/cmd_vel` consequence: a failed lookup leaves the last known pose unchanged. Wrapped in `refresh_robot_pose()`. |
| Function, adapted | `CSE180/src/patrol/src/patrol.cpp` | `odom_callback()`: quaternion → yaw, `atan2(2(qw·qz + qx·qy), 1 − 2(qy² + qz²))` | Robot yaw from its orientation quaternion. | Same conversion, already used and checked in this course. | Becomes the free function `yaw_of(Quaternion)`, also used to log the home heading. |
| Structural pattern | `scout.cpp` | `enum Phase { DETECT, APPROACH, RETURN, DONE }` and the `switch` in `timer_callback()` | An explicit, readable state machine. | Hands the project the simple style it asks for: named states, one action per transition. | States are the search sequence with separate send/wait states; transitions depend on `GetResult()` and a deadline instead of a timer tick. |
| Structural pattern | `patrol.cpp` | `waypoint_s_x[]`/`waypoint_s_y[]`, `waypoint_index`, "Reached W%d / Heading to W%d" logging, one-way advance | Waypoint table and index bookkeeping. | The state machine needs the same "one waypoint per completion, never cycle" accounting. | Waypoints are map-frame `{x, y, yaw, name}` entries sent with `GoToPose`; no `% 4` cycling, no `D_STOP` scan test, no `/cmd_vel`. |
| Idea, adapted | `scout.cpp` | `home_odom` saved when the target is found, `RETURN` phase | Remember where the robot started and go back. | The final leg of the search is exactly this. | Saved *before* anything moves, as a full map-frame pose with orientation read from `lookupTransform(map, base_link)`, restored with `Navigator::GoToPose`. |

No function body was copied verbatim from the labs: the adapted entries above are
re-implemented for map-frame poses and for the Navigator, and each is marked in the source.
What `search` adds is the deadline/cancel handling, the bounded readiness gate and the
map-frame waypoint plan, none of which exists in the labs.

### Structural patterns followed (not code reuse)

| Pattern | Where it comes from | Note |
|---|---|---|
| `cmake_minimum_required` / `project` / `CMAKE_CXX_STANDARD 17` / `-Wall -Wextra -Wpedantic` / `find_package(ament_cmake …)` / `add_executable` / `ament_target_dependencies` / `install(TARGETS … DESTINATION lib/${PROJECT_NAME})` / `ament_package()` | Standard ament_cmake boilerplate; the same shape as `CSE180/src/scout/CMakeLists.txt` and `CSE180/src/patrol/CMakeLists.txt` | This is ROS boilerplate, **not** code that originated in the labs. Additions: `find_package(navigation REQUIRED)` with `target_link_libraries(search navigation::navigator)`, and `geometry_msgs`/`tf2`/`tf2_ros` as dependencies. |
| `package.xml` format 3 with `ament_cmake` buildtool, `ament_lint_auto`/`ament_lint_common` test deps, the same maintainer/license placeholders | Same files | Again boilerplate, kept identical so the four lab packages and `search` stay consistent. The dependency list itself is not boilerplate: it contains only `rclcpp`, `geometry_msgs`, `tf2`, `tf2_ros` and `navigation`, i.e. what the code above actually uses. |
| One node created in `main()`, free functions rather than a node subclass | `gotogoal.cpp`, `patrol.cpp`, `scout.cpp` share this shape | Deliberate style choice: no custom node class, no extra abstraction. In this stage `main()` also owns the `Navigator` and the TF listener, and the state-machine loop replaces `rclcpp::spin(node)`. |
| `ament_cmake_copyright_FOUND` / `ament_cmake_cpplint_FOUND` set in `BUILD_TESTING` | `safedrive/CMakeLists.txt`, `gotogoal/CMakeLists.txt` | Prevent lint failures for a package that has no copyright header and is not a git repo of its own. |

### Planned reuse for later stages (not implemented yet)

| Kind | Source | Name / block | What it does | Planned change | Why |
|---|---|---|---|---|---|
| Adapted block | `CSE180/src/scout/src/scout.cpp` | `scan_callback()` (store the newest sample only) and `tf2::doTransform(PointStamped, …)` | Latest-sample storage and point transformation. | Input becomes `Detection2DArray` + depth + `CameraInfo`, and the transform is applied to a back-projected point at the image stamp instead of a laser point. | The "callback stores, state machine computes" split is what keeps perception off the Navigator's spins. |
| Idea only | `scout.cpp` | `D_PARK` stopping distance, `closest_distance` search loop | Stop at a fixed distance from the target. | Not planned for reuse: `search` has no reason to approach a person in stage 5–7; it observes from the viewpoint. | Prevents copying steering logic the project does not need. |
| None (new work) | — | — | — | Depth ring buffer keyed by stamp, back-projection, person association/merging, `MarkerArray`, `vision_msgs` plumbing, the detector node. | No lab contains anything equivalent. |

### New code that the project genuinely needs

Written in this stage: the search state machine with its send/wait states, the deadline and
cancel handling (`wait_for_task`), the bounded Nav2 readiness gate, the map-frame waypoint
table, pose refresh from TF, and the summary accounting. None of it comes from the labs.

Still to write, each only in the stage that needs it: depth ring buffer keyed by stamp
(exact-match then tolerance), back-projection, person observation estimation and merging
(0.6 m association, running average, distinct-observation counting), `MarkerArray`
publication, `vision_msgs` plumbing, and the detector node.

---

## Where the remaining functionality will go

`src/search.cpp` still holds no scaffolding for work that does not exist. The placement is:

* **State machine** — `run_search()`, already in place; `main()` keeps ownership of the
  `Navigator` so its blocking calls never collide with a spinning executor, and destroys it
  before `rclcpp::shutdown()` so the destructor's `CancelTask()` can still spin.
* **The observation step** — a new `START_LOOK`/`WAIT_LOOK` pair replaces the current
  `SEND_SPIN`/`WAIT_SPIN` when perception arrives; the spin stays part of it.
* **RGB / depth / `CameraInfo` subscriptions** — on the `search` node (or the detector's
  arrays), callbacks that only store the newest messages, as `scan_callback()` does in
  `scout.cpp`. They must not live on the Navigator's node.
* **Inference** — a ~2 Hz timer in the detector node (2 Hz ≈ 500 ms budget, against the
  measured 222–318 ms for `yolo26n` at `imgsz=480`).
* **Back-projection** — at the observation step, using the latest `CameraInfo` and a bounded
  timestamped `map ← optical_frame` lookup; the TF buffer itself already exists here.
* **Merging and markers** — free functions next to `search.cpp` when stage 6 starts.
