# `search` — final project (CSE180)

Package that will make the TurtleBot4 search the warehouse map for people, report where
they are, and drive back home.

**Status: stage 2 — package skeleton only.** There is no detector ROS package yet, and no
navigation behaviour. This README is the project's development record: it carries forward
the verified environment facts, the runtime interfaces, and the external detector
experiment, so later stages do not have to rediscover them.

---

## A. Current implementation status

### What this skeleton actually does

`src/search.cpp`:

1. initialises rclcpp and creates one node named `search`;
2. constructs the course `Navigator` (`Navigator navigator(true, false)`), linking against
   the MRTP package's exported `navigation::navigator` target;
3. logs that both steps succeeded, then `rclcpp::spin(node)` until shutdown;
4. on SIGINT, logs `shutdown requested, exiting` and returns 0.

That is all. It does not set an initial pose, send goals, subscribe to anything, look up a
transform, or process depth.

### What has been tested

| Check | Command | Result |
|---|---|---|
| Build (compile + link) | `colcon build --packages-select search` in `/MRTP/CSE180` | **pass**, `Finished <<< search [9.80s]`, exit 0, no warnings |
| Executable starts | `./install/search/lib/search/search --ros-args -p use_sim_time:=true` | **pass**; logs "search skeleton…", "Created instance of Navigator", "Navigator constructed…" |
| Clean shutdown | `kill -INT <pid>` after 4 s | **pass**; logs `signal_handler(SIGINT/SIGTERM)` then `shutdown requested, exiting`, exit code **0** |
| `ros2 run` entry point | `ros2 run search search --ros-args -p use_sim_time:=true` | **pass**; the same binary starts through `ros2 run` |

Simulation was **not** running during these tests (no `gz sim` / Nav2 processes; the stage
did not need it). Therefore this proves only that the package builds, that
`navigation::navigator` links and constructs, that the `use_sim_time` argument is accepted,
and that the process starts and stops cleanly. It proves nothing about navigation,
perception, TF, or the state machine.

### What remains unimplemented

Map-frame goals and `Spin`/`Backup`; the search state machine; RGB/depth subscriptions;
depth buffering and back-projection; person association/merging; `MarkerArray` output;
the detector node and detector integration; failure/timeout reporting; the return-home goal.

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

---

## F. Reproducibility and evidence

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

1. **Stage 3 — search state machine, navigation only.** `WAIT_READY → SEND_WAYPOINT →
   WAIT_WAYPOINT → START_LOOK → WAIT_LOOK → SEND_HOME → WAIT_HOME → DONE`, with the home
   pose saved in `map`, sim-time deadlines, `CancelTask()` on expiry, and explicit
   `ABORTED`/`CANCELED` handling. This validates the highest-risk interface (blocking
   Navigator calls, executor arrangement, `use_sim_time`) before any perception exists.
2. **Stage 4 — detector ROS package** (`ament_python`) publishing `Detection2DArray` from
   the latest frame at ~2 Hz, header copied from the source image, empty array when nothing
   is found. Requires the `detector` package, which does not exist yet.
3. **Stage 5 — stationary depth + TF2 localisation.** Subscribe detections, depth and
   `CameraInfo`; back-project with `K`; transform to `map` with a bounded lookup; print the
   estimate against person 1's known world coordinates and check the ~0.5 m target while
   stationary and while spinning.
4. **Stage 6 — merging and markers.** 0.6 m association, running average, distinct
   observation counting, tentative vs confirmed, `MarkerArray`.
5. **Stage 7 — integration**, perception live during navigation, return home, final report.
6. **Stage 8 — coverage and hard cases.** Waypoints derived from the map (reachability,
   71.6° HFOV, ~3–6 m observation distances, overlapping viewpoints, aisle ends), plus
   added/occluded people.

---

## Code provenance and reuse

### Reused in this stage

| Kind | Source | Name / block | What it does | Why appropriate | Changes |
|---|---|---|---|---|---|
| Library call, used directly | `MRTP/src/navigation/` (package `navigation`, installed at `/MRTP/MRTP/install/navigation`) | `Navigator::Navigator(bool debug, bool verbose)` from `include/navigation/navigation.hpp` / `src/navigation.cpp` | Creates the rclcpp node `navigator` with all Nav2 action clients, services, `/initialpose` publisher and `/amcl_pose` subscription. | It is the course-provided navigation library; calling it instead of writing action-client plumbing is exactly the intended reuse. | None. Constructed with `(true, false)` only so the library's own log line proves it ran. |

That is the **only** reuse of executable code in this stage. No function from
`scout.cpp`, `patrol.cpp`, `gotogoal.cpp` or `whatever.cpp` was copied into `search.cpp`.

### Structural patterns followed (not code reuse)

| Pattern | Where it comes from | Note |
|---|---|---|
| `cmake_minimum_required` / `project` / `CMAKE_CXX_STANDARD 17` / `-Wall -Wextra -Wpedantic` / `find_package(ament_cmake …)` / `add_executable` / `ament_target_dependencies` / `install(TARGETS … DESTINATION lib/${PROJECT_NAME})` / `ament_package()` | Standard ament_cmake boilerplate; the same shape as `CSE180/src/scout/CMakeLists.txt` and `CSE180/src/patrol/CMakeLists.txt` | This is ROS boilerplate, **not** code that originated in the labs. The only additions are `find_package(navigation REQUIRED)` and `target_link_libraries(search navigation::navigator)`. |
| `package.xml` format 3 with `ament_cmake` buildtool, `rclcpp` dependency, `ament_lint_auto`/`ament_lint_common` test deps, the same maintainer/license placeholders | Same files | Again boilerplate, kept identical so the four lab packages and `search` stay consistent. |
| One node created in `main()`, then `rclcpp::spin(node)` and `rclcpp::shutdown()`; free functions rather than a node subclass | `gotogoal.cpp`, `patrol.cpp`, `scout.cpp` share this shape | Deliberate style choice: no custom node class, no extra abstraction. Only the rclcpp idiom is shared. |
| `ament_cmake_copyright_FOUND` / `ament_cmake_cpplint_FOUND` set in `BUILD_TESTING` | `safedrive/CMakeLists.txt`, `gotogoal/CMakeLists.txt` | Prevent lint failures for a package that has no copyright header and is not a git repo of its own. |

### Planned reuse for later stages (not implemented yet)

| Kind | Source | Name / block | What it does | Planned change | Why |
|---|---|---|---|---|---|
| Setup block, reused unchanged | `CSE180/src/scout/src/scout.cpp` | `main()`: `tf_buffer = make_shared<tf2_ros::Buffer>(node->get_clock())`, `tf_listener = make_shared<tf2_ros::TransformListener>(*tf_buffer, node, true)` | TF buffer + listener with its own thread. | None expected; only `map` … `optical_frame` frames will be queried. | Same requirement: transform image-frame points into `map` while the executor keeps running. |
| Adapted function | `scout.cpp` | `report_tf_recovery()` + the `catch (const tf2::TransformException &)` block in `timer_callback()` | Logs a TF failure once, stops publishing motion, reports recovery. | No `/cmd_vel` at all in `search`; instead of stopping wheels it will skip the observation and keep its own failure counter. | Detection cannot be localised without TF; the "warn once, don't flood the log" behaviour is still right. |
| Adapted block | `scout.cpp` | `scan_callback()` (store newest scan only) and the `tf2::doTransform(PointStamped, …)` usage | Latest-sample storage and point transformation. | Input becomes `Detection2DArray` + depth + `CameraInfo`; the transform is applied to a back-projected point instead of a laser point. | The "callback stores, timer/state machine computes" split is what keeps perception off the Navigator's spins. |
| Structural pattern | `patrol.cpp` | `waypoint_odom_x/y[]`, `waypoint_index`, "Reached W%d / Heading to W%d" logging, one-way advance | Waypoint table and index bookkeeping. | Waypoints will be map-frame `PoseStamped`s sent with `Navigator::GoToPose`; no `% 4` cycling; no `D_STOP` scan check; no `/cmd_vel`. | The state machine needs the same "advance one waypoint per task completion" bookkeeping, nothing more. |
| Structural pattern | `scout.cpp` | `enum Phase { DETECT, APPROACH, RETURN, DONE }` and the free-function `timer_callback()` switch | Explicit, readable state machine. | States become the search sequence (`WAIT_READY`, `SEND_WAYPOINT`, …); transitions also depend on `GetResult()` and deadlines. | Matches the simple style of the labs, which is what the project asks for. |
| Idea, adapted | `scout.cpp` | `home_odom` saved on detection, `RETURN` phase | Remember where the robot started and go back. | Stored as a full map-frame pose **with orientation** from `lookupTransform("map", "base_link")`, restored with `Navigator::GoToPose`. | Nav2 needs a pose, not a point, and the map frame is the frame goals use. |

### New code that the project genuinely needs

Depth ring buffer keyed by stamp (exact-match then tolerance), person observation
estimation and merging (0.6 m association, running average, distinct-observation counting),
`MarkerArray` publication, `vision_msgs` plumbing, the concrete search state machine with
its deadline/cancel logic, and the detector node. None of the labs contains anything
equivalent, so this will be written for `search` — and only in the stage that needs it.

---

## Where the remaining functionality will go

`src/search.cpp` stays deliberately empty of unused scaffolding. The placement is:

* **State machine** — replaces `rclcpp::spin(node)` in `main()`; `main()` keeps ownership of
  the `Navigator` so its blocking calls never collide with a spinning executor.
* **RGB / depth / `CameraInfo` subscriptions** — a second node (`search_perception` or the
  detector's arrays), callbacks that only store the newest messages, exactly like
  `scan_callback()` in `scout.cpp`.
* **Inference** — a ~2 Hz timer in the detector node (2 Hz ≈ 500 ms budget, against the
  measured 222–318 ms for `yolo26n` at `imgsz=480`).
* **Back-projection and TF** — inside the state machine, at the observation state, using the
  latest `CameraInfo` and a bounded timestamped `map ← optical_frame` lookup.
* **Merging and markers** — free functions next to `search.cpp` when stage 6 starts.
