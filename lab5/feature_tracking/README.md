# Lab5: verified SIFT and LK course package

ROS package name: `lab_5`; directory name: `feature_tracking`.
Tested on the existing isolated VNAV ROS1 Noetic environment with OpenCV
4.13.0 and C++14. OpenCV >=4.4 is required by the `cv::SIFT` API; other
versions/platforms have not been runtime-verified. Stock Ubuntu Noetic's
OpenCV 4.2 alone is not a supported claim. Do not mix system cv_bridge with
an ABI-incompatible custom OpenCV. No dependency upgrades are needed here.

Dependencies: catkin, roscpp, rospy, sensor_msgs, image_transport, cv_bridge,
OpenCV core/features2d/calib3d/video/imgproc/imgcodecs/highgui;
roslaunch and rosbag for launch/playback. Python verification additionally
uses the existing ROS Python environment. Compiled SIFT/LK do not need contrib.

## Clean build on this server

Use a fresh shell (not a ROS2 shell). This creates a separate workspace,
without touching 005, existing build products or shared environments:

```bash
source /root/autodl-tmp/vnav2023/scripts/activate_lab5.sh
export CMAKE_PREFIX_PATH="$VNAV_ENV"
export ROS_PACKAGE_PATH="$VNAV_ENV/share"
LAB5_WS=$(mktemp -d /root/autodl-tmp/vnav2023/lab5/results/submission-ws-XXXXXX)
mkdir -p "$LAB5_WS/src"
cp -a /root/autodl-tmp/vnav2023/lab5/versions/006/submission/feature_tracking "$LAB5_WS/src/"
cd "$LAB5_WS"
catkin_make -DPYTHON_EXECUTABLE="$VNAV_ENV/bin/python" -DCMAKE_PREFIX_PATH="$VNAV_ENV" -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_BUILD_TYPE=Release -j4
source "$LAB5_WS/devel/setup.bash"
rospack find lab_5
```

On another machine, replace the source path with this package's actual path
and activate a compatible ROS1/OpenCV environment. The server activation
script is environment setup, not a portable dependency installer.

## Two-frame SIFT

`output_dir` is REQUIRED and must already exist. Run headlessly:

```bash
mkdir -p "$LAB5_WS/run-output/box"
roslaunch lab_5 two_frames_tracking.launch descriptor:=SIFT output_dir:="$LAB5_WS/run-output/box"
```

Expected box pair: 604/969 keypoints, 128 descriptor dimensions, 604 raw
matches, 101 ratio-filtered matches, 79 RANSAC inliers. Save CSV and PNGs.
After output is written, Ctrl-C ends the launch normally.

## Official sequences

Bags are external, not packaged. Input hashes are in ../evidence/INPUTS.json.
The launch uses `/use_sim_time=true`, rosbag `--clock`, and remaps the
TESSE image topic to `/camera/color/image_raw`. Do not play a second copy.

```bash
LAB5_DATA=/root/autodl-tmp/vnav2023/lab5/data/official
mkdir -p "$LAB5_WS/run-output/realsense-sift"
roslaunch lab_5 video_tracking.launch descriptor:=SIFT rate:=0.2 path_to_dataset:="$LAB5_DATA/30fps_424x240_2018-10-01-18-35-06.bag" output_dir:="$LAB5_WS/run-output/realsense-sift"
```

For TESSE, use `vnav-lab5-smooth-trajectory.bag`; for LK, change
`descriptor:=LK` and use a distinct pre-created output directory.
Repeat for SIFT/LK on both bags. After playback completes, Ctrl-C stops
only that launch. Do not use global kill commands. No Unity/controller is used.

Expected processed image/pair counts: RealSense 1584/1583, TESSE 1386/1385.
`events.csv` records accepted stamps, invalid images and duplicate rejection.
`sift_frames.csv`/`LK.csv` contain per-pair metrics. A failed fundamental
matrix estimate is recorded as invalid geometry with zero inliers; the
frame is preserved. TESSE LK has 1367 valid F estimates in 1385 pairs.

## Implementation scope and evidence

Compiled: src/feature_tracker.cpp, sift_feature_tracker.cpp,
lk_feature_tracker.cpp, course_entry.cpp. The executable is track_features.
Original track_features.cpp/self_flow.cpp/ORB/FAST/SURF source files are
uncompiled course reference/templates, NOT completed implementations.
Supplementary Python comparisons are historical evidence, not replacements
for unimplemented optional C++ trackers. SURF was skipped: nonfree unavailable.

Full-sequence evidence is from unchanged 005. Packaging-only 006 is separately
clean-built and smoke-tested; see ../evidence/CLEAN_RUNTIME_CHECK.json and
../evidence/PACKAGING_VERIFICATION.json. Report and figures are one level up.
