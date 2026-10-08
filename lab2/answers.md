

# MIT VNAV 2023 - Lab 2 Solutions

**Topic:** ROS, TF, rigid-body transformations, and quaternions  
**Deliverables:** D1-D5 completed; optional D6 not submitted.

This file deliberately uses GitHub-flavored Markdown, ordinary text, and
plain-text mathematical notation. No MathJax, LaTeX commands, special Markdown
extensions, or external equation renderers are required. The ROS C++ source
files are in `two_drones_pkg/src/`.

## Deliverable 1 - Nodes, Topics, and Launch Files (10 points)

### D1.1 - Nodes in the static scenario

Start the static configuration with:

```bash
roslaunch two_drones_pkg two_drones.launch static:=True
```

Ignoring `/rosout` and temporary rqt nodes, the relevant nodes are:

| Node                    | Function                                                |
| ----------------------- | ------------------------------------------------------- |
| `/av1broadcaster`       | Static TF publisher for `world -> av1`                  |
| `/av2broadcaster`       | Static TF publisher for `world -> av2`                  |
| `/plots_publisher_node` | Looks up transforms and publishes visualization markers |
| `/rviz`                 | Displays the markers and coordinate frames              |

The two static broadcasters are instances of `tf2_ros/static_transform_publisher`.
Their translations and orientations are:

```text
world -> av1: translation = [1, 0, 0], rotation = identity
world -> av2: translation = [0, 0, 1], rotation = identity
```

The TF tree has `world` as the parent of both `av1` and `av2`.

### D1.2 - Starting the static scenario without roslaunch

Use separate terminals with the appropriate ROS workspace sourced.
Start the ROS master first (if it is not already running):

```bash
roscore
```

Then start the two static broadcasters:

```bash
rosrun tf2_ros static_transform_publisher 1 0 0 0 0 0 1 world av1 __name:=av1broadcaster
```

```bash
rosrun tf2_ros static_transform_publisher 0 0 1 0 0 0 1 world av2 __name:=av2broadcaster
```

Start the marker publisher and RViz in their own terminals:

```bash
rosrun two_drones_pkg plots_publisher_node
```

```bash
rosrun rviz rviz -d "$(rospack find two_drones_pkg)/config/default.rviz"
```

### D1.3 - Published and subscribed topics

| Node                    | Publishes                                              | Subscribes / consumes                                        |
| ----------------------- | ------------------------------------------------------ | ------------------------------------------------------------ |
| `/av1broadcaster`       | `/tf_static`: `world -> av1`                           | No user-defined input topic                                  |
| `/av2broadcaster`       | `/tf_static`: `world -> av2`                           | No user-defined input topic                                  |
| `/plots_publisher_node` | `/visuals` (`visualization_msgs/MarkerArray`)          | TF data through `tf2_ros::TransformListener`, normally `/tf` and `/tf_static` |
| `/rviz`                 | Not required to publish a user-defined data topic here | `/visuals` and the relevant TF topics according to enabled displays |

Both drone coordinate frames are published by the static broadcasters.
The drone meshes are visualization markers published by
`/plots_publisher_node` on `/visuals`; RViz displays them by subscribing to
that topic.

### D1.4 - Omitting `static:=True`

The launch file declares:

```xml
<arg name="static" default="false"/>
```

The static publishers are inside an `if="$(arg static)"` group; the
`frames_publisher_node` is inside an `unless="$(arg static)"` group.
Therefore:

- With `static:=True`: the two drones keep fixed positions and orientations.
- With the default `static:=false`: `frames_publisher_node` publishes moving
  `world -> av1` and `world -> av2` transforms on `/tf`.
- `plots_publisher_node` and RViz are launched in both cases.

---

## Deliverable 2 - Publishing Dynamic Transforms (30 points)

**Implementation:** `two_drones_pkg/src/frames_publisher_node.cpp`.

The ROS node periodically fills two `geometry_msgs::TransformStamped`
messages and publishes them via a TF broadcaster. Its timer callback runs
at approximately 50 Hz.

### D2.1 - AV1 translation in world

```text
origin_AV1_in_world(t) = [cos(t), sin(t), 0]^T
```

AV1 travels on the unit circle in the world x-y plane.

### D2.2 - AV1 orientation in world

The roll and pitch are zero; the yaw is the current time `t`:

```text
roll = 0, pitch = 0, yaw = t

R_AV1_to_world(t) = Rz(t)

                    [ cos(t)  -sin(t)  0 ]
                    [ sin(t)   cos(t)  0 ]
                    [   0        0     1 ]
```

The second column, `[-sin(t), cos(t), 0]^T`, is the tangent to AV1's
circle, so the body y-axis points in its direction of travel.
A TF quaternion can be built from roll, pitch, and yaw using
`tf2::Quaternion::setRPY(0, 0, t)`.

### D2.3 - AV2 translation and orientation

```text
origin_AV2_in_world(t) = [sin(t), 0, cos(2t)]^T
R_AV2_to_world(t) = I_3  (identity orientation)
```

AV2 translates without rotating. The relative frame structure is:

```text
       world
       /   \
     av1   av2
```

The implementation was compiled and tested in the running ROS scenario.

---

## Deliverable 3 - Looking Up a Relative Transform (30 points)

**Implementation:** `two_drones_pkg/src/plots_publisher_node.cpp`.

The required TF query is:

```cpp
tf_buffer.lookupTransform(ref_frame, dest_frame, ros::Time(0));
```

`ref_frame` is the coordinate system in which the transform is expressed,
`dest_frame` is the frame being looked up, and `ros::Time(0)` asks for the
latest available transform.

For example, the AV2 pose relative to AV1 is obtained by the transformation:

```text
T_AV2_to_AV1 = inverse(T_AV1_to_world) * T_AV2_to_world
```

Three trajectories were observed:

| Marker            | Expressed in | Geometric shape                     |
| ----------------- | ------------ | ----------------------------------- |
| `Trail av1-world` | `world`      | Circle in the x-y plane             |
| `Trail av2-world` | `world`      | Parabolic arc in the x-z plane      |
| `Trail av2-av1`   | `av1`        | Closed ellipse on an inclined plane |

The `/visuals` topic ran at approximately 50 Hz during validation, with no
transform lookup errors.

---

## Deliverable 4 - Mathematical Derivations (25 points)

**Notation:**

- `o1_w`: origin of AV1 expressed in the world frame.
- `o2_w`: origin of AV2 expressed in the world frame.
- `o2_1`: origin of AV2 expressed in the AV1 frame.
- `T1_w`: homogeneous transformation from AV1 coordinates to world coordinates.
- `T2_w`: homogeneous transformation from AV2 coordinates to world coordinates.
- `T2_1`: homogeneous transformation from AV2 coordinates to AV1 coordinates.
- `^T`: matrix transpose; `^-1`: matrix inverse.

### D4.1 - AV2 moves on a parabolic arc in world

The world-frame position of AV2 is:

```text
x_w = sin(t)
y_w = 0
z_w = cos(2t)
```

By the double-angle identity:

```text
cos(2t) = 1 - 2*sin(t)^2
```

Substitute `x_w = sin(t)`:

```text
z_w = 1 - 2*x_w^2
y_w = 0
```

Thus AV2 lies in the world x-z plane and traces a segment of the parabola
`z_w = 1 - 2*x_w^2`, where `-1 <= x_w <= 1`.

### D4.2 - AV2 position in the AV1 body frame

First write the two world-frame homogeneous transforms:

```text
T1_w(t) =
[ cos(t)  -sin(t)  0  cos(t) ]
[ sin(t)   cos(t)  0  sin(t) ]
[    0        0    1     0   ]
[    0        0    0     1   ]

T2_w(t) =
[ 1  0  0     sin(t) ]
[ 0  1  0       0    ]
[ 0  0  1    cos(2t) ]
[ 0  0  0       1    ]
```

For a rigid transform `T = [R, p; 0, 1]`, its inverse is
`T^-1 = [R^T, -R^T*p; 0, 1]`.

The inverse of AV1's transform is:

```text
inverse(T1_w) = T_w1 =
[  cos(t)   sin(t)  0  -1 ]
[ -sin(t)   cos(t)  0   0 ]
[     0        0    1   0 ]
[     0        0    0   1 ]
```

The relative transform is:

```text
T2_1 = inverse(T1_w) * T2_w
```

Its translation is equivalently:

```text
o2_1 = transpose(R1_w) * (o2_w - o1_w)

     = [  cos(t)   sin(t)  0 ]   [ sin(t)-cos(t) ]
       [ -sin(t)   cos(t)  0 ] * [     -sin(t)   ]
       [     0        0    1 ]   [     cos(2t)  ]
```

Work through each coordinate:

```text
x_1 = cos(t)*(sin(t)-cos(t)) - sin(t)^2
    = sin(t)*cos(t) - 1
    = -1 + (1/2)*sin(2t)

y_1 = -sin(t)*(sin(t)-cos(t)) - cos(t)*sin(t)
    = -sin(t)^2
    = -1/2 + (1/2)*cos(2t)

z_1 = cos(2t)
```

Therefore the requested position is:

```text
o2_1(t) = [ -1 + (1/2)*sin(2t),
             -1/2 + (1/2)*cos(2t),
              cos(2t) ]^T
```

### D4.3 - Plane containing the relative trajectory

From the previous result:

```text
y_1 = -1/2 + (1/2)*cos(2t)
z_1 = cos(2t)
```

Eliminate `t`:

```text
2*y_1 = -1 + cos(2t)
z_1 = 2*y_1 + 1
```

Thus the plane containing the trajectory is:

```text
Pi: z_1 - 2*y_1 - 1 = 0
```

Its normal vector is `[0, -2, 1]^T`. Hence the complete relative
trajectory is planar.

### D4.4 - Define a centered 2D coordinate frame on the plane

Place the new frame origin at the ellipse center, expressed in AV1:

```text
p_1 = [-1, -1/2, 0]^T
```

Select three mutually orthogonal unit axes:

```text
x_hat_p = [1, 0, 0]^T

y_hat_p = [0, 1, 2]^T / sqrt(5)

z_hat_p = cross(x_hat_p, y_hat_p)
        = [0, -2, 1]^T / sqrt(5)
```

The rotation matrix and homogeneous transform from the plane frame to
AV1 are:

```text
R_p_to_1 =
[ 1       0            0      ]
[ 0    1/sqrt(5)   -2/sqrt(5) ]
[ 0    2/sqrt(5)    1/sqrt(5) ]

T_p_to_1 =
[ 1       0            0        -1   ]
[ 0    1/sqrt(5)   -2/sqrt(5)   -1/2 ]
[ 0    2/sqrt(5)    1/sqrt(5)    0   ]
[ 0       0            0         1   ]
```

Transform the relative position into the plane frame:

```text
o2_p = transpose(R_p_to_1) * (o2_1 - p_1)

o2_1 - p_1 = [(1/2)*sin(2t), (1/2)*cos(2t), cos(2t)]^T

x_p = (1/2)*sin(2t)

y_p = (1/sqrt(5))*(1/2)*cos(2t)
      + (2/sqrt(5))*cos(2t)
    = (sqrt(5)/2)*cos(2t)

z_p = (-2/sqrt(5))*(1/2)*cos(2t)
      + (1/sqrt(5))*cos(2t)
    = 0
```

The coordinates in the new frame are:

```text
o2_p(t) = [(1/2)*sin(2t), (sqrt(5)/2)*cos(2t), 0]^T
```

The vanishing third component confirms the curve lies in the new x-y
plane, centered at its origin.

### D4.5 - Ellipse equation and semi-axis lengths

From D4.4:

```text
sin(2t) = 2*x_p
cos(2t) = 2*y_p/sqrt(5)
```

Use `sin(2t)^2 + cos(2t)^2 = 1`:

```text
(2*x_p)^2 + (2*y_p/sqrt(5))^2 = 1

4*x_p^2 + (4/5)*y_p^2 = 1

x_p^2/(1/2)^2 + y_p^2/(sqrt(5)/2)^2 = 1
```

This is an axis-aligned ellipse with:

```text
semi-major axis = sqrt(5)/2  (along y_p)
semi-minor axis = 1/2        (along x_p)
```

---

## Deliverable 5 - Quaternion Properties (5 points)

Use the scalar-last convention:

```text
q = [q1, q2, q3, q4]^T
```

Here `q4` is the scalar component. The matrices defined by the course
are:

```text
Omega1(q) =
[  q4  -q3   q2   q1 ]
[  q3   q4  -q1   q2 ]
[ -q2   q1   q4   q3 ]
[ -q1  -q2  -q3   q4 ]

Omega2(q) =
[  q4   q3  -q2   q1 ]
[ -q3   q4   q1   q2 ]
[  q2  -q1   q4   q3 ]
[ -q1  -q2  -q3   q4 ]
```

For quaternions `qa` and `qb`, the product satisfies:

```text
qa (x) qb = Omega1(qa)*qb = Omega2(qb)*qa
```

Here `(x)` denotes quaternion multiplication (not the vector cross
product). `I4` is the 4-by-4 identity matrix.

### D5.1 - Orthogonality of Omega1 and Omega2

The quaternion norm is multiplicative:

```text
norm(qa (x) qb) = norm(qa) * norm(qb)
```

Let `q` be a unit quaternion, so `norm(q) = 1`. For every four-component
quaternion `v`:

```text
norm(Omega1(q)*v) = norm(q (x) v)
                  = norm(q)*norm(v)
                  = norm(v)

norm(Omega2(q)*v) = norm(v (x) q)
                  = norm(v)*norm(q)
                  = norm(v)
```

Thus both linear maps preserve the Euclidean norm of every vector in
four-dimensional space. Since their matrices are square, both are
orthogonal:

```text
transpose(Omega1(q))*Omega1(q) = I4
Omega1(q)*transpose(Omega1(q)) = I4

transpose(Omega2(q))*Omega2(q) = I4
Omega2(q)*transpose(Omega2(q)) = I4
```

Intuitively, left or right multiplication by a unit quaternion is a
length-preserving transformation.

### D5.2 - Mapping q to the identity quaternion

Define the identity quaternion:

```text
e4 = [0, 0, 0, 1]^T
```

The fourth column of **both** `Omega1(q)` and `Omega2(q)` is `q`. Hence:

```text
Omega1(q)*e4 = q
Omega2(q)*e4 = q
```

By the orthogonality established in D5.1:

```text
transpose(Omega1(q))*q
  = transpose(Omega1(q))*Omega1(q)*e4
  = I4*e4
  = e4

transpose(Omega2(q))*q
  = transpose(Omega2(q))*Omega2(q)*e4
  = I4*e4
  = e4
```

Therefore:

```text
transpose(Omega1(q))*q = transpose(Omega2(q))*q
                       = [0, 0, 0, 1]^T
```

This is the identity rotation quaternion in scalar-last convention.

### D5.3 - Commutation of left and right multiplication operators

Let `x`, `y`, and `z` be arbitrary four-component quaternions. By the
operator definitions:

```text
Omega1(x)*z = x (x) z
Omega2(y)*z = z (x) y
```

Associativity of quaternion multiplication gives:

```text
Omega1(x)*Omega2(y)*z
  = x (x) (z (x) y)
  = (x (x) z) (x) y
  = Omega2(y)*Omega1(x)*z
```

Since this holds for every `z`:

```text
Omega1(x)*Omega2(y) = Omega2(y)*Omega1(x)
```

To prove the transposed version, let the quaternion conjugate of `y` be:

```text
conj(y) = [-y1, -y2, -y3, y4]^T
```

Direct inspection of the defined matrices shows:

```text
transpose(Omega2(y)) = Omega2(conj(y))
```

Apply the first commutation identity with `conj(y)` in place of `y`:

```text
Omega1(x)*transpose(Omega2(y))
  = Omega1(x)*Omega2(conj(y))
  = Omega2(conj(y))*Omega1(x)
  = transpose(Omega2(y))*Omega1(x)
```

Hence both required commutation identities hold. They rely on the
**associativity** of quaternion multiplication, not its commutativity.

---

## Optional Deliverable 6 - Intrinsic vs. Extrinsic Rotations

D6 is optional (+20 points) and was **not** included in this submission.

## Completion and Reproducibility Notes

| Required deliverable               | Status                       | Where to inspect                                     |
| ---------------------------------- | ---------------------------- | ---------------------------------------------------- |
| D1 - ROS nodes, topics, and launch | Completed                    | `two_drones_pkg/launch/two_drones.launch`; this file |
| D2 - Dynamic transforms            | Completed and runtime-tested | `two_drones_pkg/src/frames_publisher_node.cpp`       |
| D3 - Relative transform lookup     | Completed and runtime-tested | `two_drones_pkg/src/plots_publisher_node.cpp`        |
| D4 - Mathematical derivations      | Completed                    | This file, D4.1-D4.5                                 |
| D5 - Quaternion proofs             | Completed                    | This file, D5.1-D5.3                                 |

Build and runtime verification evidence is recorded separately in
`VALIDATION.md`. These plain-text derivations have the same mathematical
content as the typeset Lab 2 report; use the typeset report when formally
submitting mathematical working if required by the instructor.
