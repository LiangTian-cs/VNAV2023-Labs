# MIT VNAV 2023 — Lab 2

## Deliverable 1 — Nodes, Topics, and Launch Files

### 1. Nodes in the static scenario

The static scenario is launched with:

    roslaunch two_drones_pkg two_drones.launch static:=True

Ignoring `/rosout` and temporary rqt nodes, the relevant nodes are:

- `/av1broadcaster`
- `/av2broadcaster`
- `/plots_publisher_node`
- `/rviz`

The first two nodes are instances of `tf2_ros/static_transform_publisher`.

They provide the transforms

$$
world \rightarrow av1
$$

and

$$
world \rightarrow av2.
$$

The static transforms used by the launch file are:

$$
{}^w p_1 =
\begin{bmatrix}
1\\0\\0
\end{bmatrix},
\qquad
{}^w p_2 =
\begin{bmatrix}
0\\0\\1
\end{bmatrix},
$$

with identity orientation.

---

### 2. Running the static scenario without roslaunch

The equivalent nodes can be launched manually in separate terminals.

AV1 static transform:

    rosrun tf2_ros static_transform_publisher 1 0 0 0 0 0 1 world av1 __name:=av1broadcaster

AV2 static transform:

    rosrun tf2_ros static_transform_publisher 0 0 1 0 0 0 1 world av2 __name:=av2broadcaster

Visualization publisher:

    rosrun two_drones_pkg plots_publisher_node

RViz:

    rosrun rviz rviz -d $(rospack find two_drones_pkg)/config/default.rviz

---

### 3. Topics

#### `/av1broadcaster`

Publishes the static transform

$$
world \rightarrow av1
$$

on:

    /tf_static

#### `/av2broadcaster`

Publishes the static transform

$$
world \rightarrow av2
$$

on:

    /tf_static

#### `/plots_publisher_node`

The node contains a `tf2_ros::TransformListener`, so it receives TF data
from the TF system.

It publishes:

    /visuals

with message type:

    visualization_msgs/MarkerArray

The `/visuals` topic contains the quadrotor mesh markers and the trajectory
markers displayed by RViz.

#### `/rviz`

RViz subscribes to the visualization and TF information needed by its
configured displays, in particular:

    /visuals
    /tf
    /tf_static

The quadrotor meshes themselves are supplied by the markers published on
`/visuals`.

---

### 4. Effect of omitting `static:=True`

The launch file defines:

    <arg name="static" default="false"/>

Therefore, if `static:=True` is omitted, the value of `static` is false.

The group containing the two static transform publishers is guarded by:

    if="$(arg static)"

so those nodes only run when `static` is true.

The dynamic frame publisher is guarded by:

    unless="$(arg static)"

so `frames_publisher_node` runs when `static` is false.

Therefore:

- with `static:=True`, AV1 and AV2 have fixed transforms;
- without it, `frames_publisher_node` publishes the time-varying transforms.

The `plots_publisher_node` and RViz run in both cases.

---

# Deliverable 2 — Publishing Transforms

Implemented in:

    two_drones_pkg/src/frames_publisher_node.cpp

The trajectories specified by the problem are

$$
o_1^w(t)
=
\begin{bmatrix}
\cos t\\
\sin t\\
0
\end{bmatrix},
$$

and

$$
o_2^w(t)
=
\begin{bmatrix}
\sin t\\
0\\
\cos 2t
\end{bmatrix}.
$$

AV1 has

$$
\phi=0,\qquad
\theta=0,\qquad
\psi=t,
$$

so its rotation with respect to the world is

$$
R_1^w(t)
=
R_z(t)
=
\begin{bmatrix}
\cos t & -\sin t & 0\\
\sin t & \cos t & 0\\
0 & 0 & 1
\end{bmatrix}.
$$

The second column is

$$
\begin{bmatrix}
-\sin t\\
\cos t\\
0
\end{bmatrix},
$$

which is exactly the tangent direction of AV1's circular trajectory.

AV2 undergoes pure translation, therefore

$$
R_2^w=I_3.
$$

The implementation was compiled and runtime-tested successfully.

---

# Deliverable 3 — Looking Up a Transform

Implemented in:

    two_drones_pkg/src/plots_publisher_node.cpp

The relevant TF query is

    tf_buffer.lookupTransform(ref_frame, dest_frame, ros::Time(0))

which requests the latest available transform describing `dest_frame`
relative to `ref_frame`.

Runtime validation produced all three required trajectories:

- `Trail av1-world`, expressed in `world`;
- `Trail av2-world`, expressed in `world`;
- `Trail av2-av1`, expressed in `av1`.

The `/visuals` topic was observed at approximately 50 Hz and no transform
lookup errors occurred.

---

# Deliverable 4 — Mathematical Derivations

## 4.1 AV2 follows a parabola in the world x-z plane

AV2 has position

$$
o_2^w(t)
=
\begin{bmatrix}
\sin t\\
0\\
\cos 2t
\end{bmatrix}.
$$

Let

$$
x_w=\sin t,
\qquad
y_w=0,
\qquad
z_w=\cos 2t.
$$

Using

$$
\cos 2t=1-2\sin^2 t,
$$

and substituting

$$
x_w=\sin t,
$$

we obtain

$$
z_w=1-2x_w^2.
$$

Therefore AV2 lies in the plane

$$
y_w=0
$$

and satisfies

$$
\boxed{z_w=1-2x_w^2},
$$

which is a parabola in the world x-z plane.

---

## 4.2 Position of AV2 relative to AV1

The homogeneous transformation from AV1 coordinates to world coordinates is

$$
T_1^w
=
\begin{bmatrix}
R_1^w & o_1^w\\
0_{1\times3} & 1
\end{bmatrix},
$$

where

$$
R_1^w
=
\begin{bmatrix}
\cos t & -\sin t & 0\\
\sin t & \cos t & 0\\
0 & 0 & 1
\end{bmatrix}
$$

and

$$
o_1^w
=
\begin{bmatrix}
\cos t\\
\sin t\\
0
\end{bmatrix}.
$$

Thus

$$
T_1^w
=
\begin{bmatrix}
\cos t & -\sin t & 0 & \cos t\\
\sin t & \cos t & 0 & \sin t\\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1
\end{bmatrix}.
$$

AV2 has identity orientation, therefore

$$
T_2^w
=
\begin{bmatrix}
1 & 0 & 0 & \sin t\\
0 & 1 & 0 & 0\\
0 & 0 & 1 & \cos 2t\\
0 & 0 & 0 & 1
\end{bmatrix}.
$$

The inverse of $T_1^w$ is

$$
T_w^1
=
(T_1^w)^{-1}
=
\begin{bmatrix}
(R_1^w)^T & -(R_1^w)^T o_1^w\\
0 & 1
\end{bmatrix}.
$$

Since

$$
(R_1^w)^T
=
\begin{bmatrix}
\cos t & \sin t & 0\\
-\sin t & \cos t & 0\\
0 & 0 & 1
\end{bmatrix},
$$

we have

$$
(R_1^w)^T o_1^w
=
\begin{bmatrix}
1\\0\\0
\end{bmatrix}.
$$

Hence

$$
T_w^1
=
\begin{bmatrix}
\cos t & \sin t & 0 & -1\\
-\sin t & \cos t & 0 & 0\\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1
\end{bmatrix}.
$$

The transformation from AV2 to AV1 is

$$
T_2^1
=
T_w^1 T_2^w.
$$

Its translation component is

$$
o_2^1
=
(R_1^w)^T
\left(
o_2^w-o_1^w
\right).
$$

Now

$$
o_2^w-o_1^w
=
\begin{bmatrix}
\sin t-\cos t\\
-\sin t\\
\cos 2t
\end{bmatrix}.
$$

Therefore

$$
o_2^1(t)
=
\begin{bmatrix}
\cos t & \sin t & 0\\
-\sin t & \cos t & 0\\
0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
\sin t-\cos t\\
-\sin t\\
\cos 2t
\end{bmatrix}.
$$

For the first coordinate,

$$
x_2^1
=
\cos t(\sin t-\cos t)-\sin^2t
$$

$$
=
\sin t\cos t-\cos^2t-\sin^2t
$$

$$
=
-1+\sin t\cos t
$$

$$
=
-1+\frac{1}{2}\sin 2t.
$$

For the second coordinate,

$$
y_2^1
=
-\sin t(\sin t-\cos t)-\cos t\sin t
$$

$$
=
-\sin^2t
$$

$$
=
-\frac12+\frac12\cos2t.
$$

For the third coordinate,

$$
z_2^1=\cos2t.
$$

Thus

$$
\boxed{
o_2^1(t)
=
\begin{bmatrix}
-1+\frac12\sin2t\\
-\frac12+\frac12\cos2t\\
\cos2t
\end{bmatrix}
}.
$$

---

## 4.3 Plane containing the relative trajectory

From the previous result,

$$
y_2^1
=
-\frac12+\frac12\cos2t
$$

and

$$
z_2^1=\cos2t.
$$

Therefore

$$
2y_2^1=-1+\cos2t,
$$

so

$$
z_2^1=2y_2^1+1.
$$

Hence every point of the trajectory satisfies

$$
\boxed{z_1-2y_1-1=0}.
$$

The trajectory therefore lies entirely on the plane

$$
\boxed{\Pi:\ z_1-2y_1-1=0}.
$$

A normal vector of this plane is

$$
n=
\begin{bmatrix}
0\\-2\\1
\end{bmatrix}.
$$

---

## 4.4 A 2D reference frame on the plane

The center suggested in the assignment is

$$
p^1=
\begin{bmatrix}
-1\\
-\frac12\\
0
\end{bmatrix}.
$$

Choose the first in-plane unit axis as

$$
\hat{x}_p
=
\begin{bmatrix}
1\\0\\0
\end{bmatrix}.
$$

A second in-plane direction must be orthogonal to the plane normal.
A convenient choice is

$$
\hat{y}_p
=
\frac{1}{\sqrt5}
\begin{bmatrix}
0\\1\\2
\end{bmatrix}.
$$

Finally,

$$
\hat{z}_p
=
\hat{x}_p\times\hat{y}_p
=
\frac{1}{\sqrt5}
\begin{bmatrix}
0\\-2\\1
\end{bmatrix}.
$$

Therefore the rotation from the p frame to AV1 is

$$
R_p^1
=
\begin{bmatrix}
1 & 0 & 0\\
0 & \frac1{\sqrt5} & -\frac2{\sqrt5}\\
0 & \frac2{\sqrt5} & \frac1{\sqrt5}
\end{bmatrix}.
$$

The homogeneous transformation is

$$
T_p^1
=
\begin{bmatrix}
R_p^1 & p^1\\
0 & 1
\end{bmatrix}.
$$

Coordinates in the p frame are obtained from

$$
o_2^p
=
(R_p^1)^T
\left(
o_2^1-p^1
\right).
$$

First,

$$
o_2^1-p^1
=
\begin{bmatrix}
\frac12\sin2t\\
\frac12\cos2t\\
\cos2t
\end{bmatrix}.
$$

Therefore,

$$
x_2^p
=
\frac12\sin2t.
$$

For the second coordinate,

$$
y_2^p
=
\frac1{\sqrt5}
\left(
\frac12\cos2t
\right)
+
\frac2{\sqrt5}
\cos2t
$$

$$
=
\frac{\sqrt5}{2}\cos2t.
$$

For the third coordinate,

$$
z_2^p
=
-\frac2{\sqrt5}
\left(
\frac12\cos2t
\right)
+
\frac1{\sqrt5}\cos2t
=0.
$$

Hence

$$
\boxed{
o_2^p(t)
=
\begin{bmatrix}
\frac12\sin2t\\
\frac{\sqrt5}{2}\cos2t\\
0
\end{bmatrix}
}.
$$

The zero third coordinate confirms that the trajectory lies in the
$x_p-y_p$ plane.

---

## 4.5 Ellipse equation and semi-axes

From

$$
x_p=\frac12\sin2t,
$$

we obtain

$$
\sin2t=2x_p.
$$

From

$$
y_p=\frac{\sqrt5}{2}\cos2t,
$$

we obtain

$$
\cos2t=\frac{2y_p}{\sqrt5}.
$$

Using

$$
\sin^2 2t+\cos^2 2t=1,
$$

gives

$$
(2x_p)^2+
\left(
\frac{2y_p}{\sqrt5}
\right)^2
=1.
$$

Therefore

$$
\boxed{
4x_p^2+\frac45y_p^2=1
}.
$$

Equivalently,

$$
\boxed{
\frac{x_p^2}{(1/2)^2}
+
\frac{y_p^2}{(\sqrt5/2)^2}
=1
}.
$$

Thus the two semi-axis lengths are

$$
\boxed{
a=\frac{\sqrt5}{2},
\qquad
b=\frac12
}.
$$

The major axis is along $y_p$ and the minor axis is along $x_p$.

---

# Deliverable 5 — Quaternion Properties

We use the convention

$$
q=
\begin{bmatrix}
q_1\\q_2\\q_3\\q_4
\end{bmatrix},
$$

where $q_4$ is the scalar component.

The two matrices are

$$
\Omega_1(q)
=
\begin{bmatrix}
q_4&-q_3&q_2&q_1\\
q_3&q_4&-q_1&q_2\\
-q_2&q_1&q_4&q_3\\
-q_1&-q_2&-q_3&q_4
\end{bmatrix},
$$

and

$$
\Omega_2(q)
=
\begin{bmatrix}
q_4&q_3&-q_2&q_1\\
-q_3&q_4&q_1&q_2\\
q_2&-q_1&q_4&q_3\\
-q_1&-q_2&-q_3&q_4
\end{bmatrix}.
$$

Quaternion multiplication can be written as

$$
q_a\otimes q_b
=
\Omega_1(q_a)q_b
=
\Omega_2(q_b)q_a.
$$

---

## 5.1 Orthogonality of Omega1 and Omega2

The quaternion norm is multiplicative:

$$
\|q_a\otimes q_b\|
=
\|q_a\|\,\|q_b\|.
$$

Let $q$ be a unit quaternion and let $x$ be any quaternion vector.
Then

$$
\|\Omega_1(q)x\|
=
\|q\otimes x\|
=
\|q\|\,\|x\|
=
\|x\|.
$$

Therefore the linear map $\Omega_1(q)$ preserves the Euclidean norm for
every $x\in\mathbb{R}^4$.

A real square matrix that preserves the Euclidean norm is orthogonal.
Hence

$$
\boxed{
\Omega_1(q)^T\Omega_1(q)
=
\Omega_1(q)\Omega_1(q)^T
=
I_4
}.
$$

Similarly,

$$
\|\Omega_2(q)x\|
=
\|x\otimes q\|
=
\|x\|\,\|q\|
=
\|x\|,
$$

and therefore

$$
\boxed{
\Omega_2(q)^T\Omega_2(q)
=
\Omega_2(q)\Omega_2(q)^T
=
I_4
}.
$$

The intuitive reason is that multiplication by a unit quaternion does not
change quaternion norm, so left and right multiplication act as
norm-preserving linear transformations in $\mathbb{R}^4$.

---

## 5.2 Omega1(q)^T q and Omega2(q)^T q

Let

$$
e_4=
\begin{bmatrix}
0\\0\\0\\1
\end{bmatrix}.
$$

From the definitions of both matrices, their fourth columns are equal to
$q$. Therefore

$$
\Omega_1(q)e_4=q,
$$

and

$$
\Omega_2(q)e_4=q.
$$

Since the matrices are orthogonal,

$$
\Omega_1(q)^T\Omega_1(q)=I_4.
$$

Multiplying

$$
q=\Omega_1(q)e_4
$$

from the left by $\Omega_1(q)^T$ gives

$$
\boxed{
\Omega_1(q)^Tq=e_4
}.
$$

The same argument gives

$$
\boxed{
\Omega_2(q)^Tq=e_4
}.
$$

Hence

$$
\boxed{
\Omega_1(q)^Tq
=
\Omega_2(q)^Tq
=
\begin{bmatrix}
0\\0\\0\\1
\end{bmatrix}
}.
$$

This is the unit quaternion corresponding to the identity rotation.

---

## 5.3 Commutation of the two operators

Let $z\in\mathbb{R}^4$ be arbitrary.

Because $\Omega_1(x)$ represents left quaternion multiplication by $x$,

$$
\Omega_1(x)z=x\otimes z.
$$

Because $\Omega_2(y)$ represents right multiplication by $y$,

$$
\Omega_2(y)z=z\otimes y.
$$

Therefore

$$
\Omega_1(x)\Omega_2(y)z
=
x\otimes(z\otimes y).
$$

Quaternion multiplication is associative, so

$$
x\otimes(z\otimes y)
=
(x\otimes z)\otimes y.
$$

Thus

$$
(x\otimes z)\otimes y
=
\Omega_2(y)\Omega_1(x)z.
$$

Since this holds for every $z$,

$$
\boxed{
\Omega_1(x)\Omega_2(y)
=
\Omega_2(y)\Omega_1(x)
}.
$$

Now define the quaternion conjugate

$$
\bar y=
\begin{bmatrix}
-y_1\\
-y_2\\
-y_3\\
y_4
\end{bmatrix}.
$$

Directly from the definition of $\Omega_2$,

$$
\Omega_2(y)^T=\Omega_2(\bar y).
$$

The first commutation result is valid for every vector in
$\mathbb{R}^4$, so it also holds for $\bar y$:

$$
\Omega_1(x)\Omega_2(\bar y)
=
\Omega_2(\bar y)\Omega_1(x).
$$

Substituting

$$
\Omega_2(\bar y)=\Omega_2(y)^T
$$

gives

$$
\boxed{
\Omega_1(x)\Omega_2(y)^T
=
\Omega_2(y)^T\Omega_1(x)
}.
$$

This completes Deliverable 5.

---

# Required Lab 2 Status

- Deliverable 1: complete
- Deliverable 2: complete and runtime-tested
- Deliverable 3: complete and runtime-tested
- Deliverable 4: complete
- Deliverable 5: complete
- Deliverable 6: optional, not included

The required Lab 2 deliverables are complete.
