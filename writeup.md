## Project: Kinematics Pick & Place

---

[//]: # (Image References)

[pick_gazebo]: ./writeup_material/pick_gazebo.png
[rviz_inefficient]: ./writeup_material/rviz_inefficient.png
[fk]: ./writeup_material/fk.png
[theta1]: ./writeup_material/theta1.jpg
[theta23]: ./writeup_material/theta23.jpg

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points 

Parts of the code and theory are refer to the walk-through video.
https://www.youtube.com/watch?v=Gt8DRm-REt4

---

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

I derive DH parameters base on the following image. Twist angle alpha(i-1) is the angle between Zi-1 to Zi measured about Xi-1. Link length a(i-1) is the  distance from Zi-1 to Zi measured along Xi-1. Offset length d(i) is the distance from Xi-1 to Xi measured along Zi. Joint angle q(i) is the angle between Xi-1 to Xi measured about Zi.

![alt text][fk]

We use `Sympy` to do __Symbolic Computation__. When using this, we can review equations with symbols which is easier for human to understand what's going on. Meanwhile, `Sympy` keeps Symbolic representation, and yield the actual number till you want. This can reduce error accumulation from a series of calculations and a limit precision.

Here is an example from `Sympy` doc.

```
import math
math.sqrt(8) # 2.82842712475

import sympy
sympy.sqrt(8) # 2*sqrt(2)
```

Although the precision is down to 11 digits after the point for `math`, it still lose precision after 12 digits. On the other hand, `Sympy` continues to use symbols without losing information.

We create symbols for DH parameters.

```
# Create symbols
# Twist angle
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
# Link length
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') # link length
# Offset length
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') # link offset
# Joint angle
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
```

Here is the DH table combined values from the kr210.urdf.xacro file.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1  | 0       | 0 | 0.75 | q1
1->2  | - pi/2. | 0.35 | 0 | q2 - pi/2.
2->3  | 0       | 1.25 | 0 | q3
3->4  | -pi/2.  | -0.054 | 1.5 | q4
4->5  | pi/2.   | 0 | 0 | q5
5->6  | -pi/2.  | 0 | 0 | q6
6->EE | 0       | 0 | 0.303 | 0

```
# Create DH parameters
dh = {  
    alpha0:      0,   a0:     0, d1:  0.75,  q1: q1,
    alpha1: -pi/2.,   a1:  0.35, d2:     0,  q2: q2 - pi/2.,
    alpha2:      0,   a2:  1.25, d3:     0,  q3: q3,
    alpha3: -pi/2.,   a3:-0.054, d4:   1.5,  q4: q4,
    alpha4:  pi/2.,   a4:     0, d5:     0,  q5: q5,
    alpha5: -pi/2.,   a5:     0, d6:     0,  q6: q6,
    alpha6:      0,   a6:     0, d7: 0.303,  q7: 0
}
```

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

With DH parameters, we can get transformation matrices from the following equation `tf_matrix(alpha, a, d, q)`. It describes transformation between two links, and it is just the combination of 2 rotations(joint and twist angles) and 2 transformations(link and offset lengths).

```
# Define Modified DH Transformation matrix
def tf_matrix(alpha, a, d, q):
    return Matrix([ 
        [            cos(q),           -sin(q),          0,                  a],
        [ sin(q)*cos(alpha), cos(q)*cos(alpha),  -sin(alpha),  -sin(alpha) * d],
        [ sin(q)*sin(alpha), cos(q)*sin(alpha),   cos(alpha),   cos(alpha) * d],
        [                 0,                 0,            0,                1]
    ])
```

Here, we adopt intrinsic rotations, and connect transformations from the base link to the end-effector by multiplications.

```
# Create individual transformation matrices
T0_1 = tf_matrix(alpha0, a0, d1, q1).subs(dh)
T1_2 = tf_matrix(alpha1, a1, d2, q2).subs(dh)
T2_3 = tf_matrix(alpha2, a2, d3, q3).subs(dh)
T3_4 = tf_matrix(alpha3, a3, d4, q4).subs(dh)
T4_5 = tf_matrix(alpha4, a4, d5, q5).subs(dh)
T5_6 = tf_matrix(alpha5, a5, d6, q6).subs(dh)
T6_EE =tf_matrix(alpha6, a6, d7, q7).subs(dh)
# Extract rotation matrices from the transformation matrices
T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE
```

With all joint angles, the rotation and translation of the end-effector with respect to the base link can be calculated, and these are the forward kinematics.

```
FK = T0_EE.evalf(subs = { q1: theta1, q2: theta2, q3: theta3,q4: theta4,q5: theta5,q6: theta6 })
```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

For inverse Kinematics, we can decompose into WC(wrist center) position, and WC to EE(end-effector) orientation problems. 

First, we can solve the position of WC base on vector algerbra:

```
Vector(0->EE) = Vector(0->WC) + Vector(WC->EE)
```

 `Vector(0->EE)` is the vector from origin to the EE, and we can represent it using the position of EE: `EE = Matrix([[px], [py], [pz]])`.

On the other hand, The magnitude of `Vector(WC->EE)` is offset length from link 6 to EE; the direction is the same as the Z-axis of the EE relative to the base frame.

Therefore, WC position is:

```
WC = EE - (d7) * ROT_EE[:, 2]
```

`Theta1`, `theta2` and `theta3` are explained in the following graph. `Theta1` can be solved by WC projection to x-y plane, and `theta2` & `theta3` can be solved by trigonometry and geometry.

![alt text][theta1]

![alt text][theta23]

With `theta1`, `theta2` and `theta3`, we can reduce `R0_6` to `R3_6` by decomposing `theta1~3`. Now, `R3_6` rotation matrix contains only `theta4`, `theta5` and `theta6`. Hence we can retrieve them as we did in __Euler Angles from a Rotation Matrix__.

```
R3_6 = R0_3.inv("LU") * ROT_EE
```

```
T3_6 = T3_4 * T4_5 * T5_6 = Matrix([
[-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4), -0.054],
[                           sin(q5)*cos(q6),                           -sin(q5)*sin(q6),          cos(q5),    1.5],
[-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5),      0],
[                                         0,                                          0,                0,      1]])

R3_6 = T3_6[0:2, 0:2]
```

These 3 angles can be solved by:

```
# Euler angles from rotation matrix
theta4 = atan2(R3_6[2,2], - R3_6[0,2])
theta5 = atan2(sqrt(R3_6[0,2] * R3_6[0,2] + R3_6[2,2] * R3_6[2,2]), R3_6[1,2])
theta6 = atan2(-R3_6[1,1], R3_6[1,0])
```

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


When executing `$ rosrun kuka_arm IK_server.py`, it actually creates a service. Other ROS nodes can send request, containing the target position and orientation of the end-effector, to this service, and the service invokes a `handle_calculate_IK` function in `IK_server.py`. This function takes positions and orientations, and it apply inverse kinemtaic described above to retrieve all joint angles of the arm. The service , then, returns reponse with angles. As a result, the robot arm can work base on these joint angles.

Here is a result. Base on `IK_server.py`, the arm can achieve 8/10 success.

![alt text][pick_gazebo]

During the simulation, the arm has reduantant rotations, and the motion path is inefficient. The following picture is one example. 

### Future work

#### 1.Improve the motion planning, and reduce redundant rotation and movement.

![alt text][rviz_inefficient]

The reviewer suggests: https://udacity-robotics.slack.com/archives/C5HUQ0HB9/p1499136717183191

#### 2. Test whether `numpy` can speed up calculations.
