--- 
layout: post
title:  "Differential kinematics and inverse kinematics"
date:   2015-12-11
---

This learning module covers the relationship between movement in the 
generalized coordinates
and corresponding movement at a designated point on the robot. You may find
it helpful to consider an industrial robot, in which case you can substitute
the term "joint positions" or "joint angles" for "generalized coordinates and
the term "joint velocities" for "generalized velocities", 
rather than more general robots (like humanoids). 

### Analytical inverse kinematics

Recall that the forward kinematics equation is:

\begin{equation}
\mathbf{x} = f^p(\mathbf{q})
\end{equation}

where \\(p\\) is a frame attached to a rigid link of the robot,
\\(\mathbf{q}\\) are the \\(n\\) generalized coordinates of the robot, and \\(\mathbf{x}\\) is the pose of \\(p\\) in another frame (typically the world frame). If \\(\mathbf{x}\\) is represented by \\(m\\) real numbers, then \\(f : \mathbb{R}^n \to \mathbb{R}^m \\).

**We will use \\(p\\) throughout this learning module to refer to
a pose attached to a rigid link on the robot.** 

{% include image.html url="../../assets/img/frame-p-on-robot.png" description="Depiction of Frame p, which is defined with respect to one of the robot's links. While forward kinematics seeks to determine how p is positioned and oriented with respect to the global frame, differential kinematics seek to determine how quickly p changes as a function of the robot's current configuration." %} 

#### Inverting \\(f(.)\\)

Inverting \\(f(.)\\) means determining \\(f^{-1}(\mathbf{x} = \mathbf{q}\\), where \\(\mathbf{x}\\) is one of the possible outputs of the forward kinematics function (SO(2), SO(3), \\(\mathbf{R}^2, \mathbf{R}^3\\), SE(2), SE(3)). This inversion is generally challenging because \\(f(.)\\) is a nonlinear function. Additional challenges are that:

* \\(f^{-1}(\mathbf{x})\\) may have no solutions
* \\(f^{-1}(\mathbf{x}\\)) may have multiple solutions
* Computation of \\(f^{-1}(.)\\) generally recommends the use of a symbolic mathematics package (like [Macsyma](http://maxima.sourceforge.net)), which means that changing the kinematics of the robot means re-derivation of the inverse kinematics function

Analytical inverse kinematics requires that the number of constraints be
equal to the robot's number of generalized coordinates. Artificial constraints
may be introduced to satisfy this requirement. For example, a "typical"
anthropomorphic arm provides seven degrees of freedom, while controlling
position and orientation in 3D presents six constraints. Analytical inverse
kinematics solutions for such arms typically leave the elbow position as
an open parameter for the user to adjust, reducing the controllable degrees
of freedom of the arm to six. 

{% include image.html url="../../assets/img/ik.png" description="An example of analytical inverse kinematics, using a two link planar arm. Image cribbed from Stefan Schaal's course notes." %}
 
### Differential kinematics

Another way to do inverse kinematics is using _differential kinematics_. Differential kinematics examines the change in \\(\mathbf{x}\\) given small
changes in \\(\mathbf{q}\\). [Recall that \\(\mathbf{x}\\) can represent position and/or orientation in 2D or 3D](../forward-kinematics), so the aforementioned "small changes" refer to small changes in position and/or orientation.

\begin{equation}
\dot{\mathbf{x}} = \begin{bmatrix} 
\frac{\partial f\_1}{\partial q\_1} & \ldots & \frac{\partial f\_1}{\partial q\_n} \\\\
& \vdots & \\\\
\frac{\partial f\_m}{\partial q\_1} & \ldots & \frac{\partial f\_m}{\partial q\_n}
\end{bmatrix}
\begin{bmatrix}
\dot{q}\_1 \\\\
\vdots \\\\
\dot{q}\_n
\end{bmatrix} \label{eqn:Jacobians-full}
\end{equation}

The first matrix is the _Jacobian of pose \\(\mathbf{p}\\) with respect to
the generalized configuration_. Refer back to the [material on linear algebra](../linear-algebra/) if you need a refresher on Jacobian matrices. The second matrix is just the generalized
velocity, \\(\dot{\mathbf{q}}\\). So, we can write Equation \ref{eqn:Jacobians-jull} as:
\begin{equation}
\dot{\mathbf{x}} = \mathbf{J}\dot{\mathbf{q}} \label{eqn:Jacobians}
\end{equation}
_You will frequently see this equation in robotic manipulation_.

#### An example: analytically computing the Jacobian of a double pendulum

Consider the [double pendulum from the learning module on forward kinematics](../forward-kinematics). At this point in time, assume that we wish only to
compute the Jacobian matrix of the double pendulum's endpoint. Stated another way, \\(\mathbf{f}(\mathbf{q}) \to \mathbb{R}^2\\) and is defined as:
\begin{equation}
\begin{bmatrix}
l\_1 c\_1 + l\_2 c\_{1+2} \\\\
l\_1 s\_1 + l\_2 s\_{1+2}
\end{bmatrix}
\end{equation}

Before reading further, make sure you know (a) the dimension of \\(\mathbf{q}\\), (b) the dimension of \\(\mathbf{f}(.)\\), and the dimension of the Jacobian matrix that will be produced. 

The Jacobian matrix for this equation will be:
\begin{equation}
\begin{bmatrix}
\frac{\partial f\_1}{\partial q\_1} & \frac{\partial f\_1}{\partial q\_2} \\\\
\frac{\partial f\_2}{\partial q\_1} & \frac{\partial f\_2}{\partial q\_2}
\end{bmatrix}
\end{equation}

When I determine the derivatives, I get this:
\begin{equation}
\begin{bmatrix}
-l\_1 s\_1 - l\_2 s\_{1+2} & -l\_2 s\_{1+2} \\\\
l\_1 c\_1  + l\_2 c\_{1+2} & l\_2 c\_{1+2}
\end{bmatrix}
\end{equation}

#### Consideration for orientation

What about when the \\(\mathbf{f}(.)\\) mapping contains orientation components (i.e., SO(2), SO(3), SE(2), SE(3))? If we use the \\(3 \times 3\\) homogeneous 
transformation matrix from [the forard kinematics module](../forward-kinematics), then the size of the Jacobian matrix will be \\(3 \times 3 \times 2\\).
Such higher dimensional matrices are not fun to deal with! So it makes sense
for us to use a non-matrix representation of orientation in this case. The orientation of the endpoint (or any point, for that matter) of the second link in the double pendulum is:
\begin{equation}
\theta\_1 + \theta\_2 \label{eqn:dp-orientation}
\end{equation}

**Practice computing the Jacobian matrix for Equation \ref{eqn:dp-orientation}. Then make sure you know what relationship this Jacobian is computing in Equation \ref{eqn:Jacobians}.**

#### What does the Jacobian tell us?
Column \\(i\\) of the Jacobian gives us the scale of joint \\(i\\)'s 
instantaneous contribution to the 
movement in all dimensions of operational space. Row \\(j\\) of the Jacobian 
gives us instantaneous contribution of all joints to dimension \\(j\\) of
operational space.

**Note that the contribution of joint \\(i\\) to \\(\mathbf{p}\\) is zero if joint \\(i\\) does not affect \\(\mathbf{p}\\)'s movement** (and therefore, column \\(i\\) of the Jacobian will be a zero vector).

{% include image.html url="../../assets/img/frame-p-on-robot.png" description="Axis six does not contribute to movement of Frame p. The column of the Jacobian matrix corresponding to Axis six should therefore be zero." %} 

#### Jacobians and statics

The velocity of a point \\(\mathbf{p}\\) on a rigid body due to angular velocity is:
\begin{equation}
\dot{\mathbf{p}} = \dot{\mathbf{x}} + \mathbf{\omega} \times (\mathbf{p} - \mathbf{x})
\end{equation}
where \\(\mathbf{x}\\) and \\(\dot{\mathbf{x}}\\) are the position and velocity of the center of mass of the body and \\(\mathbf{\omega}\\) is the body's angular velocity.

Note the similarity with the relationship with the [couple](https://www.quora.com/What-is-the-difference-between-torque-and-moment-3) \\(\hat{\mathbf{\tau}}\\) that results from the addition of a torque, \\(\mathbf{\tau}\\), and a [moment](https://www.quora.com/What-is-the-difference-between-torque-and-moment-3) that results from applying a force \\(\mathbf{f}\\) on the body at point \\(\mathbf{p}\\):
\begin{equation}
\hat{\mathbf{\tau}} = \mathbf{\tau} + (\mathbf{p} - \mathbf{x}) \times \mathbf{f}
\end{equation}

**These equations represent a key relationship in mechanics and robotics, between force and motion**. _Moving from single rigid bodies to robots now_, we can add the following rule in addition to Equation \ref{eqn:Jacobians}:  
\begin{equation}
\hat{\mathbf{\tau}} = \mathbf{J}^{\mathsf{T}}\begin{bmatrix} \mathbf{f} \\\\ \mathbf{\tau} \end{bmatrix} \label{eqn:Jacobians-torque}
\end{equation}

I cannot overstate the importance of Equations \ref{eqn:Jacobians} and \ref{eqn:Jacobians-torque}. These equations tell us how fast a point on the robot is moving in operational space as a function of its joint speeds (Equation \ref{eqn:Jacobians}) and how much torque acts at a robot's joints as a function of a force applied to a point on the robot (Equation \ref{eqn:Jacobians-torque}).

### Numerical inverse kinematics

State of the art numerical approaches for inverse kinematics use a [Newton-Raphson based process for finding roots of nonlinear systems of equations](https://en.wikipedia.org/wiki/Newton%27s_method). Such algorithms are susceptible to failing to find a solution,
even when one or more solutions is known to exist. I will give an overview
of the basic approach, but extensions to this approach can yield significantly 
greater solvability.  

While we can't necessarily compute \\(\mathbf{f}^{-1}(.)\\), we _can invert \\(\mathbf{f}(.)\\) in a small neighborhood around a generalized configuration_. So 
the idea behind the _resolved motion rate control (RMRC)_ Algorithm (to be described below) is this: repeatedly (1) invert \\(\mathbf{f}(.)\\) around a 
generalized configuration and (2) use that inverse to determine how to alter 
the generalized coordinates to move toward the goal. The hope is that the
process converges to an answer.

{% comment %}
Using Jacobian for double pendulum show how Jacobian is accurate only in
its neighborhood.
{% endcomment %}

#### The resolved motion rate control (RMRC) algorithm

Given \\(\mathbf{q}\_{\textrm{init}}\\), do:

1. \\(\mathbf{q} \leftarrow \mathbf{q}\_{\textrm{init}}\\)
2. Compute \\(\Delta\mathbf{x} = \mathbf{x}\_{\textrm{des}} - f(\mathbf{q}\\))
3. If \\(||\Delta \mathbf{x}|| < \epsilon \\) __return__ _success_ 
4. Compute Jacobian (\\(\mathbf{J}\\)), evaluated at \\(\mathbf{q}\\)
5. Solve least squares problem, \\(\min_{\mathbf{q}} ||\mathbf{J}\Delta \mathbf{q} - \Delta \mathbf{x}||\\) 
6. Determine \\(t \le 1\\) such that \\(||\mathbf{x}\_{\textrm{des}} - f(\mathbf{q + t\Delta \mathbf{q}}\\)) is minimized
7. Update \\(\mathbf{q}\\): \\(\mathbf{q} \leftarrow \mathbf{q} + t\Delta \mathbf{q}\\)
8. Repeat (2) until maximum iterations exceeded
9. __return__ _failure_

##### Minimizing  \\(||\mathbf{J}\Delta \mathbf{q} - \Delta \mathbf{x}||\\)
Due to unit mixing (angular vs. metric), a least squares solution is not 
necessarily desirable when attempting to find
joint angles that minimize residual error in desired position _and_
orientation simultaneously. One can attempt to use scaling factors to bias
the least squares solution, but I generally attempt to avoid such hacking.

I present the least squares options without further comment on this issue.

__Jacobian transpose__:

Least squares problems cannot usually be approached using a matrix transpose
approach: \\(\mathbf{A}^\mathsf{T}\mathbf{b}\\) does not generally yield a 
_descent direction_ (a change in \\(\mathbf{x}\\)) that reduces \\(||\mathbf{Ax} - \mathbf{b}||\\). The Jacobian transpose method is able to leverage rigid body dynamics' duality between force and
velocity:

\begin{align*}
\mathbf{J}\dot{\mathbf{q}} & = \dot{\mathbf{x}} \\\\
\mathbf{J}^\mathsf{T}\mathbf{f} & = \mathbf{\tau}
\end{align*}

The Jacobian transpose approach functions as if there were a spring attached 
between frame \\(p\\) and \\(\mathbf{x}\_{\textrm{des}}\\). Proof
that the Jacobian transpose approach yields a descent direction follows.

__Theorem__: \\((\mathbf{JJ}^\mathsf{T}\Delta \mathbf{x})^\mathsf{T}\Delta \mathbf{x} \ge 0\\)

_Proof_: \\((\mathbf{JJ}^\mathsf{T}\Delta \mathbf{x})^\mathsf{T}\Delta \mathbf{x} = (\mathbf{J}^\textsf{T}\Delta \mathbf{x})^\mathsf{T}(\mathbf{J}^\textsf{T}\Delta \mathbf{x}) = ||\mathbf{J}^\textsf{T}\Delta \mathbf{x}||^2\\)

Properties of the Jacobian transpose method:

* Slow convergence to a solution (but requires only a \\(O(n^2)\\) operation, in place of expensive \\(O(n^3)\\) operations)
* No numerical problems; robust to singularities and near singularities

{% comment %}
Graduate student problem: derive the Jacobian transpose method using
least squares and gradient descent.
{% endcomment %}

__Unregularized pseudo-inverse-based least squares__:

We can use the right pseudo inverse to solve the least squares problem.

\begin{align}
\Delta \mathbf{q} & = \mathbf{J}^+\Delta \mathbf{x} \\\\
 & = \mathbf{J}^\mathsf{T}(\mathbf{JJ}^\mathsf{T})^{-1}\Delta \mathbf{x}
\end{align}

Warning: if the matrix is _nearly singular_, it is possible that the factorization of \\(\mathbf{JJ}^\mathsf{T}\\) does not report that the matrix is singular  and yet \\(\Delta \mathbf{q}\\) is not be a descent direction.

{% comment %}
1. What is the size of the square matrix in J*J'?
2. What is the size of the square matrix in J'*J?
3. What is the rank of both?
4. Why do we use the right inverse?
{% endcomment %}



__(QR/SVD)-based least squares__:

Any standard numerical approach for solving least squares problems, including
QR factorization and singular value decomposition (as described in the [linear algebra material](../linear-algebra/)) can be applied to this problem.

__Least squares with nullspace__:

If the robot has more degrees of freedom available than there are position
and orientation constraints, we say that the kinematics are _redundant_.
Redundancy can be exploited when the number of linearly independent columns 
in the Jacobian matrix is greater than the number of linearly dependent rows.
The nullspace of the Jacobian matrix can then be used to find a solution \\(\Delta \mathbf{q}\\) that reduces the residual error in the least squares problem _while
simultaneously satisfying secondary goals_. Such secondary goals have included
selecting (1) the solution that minimizes distance from a desired joint 
configuration, (2) the solution that maximizes distance from obstacles, and
(3) the solution that maximizes manipulability (minimizes the new Jacobian's condition number).

If the singular value decomposition is used to compute the least squares
solution to \\(||\mathbf{J}\Delta \mathbf{q} - \Delta \mathbf{x}||\\), the nullspace of the Jacobian is provided [as a free byproduct](../linear-algebra/). Otherwise, [the nullspace of the Jacobian is given by the matrix](http://www.springer.com/us/book/9781846286414) (page 125):

\begin{equation}
\mathbf{R} \equiv (\mathbf{I} - \mathbf{J}^+\mathbf{J})
\end{equation}

- if delta q + R y minimizes least squares problem
- can optimize g(q), if g(q) is linear
- 

Typical objective functions:

- Maximizing a manipulability measure: \\(g(\mathbf{q}) \equiv \sqrt{\textrm{det}(\mathbf{J(q)J}^\mathsf{T}(\mathbf{q})}\\). Moves the robot away from singular configurations.
- Distance from joint limits: \\(g(\mathbf{q}) \equiv \\)
- Distance from an objstacle: \\(g(\mathbf{q}) \equiv \min\_{\mathbf{p}, \mathbf{o}} ||\mathbf{p}(\mathbf{q}) - \mathbf{o}||\\)

Finally, satisfying a hierarchy of task goals is possible using multiple nullspace projections.

{% comment %}
What happens when we compute numerical inverse kinematics without specifying a target position or orientation task?
{% endcomment %}

##### Computing \\(\Delta \mathbf{x}\\)
When \\(\mathbf{x}\_{\textrm{des}}\\) represents a target in Cartesian space (we do not care about \\(p\\)'s orientation), \\(\Delta \mathbf{x}\\) is computed using only a simple vector subtraction operation.

Similarly, when \\(\mathbf{x}\_{\textrm{des}}\\) represents a target _2D_ orientation, \\(\Delta \mathbf{x}\\) can be computed using _nearly_ a simple scalar subtraction operation. Why "nearly"? Consider the example where the current orientation is \\(\theta \equiv \frac{\pi}{15}\\) and target orientation is \\(\theta\_{\textrm{des}} \equiv \frac{29\pi}{15}\\). Why is the simple subtraction \\(\theta\_{\textrm{des}} - \theta\\) not recommended?

When \\(\mathbf{x}\_{\textrm{des}}\\) represents a target _3D_ orientation, 
matters become more complicated. In this case, \\(\mathbf{x}\_{\textrm{des}}\\) and \\(\Delta \mathbf{x}\\) will take a different form (why they must is a question for you to answer). We assume that \\(\mathbf{x}\_{\textrm{des}}\\) is given as a \\(3 \times 3\\) rotation matrix and that \\(\Delta \mathbf{x} \in \mathbb{R}^3\\). 

[Recall that](../poses3):
\begin{equation}
_w\dot{\mathbf{R}}_i\cdot\ \_w{\mathbf{R}_i}^{\mathsf{T}} = \tilde{\mathbf{\omega}}\_w
\end{equation}

We can use [Euler integration ](../dynamical-systems) and this equation to solve for \\(\tilde{\mathbf{\omega}}\\) given (1) the current orientation of the body and (2) the desired orientation of the body:
\begin{equation}
\_w\mathbf{R}\_i + \Delta t \tilde{\mathbf{\omega}}\ \_w\mathbf{R}\_i = \mathbf{x}\_{\textrm{des}}
\end{equation}
Assume that \\(\Delta t = 1\\)- it doesn't matter what we set \\(\Delta t\\) to since we will not be considering how much time it requires to move between the two orientations. We want \\(\mathbf{\omega}\\): 
\begin{align}
\tilde{\mathbf{\omega}}\ \_w\mathbf{R}\_i & = \mathbf{x}\_{\textrm{des}} - \ \_w\mathbf{R}\_i \\\\
\tilde{\mathbf{\omega}} & = (\mathbf{x}\_{\textrm{des}} - \ \_w\mathbf{R}\_i)\ \_w\mathbf{R}\_i^\mathsf{T}
\end{align}

{% comment %}
How can \tilde{\omega} above be simplified further?
{% endcomment %}

Recall that we desire for \\(\Delta \mathbf{x}\\) to be a three dimensional vector, while \\(\tilde{\mathbf{\omega}}\\) is a \\(3 \times 3\\) matrix. In fact, 
\\(\tilde{\mathbf{\omega}}\\) _should_ be a skew symmetric matrix ([as you hopefully recall](../poses3) but the
first order approximation and lack of re-orthogonalization mean that it
generally will not be. So, to get \\(\omega\\), we use the skew-symmetric form of \\(\mathbf{\omega} \times\\) (again, [as you should recall](../poses3) resulting in:
\begin{equation}
\Delta \mathbf{x} = \frac{1}{2} \begin{bmatrix}
\tilde{\omega}\_{32} - \tilde{\omega}\_{23} \\\\
\tilde{\omega}\_{13} - \tilde{\omega}\_{31} \\\\
\tilde{\omega}\_{21} - \tilde{\omega}\_{12}
\end{bmatrix}
\end{equation}

##### Ray search

As with any linearization, the approximation of \\(\dot{\mathbf{f}}(.)\\) using \\(\mathbf{J}\dot{\mathbf{q}}\\) becomes less accurate the farther we move
from the generalized coordinates where \\(\mathbf{J}\\) is evaluated. This means that we do not generally want to update the generalized configuration using:
\begin{equation}
\mathbf{q}\_{i+1} \leftarrow \mathbf{q}\_i + \Delta \mathbf{q}
\end{equation}
because \\(\mathbf{q}\_{i+1}\\) might be farther from the goal! Instead, we do the update like this:
\begin{equation}
\mathbf{q}\_{i+1} \leftarrow \mathbf{q}\_i + \alpha \Delta \mathbf{q}
\end{equation}
This is called a _ray search_ or (less accurately) a line search, because we search along the ray \\( 0 < \alpha < \infty \\). Best practice is to constrain the search to \\(\alpha < 1\\).

**As an aside, the ray/line search process is used in optimization, and therefore in machine learning, so you may see these formulas in the future.**

There are a few options for the ray search:

1. Set \\(\alpha\\) to some small value (say 0.001), that you have found
   works well empirically for the robot and task. This approach is clearly
   not adaptive- it takes small steps even when big stops might be possible.
2. Use a univariate optimization method like [Brent's Method](http://fedc.wiwi.hu-berlin.de/xplore/tutorials/xegbohtmlnode62.html) to establish
   a (local) optimum \\(\alpha\\). This approach is computationally expensive.
3. Use [backtracking line search](https://en.wikipedia.org/wiki/Backtracking_line_search), which requires a little information, but
   is adaptive, like (2), but much faster.

For your assignment, you will use the easiest option (1).

#### Manipulability

In most cases in robotics, the _rank_ of the Jacobian matrix will be determined
by the number of independent rows, as this number is usually much smaller than 
the number of independent columns. If the Jacobian does not have full row
rank (meaning that one or more rows of the Jacobian matrix are linearly
dependent), the robot will lack the ability to move frame \\(p\\)
in every possible direction (for example, the robot may be unable to move \\(p\\) vertically). 

This loss of rank can occur for the following reasons:

* _workspace singularity_: the robot's arm is fully outstretched. There is no joint movement that could cause the arm to move further in the outstretched direction. Workspace singularities often occur on anthropomorphic robots when effecting pointing or when a leg is fully extended (as when standing with ``knees'' locked).
* _internal singularity_: two or more rotational degrees of freedom have aligned such that movement in one is equivalent to movement in the other. This phenomenon is known as [Gimbal lock](https://www.youtube.com/watch?v=zc8b2Jo7mno).

Manipulability can be determined by examining the condition number of the
Jacobian matrix at configuration \\(\mathbf{q}\\). Recall from the [lecture material on linear algebra](../linear-algebra/) that the condition number of a matrix is the ratio of the largest
to the smallest singular values. The closer the condition number is to infinity, the less manipulability that the robot possesses at configuration \\(\mathbf{q}\\).

_It is wise to avoid moving the robot to a configuration where manipulability
is reduced, even though computing the condition number requires a (relatively)
computationally expensive singular value decomposition._ 

##### State of the art approaches for numerical IK
[State of the art IK approaches](https://www.researchgate.net/publication/282852814_TRAC-IK_An_Open-Source_Library_for_Improved_Solving_of_Generic_Inverse_Kinematics) use quasi-Newton methods for nonlinear programming
with inequality constraints to compute inverse kinematics with joint limits.
These are generic (albeit carefully crafted) approaches applied to a standard
nonlinear programming description of the problem. Like methods described above,
the _convergence of numerical inverse kinematics approaches is heavily
dependent upon the starting configuration_.

### Computing the Jacobian matrix
TODO: What is the frame that the Jacobian is computed in?
Ans: frame 

_One of the biggest challenges with programming robots that manipulate is
that matrices (like Jacobians) lose their meaning_: they are simply a
collection of numbers. If the matrix is computed incorrectly, the effect
may not be obvious.

**To help with this problem, you should always consider the frame of 
reference in which you are computing such quantities.** 

We will only consider computing the Jacobian matrix for revolute and prismatic
joints, as these are the most common powered joints, and I will only discuss
robots with bases affixed to their environment (like industrial robots). 
Therefore, we can assume that column \\(i\\) of the
Jacobian matrix will correspond to joint \\(i\\) of the robot, _where
the numbering is arbitrary_.

I will designate the axis of joint \\(i\\) to point along \\(\hat{\mathbf{z}}\_i\\) (the hat indicates that the vector is normalized) and the current
location of joint \\(i\\) as \\(\mathbf{j}\_i\\).  

**Bonus question: how would the Jacobian matrix change for a robot with a floating base?**

{% comment %}
Show a picture of a planar biped's kinematic tree and ask them to compute the Jacobian for the left toe with respect to the robot's right leg joints from the standing
configuration (trick question). Give link lengths.
{% endcomment %}

#### Computing the Jacobian matrix for translational motion only (2D)
Translational motion in 2D will yield a Jacobian matrix with two rows. For a revolute joint, we assume that the joint's rotation is about the positive _z_-axis. 

Therefore, a column of the Jacobian matrix takes the following form for translational motion with a revolute joint:

\begin{equation}
\begin{bmatrix}
j\_{i\_2} - p^o\_2\\\\
p^o\_1 - j\_{i\_1}
\end{bmatrix}
\end{equation}

This equation is a cross product operation, as will be seen when we compute the Jacobian matrix for translational motion in 3D. 

A prismatic joint's contribution to translational motion is just the joint axis:

\begin{equation}
\begin{bmatrix}
\hat{\mathbf{z}}\_i
\end{bmatrix}
\end{equation}

#### Computing the Jacobian matrix for translational motion only (3D)
Translational motion in 3D will yield a Jacobian matrix with three rows.
 
TODO: define p^o as origin of frame p in the global frame

A column of the Jacobian matrix takes the following form for translational motion with a revolute joint:

\begin{equation}
\begin{bmatrix}
\hat{\mathbf{z}}\_i \times (\mathbf{p}^o - \mathbf{j}\_i)
\end{bmatrix}
\end{equation}

This equation can be intuited using the figure below:
{% include image.html url="../../assets/img/geometric-Jacobian.png" description="Intuition behind the translational motion due to a revolute joint. As the link turns counter-clockwise about the joint, the linear contribution to the motion will be proportional to the rate of movement and the distance between p and the joint." %} 

As in 2D, a prismatic joint's contribution to translational motion is just the joint axis:

\begin{equation}
\begin{bmatrix}
\hat{\mathbf{z}}\_i
\end{bmatrix}
\end{equation}

{% comment %}
How can we get the translational motion of a rotational joint in 2D from
the translational motion of a rotational joint in 3D?
{% endcomment %}

#### Computing the Jacobian matrix for rotational motion only (2D)
Rotational motion in 2D will yield a Jacobian matrix with a single row. For rotational motion with a revolute joint, a column of the Jacobian matrix takes the form:

\begin{equation}
\begin{bmatrix}
1
\end{bmatrix}
\end{equation}

For rotational motion, a prismatic joint makes no contribution, so the column of the Jacobian matrix takes the form:

\begin{equation}
\begin{bmatrix}
0
\end{bmatrix}
\end{equation}


#### Computing the Jacobian matrix for rotational motion only (3D)
Rotational motion in 3D will yield a Jacobian matrix with three rows. For rotational motion with a revolute joint, a column of the Jacobian matrix takes the form:

\begin{equation}
\begin{bmatrix}
\hat{\mathbf{z}}\_i
\end{bmatrix}
\end{equation}

For rotational motion, a prismatic joint makes no contribution, so the column of the Jacobian matrix takes the form:

\begin{equation}
\begin{bmatrix}
\mathbf{0}
\end{bmatrix}
\end{equation}

#### Computing the Jacobian matrix for translational and rotational motion 

Combining translational and rotational motion entails simply stacking the
Jacobian rows corresponding to translation on top of the rows corresponding
to rotation. The ordering- linear components on top, angular on bottom, for
example- is arbitrary, but must be consistent: \\(\Delta \mathbf{x}\\)
must follow the same convention. You can see why the ordering is arbitrary
in the equation below:

\begin{equation}
\begin{bmatrix}
\dot{\overrightarrow{\mathbf{x}}} \\\\
\mathbf{\omega}
\end{bmatrix} = 
\begin{bmatrix}
\mathbf{J}\_{\overrightarrow{\mathbf{x}}} \\\\
\mathbf{J}\_{\mathbf{\omega}}
\end{bmatrix}
\dot{\mathbf{q}}
\end{equation}

where \\(\dot{\overrightarrow{\mathbf{x}}}\\) is the linear motion of a point
on the robot, \\(\mathbf{J}\_{\overrightarrow{\mathbf{x}}}\\) represents the
components of the Jacobian matrix that contribute to linear motion, and \\(\mathbf{J}\_{\mathbf{\omega}}\\) represents the components of the Jacobian that contribute to angular motion. 

**Verify that, if I permute rows of the Jacobian, I will get the same output for \\(\dot{\mathbf{x}}\\) (under the same permutation).**

Similarly, I can permute columns of the Jacobian as long as I permute the
same entries in \\(\dot{\mathbf{q}}\\).

#### An example: computing the Jacobian matrix for the double pendulum

We again use the double pendulum example from [the forward kinematics module](../forward-kinematics). We need the following pieces of information:

1. The endpoint of the mechanism ([already determined](../forward-kinematics))
2. The location of each joint (\\(\mathbf{j}\_i\\)
3. Each joint's axis: this always points along the \\(z\\)-axis, since this example is in 2D

Finally, we also have to determine the form of the Jacobian that we want: I'll say that we want a \\(3 \times 2\\) Jacobian matrix so that \\(\dot{\mathbf{x}}\\) corresponds to:
\begin{equation}
\begin{bmatrix}
\dot{x}\\\\
\dot{y}\\\\
\dot{\theta}
\end{bmatrix}
\end{equation}

The only piece of information that we lack is the locations of the joints.  
The first joint is located at the origin (0,0). The origin of the
second joint is located at Frame 1' [in the example](../forward-kinematics), which
is:

\begin{align}
\_w\mathbf{T}\_1 \cdot\ \_1\mathbf{T}\_{1'} = 
\begin{bmatrix} 
c\_1 & -s\_1 & 0 \\\\
s\_1 & c\_1 & 0 \\\\
0 & 0 & 1
\end{bmatrix} cdot
\begin{bmatrix} 
1 & 0 & \ell\_1 \\\\
0 & 1 & 0 \\\\
0 & 0 & 1
\end{bmatrix} = 
\begin{bmatrix} 
c\_1 & -s\_1 & l\_1 c\_1 \\\\
s\_1 & c\_1 & l\_1 s\_1 \\\\
0 & 0 & 1
\end{bmatrix}
\end{align}

Summarizing:
\begin{align}
\mathbf{j}\_1 & = \begin{bmatrix} 0 \\\\ 0 \end{bmatrix} \\\\
\mathbf{j}\_2 & = \begin{bmatrix} l\_1 c\_1 \\\\ l\_1 s\_1 \end{bmatrix}
\end{align}

\begin{equation}
\begin{bmatrix}
-(l\_1 s\_1 + l\_2 s\_{1+2}) & -l\_2 s\_{12}  \\\\
l\_1 c\_1 + l\_2 c\_{1+2} & l\_2 c\_{12} \\\\
1 & 1
\end{bmatrix}
\end{equation}

Note that this is equivalent to the 3D Jacobian:
\begin{equation}
\begin{bmatrix}
-(l\_1 s\_1 + l\_2 s\_{1+2}) & -l\_2 s\_{12}  \\\\
l\_1 c\_1 + l\_2 c\_{1+2} & l\_2 c\_{12} \\\\
0 & 0 \\\\
0 & 0 \\\\
0 & 0 \\\\
1 & 1
\end{bmatrix}
\end{equation}

where I have designated the top three components to be linear motion and
the bottom three components to be angular motion.

{% comment %}
Check intuition that Jacobian is correct by looking at instantaneous change
at end effector.
{% endcomment %}

#### Computing the Jacobian matrix numerically (using a finite difference approach)

Recall that the Jacobian matrix is computed at a generalized configuration, \\(\mathbf{q} \in \mathbb{R}^n\\). 

1. Compute \\(\mathbf{x} \leftarrow f(\mathbf{q})\\)
2. __for__ \\(i=1,\ldots,n\\):
    1. Set \\(q\_i \leftarrow q\_i + \epsilon\\)
    2. Compute \\(\mathbf{x}' \leftarrow f(\mathbf{q})\\)
    3. Set column i of the Jacobian matrix to \\((\mathbf{x}' - \mathbf{x})/\epsilon\\)
    4. Set \\(q\_i \leftarrow q\_i - \epsilon\\)

Step 2.3 requires computing the differential between two operational
space configurations (i.e., positions, orientations, or mixed position and 
orientation), as described above. 

