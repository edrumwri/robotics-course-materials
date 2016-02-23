--- 
layout: post
title:  "Forward kinematics"
date:   2015-12-12
---

<p class="intro">
Forward kinematics is the process of determining the location and orientation of a designated point on a robot as a function of the robot's configuration.
</p>

## Constraining the movement of rigid bodies 

In a previous learning module, we learned that rigid bodies have six
degrees of freedom: three for translation and three for rotation. Rigid
robots are composed of a number of these rigid bodies (which we call "links")
and some constraints on the links' motion (called "joints"). Some common joints are depicted below: 

{% include image.html url="https://www.accessiblesystems.com/images/bul/weld.jpg" description="A weld (or 'fixed joint') prevents both relative linear and angular motion between two rigid bodies." width="600" %}

{% include image.html url="https://sjeyr7pe.wikispaces.com/file/view/BALLSOCK.JPG.jpeg/240766305/BALLSOCK.JPG.jpeg" description="A ball and socket joint prevents relative linear motion between two bodies." width="600" %}

{% include image.html url="http://200.126.14.82/web/help/toolbox/physmod/mech/mech_building18.gif" description="Revolute joints prevents relative linear motion between two bodies (like a ball and socket joint) and allow relative rotational motion around only one axis. Revolute joints are common in robotics applications, particularly for robots that emulate biological organisms." width="600" %}

{% include image.html url="http://www.flamingriver.com/sysimg/34-dd-x-34-dd-forged-u-joint-fr2644.jpg" description="Universal joints prevent relative linear motion between two bodies (like a ball and socket joint) and allow relative rotational motion around two axes." width="600" %}

{% include image.html url="http://ode-wiki.org/wiki/images/5/5f/PistonJoint.jpg" description="Prismatic joints eliminate relative rotational motion between two bodies and allow relative linear motion along one axis between the two bodies. Prismatic joints are particularly common in industrial robots." width="600" %}

### An example of constrained motion

[The Foucault pendulum](https://en.wikipedia.org/wiki/Foucault_pendulum) was
devised as an experiment to demonstrate the rotation of the Earth. It consists
of a bob suspended from the ceiling by a long wire.

{% include image.html url="http://i.imgur.com/2UoaPJe.gif" description="Animation of the Foucault pendulum. The Earth's rotation causes the trajectory of the pendulum to change over time, causing a spiral pattern to emerge on the floor." %}

If we assume that the wire is massless, then the only rigid body is the
pendulum bob (since the ceiling is massive and effectively still, we do not
consider it). If the ceiling is located at the origin of the world \\(\begin{bmatrix}0 & 0 & 0\end{bmatrix}^\mathsf{T}\\), then the pendulum must satisfy the [implicit equation](https://en.wikipedia.org/wiki/Implicit_function):

\begin{equation}
\mathbf{x} + \ \_w\mathbf{R}\_i \mathbf{u}\_i = \mathbf{0} \label{eqn:ball-and-socket}
\end{equation} 

where \\(\mathbf{x}\\) is the center-of-mass of the bob, \\(\_w\mathbf{R}\_i\\) is the orientation of the bob, and \\(\mathbf{u}\_i\\) is a vector from the center-of-mass of the bob to the origin of the world. These variables are depicted in the figure below.

![Depiction of variables for Foucault pendulum](../../assets/img/foucault-diagram.png)

## Precise definition of forward kinematics

Forward kinematics can take multiple forms, depending on the output sought:

\begin{equation}
\mathbf{f}(\mathbf{q}) \to \begin{cases} 
\mathbb{R}^2 & \textrm{ for Cartesian position in 2D} \\\\
\mathbb{R}^3 & \textrm{ for Cartesian position in 3D} \\\\
SO(2) & \textrm{ for planar orientation in 2D} \\\\
SO(3) & \textrm{ for 3D orientation} \\\\
SE(2) & \textrm{ for Cartesian position in 2D and planar orientation} \\\\
SE(3) & \textrm{ for Cartesian position in 2D and 3D orientation}
\end{cases}
\end{equation} 

The output of this function is in what is known as _operational space_: we
use this generic term because of all of the possible mappings described above.
To make these mappings more concrete, the planar orientation SO(2) is just an angle.
Similarly, SE(2) is two real numbers plus an angle. These numbers are generally stacked into a vector like this:

\begin{equation}
\begin{bmatrix}
x\\\\
y\\\\
\theta
\end{bmatrix}
\end{equation} 

_though they are equally valid stacked like this_:

\begin{equation*}
\begin{bmatrix}
\theta\\\\
x\\\\
y
\end{bmatrix}
\end{equation*} 

_or even like this_:

\begin{equation*}
\begin{bmatrix}
x\\\\
\theta\\\\
y
\end{bmatrix}
\end{equation*} 

Another possibility is using a \\(3 \times 3\\) homogeneous transformation
matrix:

\begin{equation}
\begin{bmatrix}
c\_{\theta} & -s\_{\theta} & x \\\\
s\_{\theta} & c\_{\theta} & y \\\\
0 & 0 & 1
\end{bmatrix}
\end{equation}

where \\(c\_{\theta}\\) means \\(\cos{\theta}\\) and \\(s\_{\theta}\\) means \\(\sin{\theta}\\).

We will see shortly how this representation can be useful.

3D orientation can be described using any of the representations in [the learning module on 3D poses](../poses3), include Euler angles, axis-angle, unit quaternions, or rotation matrices. The \\(4 \times 4\\) homogeneous transformation 
matrix is common:

\begin{equation}
\begin{bmatrix}
r\_{11} & r\_{12} & r\_{13} & x \\\\
r\_{21} & r\_{22} & r\_{23} & y \\\\
r\_{31} & r\_{32} & r\_{33} & z \\\\
0 & 0 & 0 & 1
\end{bmatrix}
\end{equation}

### An example

Consider a double pendulum in 2D with two joint angles, \\(q\_1\\) and \\(q\_2\\), and link lengths \\(\ell\_1\\) and \\(\ell\_2\\). We will focus on the pendulum's endpoint location and orientation. Because the pendulum is in 2D, its forward kinematics function maps to SE(2).

The configuration of the pendulum in SE(2), using a \\(3 \times 3\\)
homogeneous transformation matrix, when \\(q\_1 = 0, q\_2 = 0\\) is:

\begin{equation*} 
\begin{bmatrix}
1 & 0 & \ell_1 + \ell_2 \\\\
0 & 1 & 0 \\\\
0 & 0 & 1
\end{bmatrix}
\end{equation*}

What is the operational space configuration of the endpoint at generalized configuration \\(q\_1 = \frac{\pi}{2}, q\_2 = \frac{-\pi}{2}\\)?

We will use the following reference frames.

\begin{align}
\_w\mathbf{T}\_1 & \equiv 
\begin{bmatrix} 
c\_1 & -s\_1 & 0 \\\\
s\_1 & c\_1 & 0 \\\\
0 & 0 & 1
\end{bmatrix}\\\\ 
\_1\mathbf{T}\_{1'} & \equiv
\begin{bmatrix} 
1 & 0 & \ell\_1 \\\\
0 & 1 & 0 \\\\
0 & 0 & 1
\end{bmatrix}\\\\
\_{1'}\mathbf{T}\_{2} & \equiv
\begin{bmatrix} 
c\_2 & -s\_2 & 0 \\\\
s\_2 & c\_2 & 0 \\\\
0 & 0 & 1
\end{bmatrix}\\\\ 
\_2\mathbf{T}\_{2'} & \equiv
\begin{bmatrix} 
1 & 0 & \ell\_2 \\\\
0 & 1 & 0 \\\\
0 & 0 & 1
\end{bmatrix}
\end{align}

where \\(c\_1, s\_1, c\_2, s\_2\\) denote \\(\cos{q\_1}, \sin{q\_1}, \cos{q\_2}, \sin{q\_2}\\), respectively.

When we substitute \\(q\_1 = \frac{\pi}{2}, q\_2 = \frac{-\pi}{2}\\) and multiply \\(\_w\mathbf{T}\_1 \cdot\ \_1\mathbf{T}\_{1'} \cdot \_{1'}\mathbf{T}\_2 \cdot \_{2}\mathbf{T}\_{2'}\\), we arrive at:

\begin{equation}
\_w\mathbf{T}\_{2'} = \begin{bmatrix}
c\_{1}c\_2 - s\_1s\_2 & -c\_2s\_1 - c\_1s\_2 & l\_1 c\_1 + l\_2 (c\_{1}c\_2 - s\_1s\_2) \\\\
c\_2s\_1 + c\_1s\_2 & c\_1 c\_2 - s\_1s\_2 & l\_1 s\_1 + l\_2 (c\_2s\_1 + c\_1s\_2) \\\\
0 & 0 & 1
\end{bmatrix}
\end{equation}

which simplifies to (I used Macsyma to do the simplification):

\begin{equation}
\_w\mathbf{T}\_{2'} = \begin{bmatrix}
c\_{1+2} & -s\_{1+2} & l\_1 c\_1 + l\_2 c\_{1+2} \\\\
s\_{1+2} & c\_{1+2} & l\_1 s\_1 + l\_2 s\_{1+2} \\\\
0 & 0 & 1
\end{bmatrix}
\end{equation}

The upper left \\(2 \times 2\\) part of this matrix gives the orientation of the second link. The upper right \\(2 \times 1\\) part of this matrix gives the position of the end point of the second link.

{% include image.html url="../../assets/img/two-link-frames.png" description="Depiction of the various frames in the example above for q1 = pi/4, q2 = -pi/4." %}


### Denavit-Hartenberg parameters

If a robot manufacturer draws a figure providing the data above, you might
make a mistake computing the forward kinematics function (particularly as
we move from two joints to, say, seven). The [Denavit-Hartenberg parameters](https://en.wikipedia.org/wiki/Denavit–Hartenberg_parameters) provide a more
compact, less error prone encoding using four parameters per reference frame.

[Here](https://www.youtube.com/watch?v=rA9tm0gTln8) is a nice video depicting
Denavit-Hartenberg. 

_The nice part of D-H parameters_ is that a simple algorithm can compute
the forward kinematics for a robot given a small table of parameters. _I
personally skip using D-H parameters_ because I find that determining them is
error prone. D-H parameters transfer the likelihood of making an error from 
the forward kinematics programmer to the robotics manufacturer. 

## Determining a robot's number of degrees-of-freedom

For a robot affixed to its environment, the robot's degrees of freedom (DoF) is
equal to its number of joint variables (if the robot has kinematic loops, its
DOF is equal to its _independent_ joint variables). 

{% include image.html url="../../assets/img/veloce-frontal.jpg" description="A robot with kinematic loops." width="600" %}

{% include image.html url="../../assets/img/veloce.png" description="A graph depicting the kinematic structure of the veloce robot depicted previously." width="600" %}

{% include image.html url="https://upload.wikimedia.org/wikipedia/commons/2/2c/Modele_cinematique_corps_humain.svg" description="A graph depicting the kinematic structure of a human. Note that the graph is a tree." width="400" %}

If a robot is not affixed to its environment, we say that its base is
"floating", and that robot has six degrees of freedom plus its number of
independent joint variables. In fact, a minimum (there may be more than one) 
set of variables used to specify a robot's configuration is known as
_minimum coordinates_ or _independent coordinates_.

As a quick aside, there may not be an actuator at every joint. If the number
of actuators is fewer than the robot's DoF, we say that the robot is
_underactuated_. Such robots are particularly challenging to control.
Legged robots, for instance, are underactuated, as are quadrotors. 

### An alternative approach to determining the number of degrees of freedom

Each rigid body in three dimensions has six degrees of freedom (three translation, three rotation). So, multiply the number of robot links by six, and then subtract the total number of constraint equations.

For the example of the Foucault pendulum, there is one rigid body (the body) and three constraint equations (Equation \ref{eqn:ball-and-socket}; note the vector form of the equation). The total number of degrees-of-freedom is therefore \\(6 - 3 = 3\\).

