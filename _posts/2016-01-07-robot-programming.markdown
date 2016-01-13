--- 
layout: post
title:  "Programming autonomous robots"
date:   2016-1-7
---

This module will discuss some topics for those new to robot programming. 
I'll start with programming general real-time systems, talk next about
an architecture for a typical robot system- including _planners_ and 
_controllers_, and conclude with many thoughts on the open problem of 
combining reactive and deliberative behaviors for dextrous robots.

## Programming real-time systems

Real-time systems are generally best controlled by a real time operating 
system, which provide specific process scheduling policies and minimal
interrupt latency. The key feature of a real time operating system is its
predictability.

For controlling robots, the user selects a control frequency (e.g., 100 Hz),
and the real time operating system ensures that this process is run 
regularly- every 10 ms (plus or minus some small amount of error). _It is the
programmer's responsibility to ensure that their process does not overrun the
process' allotted time_ (10 ms in the example above). If such an overrun
occurs, the real time operating system will typically generate an exception.
Programmers typically avoid I/O and memory allocation in real-time control
loops, as these procedures are often unpredictable. 

## Architecture of a robotic system

_Autonomous_ robotic systems- as opposed to those robots situated in controlled
environments (like factories)- are typically driven using a small number of 
_behaviors_, modular components that focus on getting the robot to perform a single task (like avoiding obstacles, homing to a light source, or following a
wall). Many robot architectures are built on the [finite state machine](https://en.wikipedia.org/wiki/Finite-state_machine) model, with behaviors representing the states and transitions between behaviors occuring in response to events. 

The reason these interacting behaviors are used, instead of the previously
dominant _sense-plan-act_ approach, is depicted in [this video of an early robot, Shakey](https://www.youtube.com/watch?v=qXdn6ynwpiI): the robot moves too
slowly to act and react in uncontrolled environments.

{% include image.html url="../../assets/img/SPA.png" description="Data flow and computation using the sense-plan-act approach to robot architecture." %}

For example, here is a finite state machine-based architecture for a simple
foraging robot:

{% include image.html url="http://www.brl.ac.uk/images/finite-state-machine.png" description="Architecture for a simple foraging robot. Graph nodes represent machine states and transitions represent events and sensory signals. Image from Bristol Robotics Laboratory." %}

The figure below shows the architecture of a more sophisticated autonomous 
robot capable of manipulating its environment. The depicted architecture does 
not represent a single task, but rather depicts the  
flow of data between software and hardware and real-time and non-real-time
software components. 

{% include image.html url="../../assets/img/robot-software-architecture.png" description="An example of a software architecture for an autonomous robot, the Willow Garage PR2 mobile manipulator." %}

### Important components

#### Interprocess communication

As can be seen from the figure, communication between multiple computational
processes running simultaneously is important. The particular mechanism
chosen for _interprocess communication_ (IPC) is critical: latency between
one process transmitting data and the other receiving it should be minimized.
The IPC mechanism depicted in this figure is _shared memory_, which is not
necessarily friendly to program with, but is quite fast.


#### Sensors

The robot depicted in this example possesses several types of sensors:

* __LIDAR__: a time of flight sensing system that operates in the same manner as police using lasers to catch speeding motorists. LIDAR sensors for robots use a rotating laser beam to capture a two-dimensional or three-dimensional depth scan of a surface. 
* __RGB__: a camera that captures a color (red-green-blue) image from its viewpoint for processing using computer vision algorithms. Computer vision is not a current mainstay of robotic sensing for manipulation, likely because of the significant processing time required, imperfect identification (see Google image search), and susceptibility to sensory noise and artifacts. 
* __Kinect__: a single hardware unit that combines both depth sensing and RGB
* __IMU__: an _inertial measurement unit_ that combines accelerometers, gyroscopic sensors, magnetometers, and (often) GPS measurements. The accelerometers give linear acceleration. The gyroscopic sensors yield angular acceleration. 
Magnetometers can help determine orientation.

{% include image.html url="https://upload.wikimedia.org/wikipedia/commons/4/4c/Point_cloud_torus.gif" description="A point cloud, as might be produced by LIDAR or the Kinect sensor." %}

#### Controller
A _controller_ is a real-time process that runs at a specified frequency
(commonly between 100 Hz and 10,000 Hz for robots). Controllers attempt to
regulate a dynamical system using a model of the system and/or error
between the desired state of the system and its current state.

{% include image.html url="../../assets/img/thermostat.png" description="Extremely simple controller (thermostat), actuators (A/C unit and furnace), and sensor (thermometer). The dynamical system is the temperature in the dwelling. The controller operates using a simple _error feedback_ mechanism: a voltage signal is sent to the furnace or A/C unit to increase/decrease temperature until the desired setpoint is reached." %}

One of the earliest controllers is the centrifugal governor. A video depiction of the centrifugal governor is shown [here](https://www.youtube.com/watch?v=iO0CxOTe4Uk). An engine causes the governor to rotate, and centrifugal force on the governor- caused by its faster rotation- closes the throttle valve. When the engine slows, the centrifugal force attenuates, and the throttle re-opens. 

{% include image.html url="https://upload.wikimedia.org/wikipedia/commons/1/1e/Centrifugal_governor.png" description="A depiction of a centrifugal governor." width="500" %}

#### Motor servos
A motor servo is the electronic interface between the robot software and
the hardware. We can communicate with this interface using low level commands
over the serial port, USB, or CAN bus or using higher level driver software.
The commands we send to the interface may consist of desired current
(for electromagnetic motors), desired torque, or desired position and velocity.
The interface will typically output some data, including joint position and-
sometimes- speed, and torque.

#### Planning modules
Many computational processes require more time to compute than the control 
loop frequency would allow. For example, determining viable foot placements
for a humanoid robot can require on the order of a second. Planning modules
are non-real-time processes that run "in the background". When a planning
module has completed its computation, it is able to begin feeding inputs
to the controller. 

#### Perceptual modules

Raw sensory data from robots is generally not usuable without further 
processing. We may wish to remove outlier points from range data, to
"self filter" the robot from RGB or range data, and fuse measurements from
inertial measurement units with other sensors to combat drift.
 
### Mixing real-time reactivity and planning

Control strategies for highly dextrous (dynamic) robots, like walking robots,
humanoid robots, and mobile manipulators are still an active area of research. The researchers must usually be focused on basic functionality (standing, walking, opening a door), causing issues like providing near autonomy in dynamic environments (like those populated by humans) to be ignored.  

Researchers _have_ investigated [getting wheeled robots to move rapidly through
such environments](http://ieeexplore.ieee.org/xpl/login.jsp?tp=&arnumber=219995&url=http%3A%2F%2Fieeexplore.ieee.org%2Fxpls%2Fabs_all.jsp%3Farnumber%3D219995), for example. The strategy allowing such autonomy has been a [three layer (hybrid reactive/deliberative) architecture](https://en.wikipedia.org/wiki/Three-layer_architecture). Consider the problem of having a robot navigate amongst furniture to reach a corner of a room. In a three layer architecture, a planner finds a path through the furniture, a reactive component avoids humans and other unforeseen obstacles, and an arbitration layer decides whether to follow the command from the planner or from the reactive component. 


{% include image.html url="../../assets/img/plannercontroller.png" description="The planner generates a sequence of desired state configurations, which the controller transforms into commands." %}

In the case of wheeled robots, stopping movement is usually an option (though moving away from the human might be best). There is not usually such a good default command available for walking robots, however. Failing to give a good enough command to a walking robot may cause a multi-hundred pound machine to fall over, breaking itself or hurting or
killing a human.

#### Doesn't the passage of time invalidate plans consisting of long command sequences? 

_(This question is relevant to current questions in AI.)_ 

Autonomous robots are expected to execute in dynamic, human populated environments. The right level of abstraction is important: "go to the grocery" is likely a safe action for timespans of years, while "pickup the hundred dollar bill from the sidewalk" may be valid for only seconds. 

#### How can plans be adapted as a robot deviates from its plan?

Robots are nondeterministic systems. A robot may move one degree to the right when we command it to move ten degrees to the left. Deviation from a plan is
nearly guaranteed. Depending on the dynamic characteristics of the system and
the commands sent to the robot, deviation may become magnified or attenuated.
The conditions under which a dynamical system _stabilizes_ to a equilibrium
point or under which deviations do not tend to grow is studied under the
theory of stability of dynamical systems. 

#### What happens if a plan does not consider effects sufficiently far into the future? 
An engine can be controlled using a simple error feedback strategy: make
throttle closure proportional to speed above the designated setpoint (the greater the speed above the setpoint, the faster the throttle closure). Many robotic systems- particularly underactuated robots- do not admit such a straightforward control strategy. Russ Tedrake, an MIT roboticist, has an entire [online course](http://underactuated.csail.mit.edu/underactuated.html) dedicated to this topic. 

If a control strategy does not consider likely next effects of actions, the
control strategy is a _Horizon-One_ strategy. If the control stategy considers the (long-term) effect, \\(n\\) steps into the future, of an action, the
control strategy is a _Horizon-\\(n\\)_ strategy (\\(n = \infty\\) is commonly considered).

{% include image.html url="http://www.expert-chess-strategies.com/images/poisoned-pawn.jpg" description="If Black greedily captures the pawn at F3, White can move the rook from D1 to F1, capturing Black's knight, and likely winning the game. Like playing Chess, robots in dynamic environments must consider longer term consequences of actions." %}

#### A lookup table for control

Since computing controls fast enough is such a problem, one holy grail would
be a lookup table (also known as a _control policy_) for dextrous robot control.A sketch of such a table is below. 

| x\\(\_1\\) | x\\(\_2\\) | Action |
|------|------|--------|
| a    | a    | Move up|
| a    | b    | Move down|
| \\(\vdots\\) | \\(\vdots\\) | \\(\vdots\\) |
| z    | z    | Climb right |

For robots, such a lookup table would have to consider- at minimum- each 
joint's angle and speed; these are the [state variables](../dynamical-systems). For a "simple" humanoid robot, over seventy state variables may be required.
To make a table, we have to discretize these state variables. If we assume
ten (only ten!) gradations per variable, the size of the lookup table will be \\(10^{70}\\) entries. If we want to explore the effect of taking each action _just once_, and our robot could somehow perform and evaluate one million actions per second, \\(10^{64}\\) seconds would be required, This number far exceeds the \\(10^{18}\\) seconds believed to be the age of the universe. Even if this could be magically computed quickly enough, tem gradations is at least an order of magnitude too coarse! 

What about function approximation approaches, which have been studied heavily
by machine learning researchers? Function approximation can often
approximate highly nonlinear functions using much smaller numbers of basis
functions. How would we know that an approximation would be good enough?

Even before we ask that question, we need to know how an action is chosen.
As I established above, greedily choosing the action that appears best at any time is often a poor
choice. Similarly, a legged robot might fall over if a good enough _sequence_ of commands isn't found. We wish for the robot to act to optimize an objective function over a series of commands. 

This problem falls under the domain of _optimal control_, for which globally
optimal solutions are often intractable to find. Current optimal control and
[model predictive control](https://en.wikipedia.org/wiki/Model_predictive_control) approaches often focus on algorithms for robustly finding locally optimal solutions (numerically "brittle" algorithms are presently required). Even if a tractable algorithm for solving optimal control problems were to exist, selecting 
a good objective function would remain a challenging problem. 

