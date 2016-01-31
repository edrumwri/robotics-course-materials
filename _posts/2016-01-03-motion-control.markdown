--- 
layout: post
title:  "Controlling robot motion"
date:   2015-12-09
---

This learning module focuses on approaches for regulating the _motion_ of a 
robot. Concepts covered include model-based (feedforward) and model-free (error feedback) approaches, composite approaches, and stability.

A caveat: even if we could control the motion of a robot with perfect accuracy
does not mean that it's a good idea. A robot that moves perfectly, minimizing 
the effects of all disturbances on its motion, is a robot that will crush
humans, damage the environment, or destroy itself. [Force control](http://users.softlab.ntua.gr/~ktzaf/Courses/literature/07_Force_Control.pdf) regulates
the force applied to the environment, while 
[impedance control](https://en.wikipedia.org/wiki/Impedance_control) regulates 
the relationship between force and motion. Such schemes should be 
considered for uncontrolled environments or when the accuracy necessary
to perform a task is higher than the accuracy of motion and/or modeling. 

Some definitions of basic concepts in control:


- **Controller**: sends inputs to the dynamical system that may change its state
- **Plant**: another name for the dynamical system that the controller can change 
- **Reference signal**: the value (state or a function of state) that the system is to attain. The reference signal may be fixed (a thermostat setting is effectively fixed) or changing over time
- **Error signal**: the difference between the reference signal and the system's output

An example that we will use for illustrative purposes is a cruise control
system in a car. In this example, the car is the plant, the cruise control
is the controller, the driver's set speed is the reference signal, and the
control input is the throttle (the gas pedal).

{% include image.html url="../../assets/img/nfb.png" description="A depiction of a negative feedback controller for controlling a robot. Where do the desireds (qd and dqd/dt) come from?" %}

## Inverse dynamics control

Without loss of generality, let's consider the _linear_ dynamical system:

\begin{equation}
\dot{\mathbf{x}} = \mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u}
\end{equation}

where \\(\mathbf{u}\\) are the controls that we can apply. Assume
that \\(\mathbf{A}, \mathbf{B} \in \mathbb{R}^{n \times n}\\) and that
\\(\mathbf{B}\\) is a full rank matrix. Then if we want to determine
controls \\(\mathbf{u}\\) to drive the system to change in state \\(\dot{\mathbf{x}}\_{\textrm{des}}\\), algebra yields:

\begin{equation}
\mathbf{u} = \mathbf{B}^{-1}(\dot{\mathbf{x}}\_{\textrm{des}} - \mathbf{A}\mathbf{x})
\end{equation}

Controlling a dynamical system in this way is termed _inverse dynamics 
control_. This is straightforward- so why is this a full learning module?

- The control system designer may not possess a mathematical model (\\(\dot{\mathbf{x}} = \mathbf{A}\mathbf{x} + \mathbf{Bu}\\) in the case above) of the dynamical system, so alternative control techniques may be necessary. With respect to robotics, obtaining accurate and rapidly computable mathematical models of robots with compliant elements is challenging. 
- _A mathematical model is just a model_. There will usually be 
effects- [transmission backlash](https://en.wikipedia.org/wiki/Backlash_(engineering)), imperfectly rigid links, [counter-electromotive forces](https://en.wikipedia.org/wiki/Counter-electromotive_force), joint friction, and other
influences- that the model will not account for, so the inverse dynamics 
control will generally not influence the system exactly as desired. Some form
of error feedback control is usually necessary as well.
- In tandem with the previous point, [system identification](https://en.wikipedia.org/wiki/System_identification) is necessary to identify modeling parameters. For example, if our dynamics model incorporates Coulomb and viscous friction models at the joints, then both Coulomb and viscous friction parameters must be determined. Determining these parameters is often a tedious process that requires extensive physical experimentation and can change as the robot is operated (as it enters its normal operating temperature, leaks hydraulic fluid, etc.) or manipulates or carries objects. 
- Even if we possess a perfect mathematical model, we generally  
do not know the contact forces acting on the robot, which implies that we cannot
invert the model.

{% comment %}
What would ID model of car for autopilot have to account for (assume no slip at wheels)?
- altitude
- slope
- turning radius
- RPM / horsepower / torque
{% endcomment %}

Inverse dynamics control is a _feedforward_ control scheme, which is 
distinguished from an _open loop_ control scheme in that the latter does
not use a mathematical model of the system. 

{% include image.html url="https://upload.wikimedia.org/wikipedia/en/c/c7/Control_Systems.png" description="The Three types of Control System (a) Open Loop (b) Feed-forward (c) Feedback (Closed Loop) (image and description from Wikipedia)" %}

## Error feedback control


Error feedback control is a simple alternative that uses the error signal
to determine controls. Error feedback control requires no mathematical
model of the system, which is a double edged sword: error feedback control
does not anticipate the system's behavior (if there is no error, no controls
are applied). 

The main challenges with error feedback control are (1) setting the rate
at which error is corrected (this process is called _tuning controller gains_)
and (2) ensuring that there is little lag from changes in the system
state to actuation.

The latter _control lag_ problem can occur because the sensing device has
low bandwidth relative to the speed with which the robot can move or because
filtering (by averaging) is necessary to combat sensory noise. Some plumbing
systems can exhibit significant control lag, for example: water temperature
may lag five to ten seconds behind adjustments to the taps.

{% include image.html url="https://upload.wikimedia.org/wikipedia/commons/2/24/Feedback_loop_with_descriptions.svg" description="The concept of the feedback loop to control the dynamic behavior of the system: this is negative feedback, because the sensed value is subtracted from the desired value to create the error signal, which is amplified by the controller (image and description from Wikipedia)" %}

### Proportional control

One of the simplest error feedback controllers is the _proportional controller_,which applies the control signal proportionally to the error signal. The
cruise control system in many cars uses proportional control (also called "P control"): the throttle is applied proportionally to the difference between the current speed and the set speed. 

The control for a P-controller is computed using the following function.

\begin{equation}
u = k\_p (x\_{\textrm{des}} - x)
\end{equation} 

\\(u\\) is the control, \\(x\_{\textrm{des}}\\) is the desired state, and
\\(x\\) is the system state. \\(k\_p \ge 0\\) is known as the "proportional
gain" or "P gain". 

{% comment %}
Assume that u is the proportion of the throttle opening [0,1]. What happens when
x\_des > x? When x\_des < x? What are reasonable gains?
{% endcomment %}

### Proportional-derivative control
If many cars operated using cruise control were to suddenly begin climbing a large hill, the speed will drop precipitously, causing the accelerator to be
applied abruptly. Alternatively, the accelerator may be applied as the car is
at the top of a steep grade, causing the car to greatly exceed the desired
speed.

A _derivative_ controller can help solve this problem by accounting for the
rate at which error is changing: control is modified depending on whether
error is increasing or decreasing. 

\begin{equation}
u = k\_d (\dot{x}\_{\textrm{des}} - \dot{x})
\end{equation} 

As in the case above, \\(k\_d \ge 0\\) is the "derivative gain" or "D gain".
It is generally the case that proportional and derivative control are combined
to make a "proportional-derivative" or "PD controller". This controller is
represented by the equation below: 

\begin{equation}
u = k\_p (x\_{\textrm{des}} - x) + k\_d (\dot{x}\_{\textrm{des}} - \dot{x})
\end{equation} 

In the cruise control example, adding derivative control would add throttle
if the car is decelerating when it is below speed and would decrease throttle if
the car is accelerating when it is above speed.

### Proportional-integrative-derivative control

PD control is not proficient at addressing _steady state error_, which (informally) is error that remains after the system's behavior has settled. Steady
state error is relevant for only fixed reference signals. A PD controller is 
often unable to eliminate steady state error without excessively large gains. 

Continuing with our cruise control example, imagine that our 
car is driving along the highway while towing a large trailor. The cruise
control will stay at some speed \\(y\\) below the set speed. If the speed
falls below \\(y\\), the positional error will be sufficiently large such 
that the proportional control component can increase the speed back to \\(y\\).
The combination of the proportional gain and positional error are insufficient to further decrease error for speeds above \\(y\\).  

{% comment %}
What is the problem of large PD gains?
Assignment: standing legged robot under PD control?
{% endcomment %}

Steady state error can be eliminated using even a small gain by accumulating 
(integrating) the error over time. This _integrative_ term results in a
proportional-integrative-derivative (PID) controller.

\begin{equation}
u = k\_p (x\_{\textrm{des}} - x) + k\_d (\dot{x}\_{\textrm{des}} - \dot{x}) + k\_i \int\_{t\_0}^{t\_c} (x\_{\textrm{des}}(t) - x(t))\textrm{ d}t
\end{equation} 

where \\(t\_0\\) and \\(t\_c\\) are the initial and current system times,
respectively. 

**Disadvantages of PID control** are that the controller can saturate the 
actuators when error is not reduced, [integral windup](https://en.wikipedia.org/wiki/Integral_windup)- which causes the integral term to rapidly accumulate 
when the setpoint is changed- can occur, and another \\(k\\) term must be
tuned.


{% comment %}
How do we approximate the integral by a sum?

What are the consequences of saturing the actuators? Ans: robot breaking
itself or environment.
{% endcomment %}

### PIDD control

PIDD or PID\\(^2\\) control, which incorporates a second derivative, is
not commonly used in robotics applications: few sensors are able to provide
clean acceleration signals.

## Decentralized control

PD and PID schemes are commonly used to control robots, in which case a
separate controller is applied at each joint: \\(2n\\) or \\(3n\\) gains
must then be tuned. This is known as _decentralized control_, because each joint
is controlled as if it is independent of the others. A torque applied to
one joint creates dynamic effects on the other joints; these dynamic effects
are treated as disturbances to be eliminated. A centralized control scheme
(like inverse dynamics control) computes controls for each joint 
simultaneously and anticipating effects.

## Combining feedforward and feedback control

The best of both worlds is a hybrid control scheme that combines the
best aspects of feedforward and feedback control. Two options are to
sum the outputs from the two control schemes and feed the output from the  
error feedback controller into the feedforward controller; only the latter
is truly a centralized controller.

Inverse dynamics is the most obvious option for feedforward control. Another
possibility for feedforward control is use a gravity compensation block.

{% include image.html url="../../assets/img/composite.png" description="A control block diagram of a composite (negative feedback + inverse dynamics controller) for controlling a robot. A good inverse dynamics model will allow error feedback gains to be really small." %}

{% include image.html url="../../assets/img/gravity.png" description="A control block diagram of an error feedback plus gravity compensation scheme for controlling a robot. Gravity compensation is less dependent on having a good model than inverse dynamics and also requires less computation, but tracking accuracy is potentially much lower." %}


## Stability of a control system 

[We have already looked at stability of dynamical systems](../dynamical-systems). Simplifying the issue, there we sought to find whether a dynamical system
would converge to an equilibrium point from points in the neighborhood of the
equilibrium point. For controlled systems (like robots), we seek guarantees 
that applying the controller to the system will result in bounded errors
(recall the definition of the error signal above), _even if the uncontrolled 
system is unstable_.

For example, if a controller causes a system's error to be describable by the linear 
differential equation

\begin{equation}
\dot{\mathbf{e}} = -\mathbf{K}\mathbf{e}
\end{equation}

where \\(\mathbf{K} \in \mathbb{R}^n\\), then the controlled system is stable
if all real eigenvalue components of \\(-\mathbf{K}\\) are negative (and
marginally stable if all real eigenvalue components are non-positive). 

Unfortunately, the combined differential equations of the plant and the 
controller are usually nonlinear and not generally amenable to such simple
analysis. Lyapunov stability analysis is often the go-to approach.

### System response

{% include image.html url="https://upload.wikimedia.org/wikipedia/commons/7/71/Second_order_under-damped_response.svg" description="Figure depicting rise time, overshoot, delay time, and peak time. Image from Wikipedia." %}

In the figure above, \\(t\_d\\) is the delay time, \\(t\_r\\) is the rise
time, \\(t\_p\\) is the peak time, and \\(M\_p\\) is the overshoot. Settling
time and steady state error are not depicted.

These definitions are adapted from [here](https://en.wikipedia.org/wiki/Transient_response).

- **rise time**: the time required for the system to move from a specified low value to a specified high value 
- **overshoot**: occurs when the system exceeds its set point (followed by a period of oscillation if the system is [underdamped](../dynamical-systems)
- **settling time**: the time that the system enters and remains within a specified error band around the set point
- **delay time**: the time required for the system to move halfway from the
specified low value to the specified high value
- **peak time**: the time required for the system to hit the first overshoot peak

{% include image.html url="http://upload.wikimedia.org/wikipedia/commons/3/33/PID_Compensation_Animated.gif" description="Animation depicting effect of different PID gains on rise time, overshoot, and settling time. Image from Wikipedia." %}

{% comment %}
## Controllability 

One commonly encountered problem in robotics is that the system may not be 
fully _controllable_. I'm still looking for a great definition for this
concept. Here are a few tries:

1. If the dynamical system's configuration is of
dimension \\(n\\), the system is not fully controllable if each state
dimension cannot be controlled independently.
2. If, for a linearization of the system at any state (yielding the linearized system \\(\dot{\mathbf{x}} = \mathbf{A}\mathbf{x} + \mathbf{Bu}\\)), the matrix \\(\mathbf{B}\\) has rank equal to the dimensionality of \\(\mathbf{x}\\), the system is fully controllable. 

A good example of a system that is not fully controllable is a car. It has
three degrees of freedom on a planar surface, and only two control inputs
(accelerator and steering). 

{% endcomment %}
