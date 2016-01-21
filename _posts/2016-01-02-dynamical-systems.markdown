--- 
layout: post
title:  "Introduction to dynamical systems and ordinary differential equations"
date:   2016-1-7
---

The _equations of motion_- that transform forces applied to a robot into motion- represent a _dynamical system_, a mathematical concept. Knowledge of 
dynamical systems allows robots to predict how a thrown ball will move and
to pick footsteps to prevent falling over, among other uses.   

{% include image.html url="http://i.giphy.com/92kIGbLtN3sZO.gif" description="Dynamical systems predict the motion of objects to which forces have been applied." %}

## Dynamical systems

Let us start by defining the _state_ of a dynamical system. The state is
typically denoted with the variable \\(\mathbf{x} \in \mathbb{R}^n\\) and
may represent position, orientation, temperature, pressure, force, current, etc. 
_Time \\(t\\) and state of the dynamical
system at \\(t\\)_ must be sufficient to predict the state of the dynamical system at some time in the future. For example, let us assume that we know all of the forces acting on the moon Europa, that Europa is well modeled as a point mass, and that we have an accurate estimate of Europa's mass. Given this information and the current location of Europa, we still cannot predict Europa's location a year into the future: we also need
Europa's current velocity. In summary, if we treat Europa as a dynamical system- and planetary objects were the first objects of study in dynamical systems- then its state should consist (at least) of its position and velocity. 

The path that the state takes over time is known as its _trajectory_ or 
_orbit_.

{% include image.html url="https://upload.wikimedia.org/wikipedia/commons/2/28/Copernican_heliocentrism_diagram-2.jpg" width="400" description="Predicting planetary motion from an initial observation was one of the first applications of dynamical systems." %}

### Evolution of a dynamical system

Another key attribute of a dynamical system is that it should have an update
or transition rule. This rule may be deterministic or stochastic (probabilistic or nondeterministic)- if the former, we can _accurately predict the state of
a dynamical system at a specified time into the future given only its state
at the current time_. An interesting example of the latter is the [N-armed bandit](https://en.wikipedia.org/wiki/Multi-armed_bandit). Stochastic dynamical systems are effectively [Markov Chains](https://en.wikipedia.org/wiki/Markov_chain)/[Markov Processes](https://en.wikipedia.org/wiki/Continuous-time_Markov_chain), for those readers familiar with stochastic processes. The remainder of this learning module will discuss only deterministic
dynamical systems.

Dynamical systems can also be discrete, meaning that they update discontinuously (the rule is then a _transition rule_). Alternatively, the system is continuous, which means that the dynamical system is continuous (and generally smooth). Some systems are [really a mix of the two](https://en.wikipedia.org/wiki/Hybrid_system), but we will not consider these hybrid systems further.

If the dynamical system is discrete, the transition rule is represented by a _difference equation_, shown in an example below:

\begin{align}
\mathbf{x}\_{i+1} = f(\mathbf{x}\_i)
\end{align}

Notice that the next state of \\(\mathbf{x}\\) is completely a function of the
current state and that the transition is immediate (discontinuous).

If the dynamical system is continuous, the update rule is represented by a 
(ordinary) _differential equation_:

\begin{align}
\dot{\mathbf{x}} = f(\mathbf{x}, t)
\end{align}

The dot notation \\(\dot{\mathbf{x}}\\) is a synonym for \\(\textrm{d}\mathbf{x}/\textrm{d}t\\).

We will only consider ordinary differential equations (ODEs), as opposed to differential equations that include partial derivatives, known as partial
differential equations (PDEs). The remainder of this learning module will
also consider only differential equations- difference equations will not be
discussed further.

### Digression: mathematical notation

For this learning module and all others, I will denote scalar real numbers using lowercase type (like \\(s \in \mathbb{R}\\)), vectors in boldface lowercase type (like \\(\mathbf{v} \in \mathbb{R}^n\\)), matrices in boldface uppercase type (like \\(\mathbf{A} \in \mathbb{R}^{n \times n}\\)).

Functions will be typeset in lowercase type, as in \\(f(.)\\), and will
be typeset in boldface type if they return a vector output, as in \\(\mathbf{g} \to \mathbb{R}^m\\).

## A simple dynamical system: the Lorenz Equations
A well-studied dynamical system was described by Lorenz for modeling climate. 
His dynamical system is a simple [chaotic system](https://en.wikipedia.org/wiki/Chaos_theory), which- like the weather- is highly sensitive to initial conditions. Lorenz's work spawned the thought that a
butterfly's wings flapping in Texas might be able to set off a tornado in 
Brazil.

The Lorenz equations follow:

\begin{align}
\frac{dx}{dt} & = \sigma(y - x) \\\\
\frac{dy}{dt} & = x(\rho - z) - y \\\\
\frac{dz}{dt} & = xy - \beta z 
\end{align}

where the system state is determined using variables \\((x, y, z)\\), \\(t\\) is time, and \\(\sigma \textrm{ (sigma)}, \rho \textrm{ (rho)}, \textrm{ and } \beta \textrm{ (beta)}\\) are constant parameters.

The Lorenz equations are plotted below using some unknown initial values and unknown \\(t\\):

{% include image.html url="https://upload.wikimedia.org/wikipedia/commons/1/13/A_Trajectory_Through_Phase_Space_in_a_Lorenz_Attractor.gif" description="Depiction of the Lorenz attractor for rho = 28, sigma=10, beta = 8/3." %}


### Plotting the Lorenz system using Octave/Matlab

I will now show how to use GNU Octave and Matlab to integrate the Lorenz equations
numerically.

First, we set the initial conditions of x, y, and z in Matlab/Octave to x=1, y=1, z=1. We will label all variables \\(\mathbf{x}\\) to use Octave/Matlab vector notation.

    >> % set the initial conditions
    >> x0 = [ 1 1 1 ]';

__In Matlab__:

Create the file ```lorenz.m``` and save it in your current directory.
The contents of this file should be:

    function dxdt = lorenz(t, x)

      % set values for rho, sigma, and beta
      rho = 28;
      sigma = 10;
      beta = 8/3;

      % setup the right hand side of the ordinary differential equation 
      dxdt(1,1) = sigma*(x(2) - x(1));
      dxdt(2,1) = x(1)*(rho - x(3)) - x(2);
      dxdt(3,1) = x(1)*x(2) - beta*x(3);

At the Matlab prompt (">>") run the following commands:

      >> t = 0:1e-2:20;
      >> [tout, x] = ode45(@lorenz, t, x0);
      >> plot3(x(:,1), x(:,2), x(:,3));

__In GNU/Octave__, the file ```lorenz.m``` should look almost exactly the same- the arguments ```t``` and ```x``` to the ```lorenz``` function are swapped. The file is otherwise identical:
 
    function dxdt = lorenz(x, t)

      % set values for rho, sigma, and beta
      rho = 28;
      sigma = 10;
      beta = 8/3;

      % setup the right hand side of the ordinary differential equation 
      dxdt(1,1) = sigma*(x(2) - x(1));
      dxdt(2,1) = x(1)*(rho - x(3)) - x(2);
      dxdt(3,1) = x(1)*x(2) - beta*x(3);

At the Octave prompt ("octave:n>") run the following commands:

      octave:1> t = 0:1e-2:20;
      octave:2> x = lsode(@lorenz, x0, t);
      octave:3> plot3(x(:,1), x(:,2), x(:,3));



## Solving ODEs 

"Solving an ordinary differential equation" indicates solving the initial value problem in closed form. 
In other words, given initial conditions, what is the state at some time in the future? As an example, consider one ODE:

\begin{equation}
\dot{x} = -10x
\end{equation}

The solution to this ODE is:

\begin{equation}
x(t) = Ce^{-10t}
\end{equation}

where \\(C\\) is an unknown constant. When we are given \\(x(0)\\), then we
can solve algebraically for \\(C\\).

{% comment %}
1. Compute C for x(0) = 10  (Ans: 10)
2. Compute x(1) for x(0) = 10 (Ans: 4.54e-4)
3. verify that taking the derivative of the solution yields the ODE.
{% endcomment %}


#### Wait, can't we just use the anti-derivative? (integrals?)

The anti-derivative is usable to solve _separable_ differential equations,
like:

\begin{equation}
\frac{\textrm{d}y}{\textrm{d}x} = 6y^2x
\end{equation}

All \\(y\\) and \\(x\\) variables simply need to be on different sides of the
equation to use anti-derivatives:

\begin{align}
\int \frac{\textrm{d}y}{y^2} & = \int 6x\ \textrm{d}x \\\\
\frac{-1}{y} + C & = 3x^2 
\end{align}

Now consider the ordinary differential equation below:

\begin{equation}
\frac{\textrm{d}y}{\textrm{d}x} + xy = x^2 
\end{equation}

See whether you can find a way to separate it as above.

A course on ODEs teaches you to solve initial value problems for 
ordinary differential equations (particularly inseparable ODEs like that above), in closed form. In robotics, our ODEs are
often unknown or too complex to compute closed form solutions for. We 
generally focus on _numerical_ solutions to ODEs.

### Solving ODEs numerically

When initial value problems for ODEs cannot be solved in closed form, they must be solved numerically. This section discusses doing just that. 

{% include image.html url="../../assets/img/Euler.png" description="Illustration of Euler's Method for integrating ODE's numerically. The derivative at the starting point of each interval is extrapolated to find the next function value. Taken from Numerical Recipes in C." %}

#### Another dynamical system, the pendulum
The regular motion of the pendulum has fascinated humans for centuries, and
the pendulum acted as the underlying mechanism of the first accurate clocks. 

{% include image.html url="https://upload.wikimedia.org/wikipedia/commons/8/80/Pendulum_60deg.gif" width="300" description="Animation of a simple model of a pendulum. The 'phase space'- position and velocity variables- of the pendulum is depicted above. Which axis corresponds to which?" %}

The ordinary differential equation of motion for the pendulum is:

\begin{equation}
\ddot{\theta} + g/L \sin \theta = 0
\end{equation}

where \\(g\\) is the acceleration due to gravity, \\(L\\) is the length of the rod, and \\(\theta\\) is the angular displacement. The pendulum is a classical dynamical system and deriving its ordinary differential equation is a seminal task in courses on ordinary differential equations and classical mechanics.

_The pendulum is an essential component of many seminal dynamic tasks
in control theory, including the [pendulum swing-up task](http://www4.ncsu.edu/~rsmith/MA731_S09/Astrom_Furuta.pdf) and the [cart-pole balancing problem](http://ieeexplore.ieee.org/xpl/login.jsp?tp=&arnumber=6313077&url=http%3A%2F%2Fieeexplore.ieee.org%2Fxpls%2Fabs_all.jsp%3Farnumber%3D6313077)._ [The cart-pole 
balancing problem has been effectively used as a balancing model for 
stabilizing legged robots.](http://ieeexplore.ieee.org/xpl/login.jsp?tp=&arnumber=973365&url=http%3A%2F%2Fieeexplore.ieee.org%2Fxpls%2Fabs_all.jsp%3Farnumber%3D973365)

#### Canonical system of first order ODEs
Most algorithms for numerically integrating ordinary differential equations
require that the ODEs be first order, where the _order_ of an ODE is the highest derivative in the equation. This problem is readily solved using variable substitution, as seen applied to the second order ODE for the pendulum below: 

\begin{align}
\dot{\theta} & = \vartheta \\\\
\dot{\vartheta} + g/L \sin \theta & = 0
\end{align}

{% comment %}
Practice using substitution method
{% endcomment %}

#### Euler's Method for integrating ODEs

Euler's Method for integrating ODEs is best derived using [Taylor's theorem](https://en.wikipedia.org/wiki/Taylor%27s_theorem) for approximating a function in a particular neighborhood using an infinite polynomial sum. However, I'll cover Euler's Method using the limit definition of the derivative instead. Recall:

\begin{equation}
\frac{\textrm{d}x}{dt} = \lim\_{\Delta t \to 0} \frac{x(t+\Delta t) - x(t)}{\Delta t}
\end{equation}

Thus, for sufficiently small \\(\Delta t\\), we have:

\begin{equation}
\frac{\textrm{d}x}{\textrm{d}t} = \frac{x(t+\Delta t) - x(t)}{\Delta t}
\end{equation}

and after just a little algebraic manipulation:

\begin{equation}
x(t+\Delta t) = x(t) + \Delta t \frac{\textrm{d}x}{\textrm{d}t}
\end{equation}

_This formula is the basis for an algorithm_. Use the current value of \\(x\\) and the current derivative, which is itself a function of the current value of \\(x\\), to compute the next value of \\(x\\). For the simplest version of this
algorithm, \\(\Delta t\\) is constant. 

#### Integrating the pendulum using Euler's Method

To numerically integrate the pendulum's equations of motion using initial
conditions \\(\theta = \pi/2, \vartheta = \dot{\theta} = 0\\), we take
the following steps, _valid for both Matlab and GNU/Octave._

__Step 1__: Setup the ODE function.

    function dxdt = pendulum(x)

      % setup gravity and pendulum rod length
      g = 9.8;
      L = 1;

      % setup theta and dtheta/dt
      theta = x(1);
      dtheta = x(2);

      % compute the ODE
      dxdt(1) = dtheta;
      dxdt(2) = -g/L * sin(theta);

In contrast to ``lorenz``, ``pendulum`` does not accept \\(t\\)
as a formal parameter, because the ODE is not explicitly dependent upon \\(t\\).

__Step 2__: Setup the initial conditions at the Matlab/Octave command prompt.

      >> x(1,:) = [pi/2 0];

__Step 3__: Pick the integration step size. I use all caps in my programming code to denote constants. We'll want to explore the effect of varying this value. 

      >> DT = .01;

__Step 4__: Pick the time interval to integrate over. Because the pendulum is a periodic dynamical system, I want the time interval to be sufficiently large to observe the periodic motion.

      >> TEND = 10;

__Step 5__: Perform the Euler integration.

      >> for t=DT:DT:TEND  % integrate for 10 seconds
      i = size(x,1); % get the last state index
      dxdt = pendulum(x(i,:)); % evaluate the ODE using current x 
      x(i+1,:) = x(i,:) + dxdt*DT; % integrate state forward 
      end

__Step 6__: Plot the joint angle of the pendulum over time.

      >> plot(0:DT:tend, x(:,1));

{% comment %}
Try integration with a few different step sizes from 0.1...1e-4, plotting the results. Why are the plots generated from using smaller step sizes more accurate? (Trust me, they are.)
{% endcomment %}

## Stability of a dynamical system

If the state of a dynamical system is two dimensional, we can readily
visualize the change in state using arrow depictions. Examine some 
visualizations of dynamical systems below carefully.

The dynamical system below can be seen to drive the state to \\(\mathbf{x}=0,0\\). This point is known as a _equilibrium point_ or a _fixed point_. This 
particular system is _stable_ in the region that we are viewing: if the
system enters this region, the state will eventually end up at the equilibrium
point.

This concept is important in airplanes. If the pilot puts the plane into a dive
and then removes her hands from the yoke, the plane will return to level
flight after a short time. _This concept is important in robotics for the same reason_:
a robot will ideally "go to rest" if no controls are applied to it for some
time. 

{% include image.html url="http://www.entropy.energy/scholar/node/dynamical-systems-maps/dynamical-map-dampedho.png" width="600" %}

The dynamical system below depicts many fixed points (where arrows are
missing). Two regions of such fixed points are stable: the left and upper
right vortices. Equilibrium points in other regions are not stable because,
if sufficiently far from the region, the state can get swept into one of the 
two vortices.

{% include image.html url="http://www.lct.jussieu.fr/pagesperso/silvi/elfhtml/dynamic.gif" width="600" %}

{% comment %}
Animate a particle in 3D subject to a gravitational sink at the origin.
{% endcomment %}

### Determining whether a dynamical system is stable around a fixed point

Determining stability in the neighborhood of an equilibrium point is
mostly beyond the scope of this learning module. I will cover a few key concepts, however.

#### Stability type

Stability considers the neighborhood of states around an equilibrium point. 

The following types of stability are listed in decreasing order from the
strongest sense of stability to the weakest. 

* __Exponential stability__: neighboring states converge exponentially fast to the fixed point
* __Asymptotic stability__: neighboring states eventually converge to the fixed point
* __Lyapunov stability__: neighboring states stay within a small neighborhood 

#### Methods for determining stability

Without going too far into stability analysis of dynamical systems, I will
quickly describe two methods for analyzing stability.

##### Linearization

A linear dynamical system can be expressed in the form:

\begin{equation}
\dot{\mathbf{x}} = \mathbf{A}\mathbf{x}
\end{equation}

where \\(\mathbf{A}\\) is a matrix. If the dynamical system is linear or
it can effectively be approximated by a linear dynamical system in the 
neighborhood of the equilibrium point, we can analyze the eigenvalues of \\(\mathbf{A}\\). Unless \\(\mathbf{A}\\) is a symmetric matrix, it will have 
both real and imaginary eigenvalues.

* If the real components of all eigenvalues are negative, the linear system is asymptotically stable
* If all real components of all eigenvalues are non-positive, the linear system is marginally stable
* If there exists a positive real eigenvalue, the linear system is unstable around the equilibrium point

Within the realm of stable systems, a system is:

- **Underdamped** if the system oscillates about equilibrium with oscillations gradually decreasing in amplitude until finally reaching zero. A system is underdamped if the eigenvalues have complex components (but the real components are necessarily non-positive).
- **Overdamped** if the system exponentially decays to equilibrium without oscillating. A system is overdamped if the eigenvalues are real and unequal (and are negative). 
- **Critically damped** if the system returns to equilibrium as quickly as possible without oscillating. A system is underdamped if all eigenvalues are real and equal (and negative).

{% include image.html url="https://www.softintegration.com/docs/ch/qanimate/examples/vibration/vibration_large.gif" description="Depiction of overdamping, critical damping, and underdamping for a mass-spring-damper system." %}

The table also depicts most of these conditions.



{% include image.html url="https://controls.engin.umich.edu/wiki/images/3/30/Eigenvalue_graphs.jpg" description="Plots of eigenvalues for a linear system." %}

{% comment %}
Identify stable, marginally stable, unstable, underdamped, overdamped, and critically damped. Which are undesirable?
{% endcomment %}


##### Lyapunov functions

[Lyapunov functions](https://en.wikipedia.org/wiki/Lyapunov_function) are the
go to approach for proving stability of _nonlinear_ dynamical systems.

Wikipedia's informal description of Lyapunov functions is accessible: "a Lyapunov function is a function that takes positive values everywhere except at any stasis in question, and decreases (or is non-increasing) along every trajectory of the ODE". If a Lyapunov
function can be found for an equilibrium point of a dynamical system, the
system is Lyapunov stable in the neighborhood of that point. Finding a
Lyapunov function for an arbitrary ODE is currently an open problem. 


