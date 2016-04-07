--- 
layout: post
title:  "Motion planning"
date:   2016-04-04
---
<p class="intro">

Robots that move within human and natural environments must navigate around
obstacles. A particularly intricate such task is depicted below. 

{% include image.html url="../../assets/img/virtual-humans.png" description="A virtual human reaching into a refrigerator. Image taken from M. Kallmann, A. Aubel, T. Abaci, and D. Thalmann. Planning collision-free reaching motions for interactive object manipulation and grasping. _Eurographics_, 22(3), 2003." width="1000" %}  

This learning module focuses on planning paths for robots to move around
obstacles in the environment and to avoid self collisions (e.g., avoiding
trying to move a robot's arm through its torso). This module will use the
following concepts: 

* __Configuration space__: The configuration space of a robot is a set of
values that can be used to completely determine its geometric description in 
Cartesian space. If the robot's links are rigid, the configuration space
consists of the information sufficient to determine the position and orientation
of each link.  This information can be the robot's [independent coordinates or absolute coordinates](../forward-dynamics), but any selection of [generalized coordinates](../forward-dynamics) will do.
* __Free configuration space__: The subset of a robot's configuration space
that does not correspond to a geometric intersection (either between two of the robot's links or between the robot and the environment). Intersecting
configurations are said to be _in collision_ and algorithms for querying
for geometric intersections are commonly called _collision detection algorithms_.

TODO: need a picture of points in Cfree and Cobs

## The Piano Mover's Problem

The problem of planning an intersection free motion is known as "The Piano Mover's Problem", and this problem is depicted well in the figure below. The top left picture shows the initial configuration of the piano, and the top right
picture shows the goal configuration of the piano.

{% include image.html url="http://planning.cs.uiuc.edu/img233.gif" description="A depiction of using mobile robots to move a piano. Image taken from S. LaValle. _Planning Algorithms_. Cambridge University Press, 2006." width="400" %}  

For two-dimensional configuration spaces, solving the Piano Mover's Problem
[can be performed in time \\(O(n \lg n)\\) in the number of vertices of the
polygonalization of the obstacle regions in configuration space](http://planning.cs.uiuc.edu/node267.html). Almost all of the problems concerned with
robots that manipulate their environments require considering more dimensions,
however, and the algorithms required to solve these problems do not exhibit
such nice properties.

### Difficulty of solving the Piano Mover's Problem 

For the special case of two-dimensional configuration spaces, I cited a
quadratic time result. However, generally motion planning is [PSPACE-complete](https://mitpress.mit.edu/books/complexity-robot-motion-planning) in the cardinality of the robot's configuration space, meaning- under current thinking on computational complexity theory- that solving the Piano Mover's Problem is likely 
computationally intractable. Just as challenging, algorithms for solving the
Piano Mover's Problem [are extremely challenging to implement](http://planning.cs.uiuc.edu/node298.html). 

## Sampling-based approaches to the Piano Mover's Problem

Sampling based approaches do not "solve" the Piano Mover's Problem because
they cannot report when a path does not exist (this may not be a problem
for many robotics applications). 

These algorithms work by sampling- either probabilistically or deterministically- some number of points in the robot's configuration space. Sampling based
approaches are usually either _probabilistically complete_ or _resolution complete_. Probabilistic completeness means that a solution is found (assuming it 
exists) as the number of probabilistic samples goes to infinity. Resolution
completeness means that a solution is found (again, assuming it exists), as
the distance between any two samples goes to zero. Probabilistic sampling is
often used because it scales well to higher dimensions (_ED: I'm having trouble locating a reference to back up this statement_), even though deterministic sampling has some advantages.

The implementation of sample-based algorithms is significantly easier than existing complete algorithms. All that is required is a mechanism for querying
whether a robot configuration is in collision, a simple graph search algorithm
implementation, and a small bit of programming code.

### Graph-based search

Algorithms for searching graphs are beyond the scope of this learning module-
they are covered in most introductory classes on algorithms. I will briefly
describe an arbitrarily chosen algorithm for this, Dijkstra's Algorithm.
Pseudocode follows (adapted from T. Cormen, C. Leiserson, R. Rivest, and C. Stein. _Introduction to Algorithms_, 3rd ed. MIT Press, 2009.)

    % Dijkstra's Algorithm is called with graph G (consisting of vertices 
    % and edges), a map of edge weights (w), and a starting vertex (start)
    Dijkstra(G, w, start):

    % initialize a priority queue
    Q := {}

    % initialize distances and pathways, add vertices to priority queue
    for each vertex v in G
      dist[v] := infinity
      prev[v] := nil
      add v to Q

    % distance from start to itself is zero
    dist[start] := 0

    % continue until the queue is empty
    while Q not empty
      u := vertex in Q with minimum dist[u]
      remove u from Q
      for each vertex v adjacent to u
        trial_dist := dist[u] + w[u, v]    % Relax path...
        if trial_dist < dist[v]            % ...to v ...
          dist[v] := trial_dist            % ...using ...
          prev[v] := u                     % ...u.
          
    return dist, prev

To simplify matters: we give the sample based motion planner two vertices 
corresponding to the robot's initial configuration and its goal configuration,
and a graph connecting these vertices using (likely) some newly created
vertices and a number of edges is output. We then feed this graph into 
Dijkstra's Algorithm- lacking better information, a weight of 1.0 is assigned 
to each edge- which returns two arrays to us, ``dist`` and ``prev``. 

``prev`` is the important one. Starting from the goal vertex, ``prev`` keeps
pointing us backward until we have reached ``start`` (``prev[start]`` will be equal to ``nil``). A list of the path from the start to the goal can be obtained using the following pseudocode:

    path := {}
    node := goal
    while node != nil
      insert node at the front of path
      node := prev[node] 

#### Asymptotic complexity of Dijkstra's Algorithm
The asymptotic time complexity of Dijkstra's Algorithm, using a simple implementation, is \\(O(|V|^2)\\); in other words, quadratic in the number of vertices
in the graph. This asymptotic complexity is identical to that exhibited to
connect nodes in the sampling based algorithms.

### Rapidly-exploring random tree (single query)

The _rapidly-exploring random tree_ (RRT) is a single query algorithm: it must be run anew any time a plan is necessary. Even though the probabilistic
roadmap (described next) generates a graph that can be used for
multiple queries, the RRT efficiently explores the configuration space.
This efficient exploration- depicted in the figure below- is particularly 
important, as the sampling based algorithms slow considerably as samples are 
added to the graph.

{% include image.html url="http://planning.cs.uiuc.edu/img2043.gif" description="Depiction of the fractal like exploration of the RRT. Image taken from LaValle's Planning Algorithms book." %}

    % constructs an RRT that attempts to connect initial
    % configuration q0 to goal configuration qgoal.
    % connect_freq > 1 is an integer that specifies how frequently
    % to attempt a connection to the goal (every other iteration = 2,
    % every tenth iteration = 10, etc.) Returns the tree when complete.
    RRT(q0, qgoal, connect_freq)

      % initialize tree (graph) G with a single vertex, q0
      init(G, q0)

      % setup the number of nodes in the tree
      n := 1

      % loop indefinitely
      while true

        % see whether to attempt to connect to the goal
        if mod(n, connect_freq) = 0
          qs = qgoal
        else
          % draw a random sample in configuration space
          qs = sample()

        % find the closest point in the "swath" of the tree to qs
        qclosest = closest_swath(G, qs)
      
        % extend qclosest as far as possible toward qs
        qext = extend(qclosest, qs)

        % ensure we were able to make some progress
        if (qext != qclosest)
          % add qclosest to the tree (ignores if qclosest already lies 
          % on a vertex of the tree)
          add(G, qclosest)

          % add qext to the tree
          add(G, qext)

          % check and see whether the goal was connected to
          if (qext = qgoal)
            return G

          % update n
          n := n + 1

A depiction of the result from the ``closest_swath`` function is shown below.
``closest_swath`` should locate the closest point on the closest edge to the
sampled configuration.

{% include image.html url="../../assets/img/swath.png" width="500" description="Depiction of the closest_swath function." %} 

The ``extend`` function extends a node as far as possible toward another node,
as depicted in the figure below.

{% include image.html url="../../assets/img/extend.png" width="500" description="Depiction of the extend(.) function." %}


A pseudocode implementation of ``extend`` is provided next:

    % extends node a as far as possible toward node b, returning a new node, c 
    % using tolerance eps << 1
    function extend(a, b, eps)

      % set last collision free node
      qlast := a

      % increase t from eps to 1-eps
      for t:=eps:eps:1-eps
        q := a*(1-eps) + b*eps
        if not collision_free(q)
          return qlast
        else
          qlast := q

      % all samples checked successfully
      return b 

``extend`` uses linear interpolation between two points on a line segment:

\begin{equation}
\mathbf{v} = \mathbf{p}\*(1-t) + \mathbf{q}\*t, \textrm{ for } t \in [0,1]
\end{equation}

Note that \\(t=0\\) yields \\(\mathbf{p}\\) while \\(t=1\\) yields \\(\mathbf{q}\\).

### Probabilistic roadmap (multi-queries)

The _probabilistic roadmap_ (PRM) is a multi-query data structure: after
the roadmap has been built, multiple queries can be performed on the graph. 
The PRM operates in two phases, a construction phase and a query phase.
The pseudocode for the construction phase is described next.

    % constructs a probabilistic roadmap of n nodes
    % Returns the graph
    Construct(n)

      % initialize graph G without nodes 
      init(G)

      % loop until the graph has the requisite number of nodes 
      for i=2:n 

        % draw a random sample in free configuration space
        repeat
          qs := sample()
        until not collision_free(qs)

        % add the sample as a vertex to the graph
        v := add(G, qs)

        % attempt to connect the vertex to other vertices in G
        for each vertex u in G
          % don't connect vertex to itself!
          if u = v
            continue

          % attempt to connect u and v
          if connectable(u,v)
            add_edge(G, u, v)

The ``connect`` function determines whether there is a path in configuration
space between two nodes. This function can be implemented naively as:

    % checks whether the straight line path in configuration space
    % between a and b is collision-free to a tolerance of eps << 1
    function connect(a, b, eps)

      % increase t from eps to 1-eps
      for t:=eps:eps:1-eps
        q := a*(1-eps) + b*eps
        if not collision_free(q)
          return false

      % all samples checked successfully
      return true

{% comment %}
Can you come up with a more efficient version of this function that rejects
paths in collision faster? 
{% endcomment %}

The query phase operates upon an initial configuration (q0) and a goal configuration (qgoal). The query phase attempts to connect each of these two 
configurations to the roadmap using a straight line path in configuration space.
The ``query`` function is a specialized version of the ``construct`` function.
``query`` returns __true__ if it is able to connect the initial and goal configurations to the roadmap and __false__ otherwise. Recall that __false__ does not necessarily mean that a path does not exist.

    % attempts to connect q0 and qgoal to a constructed roadmap 
    query(G, q0, qgoal)

      % add the initial configuration as a vertex to the graph
      v := add(G, q0)

      % indicate no success yet
      success := false

      % attempt to connect the vertex to other vertices in G
      for each vertex u in G
        % don't connect vertex to itself!
        if u = v
          continue

        % attempt to connect u and v
        if connectable(u,v)
          add_edge(G, u, v)
          success := true

      % verify we were successful
      if success = false
        return false

      % add the goal configuration as a vertex to the graph
      v := add(G, qgoal)

      % indicate no success yet
      success := false

      % attempt to connect the vertex to other vertices in G
      for each vertex u in G
        % don't connect vertex to itself!
        if u = v
          continue

        % attempt to connect u and v
        if connectable(u,v)
          add_edge(G, u, v)
          success := true

      % verify we were successful
      if success = false
        return false
      else
        return true, G

----

The pseudocode descriptions of these algorithms are simplistic
and are presented primarily for pedagogical purposes. More sophisticated 
versions of these algorithms are described [here](). These
algorithms (and many more sample based algorithms) have been an active
area of research since the 1980's. 

### Time complexity of sampling-based algorithms
The time complexity of these algorithms is dominated by the search for
nearest neighbors (probabilistic roadmap) or closest point on the swath
of the tree (RRT). [k-d tree](https://en.wikipedia.org/wiki/K-d_tree) data 
structures can yield logarithmic time such searches, but [are not efficient
in the high dimensional spaces](https://en.wikipedia.org/wiki/K-d_tree#High-dimensional_data) that robot motion planning problems typically
live within. Using exhaustive search for nearest neighbor/closest point on
the swath operations yields quadratic time complexity in the number of
samples used to construct the tree:

\begin{equation}
\sum_{i=1}^n i = O(n^2)
\end{equation}

Here \\(n\\) corresponds to the total number of PRM/RRT iterations and 
\\(i\\) counts the vertices searched over in the \\(i^{\textrm{th}}\\)
iteration.

## Criticisms of (current) motion planning approaches for robotics

I will level the following criticisms of the current state of motion planning
approaches for robotics. These criticisms are much less applicable to related
areas, like computer animation.

* **Avoiding contact with the environment is impossible**: If we are interested in robots that physically interact with their environments, avoiding contact is 
undesirable. Applying motion planning algorithms to problems like having a
robot pickup an object requires a Kabuki dance: collisions between
the robot hand and the object must be disabled while the robot is manipulating
it, and the object must be added to the robot's geometric description to 
perform motion planning during further manipulation.
* **A complete geometric description of the environment is usually not obtainable**: Parts of the environment will be obscured, sensory noise will indicate parts of the environment are obstructed (when they are not), and creating a geometric representation of the environment online requires communicating and processing prodigious amounts of sensory data. Humans frequently alter their environments in small ways- think moving a chair- so relying upon a description built offline may not be a good idea. 
* **Nonstationary environments increase the challenges significantly**: The Piano Mover's Problem assumes the environment does not change during the planning process. Dropping this assumption naturally [makes the problem harder](http://ieeexplore.ieee.org/xpl/login.jsp?tp=&arnumber=4568138&url=http%3A%2F%2Fieeexplore.ieee.org%2Fxpls%2Fabs_all.jsp%3Farnumber%3D4568138). For robotics, we either have to solve this harder problem or solve the motion planning problems extraordinarily quickly.
* **Accounting for differential constraints is challenging**: Collision-free samples between two arbitrarily chosen configurations cannot be connected using a straight line in configuration space for many robots (legged robots, for example). The RRT Algorithm, for example, has been applied to such (nonholonomically constrained)[https://en.wikipedia.org/wiki/Nonholonomic_system] robots, but approaches described in present literature do not seem to be able to generate plans frequently (_speaking strictly from my own experience, so take with a grain of salt_).
 
## Further reading

Much of the material in this learning module has been drawn from 
[LaValle's excellent, free book](http://planning.cs.uiuc.edu) on motion planning algorithms.

