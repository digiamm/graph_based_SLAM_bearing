# Graph Based SLAM (Bearing only)

### Introduction

Simultaneous Localisation And Mapping (SLAM) is a stochastic map building method which allows consistent robot navigation without requiring an a priori map. The map is built incrementally as the robot observes the environment with its on-board sensors and, at the same time, is used to localize the robot. Typically, SLAM has been performed using range-bearing sensors, but the development of a SLAM implementation using only bearing measurements is desirable as it permits the use of sensors such as CCD cameras, which are small, reliable and cheap.

However, the outcome of SLAM with bearing only sensors is strictly dependent on the initalitialization. This, in turn, depends on the generation of the dataset, hence on how the robot moves in the map and what kind of measuraments retrieve from the landmarks that observers. The main difficulty is due to the fact that a single bearing does not allow to reconstruct a 2D position. In fact, at least two bearings need to be used.

This projects provides two implemented ways of initialize the landmark pose through the bearing sensors. Among these, only the one that uses triangulation with two bearings works efficiently. 

The problem of SLAM is than solved using graph-based approach. In the graph, the nodes corresponds to the poses of the robot at different points in time and the landmarks, while the edges represent the constraint between poses. Once such a graph is constructed, the error is minimized using a least-squares approach (Gauss-Newton).

### Initialization


### Least-Squares Error Minimization

http://cal.cs.illinois.edu/~johannes/research/LS_line_intersect.pdf

http://www-personal.acfr.usyd.edu.au/tbailey/papers/icra03.pdf
