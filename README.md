# Graph Based SLAM (Bearing only)

Simultaneous Localisation And Mapping (SLAM) is a stochastic map building method which allows consistent robot navigation without requiring an a priori map. The map is built incrementally as the robot observes the environment with its on-board sensors and, at the same time, is used to localise the robot. Typically, SLAM has been performed using range-bearing sensors, but the development of a SLAM implementation using only bearing measurements is desirable as it permits the use of sensors such as CCD cameras, which are small, reliable and cheap.

However, the outcome of SLAM with bearing only sensors is strictly dependent on the initalitialization. This, in turn, depends on the generation of the dataset, hence on how the robot moves in the map and what kind of measuraments retrieve from the landmarks that observers.
