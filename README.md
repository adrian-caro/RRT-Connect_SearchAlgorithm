# Implementation of RRT-Connect Path Planning algorithm

The algorithm is implemented as part of the project https://github.com/adrian-caro/Robot-Location-PathPlanning-Control.

This RRT-Connect (Rapidly-exploring random tree) builds two random search trees and tries to connect them , finding a solution path.

A smoothing code has been developed in order to adjust the solution path to the map characteristics, making it smoother, rounding corners and difficult turns as possible.
