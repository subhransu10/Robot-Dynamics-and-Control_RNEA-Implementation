# Robot-Dynamics-and-Control_RNEA Implementation
The assignment is about the calculation of inverse dynamics of manipulators
using Recursive Newton-Euler Algorithm.Recursive is used as it is an efficient
process i.e it takes less computations and CPU time. We make a generic
algorithm that calculates the inverse dynamics by above-mentioned method.
We also focus on the calculations of inverse dynamics joint torques along a
certain axis for the three manipulators (2R,RP and 3R).Using all the given
parameters, we calculate the problems on MATLAB.

__INTRODUCTION__

Inverse dynamics is the problem of calculating the forces required to produce a given acceleration.
Given the configuration,velocity and acceleration, it amounts to finding the joint torques and
contact forces such that the constrained equations of motion are satisfied.The algorithm works in
two ways: a forward recursion which is mostly a second-order forward kinematics,followed by a
backward recursion that computes forces and joint torques.At each step of recursion, we have two
vector equations at the joint providing the torques which also contains the reaction forces/torques
at joint axis.We also take into account that they should be projected next along/around the axis.
The algorithm also takes into account that irrespective of the number of links and joint-types
(Rotational/prismatic), it calculates the joint torques.

![Screenshot (877)](https://user-images.githubusercontent.com/93926797/189097300-01131fed-985f-45b0-9c4f-0a134d10fdbf.png)

The calculations and other informations are given in the matlab file and the report attached in the repository.


__CONCLUSION__

We have calculated the inverse dynamics using the Recursive Newton-Euler approach. Since it
is a time-efficient approach , it is highly used nowadays.The various equations and algorithm are
designed to use this data in their calculations. The class of robots considered were having
revolute and prismatic.The solution is best for synthesis of model based control schemes.
