#### Introduction

This repository is used for publishing the source code of the paper "Sensorless and Coordination-Free Lane Switching on a Drone
Road Segment – A Simulation Study".

----

#### Dependency Version
| Dependency | Value |
| --------- | -----:|
| OMNeT++  | 5.6.2 |
| INET     |   inet-4.2.5-e0c1741924 |

----

#### Key Files
`src/RenewalGenerating.ned` The simualtion network.

`simulations/omnetpp.ini` The configuration file.

`src/arrivalCheck/arrivalChecker.cc` Used to detect the arrivals and collisions, and record collision data.

`src/mobility/DroneLinearMobility.cc` The mobility model for drones, it is an extension version of INET LinearMobility model.

`src/beaconing/renewalbeaconing/NodeGenerator.cc` The node generator model, used to generate and initialize drones.

`src/beaconing/processor/Processor.cc` The implementation of lane switching algorithm.

----
