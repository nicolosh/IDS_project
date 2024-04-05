# IntelligentDistributedSystems_Project
Intelligent Distributed Systems Project - 2022-2023


## Authors

- [Nicolò Cavalieri](https://www.linkedin.com/in/nicolò-cavalieri-263774194/) - MSc in Mechatronics Engineering, University of Trento
- [Federico Burgio](https://github.com/FedericoBurgio) - MSc in Mechatronics Engineering, University of Trento

### Repository description
In the folder [course_2022_2023](https://github.com/nicolosh/IntelligentDistributedSystems_Project/tree/main/course_2022_2023) you can find all the contents, divided topic by topic, covered during the edition of 2022-2023 of the Intelligent Distributed Systems course held by professor Daniele Fontanelli (Trento, Italy) while in the folder [course_lab](https://github.com/nicolosh/IntelligentDistributedSystems_Project/tree/main/course_lab) there are the main algorithms developed in class which can be useful for the exam project.

# Name of the project (TO ADD)
<img align="right" height="190" src="home-security-drone.jpg">
Multi-Camera surveillance systems have become a modern trend in many fields both for research and for the industry. These systems are being increasingly used to enhance safety and security in public places such as parks, airports, and banks, as well as in restricted areas like government and military facilities. Furthermore, the possibility to mount cameras on Unmanned Aerial Vehicles (UAVs) extends the capabilities of these surveillance systems to a whole new level. Indeed, the employment of flying cameras allow for a substantial reconfigurability of the network and enable for new perspectives on the environment and wider coverage.

### Project Abstract
The aim of this project is to find a possible solution to the Interactive Surveillance problem using multiple UAV mounted cameras, from a top down perspective, in known environments. The coordinated patrolling problem is approached in a distributed fashion using a Bayesian-based Greedy algorithm with State Exchange. The results of this strategy are tested in restricted environments. Furthermore, the overall performance is evaluated on large scale environments. Kalman’s theory is employed to implement a smart target tracking algorithm with camera zoom optimized for target containment and information loss minimization. Several simulations are performed tracking a target with different trajectory models and varying the sampling frequency of the filter and the accuracy of the detection. Finally, the robustness of such an algorithm to measurement errors and camera failures is put to the test.

For the technical results complete dissertation and the detailed description of the adopted strategies you can refer to the [Project Report :memo:](main.pdf).

### Working Simulation
The results achieved with this project can be clearly appreciated in the following simulation. Indeed, the Interactive Sourveillance task performed by our UAV mounted camera system can be divided in 4 steps:
1. All the agents of the camera network patroll the environment sharing information to autonomously select the most suitable direction to follow
2. An intruder enters the environment and it is soon detected from one of our UAV cameras
3. The camera begins the tracking task adjusting the zoom managing the tradeoff between tracking robustness and information gathering
4. On target loss the tracking camera returns to the tracking task, asking for the support of other UAVs in order to maximize the probability of tracking recovery


### Project Organization
```
.
├── Data/                                       : Contains precompiled environments
├── src/                                        : Contains developed classes and testing files
├── Camera.m                                    : Class defining a UAV camera object
├── Environment.m                               : Class defining the environment
├── Optimal_zoom.m                              : Routine to test the smart zoom feature
├── SEBS.m                                      : Routine to test the distributed approach to the patrolling problem
├── Simulation.m                                : Routine to test the complete system on large scale environments
├── Simulation_preprocessing.m                  : Defines the Simulation parameters
├── Tracking_Control.m                          : Routine to test the UAV movement system during the tracking task
├── Tracking_indices.m                          : Routine to the tracking robustness
├── envCell.m                                   : Function for environment cellularization
├── envEdges.m                                  : Function defining the allowed movement in the cellularized environment
├── kalman.m                                    : Function implementing the one step ahead Kalman filter
├── remapEdges.m                                : Function returning the set of edges associated to the connected components of a graph
├── main.pdf                                    : Project Report
└── README.md                                   : Project Summary 
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details