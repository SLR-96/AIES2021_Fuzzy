# AIES2021_Fuzzy
Fuzzy control project from the 2021 course on Artificial Intelligence and Expert Systems (AIES)

## Path Tracking of a 3 DOF Robot Arm Using PID and Fuzzy Controllers

### Abstract
In this project a path tracking controller for a 3 degree of freedom arm is designed, using PID control and fuzzy logic control methods. Both control method parameters are then optimized by using a genetic algorithm. By comparison the PID controller has shown a better overall performance.

### I. Introduction
When designing a controller for a nonlinear system, especially with multiple degrees of freedom, the designer faces lots of difficulties because of the nonlinear nature of the system dynamics. This, in turn makes the design process a difficult and time-consuming task, while the eventual control system may not be optimal. When a system is nonlinear and chaotic in nature, linearizing its equations will not correctly represent its actual behavior. In order to maintain the accuracy and nonlinearity of the system equations, while avoiding the arduous task of designing a nonlinear control system, two relevantly simple control methods are proposed in this paper. First a PID controller, and then a Fuzzy Logic Controller (FLC) is used to control the system, then the results are compared against each other. The key in designing these systems, is to set the correct parameters to effectively control the system. In PID control, these parameters are the proportional, integrator and derivative gains. While larger gains generally result in smaller errors, they require larger amounts of system input, which can be quite costly and even unrealistic. The FLC on the other hand, has lots of parameters that need to be corrected for the system to work effectively. These most important parameters in a FLC system are the consequents to the rules which are usually in large numbers and quite tricky to optimize. In this paper, the said parameters are optimized by using a genetic algorithm.

### II. PROBLEM DEFINITION
The dynamic system to be controlled is a 3-link robot arm with 3 degrees of freedom, moving in 3D space. As shown in the Fig. 1, the first link is in the z direction and only rotates around the z axis. The second and third links can only move in a single plane parallel to the first link, but the plane itself is rotating with the first link and therefore the second and third links are free to move in a sphere in 3D space. The equations of motion are taken from [1] and their parameters are as follows:

![image](https://user-images.githubusercontent.com/65850584/220757703-351cb257-a56b-465d-b323-f2cb91c3a3e6.png)
Fig. 1. Depiction of the robot arm, taken from [1]
![image](https://user-images.githubusercontent.com/65850584/220757600-b1b9b040-bd4d-4c04-b429-5bb7a302a5b6.png)

### III.	METHODOLOGY

#### A.	Genetic Algorithm Optimization
Genetic Algorithm Optimizer (GAO) is an optimization algorithm which uses the fundamentals of genetic evolution and natural selection as its basis. In this algorithm a set of parameters is given which needs to be optimized to lower a cost function, dependent on the given problem. In the first iteration a population of random sets is made. Then in each iteration the cost of each set is calculated and the sets are arranged in the order of increasing cost (best to worst). Then each pair of two random sets from the best sets in the population are mixed to some extent to produce two similar sets called “offsprings”. Also, a certain percentage of the population will go through mutation to change some of their parameters. Finally, the best of the aggregated population of offsprings, mutants and the former population will be chosen as the new population and the rest will be removed.

In optimizing the parameters of the FLC for the given system, each set contains 174 parameters. And in optimizing the PID controller, each set contains 9 parameters. The algorithm is allowed to run for up to 1000 iterations. Each iteration outputs a population of 100 sets, the best 80 percent of which are chosen as parents of the offsprings, 30 percent are chosen to be mutated by an amount of 2 percent. After the algorithm runs for a specified number of iterations, it shows a graph depicting the best cost in each iteration. Also, the last iteration’s population and its best set are saved to a file.

#### B.	Fuzzy Logic Controller
A FLC uses qualitative fuzzy input and output values to control the system. The advantage of the FLC is that it has more flexible rules, instead of absolute ones. Another advantage of FLC is its intuitive rule base, which is more comprehensible for a human designer. The basic form of a FLC consists of several membership functions for each input and output, each having a qualitative label and a quantitative range. And the rules are defined for different inputs resulting in the associated outputs.

For this control system the 6 inputs, consisting of the location and speed errors, each have 3 membership functions, representing negative, zero, and positive values. On the other hand, the 3 outputs, consisting of the input torques to the system, each have 9 membership functions in order to have more diversity to choose from in defining the rules. The system design is based on the Mamdani Method, and the outputs are defuzzified using the centroid method.

The rules concerning the torque of each link, only consider the location and speed errors of the same link. Which reduces the dimension of the problem, and since the rules are determined by a GAO, this reduction of dimensions is very helpful in avoiding getting stuck in local minima. Therefore, for each pair of location and speed errors, there are 9 rules, making up 27 rules in total. Since the GAO needs to determine the output and input gains of the FLC, the total number of parameters which need to be optimized by the GAO is 36. The cost function used in the GAO is the sum of the absolute value of all the location and speed errors throughout a simulation.

In order to reduce the computational load, the FLC and GAO are manually written in MATLAB script and using built-in functions is avoided for as much as possible.

#### C.	PID Controller
One of the most widely used control methods, the PID controller takes the feedback error and multiplies the error, its derivative and its integral by their associated gains, and outputs the result.

The PID gains for this system are determined by the GAO in order to give the optimum functionality. Since there are only 3 errors for the whole system, by taking into account, all three of the PID gains for each input, the total number of parameters to be optimized are 9. Which is a fairly simple task for the GAO.

### IV.	RESULTS
The GAO for optimizing the FLC parameters has been used for a great number of iterations. The reason why the exact number of iterations is unknown is because each time the algorithm was stuck in a local minimum, the algorithm was stopped to manually check and tune some of the parameters, in order to set the algorithm back on track. The eventual best cost of the system has been 37.8 which is the sum of the absolute value of all errors throughout a 7 seconds simulation with a time step of 0.01.

The GAO for optimizing the PID controller parameters reaches to the optimal values in 156 iterations, taking 17260 times of cost function evaluations. The eventual best cost of the system has been 68.97 which is the sum of the absolute value of all errors throughout the 7 seconds simulation with a time step of 0.01 and also one percent of the sum of all PID gains for all the inputs. This is done in order to take into account the cost of using high gains.

When compared against each other, the cost of FLC is lower and follows the path more smoothly. But the torques generated by the FLC show lots of chattering, which is undesirable. A comparison of the two system behaviors can be seen on Fig. 2, Fig. 3, and Fig. 4, depicting the angles, angular velocities, and the applied torques respectively.


 ![image](https://user-images.githubusercontent.com/65850584/220759359-b3a4c66b-3a7e-4fe7-816e-0ab31ab5ee00.png)
Fig. 2. Trajectory of the three link angles, comparing FLC and PID controller with the reference path


![image](https://user-images.githubusercontent.com/65850584/220759424-27ebbf14-5270-4599-bcbc-1cd34a8e9c18.png)
Fig. 3. Angular speed of the three links, comparing FLC and PID controller with the reference path


 ![image](https://user-images.githubusercontent.com/65850584/220759480-723be06f-9dea-4f05-89ec-2b75d7725b68.png)
Fig. 4. The applied torque to each link, comparing FLC and PID controller outputs


### V.	CONCLUSION
In this paper the performances of two control methods, namely PID control and FLC in controlling the movement of a robot arm in 3D space, and optimizing their parameters using a GAO has been discussed.

The eventual FLC design is not as robust as the designed PID controller. It’s also much more unstable regarding the outputs of the control system. But the cost of using FLC is lower, which means the path tracking is done more smoothly. It is also worth noting that if more membership functions were used for the inputs, and the rules integrated more inputs, the system behavior could have improved drastically. But the curse of dimensionality prevents us from easily optimizing such a system.

In future works, more parameters can be optimized using a GAO and designing a better FLC.

### REFERENCES

[1]	S. Dong, Z. Yuan, and F. Zhang, “A Simplified Method for Dynamic Equation of Robot in Generalized Coordinate System,” J. Phys. Conf. Ser., vol. 1345, p. 042077, Nov. 2019, doi: 10.1088/1742-6596/1345/4/042077.

