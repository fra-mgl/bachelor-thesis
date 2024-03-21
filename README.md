# Bachelor’s degree final project
Thesis work for the Bachelor's degree in Computer, Communication and Electronic Engineering @ UniTrento

## Description
This work is focused on the study of tracked robots. It investigates the kinematic model of such vehicles and the implementation of path-following feedback controllers.
It is divided into four sections, briefly summarized below:
- In the first section, the infrastructure used during the experiment is discussed.
- The second section was entirely devoted to the study of the literature in the area of mobile robots; both the kinematic model of the differential drive unicycle and the model of tracked robots were investigated in depth.
- The third section focused on the study and implementation of two different controllers, both of which are part of the state of the art and widely recognized and used. The first controller used is the Stanley Controller . Its simplicity made it the ideal candidate to be used during the validation tests of the general setup. Then, a Lyapunov-based trajectory controller was implemented. The ROS2 Humble framework was used and a dedicated ROS node was developed to interface with the node provided by the vehicle manufacturer and the laboratory instrumentation.
- In the last section, the results of the tests are presented. These two controllers resulted to be effective in path- following tasks, showing good performances when neglecting the tracks’ dynamics (at least in the environments where they were tested). Then, during the second test, the effects of the dynamic interaction between tracks and the terrain are observed by measuring the slip factor under different conditions.
