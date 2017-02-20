# SLAM-from-scratch
This repository contains a complete SLAM implementation in python for a lego robot outfitted with a 
LIDAR scanner. This class was given by [Claus Brenner](https://www.youtube.com/watch?v=JFtZnMu6PDQ&list=PLpUPoM7Rgzi_7YWn14Va2FODh7LzADBSm&index=77)
* **Unit A** we take encoder ticks from the robot and transform them inot a trajectory. We also use scan data to get 
cylinder locations in the world
* **Unit B** we use Feature based localization and Featureless Localization to imporve our robots pose estime
    * Feature Based Localization
        * Assignment of landmarks
        * Direct solution of the similarity transform
        * Correction of the pose using the transform
    * Featureless Localization
        * Assign scan points to the walls
        * Iterative Closest Point Algorithm to find the optimal transformation between scan points and the wall
* **Unit C**
    * Implemented a Bayes filter
    * Derive a 1-D kalman filter
* **Unit D**
    * Multi-dimensional kalman filter
    * Extended Kalman Filter Implemented
    
* **Unit E**
    * Particle Filter Implented
* **Unit F**
    * EKF SLAM Implemented
    
* **Unit G**
    * Particle Filter SLAM
    
    
####Packages to install 
* ```sudo apt-get install python-imaging-tk```
* ```sudo apt-get install python-pil.imagetk```