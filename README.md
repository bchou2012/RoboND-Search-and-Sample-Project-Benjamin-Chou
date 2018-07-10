Udacity Search and Sample Return Project Writeup
=

The goal of the Search and Sample Return project is to process and filter incoming images from a rover camera and translate that into maps for obstacle avoidance, navigation, and object retrieval. 

## Training and Calibration 

**Perspective Transform**

When a rover camera displays its field of view, that view must first be transformed into an approximate top-down 2D interpretation. Using dimensions taken from a grid square projected onto the rover camera, we warp the camera image to match the profile of the projected grid as a normal 1x1 grid:  

*Camera Image with Grid*

<img src="https://github.com/bchou2012/RoboND-Search-and-Sample-Project-Benjamin-Chou/blob/master/calibration_images/example_grid1.jpg?raw=true" alt="Camera Image with Grid" width="320px">

*Camera Image Transformed*

<img src="https://github.com/bchou2012/RoboND-Search-and-Sample-Project-Benjamin-Chou/blob/master/writeup_media/warped_example.jpg?raw=true" alt="Warped Camera Image with Grid" width="320px">

*Camera Image with Rock*

<img src="https://github.com/bchou2012/RoboND-Search-and-Sample-Project-Benjamin-Chou/blob/master/calibration_images/example_rock1.jpg?raw=true" alt="Camera Image with Rock" width="320px">

*Camera Image with Rock Transformed*

<img src="https://github.com/bchou2012/RoboND-Search-and-Sample-Project-Benjamin-Chou/blob/master/writeup_media/Rock_Warped.jpg?raw=true" alt="Camera Image with Rock Warped" width="320px">

The `OpenCV` library's `getPerspectiveTransform()` is used to define the transformation's matrix and `warpPerspective()` to perform the transformation. We use the same method to create the mask for obstacles, which we will go into further detail in the next section. 

**Color Thresholding**

The next step is to define three sets of data from the transformed image: the navigable terrain, the sample rocks for pickup, and the obstacle map. 

First, we define RGB color thresholds for navigable terrain and the rocks. For the purposes of this project, navigable terrain appears to have a distinct color difference from rocks and obstacles, so we use the RGB threshold of `160,160,160` discussed in class material. A function is written that creates a binary image of booleans where the light colored ground, which lies above the RGB threshold returns as true. 

*Filtered for Navigable Terrain*

<img src="https://github.com/bchou2012/RoboND-Search-and-Sample-Project-Benjamin-Chou/blob/master/writeup_media/Navigable_Terrain.jpg?raw=true" alt="Navigable Terrain Thresholding" width="320px">

The same technique is used to create a second function that filters for the sample rocks, which are gold/yellow. While calibrating for the rocks using an online RGB color picker was attempted, the results were unsatisfactory. A search on the Slack channel discussions came up with `110,110,50` as the baseline for the rock filter. 

*Filtered for Rock Images*

<img src="https://github.com/bchou2012/RoboND-Search-and-Sample-Project-Benjamin-Chou/blob/master/writeup_media/Rock_Location.jpg?raw=true" alt="Rock Thresholding" width="320px">

For the obstacle map, the navigable terrain binary image is subtracted from a mask representing the whole rover field of view, created during the perspective transform step. 

*Obstacle Map*

<img src="https://github.com/bchou2012/RoboND-Search-and-Sample-Project-Benjamin-Chou/blob/master/writeup_media/Obstacle_Map.jpg?raw=true" alt="Obstacle Map" width="320px">

**Coordinate Transformations**

The transformed and filtered images are next converted to work with rover coordinates; the image coordinates are defined as your standard X-Y grid, while for the rover the X-axis is "forward" or up, the Y-axis is left-right, and the origin of the rover is at the bottom-center of the camera images. 

The next step is to then convert the rover-centric images to be defined within the frame of reference of the simulation world. In addition to the the translation and rotation to world coordinates, a scaling will be applied to match the world scale which has a different resolution.  

The project notebook and code provides several functions, which will be described in brief as the process is outlined. 

The image pixel coordinates are converted to reflect their position with respect to the rover coordinates, with y-axis coordinates now rover x-axis coordinates and x-axis coordinates now rover y-axis coordinates and shifted so that the center of the x-axis is now the rover y-axis origin. 

To convert from rover to world coordinates, the world position and rover yaw angle are provided by the simulator. Due to coordinate transform order of operations the rotation is executed first followed by the translation. 

The rover-centric image data is multiplied by a 2D rotation matrix (in the code the matrix is broken out into two equations), then the data is scaled to the world scale and translated by the rover's world position. 

*Original Image, Warped Image, Filtered Image, Rover Coordinate Conversion Image*

<img src="https://github.com/bchou2012/RoboND-Search-and-Sample-Project-Benjamin-Chou/blob/master/writeup_media/Transform.png?raw=true" alt="Transformation Steps" width="480px">

**Image Processing**

Finally, the three image data sets are returned to the simulation for vision recognition and decision making. Each data set is assigned one of the RGB channels; R for navigable terrain, G for rocks, and B for the obstacle map.

The worldmap is then updated on its three RGB channels to represent the processed images. 

**Test Output**

[Processed Image Video (link downloads)](https://github.com/bchou2012/RoboND-Search-and-Sample-Project-Benjamin-Chou/blob/master/writeup_media/test_mapping.mp4)
  
## Decision and Perception 

When implementing the image processing functions in `perception.py`, in addition to the worldmap the vision map must also be updated. This is similar to the worldmap update, except the channel update values are scaled down to not overwhelm the vision map's output. 

The `nav_dist` and `nav_angle` class variables also need to be updated to provide possible navigation paths. As these variables are polar coordinates, the navigable map is passed through a function that converts navigable X-Y points into a distance and angle. 

In the interest of time, the optional functionality to pick up rocks was not implemented. See future improvements for more discussion.

## Code Implementation 

For a complete breakdown see comments in `/code/perception.py` , `/code/Rover_Project_Test_Notebook.ipynb`, and `/code/decision.py`.

In `decision.py` the `median` of the angle pool was taken instead of the `mean`, which during testing produced slightly better results. 

## Results

[Final Project Video (link downloads)](https://github.com/bchou2012/RoboND-Search-and-Sample-Project-Benjamin-Chou/blob/master/writeup_media/rover_project.mp4)

For the final results, we achieved over 70% of the simulation world mapped with an over 70% fidelity, with 4 rocks located. 

Past runs were inconsistent, with the rover often getting stuck in circles, or running into obstacles. An observed result was that if the rover ran over shallow rocks, the camera pitched up or down, which negatively impacted the fidelity due to the camera pitch capturing views from angles that did not correspond to the ground truth.

## Future Improvements

There are multiple areas to further refine the project

**Rock Retrieval**

Logic could be implemented to use the `near_sample` flag to set movement towards a rock, with the rock position converted to polar coordinates and used instead of `nav_angles` and `nav_dists`. In `decision.py` a step could be added if `near_sample` was set to true the rover moved towards the sample then stopping, then sending a `picking_up`flag. 

**Steering and Navigation**

Robustness in the steering could be added, with weighting towards navigable terrain that was further out and had a direct line of sight, as opposed towards navigable terrain further out blocked by the obstacle map. `mean` and `median` on their own do not produce sufficient results. 

For navigation, storing sections of regions that the rover already explored and directing the rover towards unexplored regions, perhaps with an array of grids that have flags set to `TRUE` that the rover has already explored and prioritizing `nav_dists` and `nav_angles` towards unexplored regions. 

For a stuck rover or one constantly turning in a circle, checks could be implemented based on its location and yaws, such as backing up or steering in an opposite direction. 