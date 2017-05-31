## Project: Search and Sample Return
## Author:  John Novak
## Date: 5/31/17

---


**The goals / steps of this project are :**  

**Training / Calibration**  

* Download the simulator and take data in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping**

* Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook).
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands.
* Iterate on your perception and decision function until your rover does a reasonable (need to define metric) job of navigating and mapping.  

[//]: # (Image References)

[image1]: ./misc/rover_image.jpg
[image2]: ./calibration_images/example_grid1.jpg
[image3]: ./src/test_rock1.png
[image4]: ./src/test_map.png

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

This file

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.
The thresholding function was modified to perform a range check rather than a simple on sided test.  Default values are included to allow existing code to run unmodified.  The rock test is straightforward and simply a matter of selecting an appropriate range of values.  One improvement is adding a fill function to remove holes in the detected objects.

Obstacles are modeled as a simple inversion of the navigable terrain bitmap.  


#### 2. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result.

When building the worldmap, it became clear a simple addition of values correlated with the various classes of objects was causing errors in the map due to RGB channel overflow (as values are maxed at 255 per channel).  This issue is addressed by clipping each channel to 0-255 following the update steps.

The update steps are also modified to add weighting and a variable amount to most of the updates.  This allows faster convergence early in mapping runs.  An improvement would be to model free/occupied space using a single map grid and log-odds updates.

An interesting dynamic of mapping is the interaction between occupied and free space map cells.  Using equal weighting of the free and occupied channels leads to a composite map with much navigable space marked as occupied.  To deal with this, the occupied channel is reset to 1 each time a map cell is observed as navigable, and the update rate for the occupied channel is quite low.

As can be seen in the video, the results are quite accurate and yield a pleasing map display.

![alt text][image4]

### Autonomous Navigation and Mapping

#### 3. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.

perception_step is modified to extract interesting data from the rover object into local variables to make the remainder of the function more readable.

The code adjusts roll and pitch so that the range is plus or minus a small degree range rather than then zero to 360 input range.

The current camera image is then thresholded for navigable and rock features and, if the roll and pitch are near zero, the worldmap channels are updated.

For the rock channel, the worldmap is updated only if rock feature pixels are present in the current camera image.

decision_step has one modification; when the rover heading is computed a small random factor is added in an attempt to break path loops.  This modification seems to be largely ineffecive.

#### 4. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

Simulator Resolution: 1920x1440
Simulator Graphics : Good
Simulator Frame Rate: 13-15

Results are surprisingly good; for most runs the minimum performance requirements are met, and many times surpassed.  

For mapping, the simulator reports good accuracy on all runs.

Of note is the rover does get into a navigation loop in the wide are at the right of the worldmap.  I experimented with adding small heading perturbations to the selected path, which is sometimes effective.  It is clear that a different method is needed.

One possibility is to ray trace through the worldmap based upon rover heading and bias heading changes to favor unmapped areas of the world first, then favor map areas with long stretches of navigable space, a sort of over the horizon approach for mapped sections of the world.

A third significant improvement is adding simple path finding on the world grid to direct the rover to rocks are they are identified.  This could be implemented as a Dijkstra or A* to guide the rover when a rock is detected.

Finally, adding a better motion controller using PD or EKF methods would likely improve the "drunken sailor" pathing of the rover.  After a time, it's rather nauseating as the rover moves from side to side following the changing means in the camera images.
