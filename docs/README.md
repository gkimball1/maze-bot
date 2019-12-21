## Overview
Sawyer is a single-arm robot intended for working alongside humans and for use in industrial settings. We used various strategies taught in Berkeley's Introduction to Robotics course to allow Sawyer to analyze a maze drawn on a whiteboard before drawing the solution on the same whiteboard. 

### Goal
Sawyer should precisely and accurately produce and draw a solution to a maze on a whiteboard. 

![Sawyer with the maze to solve](https://i.imgur.com/USE4Rc1.png)

### Tasks to be Completed
Before Sawyer is able to complete the main task at hand, it must first be able solve a series of smaller tasks: 
1. Vision Processing
   - Sawyer must be able to translate a real-world image into a digitized data structure that is operable on by maze-solving algorithms
   - Sawyer must also be able to identify the start and end locations
2. Solving the Maze
   - Provided with a digitized image, Sawyer must compute the optimal path to move from the start position to the end position
3. Translation to Real World
   - Sawyer must be able to use the solved maze path to determine a list of 3D locations in the real world that its end gripper (to which the marker is attached) must move through to draw the desired solution. 
4. Motion & Final Solution
   - Sawyer must use various control laws and path planning to move through the proper locations and ultimately deposit the correct ink path solving the maze onto the whiteboard. 

### Applications
There are several applications of this that could be useful in industrial or household settings. Many deal with "soft" obstacles that aren't necessarily life-ending if the robot runs into them, but still present some suboptimality in the task at hand. 
- Having Sawyer recognize >= 2 containers and dispense something into them--say, dumping peanuts into some peanut jars in a peanut processing facility. 
  - Sawyer will begin at one peanut jar (the "start" point), dispense, move to another peanut jar (the end point), and dispense again. It is sub-optimal if is Sawyer is dispensing peanuts during the whole path between the first jar and the second jar. 
- Welding two plates of metal together. 
  - If Sawyer has a thermal arc torch attached to its hand, Sawyer can recognize the path of metal that needs to be welded together and use its maze-solving skills to trace the path with its end-effector and fuse the metal into one piece. 
  - Similarly, this can be used for laser-etching a 3d object...
  - ...or precisely cutting a piece of wood
- Moving packages around in a giant warehouse.
  - If Sawyer were a few orders of magnitude larger, it would be able to pick up a package, find a path through the warehouse that wouldn't knock over the preexisting stacks of apparel, and deposit the package in a place the package should be deposited in. @Amazon--we're open for hire. 

## **Design**
Ultimately, the design of this solution must be able to correctly and cleanly solve a whiteboard maze. It must be able to locate the whiteboard's location & orientation, the starting point, and the ending point, and from there be able to trace its end-effector over the proper path through the maze. 

We chose a whiteboard with a duct-taped maze and an AR tag located directly next to the start position. This AR tag would allow for detection of whiteboard characterisitics. 
We hard coded the size of the whiteboard, and determined a general region of search for the maze start and end positions relative to this AR tag. 
In essence, if there were no walls, we had Sawyer conclude that this was the end position. This results in some tradeoffs in flexibility with regards to whiteboard sizing etc. since now we would be limited to the same sized whiteboard and general location of the end point. 
These tradeoffs could also be fixed with the introduction of another AR tag situated next to the end position, but we felt it would be slightly cleaner to the eye to have only one AR tag. 

![maze with path](https://i.imgur.com/Ofz1Kbe.png)

In addition to this, we considered the method of control of the end-effector's position relative to the whiteboard/AR tag. We decided it would be nice if we had closed-loop control around Sawyer's arm via head camera, but it continually gave incorrect coordinate transformations. 
Ultimately, we decided that given our large borders and margin for error it would be a better use of our time to have Sawyer move its end effector between points without necessarily controlling for its position relative to the whiteboard. (A future iteration of the maze-bot could include an HD external webcam to monitor Sawyer's progress)

More practically, we also considered the design of the method of ink deposition onto the whiteboard. 
Since Sawyer's precision would be limited by the lack of closed-loop control around the arm's position, we (the marker) would have to be able to physically tolerate a certain margin of error while maintaining its ink-depositing abilities. 
As such, we used an elastic suspension system that would allow the marker to be displaced a small amount from its equilibrium position, protecting ourselves from compromising neither our drawing capabilities or the structural integrity of the marker itself. 
This gives a small tradeoff of having the marker have a little bit of "lag" as it's dragged across the whiteboard, but for the much greater benefit of stability and simplicity of design.

![marker with suspension](https://i.imgur.com/fMEyy1n.png)

The result of these design choices results in a rather useful robot for use in certain applications. For instance, for operation on an assembly line, the use of just 1 AR tag on each part means less overhead in AR tag placement per part. 
The durability of the marker is significantly lengthened because of the suspension; the elasticity lends some error forgiveness as the robot draws over the whiteboard. 


## Implementation
The implemented steps were:
1. Position Arm
   - The arm has to be positioned at height in order to have the arm camera fully process the maze and its position
   
   ![arm position](https://i.imgur.com/n0s2FyI.png)
2. Take a photo
   - A photo of the maze is taken and saved for further processing
3. Transform
   - A homographic transform is used to resize the image so it can be processed accurately
   - A couple strategies are used to process this image
     - First, we begin with a high binarization threshold and gradually lower the threshold until BFS is able to find a solution. This ensures different lighting conditions do not adversely affect Sawyer's performance. (see below image)
     - Next, we pad the walls with extra pixels to mitigate the shortest path's tendency to hug the edge of a wall
     
     ![processed images](https://i.imgur.com/7nrK6wp.png)
4. Solve
   - A simple BFS is used to determine the solution to the maze. Since this is a relatively small maze/image, there's a minimal perfomance difference between using BFS versus an asymptotically faster algorithm like A-Star. 
   - Transform a list of 2d critical points into 3d coordinates
     - Transform from the 2d image to 3d AR Tag coordinates, and from there to Sawyer's base frame coordinates
     - Output a list of the most critical 3d points (corners) in space that the end-effector must pass through to draw out the solution
5. Draw Solution
   - Sawyer iterates through each of the points in the list outputted by the BFS program one-by-one and utilizes the linear path-planning algorithm to draw the final solution. 
     - We used the [descartes](http://wiki.ros.org/descartes) package to give us straight line paths between our critical points

A more detailed flowchart of the steps is below.

**Key:**

- Yellow: hardcoded
- Red: sensing/computer vision
- Green: planning
- Purple: control

![task flowchart](https://i.imgur.com/IGEqm7v.png)


## Results
Our project worked well and was relatively robust to most external factors, with the one exception of lighting. Given the poor image quality (resolution, exposure, etc) of Sawyer's wrist camera, there were plenty of issues--many of which were eventually resolved--associated with finding a path through the maze. Ultimately, our robot was able to find and execute solutions through multiple different mazes and a certain range of whiteboard orientations (Sawyer sometimes has trouble locating the AR tag when the whiteboard was rotated 180 degrees from the default orientation). 

The robot could take an image of a maze on a whiteboard, locate the AR tag, process the image, and draw the correct solution in a total timespan of about 30 seconds. 

**Video:** [link to robot doing work](https://www.youtube.com/watch?v=SH7D741mQQQ&feature=youtu.be)

## Conclusion
Our finished solution satisfied our design criteria fairly well. Our robot was able to locate and solve a whiteboard maze using only one AR tag as a location/orientation marker. 

**Hacks:**
- Somewhat janky marker setup 
- Taped black tape over duct tape since it'd be easier for the camera to see
- Hardcoded general positions of the start & end points

**Flaws/improvements:**

Since we're doing a pixel-level search the result of our algorithm can have Sawyer move in different directions within a pixel's distance, resulting in some jittery behavior during the solution-drawing process. One way we could improve this is some sort of smoothing filter run in post-processing over the list of points that our path-finding algorithm spits out. This could take the form of some sort of clustering or weighted moving average filter that reduces groupings of close-together points into a single checkpoint that Sawyer moves through. 

It would also be nice to have dynamic whiteboard sizing. Potential solutions could either use one AR tag at each corner, or use a very nice very robust edge-detection algorithm on the whiteboard perimeter in conjuction with an AR tag at one corner.

A nicer camera and/or better image processing ability would also be useful and greatly improve the robustness of our current digitized maze-generator/solver. 


## Team
#### Adrian Liu
Hello! I'm Adrian, a third-year CS major
- Image processing
  - Transformation of raw data to binarized image
  - Iterative binarization process

#### Bolun Du
Greetings! I'm Bolun, a third-year EECS major
- Path planning
- Control of Sawyer's arm
- Translation between physical coordinates to actualized motion

#### Galen Kimball
Hi! I'm Galen, a second-year EECS major
- High level overview of robot tasks and solution approach
  - General design and implementation details such as the wall-padding strat
- Website & Media

#### Mengti Sun
Hello! I'm Mengti, a fourth-year Cognitive Science major
- Maze Solver
- Pathfinding and BFS
- Transform between pixel location to AR Tag-relative coordinates

#### Ning Pang
Hi! I'm Ning, a fourth-year EECS major\
- Final solutions phase
  - Actuation & execution of the arm
- Research of packages
- Recording & Media



## Additional Materials
Code for the robot can be found in this [repository](https://github.com/gkimball1/maze-bot) under the proj folder. 

