
# The Amazing Ada (Sawyer)
## Overview
Sawyer is a single-arm robot intended for working alongside humans and for use in industrial settings. We used various strategies taught in Berkeley's Introduction to Robotics course to allow Sawyer to analyze a maze drawn on a whiteboard before drawing the solution on the same whiteboard. 
### Goal
Sawyer should precisely and accurately produce and draw a solution to a maze on a whiteboard. 
![image of sawyer and maze](https://i.imgur.com/YO6MQDU.png)
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

## Design
Ultimately, the design of this solution must be able to correctly and cleanly solve a whiteboard maze. It must be able to locate the whiteboard's location & orientation, the starting point, and the ending point, and from there be able to trace its end-effector over the proper path through the maze. 

We chose a whiteboard with a duct-taped maze and an AR tag located directly next to the start position. This AR tag would allow for detection of whiteboard characterisitics. 
We hard coded the size of the whiteboard, and determined a general region of search for the maze start and end positions relative to this AR tag. 
In essence, if there were no walls, we had Sawyer conclude that this was the end position. This results in some tradeoffs in flexibility with regards to whiteboard sizing etc. since now we would be limited to the same sized whiteboard and general location of the end point. 
These tradeoffs could also be fixed with the introduction of another AR tag situated next to the end position, but we felt it would be slightly cleaner to the eye to have only one AR tag. 

In addition to this, we considered the method of control of the end-effector's position relative to the whiteboard/AR tag. We decided it would be nice if we had closed-loop control around Sawyer's arm via head camera, but it continually gave incorrect coordinate transformations. 
Ultimately, we decided that given our large borders and margin for error it would be a better use of our time to have Sawyer move its end effector between points without necessarily controlling for its position relative to the whiteboard. (A future iteration of the maze-bot could include an HD external webcam to monitor Sawyer's progress)

More practically, we also considered the design of the method of ink deposition onto the whiteboard. 
Since Sawyer's precision would be limited by the lack of closed-loop control around the arm's position, we (the marker) would have to be able to physically tolerate a certain margin of error while maintaining its ink-depositing abilities. 
As such, we designed a two-way elastic suspension system that would allow the marker to be displaced a small amount from its equilibrium position, protecting ourselves from compromising neither our drawing capabilities or the structural integrity of the marker itself. 
This gives a small tradeoff of having the marker have a little bit of "lag" as it's dragged across the whiteboard, but for the much greater benefit of stability and simplicity of design.

The result of these design choices results in a rather useful robot for use in certain applications. For instance, for operation on an assembly line, the use of just 1 AR tag on each part means less overhead in AR tag placement per part. 
The durability of the marker is significantly lengthened because of the suspension; the


## Implementation
The implemented steps were:
1. Position Arm
   - The arm has to be positioned at height in order to have the arm camera fully process the maze and its position
2. Take a photo
   - A photo of the maze is taken and saved for further processing
3. Transform
   - A homographic transform is used to resize the image so it can be processed accurately
4. Solve
   - A simple BFS is used to determine the solution to the maze. Since this is a relatively small maze and image we can use BFS efficiently
   - Output a list of the most critical 3d points (corners) in space that the end-effector must pass through to draw out the solution
5. Draw Solution
   - Sawyer iterates through each of the points in the list outputted by the BFS program one-by-one and utilizes the linear path-planning algorithm to draw the final solution. 



# End Actual Work, Start Github Formatting (Nothing below this line will be on the final report)


## Welcome to GitHub Pages

You can use the [editor on GitHub](https://github.com/gkimball1/maze-bot/edit/master/README.md) to maintain and preview the content for your website in Markdown files.

Whenever you commit to this repository, GitHub Pages will run [Jekyll](https://jekyllrb.com/) to rebuild the pages in your site, from the content in your Markdown files.

### Markdown

Markdown is a lightweight and easy-to-use syntax for styling your writing. It includes conventions for

```markdown
Syntax highlighted code block

# Header 1
## Header 2
### Header 3

- Bulleted
- List

1. Numbered
2. List

**Bold** and _Italic_ and `Code` text

[Link](url) and ![Image](src)
```

For more details see [GitHub Flavored Markdown](https://guides.github.com/features/mastering-markdown/).

### Jekyll Themes

Your Pages site will use the layout and styles from the Jekyll theme you have selected in your [repository settings](https://github.com/gkimball1/maze-bot/settings). The name of this theme is saved in the Jekyll `_config.yml` configuration file.

### Support or Contact

Having trouble with Pages? Check out our [documentation](https://help.github.com/categories/github-pages-basics/) or [contact support](https://github.com/contact) and we’ll help you sort it out.
