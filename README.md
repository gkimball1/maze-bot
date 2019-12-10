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


## Overview
Sawyer is a single-arm robot intended for working alongside humans and for use in industrial settings. We used various strategies taught in Berkeley's Introduction to Robotics course to allow Sawyer to analyze a maze drawn on a whiteboard before drawing the solution on the same whiteboard. 
### Goal
Sawyer should precisely and accurately produce and draw a solution to a maze on a whiteboard. 
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
There are several applications of this that could be useful in industrial or household settings. 
