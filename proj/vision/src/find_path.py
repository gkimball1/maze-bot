from PIL import Image
import numpy as np
from collections import deque

def binarize(img):
	# imgb = Image.open(img_name).convert('L')
	imgb = img.convert('L')
	np_img = np.array(imgb)
	np_img = ~np_img  # invert B&W
	np_img[np_img > 0] = 1


import matplotlib.pyplot as plt
import cv2
def maze2graph(maze):
	height = len(maze)
	width = len(maze[0]) if height else 0

	# for row in range(height):
	# 	for col in range(width):
	# 		if is_left_edge(maze, row, col, width, height):
	# 			maze[row][col + 1] = 1
	# 		elif is_right_edge(maze, row, col, width, height):
	# 			maze[row][col - 1] = 1
	# 		elif is_top_edge(maze, row, col, width, height):
	# 			maze[row + 1][col] = 1
	# 		elif is_bottom_edge(maze, row, col, width, height):
	# 			maze[row - 1][col] = 1
	# 		elif is_vertical(maze, row, col, width, height):
	# 			maze[row][col - 1] = 1
	# 		elif is_horizontal(maze, row, col, width, height):
	# 			maze[row - 1][col] = 1
	graph = {(i, j): [] for j in range(width) for i in range(height) if not maze[i][j]}
	for row, col in graph.keys():
		if row < height - 1 and not maze[row + 1][col]:
			graph[(row, col)].append(("S", (row + 1, col)))
			graph[(row + 1, col)].append(("N", (row, col)))
		if col < width - 1 and not maze[row][col + 1]:
			graph[(row, col)].append(("E", (row, col + 1)))
			graph[(row, col + 1)].append(("W", (row, col)))

	return graph

def find_goal(maze, width, height):
	for i in range(height):
		for j in range(width):
			if is_right_edge(maze, i, j, width, height):
				if maze[i][j] == 0 and maze[i - 1][j] == 0 and maze[i + 1][j] == 0 and maze[i][j - 1] == 0:
					return (i, j)
			elif is_bottom_edge(maze, i, j, width, height):
				if maze[i][j] == 0 and maze[i][j - 1] == 0 and maze[i][j + 1] == 0 and maze[i - 1][j] == 0:
					return (i, j)
	return "Failed to find goal"

def is_left_edge(maze, i, j, width, height):
	if j == 0 and maze[i][j] == 1:
		return True
	return False
def is_right_edge(maze, i, j, width, height):
	if j == width - 1 and maze[i][j] == 1:
		return True
	return False
def is_top_edge(maze, i, j, width, height):
	if i == 0 and maze[i][j] == 1:
		return True
	return False
def is_bottom_edge(maze, i, j, width, height):
	if i == height - 1 and maze[i][j] == 1:
		return True
	return False

def is_vertical(maze, i, j, width, height):
	if i != height - 1 and j != width - 1 and maze[i][j] == 1 and (maze[i + 1][j] == 1 or maze[i - 1][j] == 1):
		return True
	return False 

def is_horizontal(maze, i, j, width, height):
	if i != height - 1 and j != width - 1 and maze[i][j] == 1 and (maze[i][j - 1] == 1 or maze[i][j + 1] == 1):
		return True
	return False 

def find_path_bfs(maze, start, goal):
	#start, goal = (1, 1), (len(maze) - 2, len(maze[0]) - 2)
	# find_goal(maze, len(maze[0]), len(maze))
	# start, goal = (300, 100), (550, 800)
	# start, goal = (300, 100), (550, 800)
	print(maze[start[0], start[1]], 'start', start)
	print(maze[goal[0], goal[1]], 'goal', goal)
	ref = maze.copy()
	start = (start[0], start[1])
	goal = (goal[0], goal[1])
	queue = deque([("", start)])
	visited = set()
	graph = maze2graph(maze)
	points = []
	while queue:
		path, current = queue.popleft()
		if current == goal:
			cur = list(start)
			ref[current[0], current[1]] = 128
			# maze[cur[0]][cur[1]] = 3
			# maze[goal[0]][goal[1]] = 4
			prev = path[0]
			
			for dir in path:
				# maze[cur[0]][cur[1]] = 2
				if dir == "N":
					cur[0] -= 1
				elif dir == "S":
					cur[0] += 1
				elif dir == "W":
					cur[1] -= 1
				elif dir == "E":
					cur[1] += 1
				# if prev != dir:
				points.append(cur[:])
				prev = dir
			print(points)
			return path, points
		if current in visited:
			continue
		visited.add(current)

		for direction, neighbour in graph[current]:

			queue.append((path + direction, neighbour))
	cv2.imwrite('/home/cc/ee106a/fa19/class/ee106a-adt/ros_workspaces/proj/src/vision/imgs/path.png', ref/np.max(ref) * 255)
	return None, None



maze1 = [
	[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
	[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
	[1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1],
	[1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
	[1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1],
	[1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1],
	[1, 0, 0, 0, 1, 1, 0, 1, 1, 1, 0, 1],
	[1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1],
	[1, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1],
	[1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 1],
	[1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1],
	[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
]
maze2 = [
	[1,1,1,1,1,1,1,1,1,1,1],
	[1,0,0,0,0,0,0,0,0,0,1],
	[1,0,0,0,0,0,0,0,0,0,1],
	[1,0,0,0,0,0,0,0,0,0,1],
	[1,1,1,1,1,1,1,0,0,0,1],
	[1,0,0,0,0,0,0,0,0,0,1],
	[1,0,0,0,0,0,0,0,0,0,1],
	[1,0,0,0,0,0,0,0,0,0,1],
	[1,1,1,1,1,1,1,1,1,1,1]
	]
maze3 = [
	[1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1],
	[1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1],
	[1,0,0,0,0,0,0,0,1,1,0,0,0,0,0,1],
	[1,0,0,0,0,0,0,0,1,1,0,0,0,0,0,1],
	[1,1,1,1,1,0,0,0,1,1,0,0,0,0,0,1],
	[1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,1],
	[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
	[1,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1],
	[1,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1],
	[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
	[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
	[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
	]
# maze1 = [
# 	[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
# 	[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
# 	[1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1],
# 	[1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
# 	[1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1],
# 	[1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1],
# 	[1, 0, 0, 0, 1, 1, 0, 1, 1, 1, 0, 1],
# 	[1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1],
# 	[1, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1],
# 	[1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 1],
# 	[1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1],
# 	[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
# ]
# maze2 = [
# 	[1,1,1,1,1,1,1,1,1,1,1],
# 	[1,0,0,0,0,0,0,0,0,0,1],
# 	[1,0,0,0,0,0,0,0,0,0,1],
# 	[1,0,0,0,0,0,0,0,0,0,1],
# 	[1,1,1,1,1,1,1,0,0,0,1],
# 	[1,0,0,0,0,0,0,0,0,0,1],
# 	[1,0,0,0,0,0,0,0,0,0,1],
# 	[1,0,0,0,0,0,0,0,0,0,1],
# 	[1,1,1,1,1,1,1,1,1,1,1]
# 	]
# maze3 = [
# 	[1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1],
# 	[1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1],
# 	[1,0,0,0,0,0,0,0,1,1,0,0,0,0,0,1],
# 	[1,0,0,0,0,0,0,0,1,1,0,0,0,0,0,1],
# 	[1,1,1,1,1,0,0,0,1,1,0,0,0,0,0,1],
# 	[1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,1],
# 	[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
# 	[1,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1],
# 	[1,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1],
# 	[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
# 	[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
# 	[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
# 	]

# print(find_path_bfs(maze2))

# np.savetxt('matrix.txt',np_img,fmt='%.2f')
# path = [[254, 101], [255, 668], [255, 669], [256, 670], [256, 671], [257, 672], [257, 673], [258, 673], [258, 674], [259, 674], [261, 675], [262, 675], [307, 674], [308, 674], [311, 673], [312, 673], [315, 672], [316, 672], [317, 671], [318, 671], [318, 670], [319, 670], [319, 669], [320, 668], [320, 667], [321, 663], [321, 662], [322, 651], [322, 650], [323, 642], [323, 641], [324, 630], [324, 629], [325, 625], [325, 624], [326, 623], [326, 622], [327, 620], [327, 619], [328, 607], [328, 606], [329, 599], [329, 598], [330, 577], [330, 576], [331, 546], [331, 545], [332, 494], [550, 495]]
def solve(img, start, goal):
	np_img = np.array(img)
	# np_img = ~np_img  # invert B&W
	np_img[np_img > 0] = 1
	path, pointsf = find_path_bfs(np_img, start, goal)
	np_img = np.ones(img.shape)
	if pointsf != None:
		for i in pointsf: 
			np_img[i[0], i[1]]=0
	# image = ref - image
	# Save
	cv2.imwrite('/home/cc/ee106a/fa19/class/ee106a-adt/ros_workspaces/proj/src/vision/imgs/' + 'result.png',np_img /np.max(np_img) * 255)
	return pointsf



# # Make empty black image
# image=cv2.imread('final.png',0)


# # Create a named colour
# red = 128

# ref = image.copy()
# Change one pixel

# for i in pointsf: 
# 	image[i[0], i[1]]=0
# # image = ref - image
# # Save
# cv2.imwrite('result.png',image)
