import cv2
import numpy as np
from matplotlib import pyplot as plt
def binarization(img_name):
	img = cv2.imread(img_name,0)
	img = cv2.medianBlur(img,5)

	ret,th1 = cv2.threshold(img,127,255,cv2.THRESH_BINARY)
	th2 = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_MEAN_C,\
	            cv2.THRESH_BINARY,11,2)
	th3 = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
	            cv2.THRESH_BINARY,11,2)

	sizes = np.shape(img)
	height = float(sizes[0])
	width = float(sizes[1])
	 
	fig = plt.figure()
	fig.set_size_inches(width/height, 1, forward=False)
	ax = plt.Axes(fig, [0., 0., 1., 1.])
	ax.set_axis_off()
	fig.add_axes(ax)
	 
	ax.imshow(th2, 'gray')
	plt.savefig("otsu.jpg", dpi = 300)


