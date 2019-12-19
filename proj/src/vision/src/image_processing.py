import cv2
import numpy as np
from matplotlib import pyplot as plt
import IPython.display as ipd
import scipy.signal
from scipy import signal, linalg
from scipy.io import wavfile
import math
from find_path import solve


def Hist(img):
    row, col = img.shape 
    y = np.zeros(256)
    for i in range(0,row):
        for j in range(0,col):
            y[img[i,j]] += 1
    x = np.arange(0,256)
    # plt.bar(x, y, color='b', width=5, align='center', alpha=0.25)
    # plt.show()
    return y

def variance(h, s, e):
    v = 0
    m = mean(h, s, e)
    w = wieght(h, s, e)
    for i in range(s, e):
        v += ((i - m) **2) * h[i]
    v /= w
    return v

def wieght(h, s, e):
    w = 0
    for i in range(s, e):
        w += h[i]
    return w


def mean(h, s, e):
    m = 0
    w = wieght(h, s, e)
    for i in range(s, e):
        m += h[i] * i
    
    return m/float(w)

def countPixel(h):
    cnt = 0
    for i in range(0, len(h)):
        if h[i]>0:
            cnt += h[i]
    return cnt

def threshold(h, threshold_values):
    cnt = countPixel(h)
    for i in range(1, len(h)):
        vb = variance(h, 0, i)
        wb = wieght(h, 0, i) / float(cnt)
        mb = mean(h, 0, i)
        
        vf = variance(h, i, len(h))
        wf = wieght(h, i, len(h)) / float(cnt)
        mf = mean(h, i, len(h))
        
        V2w = wb * (vb) + wf * (vf)
        V2b = wb * wf * (mb - mf)**2
        
        # fw = open("trace.txt", "a")
        # fw.write('T='+ str(i) + "\n")

        # fw.write('Wb='+ str(wb) + "\n")
        # fw.write('Mb='+ str(mb) + "\n")
        # fw.write('Vb='+ str(vb) + "\n")
        
        # fw.write('Wf='+ str(wf) + "\n")
        # fw.write('Mf='+ str(mf) + "\n")
        # fw.write('Vf='+ str(vf) + "\n")

        # fw.write('within class variance='+ str(V2w) + "\n")
        # fw.write('between class variance=' + str(V2b) + "\n")
        # fw.write("\n")
        
        if not math.isnan(V2w):
            threshold_values[i] = V2w

def regenerate_img(img, threshold):
    row, col = img.shape 
    y = np.zeros((row, col))
    for i in range(0,row):
        for j in range(0,col):
            if img[i,j] >= threshold:
                y[i,j] = 255
            else:
                y[i,j] = 0
    return y

def binarize(img, offest = 0):
    threshold_values = {}
    h = [1]
    h = Hist(img.astype(int))
    threshold(h, threshold_values)
    op_thres = get_optimal_threshold(threshold_values)
    res = regenerate_img(img, op_thres + offest)
    return res

def get_optimal_threshold(threshold_values):
    min_V2w = min(threshold_values.values())
    optimal_threshold = [k for k, v in threshold_values.items() if v == min_V2w]
    return optimal_threshold[0]

def gkern(size=5, sigma=1.0):
    """
    Returns a gaussian kernel with zero mean.
    
    Adapted from:
    https://stackoverflow.com/questions/29731726/how-to-calculate-a-gaussian-kernel-matrix-efficiently-in-numpy
    
    Parameters:
    size    - Sidelength of gaussian kernel
    sigma   - Value of standard deviation
    """
    ax = np.arange(-size // 2 + 1.0, size // 2 + 1.0)
    xx, yy = np.meshgrid(ax, ax)
    kernel = np.exp(-(xx**2 + yy**2) / (2.0 * sigma**2))
    return kernel / np.sum(kernel)

def rgb2gray(rgb):

    r, g, b = rgb[:,:,0], rgb[:,:,1], rgb[:,:,2]
    gray = 0.2989 * r + 0.5870 * g + 0.1140 * b
    
    gray = gray / np.max(gray) * 255
    
    return gray

def post_porcess(img, whiten_radius = 1, whiten_thres = 3, blacken_radius = 4):
    h, w = img.shape
    ref = img.copy()
    for i in range(h):
        for j in range(w):
            if img[i,j] == 0:
                count = 0
                for i0 in range(max(0, i-whiten_radius), min(h, i + whiten_radius)):
                    for j0 in range(max(0, j-whiten_radius), min(w, j + whiten_radius)):
                        if not img[i0, j0]:
                            count += 1
                if count <= whiten_thres:
                    ref[i,j] = 255
    img = ref.copy()

    for i in range(h):
        for j in range(w):
            if ref[i,j]:
                flag = False
                for i0 in range(max(0, i-blacken_radius), min(h, i + blacken_radius)):
                    if flag:
                        break
                    for j0 in range(max(0, j-blacken_radius), min(w, j + blacken_radius)):
                        if not ref[i0, j0]:
                            flag = True
                            break
                if flag:
                    img[i,j] = 0
    return img


def auto_canny(image, sigma=0.33):
    # compute the median of the single channel pixel intensities
    v = np.median(image)
 
    # apply automatic Canny edge detection using the computed median
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
#     lower = 100
#     upper = 200
    edged = cv2.Canny(image, lower, upper)
 
    # return the edged image
    return edged

def find(img, screen_corner, ideal, start, goal):
    img = rgb2gray(img)


    cv2.imwrite('/home/cc/ee106a/fa19/class/ee106a-adt/ros_workspaces/proj/src/vision/imgs/img.png', img)
    height_0, width_0 = img.shape
    height, width = ideal[3][0], ideal[3][1]

    ref = np.ones(img.shape)*255
    for i, j, in screen_corner[0:1]:
        for i0 in range(-20, 20):
            for j0 in range(-20, 20):
                if height_0 > i+i0 >= 0 and width_0 > j + j0 >= 0:
                    ref[i+i0,j+j0] = 0
    cv2.imwrite('/home/cc/ee106a/fa19/class/ee106a-adt/ros_workspaces/proj/src/vision/imgs/ref1.png', ref)

    ref = np.ones(img.shape)*255
    for i, j, in screen_corner[1:2]:
        for i0 in range(-20, 20):
            for j0 in range(-20, 20):
                if height_0 > i+i0 >= 0 and width_0 > j + j0 >= 0:
                    ref[i+i0,j+j0] = 0
    cv2.imwrite('/home/cc/ee106a/fa19/class/ee106a-adt/ros_workspaces/proj/src/vision/imgs/ref2.png', ref)

    ref = np.ones(img.shape)*255
    for i, j, in screen_corner[2:3]:
        for i0 in range(-20, 20):
            for j0 in range(-20, 20):
                if height_0 > i+i0 >= 0 and width_0 > j + j0 >= 0:
                    ref[i+i0,j+j0] = 0
    cv2.imwrite('/home/cc/ee106a/fa19/class/ee106a-adt/ros_workspaces/proj/src/vision/imgs/ref3.png', ref)

    ref = np.ones(img.shape)*255
    for i, j, in screen_corner[3:4]:
        for i0 in range(-20, 20):
            for j0 in range(-20, 20):
                if height_0 > i+i0 >= 0 and width_0 > j + j0 >= 0:
                    ref[i+i0,j+j0] = 0
    cv2.imwrite('/home/cc/ee106a/fa19/class/ee106a-adt/ros_workspaces/proj/src/vision/imgs/ref4.png', ref)


    def average(im, n, x, y):
        count = 0
        total = 0
        for i in range(n):
            for j in range(n):
                if (height > i+x>=0 and width >y+j>=0):
                    total += im[i+x, y+j]
                    count += 1
        return total/count

    M, mask = cv2.findHomography(screen_corner.reshape(-1, 1, 2), ideal.reshape(-1, 1, 2), method = 0)

    out = np.zeros((height, width))
    ref = np.zeros((height, width))
    for i in range(height_0):
        for j in range(width_0):
            a = np.dot(M,np.array([i,j,1]))
            a /= a[2]
            a = a.astype(int)
            if (height > a[0]>=0 and width >a[1]>=0):
                out[a[0], a[1]] += img[i,j]
                ref[a[0], a[1]] = 255
    cv2.imwrite('/home/cc/ee106a/fa19/class/ee106a-adt/ros_workspaces/proj/src/vision/imgs/out1.png', out)
    out2 = out.copy()
    for i in range(height):
        for j in range(width):
            if not ref[i,j]:
                out2[i,j] = average(out2, 5, i, j)

    cv2.imwrite('/home/cc/ee106a/fa19/class/ee106a-adt/ros_workspaces/proj/src/vision/imgs/out2.png', out2)

    kern = gkern(5, 5)


    blurred = scipy.signal.convolve2d(out2, kern, mode="same")
    blurred = blurred/ np.max(blurred) * 255


    ref = binarize(blurred, 40)
    print(start, goal)
    i,j = start
    for i0 in range(-5, 5):
        for j0 in range(-5, 5):
            if height > i+i0 >= 0 and width > j + j0 >= 0:
                ref[i+i0,j+j0] = 0
    cv2.imwrite('/home/cc/ee106a/fa19/class/ee106a-adt/ros_workspaces/proj/src/vision/imgs/start2.png', ref)

    ref = np.ones(blurred.shape)*255
    print(start, goal)
    i,j = start
    for i0 in range(-5, 5):
        for j0 in range(-5, 5):
            if height > i+i0 >= 0 and width > j + j0 >= 0:
                ref[i+i0,j+j0] = 0
    cv2.imwrite('/home/cc/ee106a/fa19/class/ee106a-adt/ros_workspaces/proj/src/vision/imgs/start.png', ref)

    ref = binarize(blurred, 40)
    i, j = goal
    print(goal)
    for i0 in range(-5, 5):
        for j0 in range(-5, 5):
            if height > i+i0 >= 0 and width > j + j0 >= 0:
                ref[i+i0,j+j0] = 0
    cv2.imwrite('/home/cc/ee106a/fa19/class/ee106a-adt/ros_workspaces/proj/src/vision/imgs/goal2.png', ref)

    ref = np.ones(blurred.shape)*255
    i, j = goal
    print(goal)
    for i0 in range(-5, 5):
        for j0 in range(-5, 5):
            if height > i+i0 >= 0 and width > j + j0 >= 0:
                ref[i+i0,j+j0] = 0
    cv2.imwrite('/home/cc/ee106a/fa19/class/ee106a-adt/ros_workspaces/proj/src/vision/imgs/goal.png', ref)

    j = 0
    path = None
    for i in range(60, -20, -5):
        binary = binarize(blurred, i)
        # cv2.imwrite(str(j)+'binary_' + str(i) +'.png', binary)
        binary = post_porcess(binary)
        # i,j = start
        # for i0 in range(-5, 5):
        #     for j0 in range(-5, 5):
        #         if height > i+i0 >= 0 and width > j + j0 >= 0:
        #             binary[i+i0,j+j0] = 0
        # i,j = goal
        # for i0 in range(-5, 5):
        #     for j0 in range(-5, 5):
        #         if height > i+i0 >= 0 and width > j + j0 >= 0:
        #             binary[i+i0,j+j0] = 0


        j += 1
        cv2.imwrite('/home/cc/ee106a/fa19/class/ee106a-adt/ros_workspaces/proj/src/vision/imgs/' + str(j)+'binary_post' + str(i) +'.png', binary)
        binary = binary <= 0
        flag = False
        for i in range(-5,5):
            for j in range(-5,5):
                if not binary[start[0] + i][start[1] + j]:
                    start0 = np.array([start[0] + i, start[1] + j])
                    flag = True
        if not flag:
            print('start not found')
            continue
        flag = False  
        for i in range(-10, 10):
            for j in range(-10,10):
                if not binary[goal[0] + i][goal[1] + j]:
                    goal0 = np.array([goal[0] + i, goal[1] + j])
                    flag = True
        if not flag:
            print('goal not found')
            continue
                        
        print(start0, goal0)       
        path = solve(binary, start0, goal0)
        if path:
            return path




    # blurred = scipy.signal.convolve2d(out2, kern, mode="same")
    # blurred = blurred/ np.max(blurred) * 255
    # binary = binarize(blurred)
    # cv2.imwrite('binary_2.png', binary)
    # paths.append(find_path_bfs(binary))

    # binary2 = post_porcess(binary)
    # cv2.imwrite('binary_2_post.png', binary2)
    # paths.append(find_path_bfs(binary2))

    # binary = post_porcess(binary, blacken_radius = 2)
    # cv2.imwrite('binary_2_post_2.png', binary)
    # paths.append(find_path_bfs(binary))


    # blurred = scipy.signal.convolve2d(out2, kern, mode="same")
    # blurred = blurred/ np.max(blurred) * 255
    # binary = binarize(blurred)
    # cv2.imwrite('binary_3.png', binary)
    # paths.append(find_path_bfs(binary))

    # binary2 = post_porcess(binary)
    # cv2.imwrite('binary_3_post.png', binary2)
    # paths.append(find_path_bfs(binary2))

    # binary = post_porcess(binary, blacken_radius = 2)
    # cv2.imwrite('binary_3_post_2.png', binary)
    # paths.append(find_path_bfs(binary))

    # blurred = scipy.signal.convolve2d(out2, kern, mode="same")
    # blurred = blurred/ np.max(blurred) * 255
    # binary = binarize(blurred)
    # cv2.imwrite('binary_4.png', binary)
    # paths.append(find_path_bfs(binary))

    # binary2 = post_porcess(binary)
    # cv2.imwrite('binary_4_post.png', binary2)
    # paths.append(find_path_bfs(binary2))

    # binary = post_porcess(binary, blacken_radius = 2)
    # cv2.imwrite('binary_4_post_2.png', binary)
    # paths.append(find_path_bfs(binary))



# img = cv2.imread('Screenshot at 2019-12-10 11_18_43.png')
# screen_corner = np.array([[610, 650], [800, 600], [600, 950], [750, 1000]])
# ideal = np.array([[0,0], [100,0],[0,140], [100,140]])
# find(img, screen_corner, ideal)