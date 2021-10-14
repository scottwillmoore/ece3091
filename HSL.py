import cv2
import matplotlib.pyplot as plt
import numpy as np

img = cv2.imread('image.png')
size = img.shape

plt.imshow(img)
# blue box, green box, red box, dfrobot box, camera box, weetbix box, chessbox
points_2D = np.array([(336.9,390.6), (335,251), (335,176.5), (160.6,249.1), (516.1,251.9), (475.6,173.7), (336,125.6), (222.8,124.6)], dtype="double")
xVec = []
yVec = []
for i in range(0,8):
    xVec.append(points_2D[i][0])
    yVec.append(points_2D[i][1])
print(xVec)
print(yVec)
plt.plot(xVec,yVec,'*',markersize=2,color='r')
plt.show()

points_3D = np.array([(30,0,25), (40,0,15), (50, 0, 25), (40,-10,61), (40,10,120),(50,10,107),(60,0,87),(60,-10,36)],dtype="double")

dist_coeffs = np.zeros((4,1))

focal_length = size[1]
center = (size[1]/2, size[0]/2)
camera_matrix = np.array([[focal_length, 0, center[0]], [0, focal_length, center[1]], [0, 0, 1]], dtype="double")

(success, rotation_vector,translation_vector) = cv2.solvePnP(points_3D,points_2D,camera_matrix,dist_coeffs,flags=0)
print(rotation_vector)
print(translation_vector)