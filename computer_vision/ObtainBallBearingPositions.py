import numpy as np
import cv2
from matplotlib import pyplot as plt
import torch
import torch.nn as nn
import csv
import os

from IPython import display
#vid_1_images
# plt.show() the image
# hover cursor over centre of ball bearing, look at numbers on bottom of screen
# write numbers down
WIDTH = 512
HEIGHT = 384

count = 0
for filepath in os.listdir('vid 1/'):
    if count >= 926:
        image = cv2.imread('vid 1/{0}'.format(filepath),1)
        imageResized = cv2.resize(image,(WIDTH,HEIGHT))
        plt.imshow(imageResized)
        plt.show()
    count = count + 1

# No ball bearing
# 221-308 inclusive
# 341 - 368 inclusive
