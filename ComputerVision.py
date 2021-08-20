import numpy as np
import cv2
from matplotlib import pyplot as plt
import torch
import torch.nn as nn
import csv
import os


# need to download torch from pytorch website
# pip install cv2


from IPython import display
#vid_1_images
# plt.show() the image
# hover cursor over centre of ball bearing, look at numbers on bottom of screen
# write numbers down

# example for above

# import this as a csv file, read function
N = 5
Y = np.random.randint(5,59,size=(N,2)) # block positions
print(Y)
print(type(Y))
ballBearingPositions = []
ballBearingPositions.append([350,387]) #1
ballBearingPositions.append([447,406]) #2
ballBearingPositions.append([437,364]) #3
ballBearingPositions.append([435,306]) #4
ballBearingPositions.append([29,514]) #5
ballArray = np.array(ballBearingPositions)
ballArray.reshape((5,2))
print(ballArray)


## Generate some data
image1 = cv2.imread('vid 1/im_0427.png')
image2 = cv2.imread('vid 1/im_0462.png')
image3 = cv2.imread('vid 1/im_0533.png')
image4 = cv2.imread('vid 1/im_0572.png')
image5 = cv2.imread('vid 1/im_0791.png')

#showing the image
plt.imshow(image1)
plt.plot(350,387,'bo')
plt.show
plt.imshow(image2)
plt.plot(447,406,'bo')
plt.imshow(image3)
plt.plot(437,364,'bo')
plt.imshow(image4)
plt.plot(435,306,'bo')
plt.imshow(image5)
plt.plot(29,514,'bo')



imageSetTrain = []
# Load in the images automatically
"""for filepath in os.listdir('vid 1/'):
    imageSetTrain.append(cv2.imread('vid 1/{0}'.format(filepath),1))
"""

# manual testing
imageSetTrain.append(image1)
imageSetTrain.append(image2)
imageSetTrain.append(image3)
imageSetTrain.append(image4)
imageSetTrain.append(image5)

imagesTrain = np.array(imageSetTrain)
Xtrain = imagesTrain # i have left as two variables to reduce confusion even if adds redundancy

imageSetTest = []
# Load in the images
"""for filepath in os.listdir('vid 2/'):
    imageSetTest.append(cv2.imread('vid 2/{0}'.format(filepath),1))
"""
imageSetTest.append(image1)
imageSetTest.append(image2)
imageSetTest.append(image3)
imageSetTest.append(image4)
imageSetTest.append(image5)

imagesTest = np.array(imageSetTest)

Xtrain = imagesTrain
Xtest = imagesTest

Ytrain = ballArray

# get list of files.
# for filename
# glob

# we have made no changes to michael's code from this point

# Define a simple CNN
class Flatten(nn.Module):
    def forward(self, input):
        return input.view(input.size(0), -1)

# The Detector class will be our detection model
class Detector(nn.Module):

    def __init__(self, output_dim, image_channels):
        super().__init__()

        self.encoder = nn.Sequential(
            nn.Conv2d(image_channels, 3, 3, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(3, 3, 3, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(3, 3, 3, stride=2, padding=1),
            nn.ReLU(),
            Flatten(),
            nn.Linear(192, 128),
            nn.ReLU(),
            nn.Linear(128, output_dim),
            nn.Tanh()
        )

    def forward(self, x):
        positions = self.encoder(x)

        return positions

network = Detector(output_dim=2, image_channels=3)

Nepochs = 100
Nbatch = 8

optimizer = torch.optim.Adam(network.parameters(), lr=1e-4)

losses = []
for j in range(Nepochs):

    # Shuffle our training data after each epoch
    idxs = np.arange(Xtrain.shape[0])
    np.random.shuffle(idxs)

    Xtrain = Xtrain[idxs, :, :, :]
    Ytrain = Ytrain[idxs, :]

    # Loop over training data in batches of images
    batch_losses = []
    for k in range(int(Xtrain.shape[0] / Nbatch)):
        # Scale images and positions to be between 0 and 1 and convert to tensors for pytorch
        Xbatch = torch.from_numpy(Xtrain[Nbatch * k:Nbatch * (k + 1), :, :, :]).float().transpose(1, 3) / 255
        Ybatch = torch.from_numpy(Ytrain[Nbatch * k:Nbatch * (k + 1), :]).float() / 64 - 0.5

        # Predict positions using neural network
        Ypred = network(Xbatch)

        # Calulate the loss (error between predictions and true values)
        loss = torch.sum((Ypred - Ybatch) ** 2)

        # Zero existing gradients
        optimizer.zero_grad()

        # Adjust neural network weights and biases by taken a step in a direction that reduces the loss
        loss.backward()
        optimizer.step()

        batch_losses.append(loss.item())

    losses.append(np.mean(batch_losses))

    plt.clf()
    plt.plot(losses)
    plt.ylabel('Loss (mean squared error)')
    plt.xlabel('Epochs')
    plt.grid()

    display.clear_output(wait=True)
    display.display(plt.gcf())

plt.show()

# testing the model - code not yet tested
k = np.random.randint(Xtest.shape[0] / Nbatch)
test_ims = torch.from_numpy(Xtest[Nbatch * k:Nbatch * (k + 1), :, :, :]).float().transpose(1, 3) / 255
Ypred = network(test_ims).detach().numpy()

plt.figure(figsize=(15, 5))
for j in range(Nbatch):
    plt.subplot(1, Nbatch, j + 1)
    plt.imshow(Xtest[Nbatch * k + j, :, :, :].squeeze(), extent=[0, 64, 64, 0])

    pos_pred = (Ypred + 0.5) * 64  # Scale predictions to pixel coordinates

    # Show predicted marker positions in image
    plt.plot([pos_pred[j, 0]], [pos_pred[j, 1]], 'bo', markersize=5)
plt.show()