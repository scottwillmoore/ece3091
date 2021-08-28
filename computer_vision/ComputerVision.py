import numpy as np
import cv2
from matplotlib import pyplot as plt
import torch
import torch.nn as nn
import csv
import os


# No ball bearing
# 221-308 inclusive
# 341 - 368 inclusive

WIDTH = 512
HEIGHT = 384

with open("Ball Bearing Position Data.csv", 'r') as csvfile:
    csvreader = csv.reader(csvfile,delimiter=',')
    lineCount = 0

    ballBearingPositions = []

    for row in csvreader:
        if (lineCount < 221) or (308 < lineCount < 341) or (368 < lineCount < 600):
            xint = float(row[1])
            yint = float(row[2])
            ballBearingPositions.append([xint,yint])
        lineCount = lineCount + 1

ballArray = np.array(ballBearingPositions)
ballArray.reshape((len(ballBearingPositions),2))
Ytrain = ballArray

# This section of code can be used to check if the ball bearing pointer is in the correct location
"""count = 0
from IPython import display
for filepath in os.listdir('vid 1/'):
    if count % 3 == 0:
        image = cv2.imread('vid 1/{0}'.format(filepath),1)
        imageResized = cv2.resize(image,(WIDTH,HEIGHT))
        plt.imshow(imageResized)
        plt.plot(ballBearingPositions[count][0],ballBearingPositions[count][1],'bo')
        plt.show()
    count = count + 1
"""

# Load in the train images automatically
imageSetTrain = []
counterTrain = 0
for filepath in os.listdir('vid 1/'):
    if (counterTrain < 221) or (308 < counterTrain < 341) or (368 < counterTrain < 600):
        image = cv2.imread('vid 1/{0}'.format(filepath),1)
        imageResized = cv2.resize(image,(WIDTH,HEIGHT))
        imageSetTrain.append(imageResized)
    counterTrain = counterTrain + 1

imagesTrain = np.array(imageSetTrain)
Xtrain = imagesTrain
print(Xtrain.shape)

# load in the test images automatically
# no ball bearing 726-927
imageSetTest = []
counterTest = 0
for filepath in os.listdir('vid 1/'):
    if (599 < counterTest < 726) or (927 < counterTest < 1286):
        image = cv2.imread('vid 1/{0}'.format(filepath), 1)
        imageResized = cv2.resize(image, (WIDTH, HEIGHT))
        imageSetTest.append(imageResized)
    counterTest = counterTest + 1

imagesTest = np.array(imageSetTest)
Xtest = imagesTest
print(Xtest.shape)
print(Ytrain.shape)

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
            nn.Linear(9216, 128), # The first number here changes based on the size. Use the errors when it runs to pick the number
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
        Ybatch = torch.from_numpy(Ytrain[Nbatch * k:Nbatch * (k + 1), :]).float()
        # The following code normalises the size of the image to be between 0-1
        size = np.array([[1 / HEIGHT, 1 / WIDTH]])
        size = torch.from_numpy(size).float()
        Ybatch = (Ybatch * size) - 0.5
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

plt.show()

# testing the model - code not yet tested
k = np.random.randint(Xtest.shape[0] / Nbatch)
test_ims = torch.from_numpy(Xtest[Nbatch * k:Nbatch * (k + 1), :, :, :]).float().transpose(1, 3) / 255
Ypred = network(test_ims).detach().numpy()
print(Ypred)

plt.figure(figsize=(15, 5))
for j in range(Nbatch):
    plt.subplot(2, 4, j + 1)
    plt.imshow(Xtest[Nbatch * k + j, :, :, :].squeeze(), extent=[0, WIDTH, HEIGHT, 0])
    size = np.array([HEIGHT, WIDTH])
    pos_pred = (Ypred + 0.5) * size  # Scale predictions to pixel coordinates
    print(pos_pred)
    # Show predicted marker positions in image
    plt.plot([pos_pred[j, 0]], [pos_pred[j, 1]], 'bo', markersize=5)
plt.show()