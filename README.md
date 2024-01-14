# Robust-plane-detection
Robust plane detection on LiDAR point clouds by SeqRANSAC

finding the three most dominant planes by Sequential RANSAC (SeqRANSAC) algorithm with optimal plane detection and Colouring the three detected planes by red, green and pink.
# Sequential RANSAC (the iterative modification of RANSAC):
it finds a plane by RANSAC, then the plane points are removed from the dataset, and RANSAC method is run again on the rest of the points. The plane detection and removal can be iterated many times. In this assignment, the three most dominant plane should be found. 

# The LiDAR clouds(inputs):
![Project Logo](https://github.com/AhmadEsmail/Robust-plane-detection/blob/main/tunnel00.png)
![Project Logo](https://github.com/AhmadEsmail/Robust-plane-detection/blob/main/street00.png)
![Project Logo](https://github.com/AhmadEsmail/Robust-plane-detection/blob/main/chainBridge00.png)


# The Results:
![Project Logo](https://github.com/AhmadEsmail/Robust-plane-detection/blob/main/tunnel.PNG)
![Project Logo](https://github.com/AhmadEsmail/Robust-plane-detection/blob/main/street.PNG)
![Project Logo](https://github.com/AhmadEsmail/Robust-plane-detection/blob/main/chainbridge.PNG)
