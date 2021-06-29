import numpy as np
import math
import airsim
import time
import cv2
import os

pi = math.pi


client = airsim.VehicleClient()
client.confirmConnection()


OUTPUT_DIR = r"./temp"

circle_steps = 20
elevation_steps = 10
center = (0, 0, 0)
sphereradius = 10

# Get points on circumference of the circle with coordinates of center
def circle(center, r, n):
    centerx, centery = center[0], center[1]
    x = [centerx + r*(math.cos(2*math.pi/n*x)) for x in range(0, n+1)]
    y = [centery + r*(math.sin(2*math.pi/n*x)) for x in range(0, n+1)]
    return x,y


# Get radius of circle when not level
def get_radius(elevation, sphereradius):
    if elevation > sphereradius:
        return False
    return math.sqrt((sphereradius ** 2) - (elevation ** 2))


# Gets the angle to look down at the center of the sphere
def lookdownangle(elevation, sphereradius):
    return math.asin(elevation/sphereradius)



client.simSetSegmentationObjectID("[\w]*", -1, True) # Sets all other StaticMesh to -1 so it won't show up in segmentation view
client.simSetSegmentationObjectID("", 1) # Set your target to unique ID so it'll show up in segmentation view

for step in range(0, elevation_steps):
    currentelevation = ((sphereradius/elevation_steps) * step) + center[2]
    # Breaks angle calculation
    if currentelevation >= sphereradius:
        break
    lookdown = lookdownangle(currentelevation, sphereradius)
    radius = get_radius(currentelevation, sphereradius)
    x, y = circle(center, radius, circle_steps)
    for i in range(circle_steps):
        tempx, tempy = x[i], y[i]
        angle = math.atan((tempy-center[1])/(tempx-center[0]))
        if tempx - center[0] >= 0:
            angle += pi
        camera_pose = airsim.Pose(airsim.Vector3r(tempx, tempy, -currentelevation), airsim.to_quaternion(-lookdown, 0, angle))
        client.simSetCameraPose("0", camera_pose)
        responses = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.Segmentation, False, False),
        airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
        response_switch = 0
        for response in responses:
            img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)
            img_rgb = img1d.reshape(response.height, response.width, 3)
            cv2.imwrite(os.path.join(OUTPUT_DIR, f"{i}_{response_switch}.png"), img_rgb)
            response_switch += 1



