from qibullet import SimulationManager
from qibullet import PepperVirtual
import numpy as np
import pybullet
import cv2
import threading
import urdfConvertor
import vhacdConvertor
import random
import time
from matplotlib import pyplot


def collision_avoidance():  # Prevents Pepper from running into anything in front of it
    pepper.subscribeLaser()
    pepper.showLaser(True)

    while True:
        for value in pepper.getFrontLaserValue():  # Checks all the front laser scanners
            if value != 0:
                if value < 1:
                    pepper.moveTo(0, 0, 0, frame=PepperVirtual.FRAME_ROBOT)
                    # And freezes pepper in place whilst any
                    # are less than 1 meter


def camera_view():  # Shows the camera and identifies faces
    global global_coords
    while True:
        img = pepper.getCameraFrame(handle)

        face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        greyscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(greyscale, 1.3, 5)

        #  TODO: Add eye detection, facial feature recognition, and save data in server to prevent re-checking

        global_coords = faces

        for (x, y, w, h) in faces:
            img = cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

        cv2.imshow('Camera: ', img)
        cv2.waitKey(1)


def thermal_face():  # Loads up an example data set of a thermal image facial and checks temperature
    print("Showing thermal face...")
    imnum = random.randint(1, 1)
    thermalim = cv2.imread("images\\thermal\\{0}.jpg".format(imnum))
    # TODO: Add in Tom's ML algorithm for temperature, add more thermal data
    cv2.imshow("Thermal: ", thermalim)
    cv2.waitKey(0)
    cv2.destroyWindow(thermalim)


def find_home():  # Finds the home position of Pepper
    global global_home
    corners = []

    while True:
        img = pepper.getCameraFrame(handle)

        mask = cv2.inRange(img, np.array([0, 0, 0]), np.array([0, 255, 0]))  # Only look for green markers
        res = cv2.bitwise_and(img, img, mask=mask)
        grey = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        grey = np.float32(grey)

        corners.append(cv2.goodFeaturesToTrack(grey, 4, 0.01, 10))  # Perform feature corner recognition

        if type(corners[0]) != "NoneType":
            global_home = corners
        else:
            corners = []

        # TODO: Still doesn't work

        cv2.imshow("Corners: ", res)
        cv2.waitKey(1)


def find_human():  # Tracks Pepper to Humans
    global global_coords

    for face in global_coords:
        print("Focusing on face: {0}".format(face))
        if face[0] < 160:  # Turn left to face human on left
            if face[0] < 160:
                deg = 160 / face[0]
            else:
                deg = 0
            pepper.moveTo(0, 0, np.radians(deg), frame=PepperVirtual.FRAME_ROBOT)  # Turn is in radians

        elif face[0] > 160:  # Turn right to face human on right
            if face[0] > 0:
                deg = 320 / face[0]
            else:
                deg = 0
            pepper.moveTo(0, 0, np.radians(-deg), frame=PepperVirtual.FRAME_ROBOT)

        if face[2] < 60:  # If they're too far away, move closer in inverse proportional steps
            len = (105 / face[2]) - 1
            print("Moving: " + str(len))
            pepper.moveTo(len, 0, 0, frame=PepperVirtual.FRAME_ROBOT)

        if face[2] >= 60:  # If close enough, take thermal reading
            thermal_face()

        global_coords = []  # Have to blank all previously seen faces each loop


if __name__ == "__main__":
    # vhacdConvertor.__init__()  # Decompose .OBJ objects to a single appropriate URDF with correct collisions
    # urdfConvertor.__init__()  # Convert all .STL and .OBJ files to .URDF for PyBullet

    global_coords = []
    global_home = []

    simulation_manager = SimulationManager()
    client_id = simulation_manager.launchSimulation(gui=True)

    humanID = pybullet.loadURDF("objects\\humanBody.urdf", basePosition=[4, 0, 1.4], baseOrientation=[1, -1, -1, 1],
                                globalScaling=0.01, useFixedBase=1, physicsClientId=client_id)  # Placeholder human

    baseID = pybullet.loadURDF("objects\\cube.urdf", basePosition=[0, 0, 0], baseOrientation=[1, 1, 1, 1],
                               globalScaling=0.3, useFixedBase=1)  # Placeholder Home station

    pepper = simulation_manager.spawnPepper(client_id, spawn_ground_plane=True)

    pepper.goToPosture("Stand", 0)
    handle = pepper.subscribeCamera(PepperVirtual.ID_CAMERA_TOP)

    cameraThread = threading.Thread(None, camera_view, name="Camera-Thread")
    cameraThread.daemon = True
    cameraThread.start()  # Start the camera (and facial recognition) on a separate thread

    """
    findHomeThread = threading.Thread(None, find_home, name="Home_Find_Thread")
    findHomeThread.daemon = True
    findHomeThread.start()  # Start the home finding camera system on a separate thread
    """

    collisionThread = threading.Thread(None, collision_avoidance, name="Collision_Avoidance")
    collisionThread.daemon = True
    collisionThread.start()  # Start the collision avoidance system on a separate thread

    try:
        while True:
            time.sleep(0.25)  # Sleep for quarter of a second each cycle to allow for now face data
            find_human()

    except KeyboardInterrupt:
        pass

    finally:
        pepper.unsubscribeCamera(handle)
        pepper.unsubscribeLaser(handle)
        simulation_manager.stopSimulation(handle)
