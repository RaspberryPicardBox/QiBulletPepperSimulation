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

def camera_view():
    while True:
        global global_coords
        img = pepper.getCameraFrame(handle)

        face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        grayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(grayscale, 1.3, 5)

        global_coords = faces

        for (x, y, w, h) in faces:
            img = cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

        cv2.imshow('Camera: ', img)

        cv2.waitKey(1)

def move_human():
    while True:
        time.sleep(2)
        side = random.randint(0, 10)
        side2 = random.randint(0, 10)

        if side > 5:
            pybullet.resetBasePositionAndOrientation(humanID, posObj=[2, 0.3, 1.2], ornObj=[1, -1, -1, 1])
        else:
            pybullet.resetBasePositionAndOrientation(humanID, posObj=[2, -0.3, 1.2], ornObj=[1, -1, -1, 1])

        if side2 > 5:
            pybullet.resetBasePositionAndOrientation(humanIDTwo, posObj=[3, 0.15, 1.2], ornObj=[1, -1, -1, 1])
        else:
            pybullet.resetBasePositionAndOrientation(humanIDTwo, posObj=[3, -0.15, 1.2], ornObj=[1, -1, -1, 1])

if __name__ == "__main__":
    #vhacdConvertor.__init__()  # Decompose .OBJ objects to a single appropriate URDF with correct collisions
    urdfConvertor.__init__()  # Convert all .STL and .OBJ files to .URDF for PyBullet

    global_coords = []

    simulation_manager = SimulationManager()
    client_id = simulation_manager.launchSimulation(gui=True)

    humanID = pybullet.loadURDF("objects\\human.urdf", basePosition=[2, -0.3, 1.2], baseOrientation=[1, -1, -1, 1],
                      globalScaling=0.01, useFixedBase=1, physicsClientId=client_id)  # Placeholder human

    humanIDTwo = pybullet.loadURDF("objects\\human.urdf", basePosition=[3, 0.15, 1.2], baseOrientation=[1, -1, -1, 1],
                      globalScaling=0.01, useFixedBase=1, physicsClientId=client_id)  # Placeholder human

    pepper = simulation_manager.spawnPepper(client_id, spawn_ground_plane=False)

    pepper.goToPosture("Stand", 0)
    handle = pepper.subscribeCamera(PepperVirtual.ID_CAMERA_TOP)

    cameraThread = threading.Thread(None, camera_view, name="Camera-Thread")
    cameraThread.daemon = True
    cameraThread.start()

    humanThread = threading.Thread(None, move_human, name="Human-Movement")
    humanThread.daemon = True
    humanThread.start()

    try:
        #pepper.moveTo(-1, 0, 0, frame=PepperVirtual.FRAME_ROBOT)
        while True:
            time.sleep(0.25)
            for face in global_coords:
                print("Focusing on face: {0}".format(face))
                if face[0] < 160:
                    if face[0] < 160:
                        deg = 160/face[0]
                    else:
                        deg = 0
                    pepper.moveTo(0, 0, np.radians(deg), frame=PepperVirtual.FRAME_ROBOT)  # Turn is in radians

                elif face[0] > 160:
                    if face[0] > 0:
                        deg = 320/face[0]
                    else:
                        deg = 0
                    pepper.moveTo(0, 0, np.radians(-deg), frame=PepperVirtual.FRAME_ROBOT)

                if face[2] < 40:
                    len = (105/face[2])-1
                    print("Moving: " + str(len))
                    pepper.moveTo(len, 0, 0, frame=PepperVirtual.FRAME_ROBOT)

                if face[2] >= 40:
                    imnum = random.randint(1, 1)
                    thermalim = cv2.imread("images\\thermal\\{0}.jpg".format(imnum))
                    cv2.imshow("Thermal: ", thermalim)
                    cv2.waitKey(0)
                    cv2.destroyWindow(thermalim)

                global_coords = []

    except KeyboardInterrupt:
        pass
    finally:
        pepper.unsubscribeCamera(handle)
        simulation_manager.stopSimulation(handle)
