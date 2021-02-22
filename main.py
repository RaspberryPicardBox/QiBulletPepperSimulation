from qibullet import SimulationManager
from qibullet import PepperVirtual
import numpy as np
import pybullet
import cv2
import threading
import urdfConvertor


def camera_view():
    while True:
        img = pepper.getCameraFrame(handle)
        cv2.imshow("Camera: ", img)
        cv2.waitKey(1)


if __name__ == "__main__":
    urdfConvertor.__init__()

    simulation_manager = SimulationManager()
    client_id = simulation_manager.launchSimulation(gui=True)

    pybullet.loadURDF("objects\wall.urdf", basePosition=[-1, -1.5, 0], baseOrientation=[1, 0, 0, 1], useFixedBase=1,
                      physicsClientId=client_id)  # Right Wall

    pybullet.loadURDF("objects\wall.urdf", basePosition=[1, -1.5, 0], baseOrientation=[1, 0, 0, 1], useFixedBase=1,
                      physicsClientId=client_id)  # Second Right Wall

    pybullet.loadURDF("objects\wall.urdf", basePosition=[-1, 2, 0], baseOrientation=[1, 0, 0, 1], useFixedBase=1,
                      physicsClientId=client_id)  # Left Wall

    pybullet.loadURDF("objects\wall.urdf", basePosition=[1, 2, 0], baseOrientation=[1, 0, 0, 1], useFixedBase=1,
                      physicsClientId=client_id)  # Second Left Wall

    pybullet.loadURDF("objects\wall.urdf", basePosition=[-1.5, -2, 0], baseOrientation=[1, 1, 1, 1], useFixedBase=1,
                      physicsClientId=client_id)  # Back Right

    pybullet.loadURDF("objects\wall.urdf", basePosition=[-1.5, 0, 0], baseOrientation=[1, 1, 1, 1], useFixedBase=1,
                      physicsClientId=client_id)  # Back Left

    pybullet.loadURDF("objects\wall.urdf", basePosition=[2.5, -2, 0], baseOrientation=[1, 1, 1, 1], useFixedBase=1,
                      physicsClientId=client_id)  # Internal Right

    pepper = simulation_manager.spawnPepper(
        client_id,
        spawn_ground_plane=True)

    pepper.goToPosture("Stand", 0)
    handle = pepper.subscribeCamera(PepperVirtual.ID_CAMERA_TOP)

    cameraThread = threading.Thread(None, camera_view, name="Camera-Thread")
    cameraThread.daemon = True
    cameraThread.start()

    try:
        while True:
            x = 1
            pepper.moveTo(1.0, 0.0, 0.0, frame=PepperVirtual.FRAME_ROBOT)
            pepper.moveTo(0, 0, np.radians(180), frame=PepperVirtual.FRAME_ROBOT)  # Turn is in radians
            pepper.moveTo(1.0, 0.0, 0.0, frame=PepperVirtual.FRAME_ROBOT)
            pepper.moveTo(0, 0, np.radians(180), frame=PepperVirtual.FRAME_ROBOT)
    except KeyboardInterrupt:
        pass
    finally:
        pepper.unsubscribeCamera(handle)
        simulation_manager.stopSimulation(handle)
