from qibullet import SimulationManager
from qibullet import PepperVirtual
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

    pybullet.loadURDF("objects\person.urdf", basePosition=[0, 3, 0], baseOrientation=[0, 0, 0, 1])

    pybullet.loadURDF("objects\wall.urdf", basePosition=[-1, -1.5, 0], baseOrientation=[1, 0, 0, 1], useFixedBase=1,
                      physicsClientId=client_id)

    pybullet.loadURDF("objects\wall.urdf", basePosition=[-1, 2, 0], baseOrientation=[1, 0, 0, 1], useFixedBase=1,
                      physicsClientId=client_id)

    pybullet.loadURDF("objects\wall.urdf", basePosition=[-1.5, -2, 0], baseOrientation=[1, 1, 1, 1], useFixedBase=1,
                      physicsClientId=client_id)

    pybullet.loadURDF("objects\wall.urdf", basePosition=[-1.5, 0, 0], baseOrientation=[1, 1, 1, 1], useFixedBase=1,
                      physicsClientId=client_id)

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
            pepper.moveTo(1.0, 0.0, 0.0, frame=PepperVirtual.FRAME_ROBOT, speed=100)
            pepper.moveTo(0, 0, 3.14159, frame=PepperVirtual.FRAME_ROBOT)
            pepper.moveTo(1.0, 0.0, 0.0, frame=PepperVirtual.FRAME_ROBOT, speed=100)
            pepper.moveTo(0, 0, 3.14159, frame=PepperVirtual.FRAME_ROBOT)
    except KeyboardInterrupt:
        pass
    finally:
        pepper.unsubscribeCamera(handle)
        simulation_manager.stopSimulation(handle)
