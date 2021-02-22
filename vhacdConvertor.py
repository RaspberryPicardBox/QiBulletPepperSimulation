import pybullet
import os


def __init__():
    for foldername in os.listdir("./objects"):
        if foldername[-5:].lower() == ".urdf":
            break
        for filename in os.listdir("./objects/" + foldername):
            if "Two" in filename:
                break
            if filename[-4:].lower() == ".obj":
                pybullet.vhacd("./objects/" + foldername + "/" + filename, "./objects/" + foldername + "/" +
                               filename[:-4] + "Two" + filename[-4:], "log.txt", convexhullApproximation=0)
