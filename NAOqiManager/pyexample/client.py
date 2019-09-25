import cv2
import numpy as np
from PIL import Image

from naoqipy import QICameraClient, QIGazeAnalysisClient

FPS = 15

if __name__ == '__main__':
    stereoCamera = QICameraClient("StereoCamera")
    gazeAnalysiser = QIGazeAnalysisClient("gazeAnalysiser")
    stereoCamera.openSharedMemory()
    gazeAnalysiser.openSharedMemory()

    while True:
        data = gazeAnalysiser.getData()
        print("QIGazeAnalysisData", data.looking_at_score,
              data.time_stamp, data.isLookingAt())

        image = stereoCamera.getImage()
        cv_im = image.data_numpy[:, :, ::-1]
        pil_im = Image.frombytes(
            "RGB", (image.width, image.height), bytes(image.data))

        print("QIImage", image.width, image.height,
              image.time_stamp, image.color_space)
        cv2.imshow("images", cv_im)
        cv2.waitKey(int(1000 / FPS))

        # cv2.imwrite("cv_image.png", cv_im)
        # pil_im.save("pil_image.png")
