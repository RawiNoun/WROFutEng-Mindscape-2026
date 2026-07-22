# camera.py

from picamera2 import Picamera2, Preview
import cv2
import time

from camera_config import (
    WIDTH,
    HEIGHT,
    MAIN_FORMAT,
    LORES_FORMAT,
    USE_LARGEST_SENSOR_MODE,
    MAX_FOV,
    PREVIEW_TYPE
)


class Camera:
    def __init__(self):
        self.picam2 = Picamera2()

    def start(self):
        if USE_LARGEST_SENSOR_MODE:
            mode = self.picam2.sensor_modes[-1]
            main_size = mode["size"]
        else:
            main_size = (WIDTH, HEIGHT)

        config = self.picam2.create_preview_configuration(
            main={
                "size": main_size,
                "format": MAIN_FORMAT
            },
            lores={
                "size": (WIDTH, HEIGHT),
                "format": LORES_FORMAT
            }
        )

        self.picam2.configure(config)

        if PREVIEW_TYPE == "QTGL":
            self.picam2.start_preview(Preview.QTGL)
        elif PREVIEW_TYPE == "DRM":
            self.picam2.start_preview(Preview.DRM)

        self.picam2.start()

        time.sleep(2)

        if MAX_FOV:
            full_crop = self.picam2.camera_controls["ScalerCrop"][1]
            self.picam2.set_controls({
                "ScalerCrop": full_crop
            })

    def get_frame(self):
        frame = self.picam2.capture_array("lores")

        # Working color pipeline from your last stable code
        frame = frame[:, :, :3]

        return frame

    def show_overlay(self, img):
        overlay = cv2.cvtColor(
            img,
            cv2.COLOR_BGR2BGRA
        )

        self.picam2.set_overlay(overlay)

    def stop(self):
        self.picam2.set_overlay(None)
        self.picam2.stop()
