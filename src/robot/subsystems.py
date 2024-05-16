import numpy as np
import pysys.pysys as pysys
import cv2 as cv

class VisionProccesingSubsystem(pysys.Subsystem):
    
    def __init__(self, name: str = "VisionProccesingSubsystem", camera_id: int = 0):
        super().__init__(self)
        self.camera = cv.VideoCapture(camera_id)
        self.frame = None
        self.green_masked_frame = None
        self.red_masked_frame = None
        self.green_mask = None
        self.red_mask = None
        self.kernel = np.ones((5, 5), np.uint8)
        
    def capture_frame(self):
        is_succesfull, frame =  self.camera.read()
        self.frame = frame
        
    def show_frame(self, frame_name, frame):
        cv.imshow(frame_name, frame)
        cv.waitKey(1)
        
    def show_frames(self, frames: list[cv.typing.MatLike]):
        count = -1
        
        for frame in frames:
            count += 1
            self.show_frame("frame" + str(count), frame)
        
    def mask_frame(self, lower_bound, upper_bound, kernel) -> cv.typing.MatLike:
        frame_hsv = cv.cvtColor(self.frame, cv.COLOR_BGR2HSV)
        mask = cv.inRange(frame_hsv, lower_bound, upper_bound)
        mask = cv.dilate(mask, kernel)
        
        return mask
    
    def update_masks(self):
        self.green_mask = self.mask_frame(np.array([25, 52, 72], np.uint8), np.array([102, 255, 255], np.uint8), self.kernel)
        self.red_mask = self.mask_frame(np.array([136, 87, 111], np.uint8), np.array([180, 255, 255], np.uint8), self.kernel)
        
        self.green_masked_frame = cv.bitwise_and(self.frame, self.frame, mask=self.green_mask)
        self.red_masked_frame = cv.bitwise_and(self.frame, self.frame, mask=self.red_mask)

    def periodic(self):
        pass
    
    def thread_periodic(self):
        while True:
            self.capture_frame()
            self.update_masks()
            self.show_frames(frames=[self.frame, self.red_masked_frame, self.green_masked_frame])
            cv.waitKey(1)