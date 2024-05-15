import pysys.pysys as pysys
import cv2 as cv

class VisionProccesingSubsystem(pysys.Subsystem):
    
    def __init__(self, name: str = "VisionProccesingSubsystem", camera_id: int = 0):
        super().__init__(self)
        self.camera: cv.VideoCapture = cv.VideoCapture(camera_id)
        self.frame: cv.typing.MatLike = None

    def capture_frame(self):
        is_succesfull, frame =  self.camera.read()
        self.frame = frame
        
    def show_frame(self):
        cv.imshow("Frame", self.frame)
        cv.waitKey(1)

    def periodic(self):
        pass
    
    def thread_periodic(self):
        while True:
            self.capture_frame()
            self.show_frame()
            cv.waitKey(1)