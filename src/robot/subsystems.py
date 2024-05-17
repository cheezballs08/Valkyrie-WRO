import numpy as np
import pysys.pysys as pysys
import cv2 as cv
import gpiozero
#from gpiozero.pins.mock import MockFactory <---> For testing purposes

class VisionProccesingSubsystem(pysys.Subsystem):
    
    def __init__(self, name: str = "VisionProccesingSubsystem", camera_id: int = 0):
        super().__init__(self)
        self.camera = cv.VideoCapture(camera_id)
        self.frame = None
        self.green_masked_frame = None
        self.red_masked_frame = None
        self.green_mask = None
        self.red_mask = None
        self.kernel = np.ones((25, 25), np.uint8)
        
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
    
    def draw_bounding_boxes(self):
        red_contours, red_hierarchy = cv.findContours(self.red_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        for contour in red_contours:
            if cv.contourArea(contour) < 70:
                break
            x, y, w, h = cv.boundingRect(contour)
            cv.rectangle(self.frame, (x,y), (x+w, y+h), (0,0,255),2)
            cv.putText(self.frame, 'Red Object', (x, y-10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
            
        green_contours, green_hierarchy = cv.findContours(self.green_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        for contour in green_contours:
            if cv.contourArea(contour) < 30:
                break
            x, y, w, h = cv.boundingRect(contour)
            cv.rectangle(self.frame, (x,y), (x+w, y+h), (0,255,0),2)
            cv.putText(self.frame, 'Green Object', (x, y-10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
            
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
            self.draw_bounding_boxes()
            self.show_frames(frames=[self.frame, self.green_masked_frame, self.red_masked_frame])
            cv.waitKey(1)

class DrivetrainSubsystem(pysys.Subsystem):

    def __init__(self, name: str = "DrivetrainSubsystem", drive_motor_pins: list[int] = [0, 0], servo_id: int = 0, servo_min_max_angle: list[float] = [0, 0]):
        super().__init__(self)
        self.drive_motor = gpiozero.Motor(drive_motor_pins[0], drive_motor_pins[1], pwm=True)
        
        self.servo = gpiozero.AngularServo(servo_id, min_angle=servo_min_max_angle[0], max_angle=servo_min_max_angle[1])

    def set_motor_speed(self, speed: float):
        if speed < 0:
            self.drive_motor.forward(speed)    
        else:
            self.drive_motor.backward(speed)

    def set_servo_angle(self, angle: float):
        self.servo.angle = angle

    def periodic(self):
        pass
    
    def thread_periodic(self):
        pass
    
