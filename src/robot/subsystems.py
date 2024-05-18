import numpy as np
import pysys.pysys as pysys
import cv2 as cv
import gpiozero
#from gpiozero.pins.mock import MockFactory <---> For testing purposes

class VisionProccesingSubsystem(pysys.Subsystem):

    def __init__(self, name: str = "VisionProccesingSubsystem", camera_id: int = 0, kernel_size: int = 25, green_bounds: list[list[int]] = [[25, 52, 72], [102, 255, 255]], red_bounds: list[list[int]] = [[136, 87, 111], [180, 255, 255]], area_limit: int = 70):
        super().__init__(self)
        self.camera = cv.VideoCapture(camera_id)
        self.frame = None
        self.green_masked_frame = None
        self.red_masked_frame = None
        self.green_mask = None
        self.red_mask = None
        self.kernel = np.ones((kernel_size, kernel_size), np.uint8)
        self.green_lower_bound = np.array(green_bounds[0], np.uint8)
        self.green_upper_bound = np.array(green_bounds[1], np.uint8)
        self.red_lower_bound = np.array(red_bounds[0], np.uint8)
        self.red_upper_bound = np.array(red_bounds[1], np.uint8)
        self.red_contours = None
        self.green_contours = None
        self.red_best_target = None
        self.green_best_target = None
        self.best_target = None
        self.area_limit = area_limit
        
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
    
    def update_contours(self):
        self.red_contours, red_hierarchy = cv.findContours(self.red_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        self.green_contours, green_hierarchy = cv.findContours(self.green_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    
    def update_best_targets(self):
        if len(self.red_contours) != 0 and len(self.green_contours) == 0:
            self.red_best_target = max(self.red_contours, key = cv.contourArea)
            self.best_target = self.red_best_target
            
        elif len(self.red_contours) == 0 and len(self.green_contours) != 0:
            self.green_best_target = max(self.green_contours, key = cv.contourArea)
            self.best_target = self.green_best_target
            
        elif len(self.red_contours) == 0 and len(self.green_contours) == 0:
            self.best_target = None
            self.red_best_target = None
            self.green_best_target = None
            
        else:
            self.red_best_target = max(self.red_contours, key = cv.contourArea)
            self.green_best_target = max(self.green_contours, key = cv.contourArea)
            self.best_target = max([self.red_best_target, self.green_best_target], key = cv.contourArea)
                
    def draw_bounding_boxes(self, area_limit: int = 70):
        count: int = 0
        
        for contour in self.red_contours:
            count += 1
            if cv.contourArea(contour) < area_limit:
                break
            x, y, w, h = cv.boundingRect(contour)
            
            cv.rectangle(self.frame, (x,y), (x+w, y+h), (0,0,255),2)
            cv.putText(self.frame, f"Red Object | Target ID: {count}" if contour is not self.red_best_target else f"Red Best Target | Target ID: {count}", (x, y-10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
        
        count = 0
        for contour in self.green_contours:
            count += 1
            if cv.contourArea(contour) < area_limit:
                break
            x, y, w, h = cv.boundingRect(contour)
            
            cv.rectangle(self.frame, (x,y), (x+w, y+h), (0,255,0),2)
            cv.putText(self.frame, f"Green Object | Target ID: {count}" if contour is not self.green_best_target else f"Green Best Target | Target ID: {count}", (x, y-10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
             
    def update_masks(self):
        self.green_mask = self.mask_frame(self.green_lower_bound, self.green_upper_bound, self.kernel)
        self.red_mask = self.mask_frame(self.red_lower_bound, self.red_upper_bound, self.kernel)
        
        self.green_masked_frame = cv.bitwise_and(self.frame, self.frame, mask=self.green_mask)
        self.red_masked_frame = cv.bitwise_and(self.frame, self.frame, mask=self.red_mask)

    def periodic(self):
        pass
    
    def thread_periodic(self):
        while True:
            self.capture_frame()
            self.update_masks()
            self.update_contours()
            self.update_best_targets()
            self.draw_bounding_boxes(self.area_limit)
            self.show_frames(frames=[self.frame, self.red_masked_frame, self.green_masked_frame])
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
    
