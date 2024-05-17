#Imports
import cv2 as cv
import os
from pysys.pysys import *
from robot.commands import *
from robot.subsystems import *
#Imports

#Object definitions
scheduler = pysys.Scheduler.get_instance()

system = pysys.System.get_instance()

logger = pysys.Logger.get_instance()
#Object definitions

#User objects
vision_proccesing_subsystem = VisionProccesingSubsystem(name="VisionProccesingSubsystem", camera_id=1)
drivetrain_subsystem = DrivetrainSubsystem(name="DrivetrainSubsystem", drive_motor_pins=[0, 0], servo_id=0, servo_min_max_angle=[0, 0])
#User objects

#Setup
scheduler.setup_scheduler(
    subsystem_commands_dictionary=
    {
        vision_proccesing_subsystem: (),
        drivetrain_subsystem: ()
    },
    subsystem_default_command_dicitionary=
    {
        vision_proccesing_subsystem: None,
        drivetrain_subsystem: None
    }
    )

system.setup_system(loop_period=2)
#Setup

#User setup
#User setup

#Main Loop (Feel free to change this however you want.)
with open("log.pylog", "w") as file:
    file.write("")
    
scheduler.remove_duplicate_items() #Just in case, I would advise you to keep this here.

while system.is_active:
    os.system('cls' if os.name == 'nt' else 'clear')

    system.run_system()
#Main Loop

#Cleanup
cv.destroyAllWindows()
vision_proccesing_subsystem.camera.release()
#Cleanup