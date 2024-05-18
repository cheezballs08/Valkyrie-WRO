#Imports
import cv2 as cv
import os
from pysys.pysys import *
from robot.commands import *
from robot.subsystems import *
#Imports

#User Functions
def percent_to_255(percentage: float) -> int:
    return int(percentage * 255 / 100)
#User Functions

#Object definitions
scheduler = pysys.Scheduler.get_instance()

system = pysys.System.get_instance()

logger = pysys.Logger.get_instance()
#Object definitions

#25, 52, 72, 102, 255, 255

#User objects
vision_proccesing_subsystem = VisionProccesingSubsystem(
    name="VisionProccesingSubsystem",
    camera_id=1,
    kernel_size=10,
    green_bounds=[[36, percent_to_255(25), percent_to_255(25)], [86, percent_to_255(100), percent_to_255(100)]],
    red_bounds=[[0, percent_to_255(50), percent_to_255(50)], [10, percent_to_255(100), percent_to_255(100)]],
    area_limit=70
    )
#User objects

#Setup
scheduler.setup_scheduler(
    subsystem_commands_dictionary=
    {
        vision_proccesing_subsystem: (),
    },
    subsystem_default_command_dicitionary=
    {
        vision_proccesing_subsystem: None,
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
    system.run_system()
#Main Loop

#Cleanup
cv.destroyAllWindows()
vision_proccesing_subsystem.camera.release()
#Cleanup