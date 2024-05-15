#Imports
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
#User objects

#Setup
scheduler.setup_scheduler(
    subsystem_commands_dictionary=
    {

    },
    subsystem_default_command_dicitionary=
    {

    }
    )

system.setup_system(loop_period=0.02)
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
    
    if system.tick_count == 17:
        system.exit_system()
#Main Loop