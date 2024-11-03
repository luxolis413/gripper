from jodellSdk.jodellSdkDemo import ClawControl
from jodellSdk.jodellSdkDemo import EpgClawControl
import time

# Initialize claw controls
claw_control = ClawControl()
epg_claw_control = EpgClawControl()

# Set up serial communication
# sudo chmod 666 /dev/ttyUSB0
claw_control.serialOperation('/dev/ttyUSB0', 115200, True)

# Define parameters for open and close actions
open_params = (5, 0, 100, 100)
close_params = (5, 180, 100, 100)

while True:
    # Close the claw
    if claw_control.runWithParam(*close_params):
        print("Claw is closing...")
    time.sleep(5)  # Wait for 5 seconds

    # Open the claw
    if claw_control.runWithParam(*open_params):
        print("Claw is opening...")
    time.sleep(5)  # Wait for 5 seconds






