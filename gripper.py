from jodellSdk.jodellSdkDemo import EpgClawControl
from jodellSdk.jodellSdkDemo import ClawControl
from jodellSdk.jodellSdkDemo import RgClawControl






# Select the corresponding model to import and create an operation object, such as importing EPG type

com = 1
baudrate = 11500
salveId = 9
readMode = 4
address = 2000
count = 2
sendCmdBuf = [1,2]
cmdId = 1


claw_control = ClawControl()


rg_claw = RgClawControl()

comlist = rg_claw.scanSalveId(1,10) # scan ID of the gripper

enb_clamp = rg_claw.enableClamp(salveId, True) # enable the clamp
stop_claw =  rg_claw.stopClaw(salveId) # Stop the jaw movement
force_open_brake = rg_claw.forceOpenBrake(salveId, True) # forcibly open the brake
switch = rg_claw.switchMode(9, 1) # Switching the control mode between serial communication and to IO control mode
switch_input_state = rg_claw.switchInputState(salveId, True) # switches the IO input state of the gripper with slave address to anti-logic
switch_output_state = rg_claw.switchOutputState(salveId, True) # switches the IO output state of the gripper with slave address to anti-logic
get_clamp_current_state = rg_claw.getClampCurrentState(salveId) # Get the clamping end status of the gripper with slave address
get_clamp_current_position = rg_claw.getClampCurrentLocation(salveId) # get the current position of the clamping end 
get_clamp_current_speed = rg_claw.getClampCurrentSpeed(salveId) # get the current speed of the clamping end
get_clamp_current_torque =  rg_claw.getClampCurrentTorque(salveId) # Get the current moment of the clamping end