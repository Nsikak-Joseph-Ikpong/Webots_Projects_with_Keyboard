"""pr2_basic_movement controller."""

# Import necessary classes
from controller import Robot, Keyboard

# Create the Robot instance and initialize keyboard
robot = Robot()
keyboard = Keyboard()

# Get the time step of the current world
TIME_STEP = 32
MAX_SPEED = 8.0

# Enable keyboard input
keyboard.enable(TIME_STEP)

# Get motors of the robot
wheel_motors = {}
wheel_motors['FLL_WHEEL'] = robot.getDevice('fl_caster_l_wheel_joint')
wheel_motors['FLR_WHEEL'] = robot.getDevice('fl_caster_r_wheel_joint')
wheel_motors['FRL_WHEEL'] = robot.getDevice('fr_caster_l_wheel_joint')
wheel_motors['FRR_WHEEL'] = robot.getDevice('fr_caster_r_wheel_joint')
wheel_motors['BLL_WHEEL'] = robot.getDevice('bl_caster_l_wheel_joint')
wheel_motors['BLR_WHEEL'] = robot.getDevice('bl_caster_r_wheel_joint')
wheel_motors['BRL_WHEEL'] = robot.getDevice('br_caster_l_wheel_joint')
wheel_motors['BRR_WHEEL'] = robot.getDevice('br_caster_r_wheel_joint')

# Set initial speed to 0 and set position to infinity (velocity control mode)
for wheel in wheel_motors.values():
    wheel.setPosition(float('inf'))
    wheel.setVelocity(0.0)

# Main loop
while robot.step(TIME_STEP) != -1:
    key = keyboard.getKey()
    
    if key == ord('W'):
        for wheel in wheel_motors.values():
            wheel.setVelocity(MAX_SPEED)
    elif key == ord('S'):
        for wheel in wheel_motors.values():
            wheel.setVelocity(-MAX_SPEED)
    else:
        for wheel in wheel_motors.values():
            wheel.setVelocity(0.0)
