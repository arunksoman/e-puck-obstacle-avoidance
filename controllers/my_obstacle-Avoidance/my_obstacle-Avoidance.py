from controller import Robot

# Get reference to the robot.
robot = Robot()

# Get simulation step length.
timeStep = int(robot.getBasicTimeStep())

# Constants of the e-puck motors and distance sensors.
maxMotorVelocity = 4.2
num_left_dist_sensors = 4
num_right_dist_sensors = 4
right_threshold = [75, 75, 75, 75]
left_threshold = [75, 75, 75, 75]

# Get left and right wheel motors.
leftMotor = robot.getMotor("left wheel motor")
rightMotor = robot.getMotor("right wheel motor")

# Get frontal distance sensors.
dist_left_sensors = [robot.getDistanceSensor('ps' + str(x)) for x in range(num_left_dist_sensors)]  # distance sensors
list(map((lambda s: s.enable(timeStep)), dist_left_sensors))  # Enable all distance sensors

dist_right_sensors = [robot.getDistanceSensor('ps' + str(x)) for x in range(num_right_dist_sensors,8)]  # distance sensors
list(map((lambda t: t.enable(timeStep)), dist_right_sensors))  # Enable all distance sensors

# Disable motor PID control mode.
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# Set ideal motor velocity.
initialVelocity = 0.7 * maxMotorVelocity

# Set the initial velocity of the left and right wheel motors.
leftMotor.setVelocity(initialVelocity)
rightMotor.setVelocity(initialVelocity)

while robot.step(timeStep) != -1:
    left_dist_sensor_values = [g.getValue() for g in dist_left_sensors]
    right_dist_sensor_values = [h.getValue() for h in dist_right_sensors]
    
    left_obstacle = [(x > y) for x, y in zip(left_dist_sensor_values, left_threshold)]
    right_obstacle = [(m > n) for m, n in zip(right_dist_sensor_values, right_threshold)]
 
    if True in left_obstacle:
        leftMotor.setVelocity(initialVelocity-(0.5*initialVelocity))
        rightMotor.setVelocity(initialVelocity+(0.5*initialVelocity))
    
    elif True in right_obstacle:
        leftMotor.setVelocity(initialVelocity+(0.5*initialVelocity))
        rightMotor.setVelocity(initialVelocity-(0.5*initialVelocity))
