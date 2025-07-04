import math

class RobotGlobals:
    def __init__(self, sim):
        self.sim = sim
        self.dt = sim.getSimulationTimeStep()

        # PID controller variables
        self.Kp = 5000.0
        self.Ki = 100.0
        self.Kd = 30.0

        self.target_angle = 0
        self.prev_error = 0
        self.integral_sum = 0
        self.cumulative_angle = 0

        # Movement and input tracking
        self.forward_speed = 0
        self.yaw_input = 0
        self.no_input_timer = 0
        self.prismatic_joint_velocity = 0
        self.arm_joint_velocity = 0

        # Robot handles
        self.bot_body = sim.getObjectHandle('/body')
        self.left_joint = sim.getObjectHandle('/body/left_joint')
        self.right_joint = sim.getObjectHandle('/body/right_joint')
        self.prismatic_joint_handle = sim.getObjectHandle('/body/Prismatic_joint')
        self.arm_joint_handle = sim.getObjectHandle('/body/arm_joint')


# Create a global instance of RobotGlobals
robot_globals = None


def sysCall_init():
    global robot_globals
    sim = require('sim')
    robot_globals = RobotGlobals(sim)


def sysCall_actuation():
    global robot_globals
    rg = robot_globals  # Alias for brevity

    # Handle user input
    new_input = False
    message, auxiliaryData, _ = rg.sim.getSimulatorMessage()

    if message == rg.sim.message_keypress:
        key = auxiliaryData[0]

        if key == 2008:  # Up arrow key
            rg.forward_speed = 5.0
            new_input = True
        elif key == 2007:  # Down arrow key
            rg.forward_speed = -5.0
            new_input = True
        elif key == 2009:  # Left arrow key
            rg.yaw_input = -3.0
            new_input = True
        elif key == 2010:  # Right arrow key
            rg.yaw_input = 3.0
            new_input = True
        elif key == 113:  # 'q' for closing the gripper
            rg.prismatic_joint_velocity = -0.1
            new_input = True
        elif key == 101:  # 'e' for opening the gripper
            rg.prismatic_joint_velocity = 0.1
            new_input = True
        elif key == 119:  # 'w' for raising the gripper
            rg.arm_joint_velocity = 1
            new_input = True
        elif key == 115:  # 's' for lowering the gripper
            rg.arm_joint_velocity = -1
            new_input = True

    if not new_input:
        rg.no_input_timer += rg.dt
        if rg.no_input_timer > 0.1:  # Reset movement after 100ms of no input
            rg.forward_speed = 0
            rg.yaw_input = 0
            rg.prismatic_joint_velocity = 0
            rg.arm_joint_velocity = 0
    else:
        rg.no_input_timer = 0

    # Get current orientation and angular velocity
    _, current_angle, _ = rg.sim.getObjectOrientation(rg.bot_body, -1)
    gyro_rate = rg.sim.getObjectVelocity(rg.bot_body, -1)[1][0]

    # Update cumulative angle
    rg.cumulative_angle += gyro_rate * rg.dt

    # PID control logic
    error = rg.cumulative_angle - rg.target_angle
    deri_error = (error - rg.prev_error) / rg.dt
    rg.integral_sum += error

    u = rg.Kp * error - rg.Kd * deri_error + rg.Ki * rg.integral_sum
    u = max(min(20, u), -20)  # Limit control output

    rg.prev_error = error

    # Set motor velocities
    left_velocity = -u + rg.forward_speed - rg.yaw_input
    right_velocity = -u + rg.forward_speed + rg.yaw_input

    rg.sim.setJointTargetVelocity(rg.left_joint, left_velocity)
    rg.sim.setJointTargetVelocity(rg.right_joint, right_velocity)
    rg.sim.setJointTargetVelocity(rg.prismatic_joint_handle, rg.prismatic_joint_velocity)
    rg.sim.setJointTargetVelocity(rg.arm_joint_handle, rg.arm_joint_velocity)


def sysCall_sensing():
    pass


def sysCall_cleanup():
    pass
