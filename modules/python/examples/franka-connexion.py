import math
from visp.robot import RobotFranka, Robot

def main():
    robot = RobotFranka()
    robot.connect('192.168.30.10', RobotFranka.RealtimeConfig.kEnforce)
    pos = robot.getPosition(Robot.JOINT_STATE)
    print(f"Initial joint position: \n{pos}")

    print("\nWARNING: This example will move the robot!")
    print("Please make sure to have the user stop button at hand!")
    print("Press Enter to continue...")
    input()

    print("Move to home position")
    pos[0] = 0
    pos[1] = 0
    pos[2] = 0
    pos[3] = -math.pi / 2
    pos[4] = 0
    pos[5] = math.pi / 2
    pos[6] = math.pi / 4
    robot.setRobotState(Robot.STATE_POSITION_CONTROL)
    robot.setPosition(Robot.JOINT_STATE, pos)

if __name__ == "__main__":
    main()
