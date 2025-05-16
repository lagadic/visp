import argparse
import math
import sys
from visp.robot import RobotFranka, Robot

def main():
    parser = argparse.ArgumentParser(description='Python wrapper example over vpRobotFranka ViSP class')
    parser.add_argument('--ip', type=str, default="192.168.30.10", dest='robot_ip', help='Robot IP address like 192.168.30.10')

    args, unknown_args = parser.parse_known_args()
    if unknown_args:
        print(f"The following args are not recognized and will not be used: {unknown_args}")
        sys.exit()

    print(f"Use robot IP: {args.robot_ip}")

    robot = RobotFranka()
    robot.connect(args.robot_ip, RobotFranka.RealtimeConfig.kEnforce)
    robot.connect("192.168.30.10", RobotFranka.RealtimeConfig.kEnforce)
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
