from visp.robot import RobotFranka

def main():
    robot = RobotFranka()
    robot.connect('192.168.30.10', RobotFranka.RealtimeConfig.kEnforce)
    pos = robot.getPosition(Robot.REFERENCE_FRAME)
    print(f"{pos=}")

if __name__ == "__main__":
    main()
