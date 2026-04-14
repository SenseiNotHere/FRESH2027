from wpimath.geometry import Pose2d, Rotation2d


class PhysicsEngine:
    def __init__(self, physics_controller, robot):
        self.robot = robot
        self.x = 0.0
        self.y = 0.0
        self.heading = Rotation2d()

    def update_sim(self, now: float, tm_diff: float) -> None:
        if not self.robot.isEnabled():
            return

        drivetrain = self.robot.robot_container.vroomvroom
        speeds = drivetrain.last_speeds

        self.heading = self.heading + Rotation2d(speeds.omega * tm_diff)
        self.x += speeds.vx * tm_diff
        self.y += speeds.vy * tm_diff

        drivetrain.reset_pose(Pose2d(self.x, self.y, self.heading))
