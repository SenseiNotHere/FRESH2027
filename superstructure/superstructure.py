from wpilib import SmartDashboard, Timer
from commands2 import FunctionalCommand
from commands2.button import CommandGenericHID

from subsystems.orchestra.orchestra_subsystem import OrchestraSubsystem
from superstructure import SuperstructureStates, SuperstructureHelpers, RobotState, RobotReadiness, AuxiliaryActions
from subsystems import DriveSubsystem


class Superstructure(SuperstructureStates, SuperstructureHelpers):
    _instance = None

    def __init__(
            self,
            drivetrain: DriveSubsystem | None = None,
            orchestra: OrchestraSubsystem | None = None,
            driverController: CommandGenericHID | None = None,
            operatorController: CommandGenericHID | None = None,
    ):
        """
        Superstructure.

        The Superstructure is the central coordination layer of the robot. It manages
        high-level robot states and orchestrates interactions between subsystems such
        as the drivetrain, intake, shooter, indexer, and agitator.

        Instead of subsystems directly controlling each other, the Superstructure
        defines robot-wide states (RobotState) and executes the appropriate subsystem
        logic for each state. This keeps subsystem logic isolated while allowing the
        robot to perform coordinated actions such as intaking, preparing a shot, and
        shooting.

        The Superstructure also tracks robot readiness conditions (RobotReadiness)
        which are used to determine when actions are safe to perform.

        This class is implemented as a single-instance subsystem and should be the
        final subsystem initialized in `RobotContainer`. Its update loop must be called
        periodically from `robotPeriodic()` to process state handlers and readiness
        logic.

        :param drivetrain: Drivetrain subsystem responsible for robot movement.
        :param orchestra: Orchestra subsystem responsible for music playback.
        :param driverController: Driver controller used.
        :param operatorController: Operator controller used.
        """

        # Single instance
        if Superstructure._instance is not None:
            raise RuntimeError("Only one instance of Superstructure is allowed.")
        Superstructure._instance = self
        
        # Subsystems
        self.drivetrain = drivetrain
        self.orchestra = orchestra
        self.driverController = driverController
        self.operatorController = operatorController
        
        # Availability Flags
        self.hasOrchestra = self.orchestra is not None
        self.hasDriverController = self.driverController is not None
        self.hasOperatorController = self.operatorController is not None
        
        # State Tracking
        self.robot_state = RobotState.IDLE
        self.robot_readiness = RobotReadiness()
        self.auxiliary_actions = AuxiliaryActions()
        
        self._state_handlers = {
            # General States
            RobotState.IDLE: self._handle_idle,
            RobotState.PLAYING_SONG: self._handle_playing_song,
            RobotState.PLAYING_CHAMPIONSHIP_SONG: self._handle_playing_championship_song,
        }

        # Auxiliary Actions
        self.auxiliary_actions.update()
        
        # Timers
        self._state_start_time = Timer.getFPGATimestamp()

    def update(self):
        """
        Superstructure update loop.
        Should be called periodically to update subsystem states.

        Called by robotPeriodic() in robot.py
        """

        SmartDashboard.putString("Superstructure/State", self.robot_state.name)

        self._update_readiness()

        handler = self._state_handlers.get(self.robot_state)
        if handler:
            handler()

    def _update_readiness(self):
        pass

    # Public Superstructure API
    def createStateCommand(self, state: RobotState, finishImmediately=False):
        """
        Creates a command that sets the robot state to the specified state.
        """

        def on_end(interrupted):
            if self.robot_state == state:
                self.setState(RobotState.IDLE)

        return FunctionalCommand(
            onInit=lambda: self.setState(state),
            onExecute=lambda: None,
            onEnd=on_end,
            isFinished=lambda: finishImmediately
        )

    def autoCreateStateCommand(self, state: RobotState):
        def on_init():
            print(f"AUTO COMMAND TRIGGERED -> {state}")
            self.setState(state)

        return FunctionalCommand(
            onInit=on_init,
            onExecute=lambda: None,
            onEnd=lambda interrupted: None,
            isFinished=lambda: True,
        )

    def setState(self, newState: RobotState, force: bool = False):
        """
        Sets the robot state to the specified state.

        To be used inside superstructure.py.

        :newState: The new state to set (RobotState enum).
        :force: Whether to force the state change even if it's the same as the current state. (Great for restarting a state)
        """
        if not force and newState == self.robot_state:
            return

        oldState = self.robot_state
        self.robot_state = newState

        # Timer
        self._state_start_time = Timer.getFPGATimestamp()

        print(f"[Superstructure] {oldState.name} -> {newState.name}")

        SmartDashboard.putString("Superstructure/State", newState.name)

    # Public General API
    def getState(self):
        """
        :return: The current robot state.
        """
        return self.robot_state