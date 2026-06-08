from typing import Optional

from commands2 import Subsystem
from phoenix6.orchestra import Orchestra
from wpilib import SendableChooser, SmartDashboard
from pykit.logger import Logger

from utils import log


class OrchestraSubsystem(Subsystem):
    """
    Orchestra Subsystem.

    Plays music on the robot's Phoenix 6 (TalonFX/Kraken) motors using the Phoenix 6
    Orchestra library. Hand it any subsystems that own TalonFX motors; each one that
    exposes a ``getMotors()`` iterator has its motors registered as instruments. Any
    argument that is ``None`` or does not provide ``getMotors()`` is skipped, so this
    subsystem never depends on a particular subsystem existing on the robot.

    This is a single-instance class. Constructing it a second time raises a
    RuntimeError; use ``OrchestraSubsystem.getInstance()`` to reach the existing one.

    :param motorSubsystems: Any subsystems whose Phoenix 6 motors should join the
        orchestra. Each is used only if it exposes ``getMotors()``.
    """

    _instance: "OrchestraSubsystem | None" = None

    def __init__(self, *motorSubsystems):
        if OrchestraSubsystem._instance is not None:
            raise RuntimeError(
                "OrchestraSubsystem is a single-instance class but was constructed twice. "
                "Use OrchestraSubsystem.getInstance() to reuse the existing instance."
            )

        super().__init__()

        self._orchestra = Orchestra()
        self._current_song: Optional[str] = None
        self._championship_mode = False
        self._championship_song_path = (
            "/home/lvuser/py/deploy/files/WinnerSong.chrp"
        )

        # Register motors from any provided subsystem that exposes getMotors().
        for subsystem in motorSubsystems:
            if subsystem is None or not hasattr(subsystem, "getMotors"):
                continue
            for motor in subsystem.getMotors():
                self._orchestra.add_instrument(motor)

        # Song Chooser (kept — Elastic needs this as a NT widget)
        self._song_chooser = SendableChooser()
        self._song_chooser.setDefaultOption(
            "Yes And? - Ariana Grande",
            "/home/lvuser/py/deploy/files/Yesand.chrp"
        )
        self._song_chooser.addOption(
            "Espresso - Sabrina Carpenter",
            "/home/lvuser/py/deploy/files/Espresso.chrp"
        )
        self._song_chooser.addOption(
            "Needy - Ariana Grande",
            "/home/lvuser/py/deploy/files/Needy.chrp"
        )
        self._song_chooser.addOption(
            "Dandelion - Ariana Grande",
            "/home/lvuser/py/deploy/files/Dandelion.chrp"
        )
        self._song_chooser.addOption(
            "When Did You Get Hot - Sabrina Carpenter",
            "/home/lvuser/py/deploy/files/WhenDidYouGetHot.chrp"
        )
        self._song_chooser.addOption(
            "Tití Me Preguntó - Bad Bunny",
            "/home/lvuser/py/deploy/files/TitiMePregunto.chrp"
        )
        self._song_chooser.addOption(
            "Stateside - PinkPantheress",
            "/home/lvuser/py/deploy/files/Stateside.chrp"
        )
        self._song_chooser.addOption(
            "Despacito - Luis Fonsi",
            "/home/lvuser/py/deploy/files/Despacito.chrp"
        )
        SmartDashboard.putData("Song Selection", self._song_chooser)

        OrchestraSubsystem._instance = self

    @classmethod
    def getInstance(cls) -> "OrchestraSubsystem":
        if cls._instance is None:
            raise RuntimeError("OrchestraSubsystem has not been constructed yet.")
        return cls._instance

    def periodic(self):
        Logger.recordOutput("Orchestra/IsPlaying", self._orchestra.is_playing())
        Logger.recordOutput("Orchestra/CurrentSong", self._current_song or "")
        Logger.recordOutput("Orchestra/ChampionshipMode", self._championship_mode)

    # Public API

    def play_selected_song(self):
        path = self._song_chooser.getSelected()

        if path is None:
            return

        if self._current_song != path:
            self._orchestra.load_music(path)
            self._current_song = path

        if not self._orchestra.is_playing():
            self._orchestra.play()

    def play_championship_song(self):
        if not self._championship_mode:
            log("Orchestra", "Championship mode not enabled.")
            return

        path = self._championship_song_path

        if self._current_song != path:
            self._orchestra.load_music(path)
            self._current_song = path

        if not self._orchestra.is_playing():
            self._orchestra.play()

    def stop(self):
        self._orchestra.stop()

    def is_playing(self) -> bool:
        return self._orchestra.is_playing()
