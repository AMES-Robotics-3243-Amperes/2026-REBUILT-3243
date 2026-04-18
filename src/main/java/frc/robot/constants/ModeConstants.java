package frc.robot.constants;

import edu.wpi.first.wpilibj.RobotBase;

public class ModeConstants {
  public enum Mode {
    REAL_COMPETITION,
    REAL,
    SIM,
    REPLAY
  }

  public static final Mode realMode = Mode.REAL;
  public static final Mode simMode = Mode.SIM;

  public static final Mode robotMode = RobotBase.isReal() ? realMode : simMode;
}
