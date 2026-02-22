package frc.robot.constants.swerve;

import frc.robot.util.ControlConstantsBuilder;

public class SimConstants {
  public static final ControlConstantsBuilder simDriveControl =
      ControlConstantsBuilder.fromRadiansAndSeconds().sva(0.31453, 0.12402, 0.0077277);
}
