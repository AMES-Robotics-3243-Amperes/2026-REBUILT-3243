package frc.robot.constants.swerve;

import frc.robot.util.ControlConstantsBuilder;

public class SimConstants {
  public static final ControlConstantsBuilder simDriveControl =
      ControlConstantsBuilder.fromRadiansAndSeconds().sva(0.022251, 0.12438, 0.0091998);

  public static final ControlConstantsBuilder simSteerControl = ModuleConstants.steerControl;
}
