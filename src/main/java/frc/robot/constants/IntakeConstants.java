package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.ControlConstantsBuilder;

public class IntakeConstants {
  // roller
  public static final int rollerId = 2;
  public static final double rollerReduction = 2.0;
  public static final Distance rollerRadius = Inches.of(1);

  public static final ControlConstantsBuilder rollerControl =
      ControlConstantsBuilder.fromRadiansAndSeconds()
          .pid(0.000002, 0, 0)
          .sva(0.30224, 0.039192, 0.0051602);

  public static final AngularVelocity rollerAbsoluteSpeed = RotationsPerSecond.of(20);
  public static final Angle pivotToleranceBeforeRollersEngage = Degrees.of(5);

  // pivot
  public static final int pivotId = 3;

  public static final double pivotReduction = 1.0;

  public static final Angle pivotMinRotation = Degrees.of(0);
  public static final Angle pivotMaxRotation = Degrees.of(110);

  public static final ControlConstantsBuilder pivotControl =
      ControlConstantsBuilder.fromRadiansAndSeconds().pid(0.00001, 0, 0).sva(0, 0, 0);
}
