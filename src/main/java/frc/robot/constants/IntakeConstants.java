package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

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

  public static final double pivotReduction = (16.0 / 34.0) * 16.0 * (54.0 / 16.0);

  public static final int pivotCurrentLimit = 30;

  public static final Angle pivotMinRotation = Degrees.of(3.92);
  public static final Angle pivotMaxRotation = Degrees.of(137.35);

  public static final Angle pivotToleranceBeforeCoast = Degrees.of(5);

  public static final ControlConstantsBuilder pivotControl =
      ControlConstantsBuilder.fromRadiansAndSeconds()
          .pid(0, 0, 0)
          .sva(0, 0, 0)
          .g(0, true)
          .constraints(RotationsPerSecond.of(0.2), RotationsPerSecondPerSecond.of(1.0));
}
