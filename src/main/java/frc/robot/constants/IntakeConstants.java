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

  public static final AngularVelocity rollerIntakeSpeed = RotationsPerSecond.of(20);
  public static final Angle pivotToleranceBeforeRollersEngage = Degrees.of(5);

  // pivot
  public static final int pivotId = 3;

  public static final double pivotReduction = (54.0 / 18.0) * (34.0 / 16.0) * (16.0 / 1.0);

  public static final int pivotCurrentLimit = 30;

  public static final Angle pivotMinRotation = Degrees.of(-1.3476);
  public static final Angle pivotMaxRotation = Degrees.of(130 - 1.3476);

  public static final Angle pivotToleranceBeforeCoast = Degrees.of(7);

  public static final ControlConstantsBuilder pivotControl =
      ControlConstantsBuilder.fromRadiansAndSeconds()
          .pid(10, 0, 0)
          .sva(0, 0.6, 0.08)
          .g(0.1, true)
          .constraints(RotationsPerSecond.of(3), RotationsPerSecondPerSecond.of(8));
}
