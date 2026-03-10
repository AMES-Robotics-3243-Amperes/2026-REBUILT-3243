package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.ControlConstantsBuilder;

public class IntakeConstants {
  // roller
  public static final int rollerId = 2;
  public static final double rollerReduction = 2.0;
  public static final Distance rollerRadius = Inches.of(1);

  public static final ControlConstantsBuilder rollerControl =
      ControlConstantsBuilder.fromRadiansAndSeconds()
          .pid(0.0033585, 0, 0)
          .sva(0.48225, 0.036694, 0.0036671);

  public static final AngularVelocity rollerIntakeSpeed = RotationsPerSecond.of(30);
  public static final AngularVelocity rollerAgitateSpeed = RotationsPerSecond.of(20);

  // pivot
  public static final int pivotId = 3;

  public static final double pivotReduction = (54.0 / 18.0) * (34.0 / 16.0) * (16.0 / 1.0);

  public static final int pivotCurrentLimit = 40;

  public static final Angle pivotMinRotation = Degrees.of(-1.3476);
  public static final Angle pivotMaxRotation = Degrees.of(130 - 1.3476);

  public static final AngularVelocity pivotVelocityToleranceBeforeStop = DegreesPerSecond.of(2);

  public static final Voltage pivotOpenLoopVolts = Volts.of(4);
}
