package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.util.ControlConstantsBuilder;

public class IntakeConstants {
  // agitating
  public static final Time timeBeforeAgitating = Seconds.of(1.2);
  public static final Time timeSpentRunningRollers = Seconds.of(4);

  // roller
  public static final int rollerId = 2;
  public static final double rollerReduction = 2.0;
  public static final Distance rollerRadius = Inches.of(1);

  public static final ControlConstantsBuilder rollerControl =
      ControlConstantsBuilder.fromRadiansAndSeconds()
          .pid(0.0006, 0, 0)
          .sva(0.19678, 0.035458, 0.0036986);

  public static final int rollerStallLimit = 65;
  public static final int rollerFreeLimit = 30;

  public static final AngularVelocity rollerIntakeSpeed = RPM.of(2200);
  public static final AngularVelocity rollerAgitateSpeed = RPM.of(1500);

  // pivot
  public static final int pivotId = 3;

  public static final ControlConstantsBuilder pivotVelocityControl =
      ControlConstantsBuilder.fromRadiansAndSeconds().pid(1, 0, 0).sva(0.6, 0, 0).g(0.98, true);
  public static final ControlConstantsBuilder pivotPositionControl =
      ControlConstantsBuilder.fromRadiansAndSeconds().pid(1.6, 0, 0);

  public static final double pivotReduction = 20.0;

  public static final int pivotCurrentLimit = 40;

  public static final Angle pivotMinRotation = Degrees.of(0);
  public static final Angle pivotRaiseTargetAngle = Degrees.of(105);
  public static final Angle pivotMaxRotation = Degrees.of(130);

  public static final Angle pivotAbsoluteEncoderZeroAngle = Degrees.of(0);
  public static final Angle pivotBacklash = Degrees.of(20);

  public static final Angle pivotPositioningTolerance = Degrees.of(3);
  public static final Angle pivotIntakeTolerance = Degrees.of(8);

  /**
   * Where the "seam" of the pivot's rotation is. If the angle passes this value (which it never
   * ever ever should), it will appear as if it randomly jumps backwards a full rotation.
   */
  public static final Angle pivotAngleWrap = Degrees.of(220);

  /**
   * This is used for keeping track of backlash. If the intake is pivoting at least as fast as the
   * rezro speed (as reported by the absolute encoder) and fighting against gravity with the
   * specified tolerance, we assume we're at the end of the backlash and can re-zero.
   */
  public static final AngularVelocity pivotAbsoluteVelocityForRezero = DegreesPerSecond.of(8);

  public static final Angle pivotAbsolutePositionToleranceForRezero = Degrees.of(20);
}
