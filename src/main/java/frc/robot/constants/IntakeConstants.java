package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.util.ControlConstantsBuilder;

public class IntakeConstants {
  // agitating
  public static final Time timeBeforeAgitating = Seconds.of(1.2);
  public static final Time timeSpentRunningRollers = Seconds.of(2);

  // roller
  public static final int rollerId = 2;
  public static final double rollerReduction = 2.0;
  public static final Distance rollerRadius = Inches.of(1);

  public static final ControlConstantsBuilder rollerControl =
      ControlConstantsBuilder.fromRadiansAndSeconds()
          .pid(0.0033585, 0, 0)
          .sva(0.48225, 0.036694, 0.0036671);

  public static final AngularVelocity rollerIntakeSpeed = RotationsPerSecond.of(32);
  public static final AngularVelocity rollerAgitateSpeed = RotationsPerSecond.of(20);

  // pivot
  public static final int pivotId = 3;

  public static final ControlConstantsBuilder pivotControl =
      ControlConstantsBuilder.fromRadiansAndSeconds().pid(0.01, 0, 0);

  public static final double pivotReduction = (54.0 / 18.0) * (34.0 / 16.0) * (16.0 / 1.0);

  public static final int pivotCurrentLimit = 40;

  public static final Angle pivotMinRotation = Degrees.of(0);
  public static final Angle pivotMaxRotation = Degrees.of(110);

  public static final Angle pivotAbsoluteEncoderZeroAngle = Degrees.of(0);
  public static final Angle pivotBacklash = Degrees.of(20);

  public static final Angle pivotPositioningTolerance = Degrees.of(5);

  /** If we've been controlling the pivot for this long and it hasn't reached the top, give up. */
  // TODO: we want to rely exclusively on the absolute encoder most of the time. make a secondary
  // driver bind to enable this
  public static final Time pivotControlSafety = Seconds.of(5);

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
