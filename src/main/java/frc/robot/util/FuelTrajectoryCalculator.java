package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.util.PointOfInterestManager.FlipType;
import java.util.function.Supplier;

public class FuelTrajectoryCalculator {
  /* The data behind a fuel shot; i.e. the shooter setpoint values. */
  public record FuelShot(LinearVelocity linearFlywheelSpeed, Angle hoodAngle, Time shotTime) {}

  /* A setpoint for the robot to follow. Most important is the actual shot, but also includes the field-relative rotation the fuel must be shot from. */
  public record FuelShotSetpoints(FuelShot shooterSetpoint, Rotation2d fuelGroundSpeedRotation) {}

  private static FuelShot calculateFuelTrajectory(Translation3d goal, Translation3d shooterStart) {
    double horizontalDistanceToSetpoint =
        shooterStart.toTranslation2d().getDistance(goal.toTranslation2d());
    double verticalDistanceToSetpoint = goal.getZ() - shooterStart.getZ();

    // we're going to construct polynomial from three points. the first point is the shooter,
    // centered at the origin. the second is the center of the hub's funnel.
    double x2 =
        Double.max(
            horizontalDistanceToSetpoint - ShooterConstants.extraPointHorizontalOffset.in(Meters),
            horizontalDistanceToSetpoint / 2);
    double y2 = verticalDistanceToSetpoint + ShooterConstants.extraPointVerticalOffset.in(Meters);

    // find the coefficients of the polynomial (c is 0)
    Matrix<N2, N2> pointMatrix =
        MatBuilder.fill(
            Nat.N2(),
            Nat.N2(),
            x2 * x2,
            x2,
            horizontalDistanceToSetpoint * horizontalDistanceToSetpoint,
            horizontalDistanceToSetpoint);
    Matrix<N2, N1> coeffSolutions =
        pointMatrix.solve(VecBuilder.fill(y2, verticalDistanceToSetpoint));
    double a = coeffSolutions.get(0, 0);
    double b = coeffSolutions.get(1, 0);

    // angle from horizon = tan^(-1) (f'(0)) = tan^(-1) (b). the clamping is because the fuel angle
    // is 0 at the horizon but the hood is 0 when shooting upwards
    Angle fuelAngle =
        Radians.of(
            MathUtil.clamp(
                Math.atan(b),
                Degrees.of(90).minus(ShooterConstants.hoodMaxRotation).in(Radians),
                Degrees.of(90).minus(ShooterConstants.hoodMinRotation).in(Radians)));

    // the angle may have been clamped, so regenerate the polynomial. if we expand our y(x)
    // polynomial as y(x(t)) = y(t (v cos theta)) and as -1 / 2 g t^2 + t (v sin theta) and match
    // the t coefficients we get the new b value. the new a value comes from fitting to the goal
    // spot with known b and c values.
    b = Math.tan(fuelAngle.in(Radians));
    a =
        (verticalDistanceToSetpoint - b * horizontalDistanceToSetpoint)
            / (horizontalDistanceToSetpoint * horizontalDistanceToSetpoint);

    // if a > 0, the hood's angle was clamped too low and it's impossible to make the shot. we'll
    // follow through with the calculations with very very small a (i.e. the setpoint is more or
    // less a straight line from the shooter)
    a = Double.min(a, -1e-5);

    // now that we have the final polynomial that our fuel will follow, we just coefficient match
    // the t^2 term to get the speed
    double gravity = 9.8;
    double fuelAngleCos = Math.cos(fuelAngle.in(Radians));

    LinearVelocity fuelSpeed = MetersPerSecond.of(Math.sqrt(-gravity / (2 * a)) / fuelAngleCos);

    // the fuel angle is 0 at the horizon but the hood is 0 when shooting upwards, hence the 90 -
    // fuelAngle
    return new FuelShot(
        fuelSpeed.times(1.05),
        Degrees.of(90).minus(fuelAngle),
        Meters.of(horizontalDistanceToSetpoint).div(fuelSpeed.times(fuelAngleCos)));
  }

  private static Pose3d getShooterPositionAndRotationFromRobotPosition(
      Translation3d goal, Translation2d robotPosition) {
    // note that this is NOT correct when the shooter doesn't point across/away from the center of
    // the robot. ours doesn't though, so...
    Rotation3d goalRobotRotation =
        new Rotation3d(
            goal.toTranslation2d()
                .minus(robotPosition)
                .getAngle()
                .minus(ShooterConstants.robotToShooter.getRotation().toRotation2d()));
    Pose3d robotPoseWithCorrectRotation =
        new Pose3d(new Translation3d(robotPosition), goalRobotRotation);
    return robotPoseWithCorrectRotation.transformBy(ShooterConstants.robotToShooter);
  }

  private static FuelShotSetpoints getFuelShot(
      Translation3d goal, Pose2d robotPose, ChassisSpeeds chassisSpeeds) {
    // the approach here is to run secant method on trajectoryTime(lookaheadTime) - lookaheadTime

    // we'll need this to build find the rotation setpoint at the end, so we store it for use after
    // the loop
    Pose3d shotStart =
        getShooterPositionAndRotationFromRobotPosition(goal, robotPose.getTranslation());

    // here, t0 is always just t_(k - 2) and t1 is always t_(k - 1)
    double lookahead0 = 0;
    FuelShot trajectory0 = calculateFuelTrajectory(goal, shotStart.getTranslation());

    shotStart =
        getShooterPositionAndRotationFromRobotPosition(
            goal,
            robotPose
                .transformBy(
                    new Transform2d(
                        chassisSpeeds.vxMetersPerSecond * trajectory0.shotTime().in(Seconds),
                        chassisSpeeds.vyMetersPerSecond * trajectory0.shotTime().in(Seconds),
                        new Rotation2d()))
                .getTranslation());

    double lookahead1 = trajectory0.shotTime().in(Seconds);
    FuelShot trajectory1 = calculateFuelTrajectory(goal, shotStart.getTranslation());

    // secant method iteration
    for (int i = 0; i < ShooterConstants.secantMethodIterations; i++) {
      if (Math.abs(lookahead1 - lookahead0) < 1e-5) break;

      // here, f(t) = length(trajectory(lookaheadPosition(t))) - t
      double newLookahead =
          (lookahead0 * (trajectory1.shotTime().in(Seconds) - lookahead1)
                  - lookahead1 * (trajectory0.shotTime().in(Seconds) - lookahead0))
              / (trajectory1.shotTime().in(Seconds)
                  - trajectory0.shotTime().in(Seconds)
                  + lookahead0
                  - lookahead1);

      // update values for next iteration
      shotStart =
          getShooterPositionAndRotationFromRobotPosition(
              goal,
              robotPose
                  .plus(
                      new Transform2d(
                          chassisSpeeds.vxMetersPerSecond * newLookahead,
                          chassisSpeeds.vyMetersPerSecond * newLookahead,
                          new Rotation2d()))
                  .getTranslation());

      lookahead0 = lookahead1;
      trajectory0 = trajectory1;

      lookahead1 = newLookahead;
      trajectory1 = calculateFuelTrajectory(goal, shotStart.getTranslation());
    }

    return new FuelShotSetpoints(trajectory1, shotStart.getRotation().toRotation2d());
  }

  //
  // Caching
  //
  private static FuelShotSetpoints hubShot = null;
  private static FuelShotSetpoints allianceZoneShot = null;
  private static FuelShotSetpoints neutralZoneShot = null;

  public static Supplier<Pose2d> robotPose = () -> new Pose2d();
  public static Supplier<ChassisSpeeds> robotSpeeds = () -> new ChassisSpeeds();

  public static void clearSavedShots() {
    hubShot = null;
    allianceZoneShot = null;
    neutralZoneShot = null;
  }

  public static FuelShotSetpoints getHubShot() {
    if (hubShot == null)
      hubShot =
          getFuelShot(
              PointOfInterestManager.flipTranslationConditionally(
                  FieldConstants.hubPosition, FlipType.REFLECT_FOR_OTHER_ALLIANCE),
              robotPose.get(),
              robotSpeeds.get());

    return hubShot;
  }

  public static FuelShotSetpoints getAllianceShot() {
    if (allianceZoneShot == null) {
      Translation2d translation =
          PointOfInterestManager.flipTranslationConditionally(
              ShooterConstants.bottomBlueAllianceZoneShot, FlipType.REFLECT_X);
      translation =
          robotPose.get().getY() > FieldConstants.fieldWidth.div(2).in(Meters)
              ? PointOfInterestManager.flipTranslation(translation, FlipType.REFLECT_Y)
              : translation;
      allianceZoneShot =
          getFuelShot(new Translation3d(translation), robotPose.get(), robotSpeeds.get());
    }

    return allianceZoneShot;
  }

  public static FuelShotSetpoints getNeutralShot() {
    if (neutralZoneShot == null) {
      Translation2d translation =
          PointOfInterestManager.flipTranslationConditionally(
              ShooterConstants.bottomBlueNeutralZoneShot, FlipType.REFLECT_X);
      translation =
          robotPose.get().getY() > FieldConstants.fieldWidth.div(2).in(Meters)
              ? PointOfInterestManager.flipTranslation(translation, FlipType.REFLECT_Y)
              : translation;
      neutralZoneShot =
          getFuelShot(new Translation3d(translation), robotPose.get(), robotSpeeds.get());
    }

    return neutralZoneShot;
  }
}
