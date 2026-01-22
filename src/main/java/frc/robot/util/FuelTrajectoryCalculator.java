package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.setpoints.BluePointsOfInterest;

public class FuelTrajectoryCalculator {
  public record ShooterSetpoint(
      LinearVelocity linearFlywheelSpeed, Angle hoodAngle, Time fuelTravelTime) {}

  // TODO: add some safety mechanisms for bad inputs, nans, and stuff like that. for example, speed
  // becomes a nan if the shooter points down and a is positive

  // TODO: if the hood angle becomes unachievable, clamp it and calculate the new velocity that
  // still makes it in

  // TODO: use the chassis speeds
  private static ShooterSetpoint calcualteShooterSetpoint(Pose2d robotPose) {
    Translation3d shooterToHub =
        PointOfInterestManager.flipTranslation(BluePointsOfInterest.hubPosition)
            .minus(
                new Pose3d(robotPose)
                    .transformBy(ShooterConstants.robotToShooter)
                    .getTranslation());

    double horizontalDistanceToHub = shooterToHub.toTranslation2d().getNorm();

    // we're going to construct polynomial from three points. the first point is the shooter,
    // centered at the origin. the second is the center of the hub's funnel.
    double x2 =
        Double.max(
            horizontalDistanceToHub - ShooterConstants.extraPointHorizontalOffset.in(Meters),
            horizontalDistanceToHub / 2);
    double y2 = shooterToHub.getZ() + ShooterConstants.extraPointVerticalOffset.in(Meters);

    // find the coefficients of the polynomial (c is 0)
    Matrix<N2, N2> pointMatrix =
        MatBuilder.fill(
            Nat.N2(),
            Nat.N2(),
            x2 * x2,
            x2,
            horizontalDistanceToHub * horizontalDistanceToHub,
            horizontalDistanceToHub);
    Matrix<N2, N1> coeffSolutions = pointMatrix.solve(VecBuilder.fill(y2, shooterToHub.getZ()));
    double a = coeffSolutions.get(0, 0);
    double b = coeffSolutions.get(1, 0);

    // angle from horizon = tan^(-1) (f'(0)) = tan^(-1) (b)
    Angle fuelAngle = Radians.of(Math.atan(b));
    double fuelAngleCos = Math.cos(fuelAngle.in(Radians));

    // if we expand our y(x) polynomial as y(x(t)) = y(t (v cos theta)) and as -1 / 2 g t^2 + t (v
    // sin theta) and match the t^2 coeff we get the linear speed
    double gravity = 9.8;
    LinearVelocity fuelSpeed =
        MetersPerSecond.of(Math.sqrt(-gravity / (2 * a * fuelAngleCos * fuelAngleCos)));

    // the fuel angle is 0 at the horizon but the hood is 0 when shooting upwards
    return new ShooterSetpoint(
        fuelSpeed,
        Degrees.of(90).minus(fuelAngle),
        Meters.of(horizontalDistanceToHub).div(fuelSpeed.times(fuelAngleCos)));
  }

  public static ShooterSetpoint calcualteShooterSetpoint(
      Pose2d robotPose, ChassisSpeeds chassisSpeeds) {
    ShooterSetpoint setpoint = calcualteShooterSetpoint(robotPose);

    for (int i = 0; i < ShooterConstants.extraLookaheadIterations; i++) {
      setpoint =
          calcualteShooterSetpoint(
              robotPose.plus(
                  new Transform2d(
                      chassisSpeeds.vxMetersPerSecond * setpoint.fuelTravelTime.in(Seconds),
                      chassisSpeeds.vyMetersPerSecond * setpoint.fuelTravelTime.in(Seconds),
                      new Rotation2d())));
    }

    return setpoint;
  }
}
