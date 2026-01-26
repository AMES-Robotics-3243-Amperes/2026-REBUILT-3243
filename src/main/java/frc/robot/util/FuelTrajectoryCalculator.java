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
  /* A setpoint for the shooter to follow. */
  public record ShooterSetpoint(LinearVelocity linearFlywheelSpeed, Angle hoodAngle) {}

  /* A setpoint for the robot to follow. Most important is the shooter setpoint, but also includes the field-relative rotation the fuel must be shot from. */
  public record FuelShotSetpoints(
      ShooterSetpoint shooterSetpoint, Rotation2d fuelGroundSpeedRotation) {}

  /* A shooter setpoint along with other information about the trajectory used in the calculation. */
  private record FuelTrajectory(ShooterSetpoint shooterSetpoint, Time fuelTravelTime) {}

  private static FuelTrajectory calculateFuelTrajectory(Pose2d robotPose) {
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
        (shooterToHub.getZ() - b * horizontalDistanceToHub)
            / (horizontalDistanceToHub * horizontalDistanceToHub);

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
    return new FuelTrajectory(
        new ShooterSetpoint(fuelSpeed, Degrees.of(90).minus(fuelAngle)),
        Meters.of(horizontalDistanceToHub).div(fuelSpeed.times(fuelAngleCos)));
  }

  public static FuelShotSetpoints getFuelShot(Pose2d robotPose, ChassisSpeeds chassisSpeeds) {
    // we have a way to calculate the necessary trajectory for a stationary robot at a certain
    // point, but now we want to compensate for a moving robot. the impact of a moving robot on the
    // fuel is more or less uniformly adding the robot's speed to the fuel's ground speed throughout
    // the duration of the launch. thus, if we know how long the fuel will be in the air, we just
    // look at how far the robot will travel in that time and pretend we're shooting from the
    // adjusted position. however, once we calculate the trajectory from the adjusted position, the
    // new trajectory the robot shoots the fuel at may have a different length. the fuel may
    // over/undershoot the hub depending on if the new trajectory is longer or shorter. in essence,
    // we're looking for a lookahead time t such that the length of the trajectory generated with
    // that lookahead time is equal to the lookahead time. all of that is to say that all we do here
    // is run the secant method to find the zero of timeInAir(lookaheadTime) - lookaheadTime.

    // here, t0 is always just t_(k - 2) and t1 is always t_(k - 1)
    double lookahead0 = 0;
    FuelTrajectory trajectory0 = calculateFuelTrajectory(robotPose);

    // we'll need this to build find the rotation setpoint at the end, so we store it for use after
    // the loop
    Pose2d adjustedShotStart =
        robotPose.plus(
            new Transform2d(
                chassisSpeeds.vxMetersPerSecond * trajectory0.fuelTravelTime().in(Seconds),
                chassisSpeeds.vyMetersPerSecond * trajectory0.fuelTravelTime().in(Seconds),
                new Rotation2d()));

    double lookahead1 = trajectory0.fuelTravelTime().in(Seconds);
    FuelTrajectory trajectory1 = calculateFuelTrajectory(adjustedShotStart);

    // secant method iteration
    for (int i = 0; i < ShooterConstants.secantMethodIterations; i++) {
      if (Math.abs(lookahead1 - lookahead0) < 1e-5) break;

      // here, f(t) = length(trajectory(lookaheadPosition(t))) - t
      double newLookahead =
          (lookahead0 * (trajectory1.fuelTravelTime().in(Seconds) - lookahead1)
                  - lookahead1 * (trajectory0.fuelTravelTime().in(Seconds) - lookahead0))
              / (trajectory1.fuelTravelTime().in(Seconds)
                  - trajectory0.fuelTravelTime().in(Seconds)
                  + lookahead0
                  - lookahead1);

      // update values for next iteration
      adjustedShotStart =
          robotPose.plus(
              new Transform2d(
                  chassisSpeeds.vxMetersPerSecond * newLookahead,
                  chassisSpeeds.vyMetersPerSecond * newLookahead,
                  new Rotation2d()));

      lookahead0 = lookahead1;
      trajectory0 = trajectory1;

      lookahead1 = newLookahead;
      trajectory1 = calculateFuelTrajectory(adjustedShotStart);
    }

    return new FuelShotSetpoints(
        trajectory1.shooterSetpoint(),
        PointOfInterestManager.flipTranslation(BluePointsOfInterest.hubPosition)
            .toTranslation2d()
            .minus(adjustedShotStart.getTranslation())
            .getAngle()
            .rotateBy(ShooterConstants.robotToShooter.getRotation().toRotation2d().unaryMinus()));
  }
}
