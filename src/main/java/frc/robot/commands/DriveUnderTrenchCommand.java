package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.swerve.SwerveConstants;
import frc.robot.util.PointOfInterestManager;
import frc.robot.util.PointOfInterestManager.FlipType;
import frc.robot.util.RobotLocationManager.RobotLocation;
import java.util.EnumSet;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

public class DriveUnderTrenchCommand {
  // private Command buildConfidenceGatedAuto(
  //       String autoName, Pose2d stagingPose, double requiredConfidence) {

  //     PathConstraints stageConstraints =
  //         new PathConstraints(
  //             MetersPerSecond.of(1),
  //             MetersPerSecondPerSecond.of(4),
  //             RotationsPerSecond.of(0.5),
  //             RotationsPerSecondPerSecond.of(2));

  //     Command runAuto = AutoBuilder.buildAuto(autoName);

  //     Command gate =
  //         AutoBuilder.pathfindToPose(stagingPose, stageConstraints)
  //             .andThen(
  //                 Commands.waitUntil(() -> drivetrain.getPoseConfidence() >=
  // requiredConfidence));

  //     return Commands.either(
  //         runAuto, gate.andThen(runAuto), () -> drivetrain.getPoseConfidence() >=
  // requiredConfidence);
  //   }

  public enum TrenchType {
    BLUE_TOP(FieldConstants.topTrench),
    BLUE_BOTTOM(FieldConstants.bottomTrench),
    RED_TOP(PointOfInterestManager.flipRectangle(FieldConstants.topTrench, FlipType.REFLECT_X)),
    RED_BOTTOM(
        PointOfInterestManager.flipRectangle(FieldConstants.bottomTrench, FlipType.REFLECT_X));

    public Rectangle2d position;

    private TrenchType(Rectangle2d position) {
      this.position = position;
    }
  }

  public static Command driveUnderNearestTrenchCommand(
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ChassisSpeeds> chassisSpeedsSupplier,
      Supplier<RobotLocation> robotLocationSupplier) {
    return Commands.deferredProxy(
        () -> {
          TrenchType closestTrench =
              nearestTrench(robotPoseSupplier.get().getTranslation()).getFirst();
          Translation2d trenchCenter = closestTrench.position.getCenter().getTranslation();
          RobotLocation robotLocation = robotLocationSupplier.get();

          boolean blueAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
          boolean neutralIntoRed =
              robotLocation == RobotLocation.NEUTRAL_ZONE
                  && (closestTrench == TrenchType.RED_BOTTOM
                      || closestTrench == TrenchType.RED_TOP);
          boolean blueIntoNeutral =
              (robotLocation == RobotLocation.ALLIANCE_ZONE && blueAlliance)
                  || (robotLocation == RobotLocation.OPPONENT_ZONE && !blueAlliance);
          boolean traversingInPositiveDirection = neutralIntoRed || blueIntoNeutral;

          // TODO: for safety reasons, return a void command if the robot location indicates we're
          // on the blue side but the nearest trench is red and vice versa

          // build the paths. the reason we use two paths is so that the robot is fully rotated (and
          // the hopper is fully protected) before traversal.
          // TODO: if the secondary driver indicates it's okay, don't worry about protecting the
          // hopper
          Pose2d robotPose = robotPoseSupplier.get();
          ChassisSpeeds chassisSpeeds = chassisSpeedsSupplier.get();

          Pose2d startTrenchPose =
              new Pose2d(
                  trenchCenter.getX()
                      + (traversingInPositiveDirection ? -1.0 : 1.0)
                          * SwerveConstants.trenchTraversalOffset.in(Meters),
                  trenchCenter.getY(),
                  traversingInPositiveDirection ? Rotation2d.kZero : Rotation2d.k180deg);

          Pose2d endTrenchPose =
              new Pose2d(
                  trenchCenter.getX()
                      + (traversingInPositiveDirection ? 1.0 : -1.0)
                          * SwerveConstants.trenchTraversalOffset.in(Meters),
                  trenchCenter.getY(),
                  traversingInPositiveDirection ? Rotation2d.kZero : Rotation2d.k180deg);

          Rotation2d initialDirection =
              Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond) > 1e-1
                  ? new Translation2d(
                          chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
                      .getAngle()
                      .rotateBy(robotPose.getRotation())
                  : startTrenchPose.getTranslation().minus(robotPose.getTranslation()).getAngle();

          List<Waypoint> toTrenchWaypoints =
              PathPlannerPath.waypointsFromPoses(
                  new Pose2d(robotPose.getX(), robotPose.getY(), initialDirection),
                  startTrenchPose);

          PathPlannerPath toTrenchPath =
              new PathPlannerPath(
                  toTrenchWaypoints,
                  SwerveConstants.automaticsConstraints,
                  null,
                  new GoalEndState(
                      SwerveConstants.automaticsConstraints.maxVelocityMPS(),
                      traversingInPositiveDirection ? Rotation2d.k180deg : Rotation2d.kZero));
          toTrenchPath.preventFlipping = true;

          List<Waypoint> underTrenchWaypoints =
              PathPlannerPath.waypointsFromPoses(startTrenchPose, endTrenchPose);

          PathPlannerPath underTrenchPath =
              new PathPlannerPath(
                  underTrenchWaypoints,
                  SwerveConstants.automaticsConstraints,
                  null,
                  new GoalEndState(
                      SwerveConstants.automaticsConstraints.maxVelocityMPS(),
                      traversingInPositiveDirection ? Rotation2d.k180deg : Rotation2d.kZero));
          underTrenchPath.preventFlipping = true;

          return AutoBuilder.followPath(toTrenchPath)
              .andThen(AutoBuilder.followPath(underTrenchPath));
        });
  }

  /** Gets a pair of the nearest trench as to the robot as well as the distance between them. */
  public static Pair<TrenchType, Distance> nearestTrench(Translation2d robotPosition) {
    TrenchType closestTrench = TrenchType.BLUE_BOTTOM;
    Distance closestDistance = Meters.of(100);

    Set<TrenchType> trenches = EnumSet.allOf(TrenchType.class);
    for (TrenchType trench : trenches) {
      Distance distanceToTrench = trench.position.getMeasureDistance(robotPosition);
      if (distanceToTrench.lt(closestDistance)) {
        closestDistance = distanceToTrench;
        closestTrench = trench;
      }
    }

    return Pair.of(closestTrench, closestDistance);
  }
}
