package frc.robot.state;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.constants.setpoints.ChoreoVars;
import frc.robot.constants.setpoints.FieldConstants;
import frc.robot.util.PointOfInterestManager;
import frc.robot.util.PointOfInterestManager.FlipType;
import java.util.List;
import java.util.Optional;

public record TrenchTraversal(TrenchType type, TraversalType traversal) implements RobotState {
  public enum TrenchType {
    TOP,
    BOTTOM,
  }

  public enum TraversalType {
    BLUE_INTO_NEUTRAL,
    RED_INTO_NEUTRAL,
    NEUTRAL_INTO_BLUE,
    NEUTRAL_INTO_RED
  }

  public static Optional<TrenchTraversal> inferTraversal(Translation2d robotTranslation) {
    double cutoff = 2;

    boolean inNeutralZone =
        FieldConstants.leftTrench.getCenter().getX() <= robotTranslation.getX()
            && robotTranslation.getX()
                <= PointOfInterestManager.flipX(FieldConstants.leftTrench.getCenter().getX());

    if (FieldConstants.leftTrench.getDistance(robotTranslation) < cutoff) {
      return Optional.of(
          inNeutralZone
              ? new TrenchTraversal(TrenchType.TOP, TraversalType.NEUTRAL_INTO_BLUE)
              : new TrenchTraversal(TrenchType.TOP, TraversalType.BLUE_INTO_NEUTRAL));
    }

    if (FieldConstants.rightTrench.getDistance(robotTranslation) < cutoff) {
      return Optional.of(
          inNeutralZone
              ? new TrenchTraversal(TrenchType.BOTTOM, TraversalType.NEUTRAL_INTO_BLUE)
              : new TrenchTraversal(TrenchType.BOTTOM, TraversalType.BLUE_INTO_NEUTRAL));
    }

    if (PointOfInterestManager.flipRectangle(
                FieldConstants.leftTrench, FlipType.REFLECT_ACROSS_WIDTH)
            .getDistance(robotTranslation)
        < cutoff) {
      return Optional.of(
          inNeutralZone
              ? new TrenchTraversal(TrenchType.TOP, TraversalType.NEUTRAL_INTO_RED)
              : new TrenchTraversal(TrenchType.TOP, TraversalType.RED_INTO_NEUTRAL));
    }

    if (PointOfInterestManager.flipRectangle(
                FieldConstants.rightTrench, FlipType.REFLECT_ACROSS_WIDTH)
            .getDistance(robotTranslation)
        < cutoff) {
      return Optional.of(
          inNeutralZone
              ? new TrenchTraversal(TrenchType.BOTTOM, TraversalType.NEUTRAL_INTO_RED)
              : new TrenchTraversal(TrenchType.BOTTOM, TraversalType.RED_INTO_NEUTRAL));
    }

    return Optional.empty();
  }

  public Command getCommand(RobotContainer robotContainer) {
    Pose2d robotPose = robotContainer.drivetrain.getPose();
    ChassisSpeeds robotSpeeds = robotContainer.drivetrain.getChassisSpeeds();
    Transform2d speedsTransform =
        new Transform2d(
            robotSpeeds.vxMetersPerSecond,
            robotSpeeds.vyMetersPerSecond,
            Rotation2d.fromRadians(robotSpeeds.omegaRadiansPerSecond));

    double trenchY =
        switch (this.type) {
          case TOP -> FieldConstants.leftTrench.getCenter().getY();
          case BOTTOM -> FieldConstants.rightTrench.getCenter().getY();
        };

    double enterX =
        switch (this.traversal) {
          case BLUE_INTO_NEUTRAL -> ChoreoVars.TrenchEnterX.in(Meters);
          case NEUTRAL_INTO_BLUE -> ChoreoVars.TrenchExitX.in(Meters);
          case RED_INTO_NEUTRAL -> PointOfInterestManager.flipX(ChoreoVars.TrenchEnterX.in(Meters));
          case NEUTRAL_INTO_RED -> PointOfInterestManager.flipX(ChoreoVars.TrenchExitX.in(Meters));
        };

    double exitX =
        switch (this.traversal) {
          case BLUE_INTO_NEUTRAL -> ChoreoVars.TrenchExitX.in(Meters);
          case NEUTRAL_INTO_BLUE -> ChoreoVars.TrenchEnterX.in(Meters);
          case RED_INTO_NEUTRAL -> PointOfInterestManager.flipX(ChoreoVars.TrenchExitX.in(Meters));
          case NEUTRAL_INTO_RED -> PointOfInterestManager.flipX(ChoreoVars.TrenchEnterX.in(Meters));
        };

    Rotation2d traverseDir =
        switch (this.traversal) {
          case BLUE_INTO_NEUTRAL, NEUTRAL_INTO_RED -> Rotation2d.kZero;
          case RED_INTO_NEUTRAL, NEUTRAL_INTO_BLUE -> Rotation2d.k180deg;
        };

    double rotationMod = MathUtil.angleModulus(robotPose.getRotation().getRadians());
    Rotation2d desiredRotation =
        -Math.PI / 2.0 <= rotationMod && rotationMod < Math.PI / 2.0
            ? Rotation2d.kZero
            : Rotation2d.k180deg;

    double speed = 3;
    List<Waypoint> alignWaypoints =
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(
                robotPose.getTranslation(),
                speedsTransform.getTranslation().getAngle().rotateBy(robotPose.getRotation())),
            new Pose2d(new Translation2d(enterX, trenchY), traverseDir));

    List<Waypoint> travelWaypoints =
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(new Translation2d(enterX, trenchY), traverseDir),
            new Pose2d(new Translation2d(exitX, trenchY), traverseDir));

    PathConstraints constraints = new PathConstraints(speed, speed * 2, 6, 12);

    PathPlannerPath alignPath =
        new PathPlannerPath(
            alignWaypoints, constraints, null, new GoalEndState(speed, desiredRotation));

    PathPlannerPath travelPath =
        new PathPlannerPath(
            travelWaypoints,
            constraints,
            new IdealStartingState(speed, Rotation2d.kZero),
            new GoalEndState(speed, desiredRotation));

    alignPath.preventFlipping = true;
    travelPath.preventFlipping = true;

    return Commands.sequence(AutoBuilder.followPath(alignPath), AutoBuilder.followPath(travelPath));
  }

  public Optional<RobotState> nextState(RobotContainer robotContainer, Command command) {
    double robotX = robotContainer.drivetrain.getPose().getX();
    boolean isFinished =
        switch (this.traversal) {
          case BLUE_INTO_NEUTRAL -> robotX + 1e-1 >= ChoreoVars.TrenchExitX.in(Meters);
          case NEUTRAL_INTO_BLUE -> robotX - 1e-1 <= ChoreoVars.TrenchEnterX.in(Meters);
          case RED_INTO_NEUTRAL ->
              robotX - 1e-1 <= PointOfInterestManager.flipX(ChoreoVars.TrenchExitX.in(Meters));
          case NEUTRAL_INTO_RED ->
              robotX + 1e-1 >= PointOfInterestManager.flipX(ChoreoVars.TrenchEnterX.in(Meters));
        };

    if (isFinished) return Optional.of(new OperatorControlled());
    return Optional.empty();
  }
}
