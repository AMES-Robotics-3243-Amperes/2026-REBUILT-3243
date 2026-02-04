package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.setpoints.ChoreoVars;
import frc.robot.constants.setpoints.FieldConstants;
import frc.robot.constants.swerve.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import java.util.List;

public class RobotStateMachine {
  public enum RobotState {
    OPERATOR_CONTROLLED,
    EXIT_TRENCH_LEFT
  }

  private SwerveSubsystem drivetrain;
  private CommandXboxController joystick;

  private Command activeCommand;
  private RobotState state;

  public RobotStateMachine(SwerveSubsystem drivetrain, CommandXboxController joystick) {
    this.drivetrain = drivetrain;
    this.joystick = joystick;
  }

  private void switchState(RobotState newState) {
    state = newState;

    if (activeCommand != null) activeCommand.cancel();

    activeCommand = getCommand();
    CommandScheduler.getInstance().schedule(activeCommand);
  }

  public Command stateMachineCommand() {
    return Commands.runEnd(this::updateState, this::reset);
  }

  private void updateState() {
    if (state == null) switchState(RobotState.OPERATOR_CONTROLLED);

    switch (state) {
      case OPERATOR_CONTROLLED:
        if (FieldConstants.leftTrench.getDistance(drivetrain.getPose().getTranslation()) < 2.5
            && joystick.rightBumper().getAsBoolean()) switchState(RobotState.EXIT_TRENCH_LEFT);
        break;

      case EXIT_TRENCH_LEFT:
        if (activeCommand.isFinished()) switchState(RobotState.OPERATOR_CONTROLLED);
        break;

      default:
        throw new RuntimeException("Invalid robot state");
    }
  }

  private void reset() {
    if (activeCommand != null) activeCommand.cancel();
    state = null;
  }

  private Command getCommand() {
    Pose2d robotPose = drivetrain.getPose();
    ChassisSpeeds speeds = drivetrain.getChassisSpeeds();
    Transform2d speedsTransform =
        new Transform2d(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            Rotation2d.fromRadians(speeds.omegaRadiansPerSecond));

    switch (state) {
      case OPERATOR_CONTROLLED:
        return drivetrain.driveSetpiontGeneratorCommand(
            drivetrain.joystickDriveLinear(
                SwerveConstants.linearTeleopSpeed,
                joystick::getLeftX,
                joystick::getLeftY,
                () -> true),
            drivetrain.joystickDriveAngular(joystick::getRightX));

      case EXIT_TRENCH_LEFT:
        double mod = 0.3;
        double speed = 3;
        List<Waypoint> alignWaypoints =
            List.of(
                new Waypoint(
                    null,
                    robotPose.getTranslation(),
                    robotPose.transformBy(speedsTransform.times(mod)).getTranslation()),
                new Waypoint(
                    new Translation2d(
                        ChoreoVars.TrenchEnterX.in(Meters) - speed * mod,
                        FieldConstants.leftTrench.getCenter().getY()),
                    new Translation2d(
                        ChoreoVars.TrenchEnterX.in(Meters),
                        FieldConstants.leftTrench.getCenter().getY()),
                    new Translation2d(
                        ChoreoVars.TrenchEnterX.in(Meters) + speed * mod,
                        FieldConstants.leftTrench.getCenter().getY())));

        List<Waypoint> travelWaypoints =
            PathPlannerPath.waypointsFromPoses(
                new Pose2d(
                    new Translation2d(
                        ChoreoVars.TrenchEnterX.in(Meters),
                        FieldConstants.leftTrench.getCenter().getY()),
                    Rotation2d.kZero),
                new Pose2d(
                    new Translation2d(
                        ChoreoVars.TrenchExitX.in(Meters),
                        FieldConstants.leftTrench.getCenter().getY()),
                    Rotation2d.kZero));

        PathConstraints constraints = new PathConstraints(speed, speed * 2, 6, 12);

        PathPlannerPath alignPath =
            new PathPlannerPath(
                alignWaypoints, constraints, null, new GoalEndState(speed, Rotation2d.kZero));

        PathPlannerPath travelpath =
            new PathPlannerPath(
                travelWaypoints,
                constraints,
                new IdealStartingState(speed, Rotation2d.kZero),
                new GoalEndState(speed, Rotation2d.kZero));

        return Commands.sequence(
            AutoBuilder.followPath(alignPath), AutoBuilder.followPath(travelpath));

      default:
        throw new RuntimeException("Invalid robot state");
    }
  }
}
