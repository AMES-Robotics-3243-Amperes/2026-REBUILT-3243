package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.setpoints.ChoreoVars;
import frc.robot.constants.setpoints.FieldConstants;
import frc.robot.constants.swerve.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import java.util.List;

public class RobotStateMachine extends SubsystemBase {
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

  public void switchState(RobotState newState) {
    state = newState;

    if (activeCommand != null) activeCommand.cancel();

    activeCommand = getCommand();
    CommandScheduler.getInstance().schedule(activeCommand);
  }

  public void updateState() {
    if (state == null) switchState(RobotState.OPERATOR_CONTROLLED);

    switch (state) {
      case OPERATOR_CONTROLLED:
        if (FieldConstants.leftTrench.getDistance(drivetrain.getPose().getTranslation()) < 1.5)
          switchState(RobotState.EXIT_TRENCH_LEFT);
        break;

      case EXIT_TRENCH_LEFT:
        if (activeCommand.isFinished()) switchState(RobotState.OPERATOR_CONTROLLED);
        break;

      default:
        throw new RuntimeException("Invalid robot state");
    }
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
        List<Waypoint> waypoints =
            List.of(
                new Waypoint(
                    null,
                    robotPose.getTranslation(),
                    robotPose.transformBy(speedsTransform).getTranslation()),
                new Waypoint(
                    new Translation2d(
                        ChoreoVars.TrenchEnterX.in(Meters) - 2,
                        FieldConstants.leftTrench.getCenter().getY()),
                    new Translation2d(
                        ChoreoVars.TrenchEnterX.in(Meters),
                        FieldConstants.leftTrench.getCenter().getY()),
                    new Translation2d(
                        ChoreoVars.TrenchEnterX.in(Meters) + 2,
                        FieldConstants.leftTrench.getCenter().getY())),
                new Waypoint(
                    new Translation2d(
                        ChoreoVars.TrenchExitX.in(Meters) - 2,
                        FieldConstants.leftTrench.getCenter().getY()),
                    new Translation2d(
                        ChoreoVars.TrenchExitX.in(Meters),
                        FieldConstants.leftTrench.getCenter().getY()),
                    null));

        PathConstraints constraints = new PathConstraints(2, 4, 4, 8);

        PathPlannerPath path =
            new PathPlannerPath(
                waypoints, constraints, null, new GoalEndState(2, Rotation2d.kZero));

        return AutoBuilder.followPath(path);

      default:
        throw new RuntimeException("Invalid robot state");
    }
  }
}
