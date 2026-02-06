package frc.robot.state;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootingCommands;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.swerve.SwerveConstants;
import java.util.Optional;

interface RobotState {
  Command getCommand(RobotContainer robotContainer);

  Optional<RobotState> nextState(RobotContainer robotContainer, Command command);

  record OperatorControlled() implements RobotState {
    public Command getCommand(RobotContainer robotContainer) {
      return Commands.parallel(
          robotContainer.drivetrain.driveSetpiontGeneratorCommand(
              robotContainer.drivetrain.joystickDriveLinear(
                  SwerveConstants.linearTeleopSpeed,
                  robotContainer.primaryJoystick::getLeftX,
                  robotContainer.primaryJoystick::getLeftY,
                  () -> true),
              robotContainer.drivetrain.joystickDriveAngular(
                  robotContainer.primaryJoystick::getRightX)),
          robotContainer.shooter.setHoodAngleCommand(ShooterConstants.hoodMinRotation));
    }

    public Optional<RobotState> nextState(RobotContainer robotContainer, Command command) {
      if (robotContainer.primaryJoystick.rightTrigger().getAsBoolean())
        return TrenchTraversal.inferTraversal(robotContainer.drivetrain.getPose().getTranslation())
            .map(traversal -> (RobotState) traversal);

      if (robotContainer.primaryJoystick.a().getAsBoolean())
        return Optional.of(new OperatorControlledShooting());

      return Optional.empty();
    }
  }

  record OperatorControlledShooting() implements RobotState {
    public Command getCommand(RobotContainer robotContainer) {
      return ShootingCommands.shootHubWithIndependentLinearDriveCommand(
          robotContainer.drivetrain.joystickDriveLinear(
              SwerveConstants.linearTeleopSpeed,
              robotContainer.primaryJoystick::getLeftX,
              robotContainer.primaryJoystick::getLeftY,
              () -> true),
          robotContainer.drivetrain,
          robotContainer.shooter);
    }

    public Optional<RobotState> nextState(RobotContainer robotContainer, Command command) {
      if (robotContainer.primaryJoystick.rightTrigger().getAsBoolean())
        return TrenchTraversal.inferTraversal(robotContainer.drivetrain.getPose().getTranslation())
            .map(traversal -> (RobotState) traversal);

      if (!robotContainer.primaryJoystick.a().getAsBoolean())
        return Optional.of(new OperatorControlled());

      return Optional.empty();
    }
  }
}
