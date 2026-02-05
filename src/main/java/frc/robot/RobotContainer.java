// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCharacterizations;
import frc.robot.constants.ModeConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.swerve.SwerveConstants;
import frc.robot.constants.swerve.TunerConstants;
import frc.robot.subsystems.drivetrain.GyroIO;
import frc.robot.subsystems.drivetrain.GyroIONavX;
import frc.robot.subsystems.drivetrain.GyroIOSim;
import frc.robot.subsystems.drivetrain.ModuleIO;
import frc.robot.subsystems.drivetrain.ModuleIORev;
import frc.robot.subsystems.drivetrain.ModuleIOTalonFXSim;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import frc.robot.subsystems.shooter.FlywheelIO;
import frc.robot.subsystems.shooter.FlywheelIOReal;
import frc.robot.subsystems.shooter.IndexerIO;
import frc.robot.subsystems.shooter.IndexerIOReal;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionSubsystem;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private final CommandXboxController primaryJoystick = new CommandXboxController(0);

  private final SwerveSubsystem drivetrain;
  private final ShooterSubsystem shooter;

  @SuppressWarnings("unused")
  private final VisionSubsystem vision;

  private final SwerveDriveSimulation driveSimulation;

  private final LoggedDashboardChooser<Command> autoChooser;

  GyroIONavX gyro = new GyroIONavX();

  public RobotContainer() {
    switch (ModeConstants.robotMode) {
      case REAL_COMPETITION:
      case REAL:
        driveSimulation = null;

        drivetrain =
            new SwerveSubsystem(
                gyro,
                new ModuleIORev(1, 2, Rotation2d.fromDegrees(90)) {},
                new ModuleIORev(3, 4, Rotation2d.fromDegrees(0)) {},
                new ModuleIORev(5, 6, Rotation2d.fromDegrees(180)) {},
                new ModuleIORev(7, 8, Rotation2d.fromDegrees(270)) {});

        shooter = new ShooterSubsystem(new FlywheelIOReal(), new IndexerIOReal());
        // shooter = new ShooterSubsystem(new FlywheelIO() {}, new IndexerIO() {});

        vision =
            new VisionSubsystem(
                drivetrain::addVisionMeasurement,
                VisionConstants.cameras.stream()
                    .<VisionIO>map(config -> new VisionIOLimelight(config, drivetrain::getRotation))
                    .toList());

        break;

      case SIM:
        driveSimulation =
            new SwerveDriveSimulation(
                SwerveSubsystem.mapleSimConfig, new Pose2d(2, 2, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

        drivetrain =
            new SwerveSubsystem(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOTalonFXSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                new ModuleIOTalonFXSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                new ModuleIOTalonFXSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                new ModuleIOTalonFXSim(TunerConstants.BackRight, driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);

        shooter = new ShooterSubsystem(new FlywheelIO() {}, new IndexerIO() {});

        vision =
            new VisionSubsystem(
                drivetrain::addVisionMeasurement,
                VisionConstants.cameras.stream()
                    .<VisionIO>map(
                        config ->
                            new VisionIOPhotonVisionSim(
                                config, driveSimulation::getSimulatedDriveTrainPose))
                    .toList());

        break;

      case REPLAY:
        driveSimulation = null;

        // Replayed robot, disable IO implementations
        drivetrain =
            new SwerveSubsystem(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        shooter = new ShooterSubsystem(new FlywheelIO() {}, new IndexerIO() {});

        vision =
            new VisionSubsystem(
                drivetrain::addVisionMeasurement,
                VisionConstants.cameras.stream()
                    .<VisionIO>map(config -> new VisionIO(config) {})
                    .toList());

        break;

      default:
        throw new RuntimeException("Invalid robot mode");
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    if (ModeConstants.robotMode == ModeConstants.Mode.SIM) {
      autoChooser.addOption(
          "Drive Wheel Radius Characterization",
          DriveCharacterizations.wheelRadiusCharacterization(drivetrain));
    }

    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
        drivetrain.driveSetpiontGeneratorCommand(
            drivetrain.joystickDriveLinear(
                SwerveConstants.linearTeleopSpeed,
                primaryJoystick::getLeftX,
                primaryJoystick::getLeftY,
                primaryJoystick.start().negate()),
            drivetrain.joystickDriveAngular(primaryJoystick::getRightX)));

    primaryJoystick
        .a()
        .whileTrue(
            Commands.sequence(
                shooter
                    .runAtSpeedCommand(RotationsPerSecond.of(48), RotationsPerSecond.of(0))
                    .withTimeout(0.7),
                shooter.runAtSpeedCommand(RotationsPerSecond.of(48), RotationsPerSecond.of(40))));

    primaryJoystick
        .y()
        .whileTrue(
            Commands.sequence(
                    shooter
                        .runAtSpeedCommand(RotationsPerSecond.of(48), RotationsPerSecond.of(0))
                        .withTimeout(0.7),
                    shooter
                        .runAtSpeedCommand(RotationsPerSecond.of(48), RotationsPerSecond.of(40))
                        .withTimeout(0.2),
                    Commands.run(
                            () ->
                                drivetrain.driveSetpointGenerator(new ChassisSpeeds(0.9, 0.05, 0)),
                            drivetrain)
                        .withTimeout(0.65),
                    Commands.run(
                            () -> drivetrain.driveSetpointGenerator(new ChassisSpeeds(0, 0, 0)),
                            drivetrain)
                        .withTimeout(1.5),
                    Commands.run(
                            () ->
                                drivetrain.driveSetpointGenerator(
                                    new ChassisSpeeds(-0.9, -0.05, 0)),
                            drivetrain)
                        .withTimeout(0.65),
                    Commands.run(
                            () -> drivetrain.driveSetpointGenerator(new ChassisSpeeds(0, 0, 0)),
                            drivetrain)
                        .withTimeout(0.5))
                .repeatedly());

    primaryJoystick.x().onTrue(Commands.runOnce(gyro::resetYaw));

    // primaryJoystick
    //     .b()
    //     .onTrue(DriveCharacterizations.sysIdCharacterization(drivetrain, primaryJoystick.b()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void updateSimulation() {
    if (ModeConstants.robotMode != ModeConstants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
  }
}
