// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCharacterizations;
import frc.robot.commands.ShootingCommands;
import frc.robot.constants.ModeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.swerve.TunerConstants;
import frc.robot.subsystems.drivetrain.GyroIO;
import frc.robot.subsystems.drivetrain.GyroIOSim;
import frc.robot.subsystems.drivetrain.ModuleIO;
import frc.robot.subsystems.drivetrain.ModuleIOTalonFXSim;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.RollerIO;
import frc.robot.subsystems.shooter.FlywheelIO;
import frc.robot.subsystems.shooter.FlywheelIOReal;
import frc.robot.subsystems.shooter.FlywheelIOSim;
import frc.robot.subsystems.shooter.HoodIO;
import frc.robot.subsystems.shooter.HoodIOReal;
import frc.robot.subsystems.shooter.HoodIOSim;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private final CommandXboxController primaryJoystick = new CommandXboxController(0);

  private final SwerveSubsystem drivetrain;
  private final IntakeSubsystem intake;
  private final ShooterSubsystem shooter;

  @SuppressWarnings("unused")
  private final VisionSubsystem vision;

  private final SwerveDriveSimulation driveSimulation;

  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    switch (ModeConstants.robotMode) {
      case REAL_COMPETITION:
      case REAL:
        driveSimulation = null;
        drivetrain =
            new SwerveSubsystem(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        intake = new IntakeSubsystem(new RollerIO() {});

        shooter = new ShooterSubsystem(new FlywheelIOReal(), new HoodIOReal());

        vision = new VisionSubsystem(drivetrain::addVisionMeasurement, List.of());
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

        intake = new IntakeSubsystem(new RollerIO() {});

        shooter = new ShooterSubsystem(new FlywheelIOSim(), new HoodIOSim());

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

        intake = new IntakeSubsystem(new RollerIO() {});

        shooter = new ShooterSubsystem(new FlywheelIO() {}, new HoodIO() {});

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
                primaryJoystick::getLeftX,
                primaryJoystick::getLeftY,
                primaryJoystick.start().negate()),
            drivetrain.joystickDriveAngular(primaryJoystick::getRightX)));

    // primaryJoystick.a().whileTrue(DriveCommands.maxSpeedCharacterization(drivetrain));

    primaryJoystick.rightTrigger().whileTrue(intake.runAtIntakeSpeedCommand(drivetrain::getSpeed));
    primaryJoystick.leftTrigger().whileTrue(intake.outtakeCommand());

    primaryJoystick
        .a()
        .onTrue(DriveCharacterizations.sysIdCharacterization(drivetrain, primaryJoystick.a()));

    primaryJoystick.b().onTrue(shooter.sysIdCommand(primaryJoystick.b()));

    primaryJoystick
        .y()
        .whileTrue(
            ShootingCommands.shootWithIndependentLinearDriveCommand(
                drivetrain.joystickDriveLinear(
                    primaryJoystick::getLeftX,
                    primaryJoystick::getLeftY,
                    primaryJoystick.start().negate()),
                drivetrain,
                shooter));

    primaryJoystick
        .leftBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  RebuiltFuelOnFly fuel =
                      new RebuiltFuelOnFly(
                          driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                          ShooterConstants.robotToShooter.getTranslation().toTranslation2d(),
                          driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                          driveSimulation
                              .getSimulatedDriveTrainPose()
                              .getRotation()
                              .plus(ShooterConstants.robotToShooter.getRotation().toRotation2d()),
                          ShooterConstants.robotToShooter.getMeasureZ(),
                          shooter.getFuelVelocity(),
                          Degrees.of(90).minus(shooter.getHoodAngle()));

                  SimulatedArena.getInstance().addGamePieceProjectile(fuel);
                }));
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
