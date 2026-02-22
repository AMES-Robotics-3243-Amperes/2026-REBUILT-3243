// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCharacterizations;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.ModeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.swerve.SwerveConstants;
import frc.robot.constants.swerve.TunerConstants;
import frc.robot.subsystems.Indexer.IndexerSubsystem;
import frc.robot.subsystems.Indexer.KickerIO;
import frc.robot.subsystems.Indexer.KickerIOReal;
import frc.robot.subsystems.Indexer.SpindexerIO;
import frc.robot.subsystems.Indexer.SpindexerIOReal;
import frc.robot.subsystems.drivetrain.GyroIO;
import frc.robot.subsystems.drivetrain.GyroIOPigeon2;
import frc.robot.subsystems.drivetrain.GyroIOSim;
import frc.robot.subsystems.drivetrain.ModuleIO;
import frc.robot.subsystems.drivetrain.ModuleIOTalonFXReal;
import frc.robot.subsystems.drivetrain.ModuleIOTalonFXSim;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.PivotIO;
import frc.robot.subsystems.intake.PivotIOSim;
import frc.robot.subsystems.intake.RollerIO;
import frc.robot.subsystems.intake.RollerIOReal;
import frc.robot.subsystems.shooter.FlywheelIO;
import frc.robot.subsystems.shooter.FlywheelIOReal;
import frc.robot.subsystems.shooter.FlywheelIOSim;
import frc.robot.subsystems.shooter.HoodIO;
import frc.robot.subsystems.shooter.HoodIOReal;
import frc.robot.subsystems.shooter.HoodIOSim;
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
  public final CommandXboxController primaryJoystick = new CommandXboxController(0);

  public final SwerveSubsystem drivetrain;
  public final IntakeSubsystem intake;
  public final IndexerSubsystem indexer;
  public final ShooterSubsystem shooter;

  private final SwerveDriveSimulation driveSimulation;

  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    switch (ModeConstants.robotMode) {
      case REAL_COMPETITION:
      case REAL:
        driveSimulation = null;

        drivetrain =
            new SwerveSubsystem(
                new GyroIOPigeon2(),
                new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                new ModuleIOTalonFXReal(TunerConstants.BackRight));
        intake = new IntakeSubsystem(new RollerIOReal(), new PivotIO() {});
        shooter = new ShooterSubsystem(new FlywheelIOReal(), new HoodIOReal());
        indexer = new IndexerSubsystem(new KickerIOReal(), new SpindexerIOReal());

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
        intake = new IntakeSubsystem(new RollerIO() {}, new PivotIOSim());
        indexer = new IndexerSubsystem(new KickerIO() {}, new SpindexerIO() {});
        shooter = new ShooterSubsystem(new FlywheelIOSim(), new HoodIOSim());

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
        intake = new IntakeSubsystem(new RollerIO() {}, new PivotIO() {});
        indexer = new IndexerSubsystem(new KickerIO() {}, new SpindexerIO() {});
        shooter = new ShooterSubsystem(new FlywheelIO() {}, new HoodIO() {});

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
                () -> true),
            drivetrain.joystickDriveAngular(primaryJoystick::getRightX)));

    primaryJoystick.leftTrigger().whileTrue(intake.intakeAtSpeedCommand(RotationsPerSecond.of(10)));

    primaryJoystick
        .rightTrigger()
        .whileTrue(
            shooter.shootAtHoodAngleCommand(
                RadiansPerSecond.of(
                    6
                        / (ShooterConstants.flywheelRadius.in(Meters)
                            * ShooterConstants.fuelToFlywheelLinearSpeedRatio)),
                ShooterConstants.hoodMaxRotation));

    primaryJoystick
        .b()
        .whileTrue(
            indexer.runAtSpeedCommand(
                MetersPerSecond.of(4), IndexerConstants.spindexerIndexingSpeed));
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

  public void updateComponents() {
    Logger.recordOutput(
        "ComponentPoses",
        new Pose3d[] {
          new Pose3d(
              Units.inchesToMeters(4.206652 + 7.266 - 0.1),
              0,
              Units.inchesToMeters(17.735146 - 6.980000 - 1.166046 - 0.21),
              new Rotation3d(0, intake.getPivotAngle().times(-1).in(Radians), 0)),
          new Pose3d(
              (Math.sin(Timer.getFPGATimestamp()) + 1) * Units.inchesToMeters(11.5) / 2,
              0,
              0,
              new Rotation3d()),
          new Pose3d(
              Units.inchesToMeters(-4.121261 - 0.21),
              0,
              Units.inchesToMeters(18.062500 - 0.27),
              new Rotation3d(0, shooter.getHoodAngle().in(Radians), 0))
        });
  }
}
