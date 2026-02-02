// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.swerve.ModuleConstants;
import frc.robot.constants.swerve.SwerveConstants;
import frc.robot.constants.swerve.SysIdConstants;
import frc.robot.constants.swerve.TunerConstants;
import frc.robot.util.Container;
import frc.robot.util.TunableControls;
import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SwerveSubsystem extends SubsystemBase {
  public static final DriveTrainSimulationConfig mapleSimConfig =
      DriveTrainSimulationConfig.Default()
          .withRobotMass(SwerveConstants.robotMass)
          .withCustomModuleTranslations(getModuleTranslations())
          .withGyro(COTS.ofPigeon2())
          .withSwerveModule(
              new SwerveModuleSimulationConfig(
                  (ModuleConstants.driveClosedLoopOutput == ClosedLoopOutputType.Voltage
                      ? DCMotor.getKrakenX60(1)
                      : DCMotor.getKrakenX60Foc(1)),
                  (ModuleConstants.steerClosedLoopOutput == ClosedLoopOutputType.Voltage
                      ? DCMotor.getFalcon500(1)
                      : DCMotor.getFalcon500Foc(1)),
                  ModuleConstants.driveGearRatio,
                  ModuleConstants.steerGearRatio,
                  ModuleConstants.driveFrictionVoltage,
                  ModuleConstants.steerFrictionVoltage,
                  ModuleConstants.wheelRadius,
                  ModuleConstants.kSteerInertia,
                  SwerveConstants.wheelCoefficientOfFriction));

  public static final RobotConfig pathPlannerConfig =
      new RobotConfig(
          SwerveConstants.robotMass.in(Kilograms),
          SwerveConstants.robotMomentOfInertia.in(KilogramSquareMeters),
          new ModuleConfig(
              ModuleConstants.wheelRadius.in(Meters),
              SwerveConstants.speedAt12Volts.in(MetersPerSecond),
              SwerveConstants.wheelCoefficientOfFriction,
              (ModuleConstants.driveClosedLoopOutput == ClosedLoopOutputType.Voltage
                      ? DCMotor.getKrakenX60(1)
                      : DCMotor.getKrakenX60Foc(1))
                  .withReduction(ModuleConstants.driveGearRatio),
              SwerveConstants.slipCurrent.in(Amps),
              1),
          getModuleTranslations());

  private final SwerveSetpointGenerator setpointGenerator =
      new SwerveSetpointGenerator(
          pathPlannerConfig, ModuleConstants.maxSetpointGeneratorModuleRotation);
  private final Timer idleTimeSinceLastSetpointReset = new Timer();
  private SwerveSetpoint previousSetpoint = null;

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final SwerveModule[] modules = new SwerveModule[4]; // FL, FR, BL, BR
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(getModuleTranslations());
  private final SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.kZero);

  private final Consumer<Pose2d> resetSimulationPoseCallBack;

  public SwerveSubsystem(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      Consumer<Pose2d> resetSimulationPoseCallBack) {
    this.gyroIO = gyroIO;
    modules[0] = new SwerveModule(flModuleIO, 0);
    modules[1] = new SwerveModule(frModuleIO, 1);
    modules[2] = new SwerveModule(blModuleIO, 2);
    modules[3] = new SwerveModule(brModuleIO, 3);

    this.resetSimulationPoseCallBack = resetSimulationPoseCallBack;

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    PhoenixOdometryThread.getInstance().start();

    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::driveFeedforwards,
        new PPLTVController(0.02),
        // new PPHolonomicDriveController(
        //     SwerveConstants.drivePid.buildConstants(),
        //     SwerveConstants.rotationPidRadians.buildConstants()),
        pathPlannerConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Drivetrain/PathPlanner/Trajectory",
              activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Drivetrain/PathPlanner/TrajectorySetpoint", targetPose);
        });

    resetSetpointGenerator();
  }

  public SwerveSubsystem(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this(gyroIO, flModuleIO, frModuleIO, blModuleIO, brModuleIO, pose -> {});
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drivetrain/Gyro", gyroInputs);
    for (SwerveModule module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (SwerveModule module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("Drivetrain/Modules/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("Drivetrain/Modules/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    gyroDisconnectedAlert.set(!gyroInputs.connected);
  }

  private void resetSetpointGenerator() {
    previousSetpoint =
        new SwerveSetpoint(
            getChassisSpeeds(),
            getModuleStates(),
            DriveFeedforwards.zeros(pathPlannerConfig.numModules));
    idleTimeSinceLastSetpointReset.restart();
  }

  /**
   * Runs at the desired field-relative velocity after processing through swerve setpoint generator
   *
   * @param speeds Speeds in meters/sec
   */
  public void driveSetpointGenerator(ChassisSpeeds speeds) {
    if (idleTimeSinceLastSetpointReset.hasElapsed(
        SwerveConstants.idleTimeUntilSetpointGeneratorReset.in(Seconds))) {
      resetSetpointGenerator();
    }

    // Calculate module setpoints
    previousSetpoint = setpointGenerator.generateSetpoint(previousSetpoint, speeds, 0.02);
    idleTimeSinceLastSetpointReset.restart();

    SwerveModuleState[] states = previousSetpoint.moduleStates();
    LinearAcceleration[] feedforwards = previousSetpoint.feedforwards().accelerations();

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("Drivetrain/Modules/Setpoints", states);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(states[i], feedforwards[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("Drivetrain/Modules/SetpointsOptimized", states);
  }

  /**
   * Runs the drive at the desired field-relative velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void driveFeedforwards(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, SwerveConstants.speedAt12Volts);

    // Log unoptimized setpoints
    Logger.recordOutput("Drivetrain/Modules/Setpoints", setpointStates);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i], feedforwards.accelerations()[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("Drivetrain/Modules/SetpointsOptimized", setpointStates);
  }

  /**
   * Runs the drive at the desired field-relative velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void drive(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, SwerveConstants.speedAt12Volts);

    // Log unoptimized setpoints
    Logger.recordOutput("Drivetrain/Modules/Setpoints", setpointStates);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("Drivetrain/Modules/SetpointsOptimized", setpointStates);
  }

  //
  // Module Getters
  //

  @AutoLogOutput(key = "Drivetrain/Modules/MeasuredStates")
  public SwerveModuleState[] getModuleStates() {
    return Arrays.stream(modules).map(SwerveModule::getState).toArray(SwerveModuleState[]::new);
  }

  public SwerveModulePosition[] getModulePositions() {
    return Arrays.stream(modules)
        .map(SwerveModule::getPosition)
        .toArray(SwerveModulePosition[]::new);
  }

  public Distance[] getModuleLinearDistances() {
    return Arrays.stream(modules).map(SwerveModule::getLinearDistance).toArray(Distance[]::new);
  }

  public Angle[] getModuleDriveAngles() {
    return Arrays.stream(modules).map(SwerveModule::getAngularDrivePosition).toArray(Angle[]::new);
  }

  public Angle[] getModuleAzimuthAngles() {
    return Arrays.stream(modules).map(SwerveModule::getAzimuthAngle).toArray(Angle[]::new);
  }

  public Current getAverageCurrent() {
    return Arrays.stream(modules)
        .map(SwerveModule::getCurrent)
        .reduce(Current::plus)
        .orElseThrow()
        .div(4);
  }

  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }

  //
  // Positional Data
  //

  public void setPose(Pose2d pose) {
    resetSimulationPoseCallBack.accept(pose);
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  @AutoLogOutput(key = "Drivetrain/EstimatedPose")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  @AutoLogOutput(key = "Drivetrain/Speeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public LinearVelocity getSpeed() {
    ChassisSpeeds speeds = getChassisSpeeds();
    double x = speeds.vxMetersPerSecond;
    double y = speeds.vyMetersPerSecond;

    return MetersPerSecond.of(Math.sqrt(x * x + y * y));
  }

  //
  // SysID
  //

  public enum SwerveSysIdRoutine {
    DRIVE_LINEAR_FEEDFORWARD,
    AZIMUTH_FEEDFORWARD,
  }

  private SysIdRoutine sysIdRoutine(SwerveSysIdRoutine routine) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            SysIdConstants.sysIdRampRate(routine),
            SysIdConstants.sysIdStepVoltage(routine),
            SysIdConstants.sysIdTimeout,
            (state) -> {
              Logger.recordOutput("Drivetrain/SysId/State", state.toString());

              Logger.recordOutput(
                  "Drivetrain/SysId/DrivePositionRadians",
                  getCharacterizationDriveAngle().in(Radians));
              Logger.recordOutput(
                  "Drivetrain/SysId/DriveVelocityRadiansPerSecond",
                  getCharacterizationAngularDriveVelocity().in(RadiansPerSecond));
              Logger.recordOutput(
                  "Drivetrain/SysId/DriveAppliedVolts", getCharacterizationDriveVolts().in(Volts));

              Logger.recordOutput(
                  "Drivetrain/SysId/AzimuthPositionRadians",
                  getCharacterizationAzimuthAngle().in(Radians));
              Logger.recordOutput(
                  "Drivetrain/SysId/AzimuthVelocityRadiansPerSecond",
                  getCharacterizationAngularAzimuthVelocity().in(RadiansPerSecond));
              Logger.recordOutput(
                  "Drivetrain/SysId/AzimuthAppliedVolts",
                  getCharacterizationAzimuthVolts().in(Volts));
            }),
        new SysIdRoutine.Mechanism(
            (voltage) -> runCharacterization(routine, voltage.in(Volts)), null, this));
  }

  /** Runs the SysId characterization of each module with the given output. */
  public void runCharacterization(SwerveSysIdRoutine routine, double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(routine, output);
    }
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(SysIdConstants.activeRoutine, 0.0))
        .withTimeout(1.0)
        .andThen(sysIdRoutine(SysIdConstants.activeRoutine).quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(SysIdConstants.activeRoutine, 0.0))
        .withTimeout(1.0)
        .andThen(sysIdRoutine(SysIdConstants.activeRoutine).dynamic(direction));
  }

  public Angle getCharacterizationDriveAngle() {
    return Arrays.stream(getModuleDriveAngles()).reduce(Angle::plus).orElseThrow().div(4);
  }

  public AngularVelocity getCharacterizationAngularDriveVelocity() {
    return Arrays.stream(modules)
        .map(SwerveModule::getAngularDriveVelocity)
        .reduce(AngularVelocity::plus)
        .orElseThrow()
        .div(4);
  }

  public Voltage getCharacterizationDriveVolts() {
    return Arrays.stream(modules)
        .map(SwerveModule::getAppliedDriveVoltage)
        .reduce(Voltage::plus)
        .orElseThrow()
        .div(4);
  }

  public Angle getCharacterizationAzimuthAngle() {
    return Arrays.stream(getModuleAzimuthAngles()).reduce(Angle::plus).orElseThrow().div(4);
  }

  public AngularVelocity getCharacterizationAngularAzimuthVelocity() {
    return Arrays.stream(modules)
        .map(SwerveModule::getAzimuthVelocity)
        .reduce(AngularVelocity::plus)
        .orElseThrow()
        .div(4);
  }

  public Voltage getCharacterizationAzimuthVolts() {
    return Arrays.stream(modules)
        .map(SwerveModule::getAppliedAzimuthVoltage)
        .reduce(Voltage::plus)
        .orElseThrow()
        .div(4);
  }

  //
  // Driving Strategies & Commands
  //

  public Command driveCommand(
      Supplier<Translation2d> linearStrategy, Supplier<AngularVelocity> angularStrategy) {
    return run(
        () -> {
          Translation2d linearSpeeds = linearStrategy.get();
          AngularVelocity angularSpeed = angularStrategy.get();

          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearSpeeds.getX(), linearSpeeds.getY(), angularSpeed.in(RadiansPerSecond));
          drive(speeds);
        });
  }

  public Command driveSetpiontGeneratorCommand(
      Supplier<Translation2d> linearStrategy, Supplier<AngularVelocity> angularStrategy) {
    return run(
        () -> {
          Translation2d linearSpeeds = linearStrategy.get();
          AngularVelocity angularSpeed = angularStrategy.get();

          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearSpeeds.getX(), linearSpeeds.getY(), angularSpeed.in(RadiansPerSecond));
          driveSetpointGenerator(speeds);
        });
  }

  public Supplier<Translation2d> joystickDriveLinear(
      DoubleSupplier leftJoystickX, DoubleSupplier leftJoystickY, BooleanSupplier fieldRelative) {
    return () -> {
      double fieldX = -leftJoystickY.getAsDouble();
      double fieldY = -leftJoystickX.getAsDouble();

      Vector<N2> rawSpeeds =
          MathUtil.applyDeadband(
              VecBuilder.fill(fieldX, fieldY), SwerveConstants.teleopJoystickDeadband);

      // Square magnitude for more precise control
      rawSpeeds = rawSpeeds.times(rawSpeeds.norm());

      // Convert to field relative speeds & send command
      ChassisSpeeds speeds =
          new ChassisSpeeds(
              rawSpeeds.get(0) * SwerveConstants.linearTeleopSpeed.in(MetersPerSecond),
              rawSpeeds.get(1) * SwerveConstants.linearTeleopSpeed.in(MetersPerSecond),
              0);

      if (fieldRelative.getAsBoolean()) {
        boolean isFlipped =
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;

        speeds =
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds, isFlipped ? getRotation().plus(new Rotation2d(Math.PI)) : getRotation());
      }

      return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    };
  }

  public Supplier<AngularVelocity> joystickDriveAngular(DoubleSupplier rightJoystickX) {
    return () -> {
      double withDeadband =
          MathUtil.applyDeadband(
              -rightJoystickX.getAsDouble(), SwerveConstants.teleopJoystickDeadband);

      return SwerveConstants.angularTeleopSpeed.times(
          Math.copySign(withDeadband * withDeadband, withDeadband));
    };
  }

  public Supplier<AngularVelocity> rotateAtAngleFeedForward(Supplier<Rotation2d> targetSupplier) {
    ProfiledPIDController pidController =
        SwerveConstants.rotationControl.profiledPIDController(
            Radians, Degrees.of(-180), Degrees.of(180));
    pidController.enableContinuousInput(-Math.PI, Math.PI);
    pidController.setTolerance(SwerveConstants.rotationFeedBackTolerance.in(Radians));
    pidController.reset(getRotation().getRadians());
    Container<Rotation2d> previousSetpoint = new Container<Rotation2d>(targetSupplier.get());

    Timer timeSinceLastLoop = new Timer();
    timeSinceLastLoop.restart();

    return () -> {
      Rotation2d setpoint = targetSupplier.get();

      AngularVelocity setpointVelocity =
          setpoint
              .minus(previousSetpoint.inner)
              .getMeasure()
              .div(Seconds.of(timeSinceLastLoop.get()));

      // four loops is picked arbitrarily
      if (timeSinceLastLoop.hasElapsed(0.08)) {
        pidController.reset(getRotation().getRadians());
        previousSetpoint.inner = setpoint;
        setpointVelocity = RadiansPerSecond.of(0);
      }

      timeSinceLastLoop.restart();

      double feedbackOutput =
          pidController.calculate(getRotation().getRadians(), targetSupplier.get().getRadians());
      previousSetpoint.inner = setpoint;

      Logger.recordOutput("pid error radians", pidController.getPositionError());

      return RadiansPerSecond.of(feedbackOutput)
          .plus(setpointVelocity.times(SwerveConstants.rotationFeedforwardCoefficient));
    };
  }
}
