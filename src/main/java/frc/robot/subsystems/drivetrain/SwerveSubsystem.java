// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import choreo.trajectory.SwerveSample;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
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
import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.GeneralPurposeCharacterization;
import frc.robot.constants.choreo.ChoreoVars;
import frc.robot.constants.swerve.ModuleConstants;
import frc.robot.constants.swerve.SwerveConstants;
import frc.robot.constants.swerve.SysIdConstants;
import frc.robot.constants.swerve.TunerConstants;
import frc.robot.util.Container;
import frc.robot.util.FuelTrajectoryCalculator;
import frc.robot.util.FuelTrajectoryCalculator.ShootTarget;
import frc.robot.util.PointOfInterestManager;
import frc.robot.util.PointOfInterestManager.FlipType;
import java.util.Arrays;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
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
          .withRobotMass(ChoreoVars.R_RobotMass)
          .withCustomModuleTranslations(getModuleTranslations())
          .withBumperSize(ChoreoVars.R_BumperLength, ChoreoVars.R_BumperLength)
          .withGyro(COTS.ofPigeon2())
          .withSwerveModule(
              new SwerveModuleSimulationConfig(
                  DCMotor.getKrakenX60(1),
                  DCMotor.getFalcon500(1),
                  ChoreoVars.R_DriveReduction,
                  ModuleConstants.steerReduction,
                  ModuleConstants.driveFrictionVoltage,
                  ModuleConstants.steerFrictionVoltage,
                  ChoreoVars.R_WheelRadius,
                  ModuleConstants.kSteerInertia,
                  ChoreoVars.R_WheelCOF));

  public static final RobotConfig pathPlannerConfig =
      new RobotConfig(
          ChoreoVars.R_RobotMass.in(Kilograms),
          ChoreoVars.R_RobotMOI.in(KilogramSquareMeters),
          new ModuleConfig(
              ChoreoVars.R_WheelRadius.in(Meters),
              SwerveConstants.speedAt12Volts.in(MetersPerSecond),
              ChoreoVars.R_WheelCOF,
              DCMotor.getKrakenX60Foc(1).withReduction(ChoreoVars.R_DriveReduction),
              ModuleConstants.driveSupplyCurrentLimit.in(Amps),
              1),
          getModuleTranslations());

  private final Field2d field = new Field2d();

  private final SwerveSetpointGenerator setpointGenerator =
      new SwerveSetpointGenerator(pathPlannerConfig, ModuleConstants.maxModuleAzimuth);
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

  private final Consumer<Pose2d> resetSimulationPoseCallback;

  private double poseConfidence = 0.0;
  private double lastPeriodicTimeSec = Timer.getFPGATimestamp();
  private double lastVisionTimeSec = 0.0;

  private boolean atRotationSetpoint = false;

  private static final double kConfidenceDecaySec = 2.0; // TODO: constants
  private static final double kBaseConfidenceBump = 0.35;

  private boolean setpointGeneratorUpToDate = false;

  public SwerveSubsystem(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      Consumer<Pose2d> resetSimulationPoseCallback) {
    this.gyroIO = gyroIO;
    modules[0] = new SwerveModule(flModuleIO, 0);
    modules[1] = new SwerveModule(frModuleIO, 1);
    modules[2] = new SwerveModule(blModuleIO, 2);
    modules[3] = new SwerveModule(brModuleIO, 3);

    this.resetSimulationPoseCallback = resetSimulationPoseCallback;

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    PhoenixOdometryThread.getInstance().start();

    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::driveFeedforwards,
        new PPHolonomicDriveController(
            SwerveConstants.driveControl,
            SwerveConstants.rotationControl.pathPlannerPIDConstants()),
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

    SmartDashboard.putData(
        "Module States",
        new Sendable() {
          @Override
          public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("SwerveDrive");

            builder.addDoubleProperty(
                "Front Left Angle", () -> modules[0].getAzimuthAngle().in(Radians), null);
            builder.addDoubleProperty(
                "Front Left Velocity",
                () -> modules[0].getLinearVelocity().in(MetersPerSecond),
                null);

            builder.addDoubleProperty(
                "Front Right Angle", () -> modules[1].getAzimuthAngle().in(Radians), null);
            builder.addDoubleProperty(
                "Front Right Velocity",
                () -> modules[1].getLinearVelocity().in(MetersPerSecond),
                null);

            builder.addDoubleProperty(
                "Back Left Angle", () -> modules[2].getAzimuthAngle().in(Radians), null);
            builder.addDoubleProperty(
                "Back Left Velocity",
                () -> modules[2].getLinearVelocity().in(MetersPerSecond),
                null);

            builder.addDoubleProperty(
                "Back Right Angle", () -> modules[3].getAzimuthAngle().in(Radians), null);
            builder.addDoubleProperty(
                "Back Right Velocity",
                () -> modules[3].getLinearVelocity().in(MetersPerSecond),
                null);

            builder.addDoubleProperty("Robot Angle", () -> getRotation().getRadians(), null);
          }
        });
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

    double now = Timer.getFPGATimestamp();
    double dt = Math.max(0.0, now - lastPeriodicTimeSec);
    lastPeriodicTimeSec = now;

    poseConfidence *= Math.exp(-dt / kConfidenceDecaySec);
    poseConfidence = MathUtil.clamp(poseConfidence, 0.0, 1.0);

    field.setRobotPose(getPose());
    SmartDashboard.putData(field);
  }

  private void resetSetpointGenerator() {
    previousSetpoint =
        new SwerveSetpoint(
            getChassisSpeeds(),
            getModuleStates(),
            DriveFeedforwards.zeros(pathPlannerConfig.numModules));
    setpointGeneratorUpToDate = true;
  }

  /**
   * Runs at the desired field-relative velocity after processing through swerve setpoint generator
   *
   * @param speeds Speeds in meters/sec
   */
  public void driveSetpointGenerator(ChassisSpeeds speeds) {
    if (!setpointGeneratorUpToDate) {
      resetSetpointGenerator();
    }

    // Calculate module setpoints
    previousSetpoint = setpointGenerator.generateSetpoint(previousSetpoint, speeds, 0.02);

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
    setpointGeneratorUpToDate = false;

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
    setpointGeneratorUpToDate = false;

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
    resetSimulationPoseCallback.accept(pose);
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);

    lastVisionTimeSec = Timer.getFPGATimestamp();
    double sx = visionMeasurementStdDevs.get(0, 0);
    double sy = visionMeasurementStdDevs.get(1, 0);
    double positionStdDev = Math.hypot(sx, sy);
    double measurementQuality = 1.0 / (1.0 + positionStdDev);
    poseConfidence =
        MathUtil.clamp(poseConfidence + (kBaseConfidenceBump * measurementQuality), 0.0, 1.0);
  }

  @AutoLogOutput(key = "Drivetrain/EstimatedPose")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  @AutoLogOutput(key = "Drivetrain/PoseConfidence")
  public double getPoseConfidence() {
    return poseConfidence;
  }

  @AutoLogOutput(key = "Drivetrain/TimeSinceVision")
  public double getTimeSinceVision() {
    return Timer.getFPGATimestamp() - lastVisionTimeSec;
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

  public boolean atRotationSetpoint() {
    return atRotationSetpoint;
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

  public Command sysIdCommand(SwerveSysIdRoutine routine, Trigger advanceRoutine) {
    return GeneralPurposeCharacterization.sysIdCommand(
        sysIdRoutine(routine), advanceRoutine, () -> runCharacterization(routine, 0), this);
  }

  /** Runs the SysId characterization of each module with the given output. */
  public void runCharacterization(SwerveSysIdRoutine routine, double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(routine, output);
    }
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
  // Path following
  //

  private boolean flipPathAcrossFieldWidth = false;
  private Optional<Supplier<AngularVelocity>> pathFollowingAngularOverride = Optional.empty();

  /**
   * These are used to control the behavior of choreo path following in a command-centric way
   * (hopefully) without running into race conditions or other weird "only one option at once"
   * issues.
   */
  private SubsystemBase choreoFlippedSubsystem = new SubsystemBase("SwerveAutoFlipped") {};

  private SubsystemBase choreoCustomRotationSubsystem = new SubsystemBase("SwerveAutoRotation") {};

  public Command driveFlippedAlongFieldWidth() {
    return Commands.runEnd(
        () -> flipPathAcrossFieldWidth = true,
        () -> flipPathAcrossFieldWidth = false,
        choreoFlippedSubsystem);
  }

  public Command rotateIndependentlyOfPath(Supplier<AngularVelocity> angularStrategy) {
    return Commands.runEnd(
        () -> pathFollowingAngularOverride = Optional.of(angularStrategy),
        () -> pathFollowingAngularOverride = Optional.empty(),
        choreoCustomRotationSubsystem);
  }

  private PIDController choreoXController =
      new PIDController(
          SwerveConstants.driveControl.kP,
          SwerveConstants.driveControl.kD,
          SwerveConstants.driveControl.kD);
  private PIDController choreoYController =
      new PIDController(
          SwerveConstants.driveControl.kP,
          SwerveConstants.driveControl.kD,
          SwerveConstants.driveControl.kD);
  private ProfiledPIDController choreoHeadingController =
      SwerveConstants.rotationControl.profiledPIDController(
          Radians, Degrees.of(-180), Degrees.of(180));

  public void followChoreoTrajecotry(SwerveSample sample) {
    if (sample.getTimestamp() == 0) {
      choreoHeadingController.reset(
          getRotation().getRadians(), getChassisSpeeds().omegaRadiansPerSecond);
    }

    Pose2d robotPose = getPose();

    Pose2d poseSetpoint = new Pose2d(sample.x, sample.y, Rotation2d.fromRadians(sample.heading));
    if (flipPathAcrossFieldWidth)
      poseSetpoint = PointOfInterestManager.flipPose(poseSetpoint, FlipType.REFLECT_Y);

    ChassisSpeeds feedbackSpeeds =
        new ChassisSpeeds(
            choreoXController.calculate(robotPose.getX(), poseSetpoint.getX()),
            choreoYController.calculate(robotPose.getY(), poseSetpoint.getY()),
            choreoHeadingController.calculate(
                robotPose.getRotation().getRadians(), poseSetpoint.getRotation().getRadians()));

    ChassisSpeeds feedforwardSpeeds =
        new ChassisSpeeds(
            sample.vx,
            flipPathAcrossFieldWidth ? -sample.vy : sample.vy,
            flipPathAcrossFieldWidth ? -sample.omega : sample.omega);

    ChassisSpeeds finalSpeeds = feedbackSpeeds.plus(feedforwardSpeeds);
    pathFollowingAngularOverride.ifPresent(
        angularStrategy ->
            finalSpeeds.omegaRadiansPerSecond = angularStrategy.get().in(RadiansPerSecond));

    // TODO: the sample contains acceleration information, we should probably use this information
    // to get module feedforwards and directly set module states
    drive(ChassisSpeeds.fromFieldRelativeSpeeds(finalSpeeds, robotPose.getRotation()));
  }

  //
  // Driving Strategies & Commands
  //

  public Command driveCommand(
      Supplier<Translation2d> linearStrategy, Supplier<AngularVelocity> angularStrategy) {
    return Commands.sequence(
        runOnce(() -> setpointGeneratorUpToDate = false),
        run(
            () -> {
              Translation2d linearSpeeds = linearStrategy.get();
              AngularVelocity angularSpeed = angularStrategy.get();

              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearSpeeds.getX(), linearSpeeds.getY(), angularSpeed.in(RadiansPerSecond));
              drive(speeds);
            }));
  }

  public Command driveSetpiontGeneratorCommand(
      Supplier<Translation2d> linearStrategy, Supplier<AngularVelocity> angularStrategy) {
    return Commands.sequence(
        runOnce(() -> setpointGeneratorUpToDate = false),
        run(
            () -> {
              Translation2d linearSpeeds = linearStrategy.get();
              AngularVelocity angularSpeed = angularStrategy.get();

              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearSpeeds.getX(), linearSpeeds.getY(), angularSpeed.in(RadiansPerSecond));
              driveSetpointGenerator(speeds);
            }));
  }

  public Supplier<Translation2d> joystickDriveLinear(
      LinearVelocity velocity, CommandXboxController controller) {
    final double velocityMps = velocity.in(MetersPerSecond);

    return () -> {
      double fieldX = -controller.getLeftY();
      double fieldY = -controller.getLeftX();

      Vector<N2> rawSpeeds =
          MathUtil.applyDeadband(
              VecBuilder.fill(fieldX, fieldY), SwerveConstants.teleopJoystickDeadband);

      // Square magnitude for more precise control
      rawSpeeds = rawSpeeds.times(rawSpeeds.norm());

      // Convert to field relative speeds & send command
      ChassisSpeeds speeds =
          new ChassisSpeeds(rawSpeeds.get(0) * velocityMps, rawSpeeds.get(1) * velocityMps, 0);

      boolean isFlipped =
          DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;

      speeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              speeds, isFlipped ? getRotation().plus(new Rotation2d(Math.PI)) : getRotation());

      return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    };
  }

  public Supplier<AngularVelocity> joystickDriveAngular(DoubleSupplier rightJoystickX) {
    return () -> {
      atRotationSetpoint = false;

      double withDeadband =
          MathUtil.applyDeadband(
              -rightJoystickX.getAsDouble(), SwerveConstants.teleopJoystickDeadband);

      return SwerveConstants.angularTeleopSpeed.times(
          Math.copySign(withDeadband * withDeadband, withDeadband));
    };
  }

  public Supplier<AngularVelocity> rotateAtAngle(Supplier<Rotation2d> targetSupplier) {
    ProfiledPIDController pidController =
        SwerveConstants.rotationControl.profiledPIDController(
            Radians, Degrees.of(-180), Degrees.of(180));
    pidController.enableContinuousInput(-Math.PI, Math.PI);
    pidController.setTolerance(SwerveConstants.rotationFeedbackTolerance.in(Radians));
    pidController.reset(getRotation().getRadians());

    Timer timeSinceLastLoop = new Timer();
    timeSinceLastLoop.restart();

    Container<Rotation2d> previousTarget = new Container<Rotation2d>(targetSupplier.get());

    return () -> {
      double elapsedTime = timeSinceLastLoop.get();

      if (timeSinceLastLoop.hasElapsed(SwerveConstants.idleTimeUntilReset))
        pidController.reset(getRotation().getRadians());

      boolean ignoreFeedback =
          Math.abs(pidController.getPositionError())
              < SwerveConstants.rotationFeedbackTolerance.in(Radians);

      timeSinceLastLoop.restart();

      Rotation2d target = targetSupplier.get();
      Angle rotationDelta = target.relativeTo(previousTarget.inner).getMeasure();
      previousTarget.inner = target;

      double feedback =
          pidController.calculate(getRotation().getRadians(), targetSupplier.get().getRadians());

      atRotationSetpoint =
          Math.abs(MathUtil.angleModulus(target.relativeTo(getRotation()).getRadians()))
              < SwerveConstants.rotationToleranceBeforeShooting.in(Radians);

      return RadiansPerSecond.of(
          (ignoreFeedback ? feedback : feedback) + rotationDelta.div(elapsedTime).in(Radians));
    };
  }

  public Supplier<AngularVelocity> rotateForShoot(ShootTarget target) {
    return rotateAtAngle(() -> FuelTrajectoryCalculator.getShot(target).fuelGroundSpeedRotation());
  }
}
