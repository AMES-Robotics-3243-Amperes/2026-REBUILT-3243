package frc.robot.constants.swerve;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.choreo.ChoreoVars;

public class TunerConstants {
  private static final boolean kInvertLeftSide = false;
  private static final boolean kInvertRightSide = true;

  private static final int kPigeonId = 1;

  public static final SwerveDrivetrainConstants DrivetrainConstants =
      new SwerveDrivetrainConstants()
          .withCANBusName(SwerveConstants.swerveCANBus.getName())
          .withPigeon2Id(kPigeonId)
          .withPigeon2Configs(new Pigeon2Configuration());

  private static final SwerveModuleConstantsFactory<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      ConstantCreator =
          new SwerveModuleConstantsFactory<
                  TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
              .withDriveMotorGearRatio(ChoreoVars.R_DriveReduction)
              .withSteerMotorGearRatio(ModuleConstants.steerReduction)
              .withCouplingGearRatio(ModuleConstants.kCoupleRatio)
              .withWheelRadius(ChoreoVars.R_WheelRadius)
              .withSteerMotorGains(
                  Slot0Configs.from(ModuleConstants.steerControl.talonFXConfigs().getFirst()))
              .withDriveMotorGains(
                  Slot0Configs.from(ModuleConstants.driveControl.talonFXConfigs().getFirst()))
              .withSteerMotorClosedLoopOutput(ModuleConstants.steerClosedLoopOutput)
              .withDriveMotorClosedLoopOutput(ModuleConstants.driveClosedLoopOutput)
              .withSlipCurrent(ModuleConstants.slipCurrent)
              .withSpeedAt12Volts(SwerveConstants.speedAt12Volts)
              .withDriveMotorType(DriveMotorArrangement.TalonFX_Integrated)
              .withSteerMotorType(SteerMotorArrangement.TalonFX_Integrated)
              .withFeedbackSource(ModuleConstants.steerFeedbackType)
              .withDriveMotorInitialConfigs(
                  new TalonFXConfiguration()
                      .withCurrentLimits(
                          new CurrentLimitsConfigs()
                              .withSupplyCurrentLimit(ModuleConstants.driveSupplyCurrentLimit)
                              .withSupplyCurrentLimitEnable(true)))
              .withSteerMotorInitialConfigs(
                  new TalonFXConfiguration()
                      .withCurrentLimits(
                          new CurrentLimitsConfigs()
                              .withSupplyCurrentLimit(ModuleConstants.steerStatorCurrentLimit)
                              .withSupplyCurrentLimitEnable(true)))
              .withEncoderInitialConfigs(new CANcoderConfiguration())
              .withSteerInertia(ModuleConstants.kSteerInertia)
              .withDriveInertia(ModuleConstants.kDriveInertia)
              .withSteerFrictionVoltage(ModuleConstants.steerFrictionVoltage)
              .withDriveFrictionVoltage(ModuleConstants.driveFrictionVoltage);

  // Front Left
  private static final int kFrontLeftDriveMotorId = 5;
  private static final int kFrontLeftSteerMotorId = 6;
  private static final int kFrontLeftEncoderId = 3;
  private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.00927734375);
  private static final boolean kFrontLeftSteerMotorInverted = false;
  private static final boolean kFrontLeftEncoderInverted = false;

  private static final Distance kFrontLeftXPos = Inches.of(10.75);
  private static final Distance kFrontLeftYPos = Inches.of(10.75);

  // Front Right
  private static final int kFrontRightDriveMotorId = 7;
  private static final int kFrontRightSteerMotorId = 8;
  private static final int kFrontRightEncoderId = 4;
  private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.146484375);
  private static final boolean kFrontRightSteerMotorInverted = false;
  private static final boolean kFrontRightEncoderInverted = false;

  private static final Distance kFrontRightXPos = Inches.of(10.75);
  private static final Distance kFrontRightYPos = Inches.of(-10.75);

  // Back Left
  private static final int kBackLeftDriveMotorId = 3;
  private static final int kBackLeftSteerMotorId = 4;
  private static final int kBackLeftEncoderId = 2;
  private static final Angle kBackLeftEncoderOffset = Rotations.of(-0.226806640625);
  private static final boolean kBackLeftSteerMotorInverted = false;
  private static final boolean kBackLeftEncoderInverted = false;

  private static final Distance kBackLeftXPos = Inches.of(-10.75);
  private static final Distance kBackLeftYPos = Inches.of(10.75);

  // Back Right
  private static final int kBackRightDriveMotorId = 1;
  private static final int kBackRightSteerMotorId = 2;
  private static final int kBackRightEncoderId = 1;
  private static final Angle kBackRightEncoderOffset = Rotations.of(0.20166015625);
  private static final boolean kBackRightSteerMotorInverted = false;
  private static final boolean kBackRightEncoderInverted = false;

  private static final Distance kBackRightXPos = Inches.of(-10.75);
  private static final Distance kBackRightYPos = Inches.of(-10.75);

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontLeft =
          ConstantCreator.createModuleConstants(
              kFrontLeftSteerMotorId,
              kFrontLeftDriveMotorId,
              kFrontLeftEncoderId,
              kFrontLeftEncoderOffset,
              kFrontLeftXPos,
              kFrontLeftYPos,
              kInvertLeftSide,
              kFrontLeftSteerMotorInverted,
              kFrontLeftEncoderInverted);
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontRight =
          ConstantCreator.createModuleConstants(
              kFrontRightSteerMotorId,
              kFrontRightDriveMotorId,
              kFrontRightEncoderId,
              kFrontRightEncoderOffset,
              kFrontRightXPos,
              kFrontRightYPos,
              kInvertRightSide,
              kFrontRightSteerMotorInverted,
              kFrontRightEncoderInverted);
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackLeft =
          ConstantCreator.createModuleConstants(
              kBackLeftSteerMotorId,
              kBackLeftDriveMotorId,
              kBackLeftEncoderId,
              kBackLeftEncoderOffset,
              kBackLeftXPos,
              kBackLeftYPos,
              kInvertLeftSide,
              kBackLeftSteerMotorInverted,
              kBackLeftEncoderInverted);
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackRight =
          ConstantCreator.createModuleConstants(
              kBackRightSteerMotorId,
              kBackRightDriveMotorId,
              kBackRightEncoderId,
              kBackRightEncoderOffset,
              kBackRightXPos,
              kBackRightYPos,
              kInvertRightSide,
              kBackRightSteerMotorInverted,
              kBackRightEncoderInverted);
}
