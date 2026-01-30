package frc.robot.constants.swerve;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import edu.wpi.first.units.measure.*;

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
              .withDriveMotorGearRatio(ModuleConstants.driveGearRatio)
              .withSteerMotorGearRatio(ModuleConstants.steerGearRatio)
              .withCouplingGearRatio(ModuleConstants.kCoupleRatio)
              .withWheelRadius(ModuleConstants.wheelRadius)
              .withSteerMotorGains(
                  Slot0Configs.from(ModuleConstants.steerControl.talonFXSlotConfigs()))
              .withDriveMotorGains(
                  Slot0Configs.from(ModuleConstants.driveControl.talonFXSlotConfigs()))
              .withSteerMotorClosedLoopOutput(ModuleConstants.steerClosedLoopOutput)
              .withDriveMotorClosedLoopOutput(ModuleConstants.driveClosedLoopOutput)
              .withSlipCurrent(SwerveConstants.slipCurrent)
              .withSpeedAt12Volts(SwerveConstants.speedAt12Volts)
              .withDriveMotorType(DriveMotorArrangement.TalonFX_Integrated)
              .withSteerMotorType(SteerMotorArrangement.TalonFX_Integrated)
              .withFeedbackSource(ModuleConstants.steerFeedbackType)
              .withDriveMotorInitialConfigs(new TalonFXConfiguration())
              .withSteerMotorInitialConfigs(
                  new TalonFXConfiguration()
                      .withCurrentLimits(
                          new CurrentLimitsConfigs()
                              .withStatorCurrentLimit(ModuleConstants.steerStatorCurrentLimit)
                              .withStatorCurrentLimitEnable(true)))
              .withEncoderInitialConfigs(new CANcoderConfiguration())
              .withSteerInertia(ModuleConstants.kSteerInertia)
              .withDriveInertia(ModuleConstants.kDriveInertia)
              .withSteerFrictionVoltage(ModuleConstants.steerFrictionVoltage)
              .withDriveFrictionVoltage(ModuleConstants.driveFrictionVoltage);

  // Front Left
  private static final int kFrontLeftDriveMotorId = 1;
  private static final int kFrontLeftSteerMotorId = 2;
  private static final int kFrontLeftEncoderId = 1;
  private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.1923828125);
  private static final boolean kFrontLeftSteerMotorInverted = false;
  private static final boolean kFrontLeftEncoderInverted = false;

  private static final Distance kFrontLeftXPos = SwerveConstants.driveBaseSideLength.div(2);
  private static final Distance kFrontLeftYPos = SwerveConstants.driveBaseFrontLength.div(2);

  // Front Right
  private static final int kFrontRightDriveMotorId = 3;
  private static final int kFrontRightSteerMotorId = 4;
  private static final int kFrontRightEncoderId = 2;
  private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.218017578125);
  private static final boolean kFrontRightSteerMotorInverted = false;
  private static final boolean kFrontRightEncoderInverted = false;

  private static final Distance kFrontRightXPos = SwerveConstants.driveBaseSideLength.div(2);
  private static final Distance kFrontRightYPos =
      SwerveConstants.driveBaseFrontLength.div(2).unaryMinus();

  // Back Left
  private static final int kBackLeftDriveMotorId = 5;
  private static final int kBackLeftSteerMotorId = 6;
  private static final int kBackLeftEncoderId = 3;
  private static final Angle kBackLeftEncoderOffset = Rotations.of(0.262451171875);
  private static final boolean kBackLeftSteerMotorInverted = false;
  private static final boolean kBackLeftEncoderInverted = false;

  private static final Distance kBackLeftXPos =
      SwerveConstants.driveBaseSideLength.div(2).unaryMinus();
  private static final Distance kBackLeftYPos = SwerveConstants.driveBaseFrontLength.div(2);

  // Back Right
  private static final int kBackRightDriveMotorId = 7;
  private static final int kBackRightSteerMotorId = 8;
  private static final int kBackRightEncoderId = 4;
  private static final Angle kBackRightEncoderOffset = Rotations.of(-0.396728515625);
  private static final boolean kBackRightSteerMotorInverted = false;
  private static final boolean kBackRightEncoderInverted = false;

  private static final Distance kBackRightXPos =
      SwerveConstants.driveBaseSideLength.div(2).unaryMinus();
  private static final Distance kBackRightYPos =
      SwerveConstants.driveBaseFrontLength.div(2).unaryMinus();

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
