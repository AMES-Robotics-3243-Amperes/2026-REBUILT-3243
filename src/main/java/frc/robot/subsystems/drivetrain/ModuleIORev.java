package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.util.ControlConstantsBuilder;

public class ModuleIORev implements ModuleIO {
  Rotation2d offset;

  SparkMax driveMotor;
  SparkMax azimuth;
  SparkClosedLoopController driveController;
  SparkClosedLoopController azimuthController;
  AbsoluteEncoder azimuthEncoder;
  RelativeEncoder driveEncoder;

  public ModuleIORev(int driveId, int turnId, Rotation2d offset) {
    ControlConstantsBuilder driveControl =
        ControlConstantsBuilder.fromRadiansAndSeconds()
            .pid(0.0001, 0, 0)
            .sva(0.1396, 0.10411, 0.013093);

    // Hardcoded constants for now. If we ever get an actual prototype bot on a rev drivetrain
    // perhaps this can be improved
    final double factor = (45.0 * 22) / (13 * 15);

    this.offset = offset;
    driveMotor = new SparkMax(driveId, MotorType.kBrushless);
    azimuth = new SparkMax(turnId, MotorType.kBrushless);

    SparkMaxConfig azimuthConfig = new SparkMaxConfig();
    azimuthConfig
        .closedLoop
        .pid(1.8, 0, 0)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, 1)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    azimuthConfig
        .absoluteEncoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1)
        .inverted(true);
    azimuth.configure(
        azimuthConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig
        .closedLoop
        .apply(driveControl.revClosedLoopConfig())
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    driveConfig.encoder.positionConversionFactor(1 / factor).velocityConversionFactor(1 / factor);
    driveConfig.idleMode(IdleMode.kBrake);
    driveMotor.configure(
        driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    azimuthController = azimuth.getClosedLoopController();
    driveController = driveMotor.getClosedLoopController();
    azimuthEncoder = azimuth.getAbsoluteEncoder();
    driveEncoder = driveMotor.getEncoder();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePosition = Rotations.of(driveEncoder.getPosition());
    inputs.driveVelocity = RPM.of(driveEncoder.getVelocity());
    inputs.driveAppliedVolts = Volts.of(driveMotor.getAppliedOutput() * driveMotor.getBusVoltage());
    inputs.driveCurrent = Amps.of(driveMotor.getOutputCurrent());

    // Update turn inputs
    inputs.turnConnected = true;
    inputs.turnEncoderConnected = true;
    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(azimuthEncoder.getPosition()).plus(offset);
    inputs.turnPosition = Rotations.of(azimuthEncoder.getPosition() + offset.getRotations());
    inputs.turnVelocity = RPM.of(azimuthEncoder.getVelocity());
    inputs.turnAppliedVolts = Volts.of(azimuth.getAppliedOutput() * azimuth.getBusVoltage());
    inputs.turnCurrentAmps = Amps.of(azimuth.getOutputCurrent());

    inputs.odometryTimestamps = new double[] {RobotController.getFPGATime() / 1e6};
    inputs.odometryDrivePositionsRad = new double[] {driveEncoder.getPosition()};
    inputs.odometryTurnPositions =
        new Rotation2d[] {Rotation2d.fromRotations(azimuthEncoder.getPosition()).plus(offset)};
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveMotor.setVoltage(output);
  }

  @Override
  public void setTurnOpenLoop(double output) {
    azimuth.setVoltage(output);
  }

  @Override
  public void setDriveVelocity(AngularVelocity velocity) {
    driveController.setSetpoint(velocity.in(RPM), ControlType.kVelocity);
  }

  @Override
  public void setDriveSetpoint(AngularVelocity velocity, AngularAcceleration acceleration) {
    driveController.setSetpoint(velocity.in(RPM), ControlType.kVelocity);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    azimuthController.setSetpoint(rotation.minus(offset).getRotations(), ControlType.kPosition);
  }
}
