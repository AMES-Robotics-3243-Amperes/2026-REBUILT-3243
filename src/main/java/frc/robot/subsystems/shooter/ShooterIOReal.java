// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.ShooterConstants;

/** Add your docs here. */
public class ShooterIOReal implements ShooterIO {
  TalonFX flywheelLeader = new TalonFX(ShooterConstants.flywheelLeaderId);
  TalonFX flywheelFollower = new TalonFX(ShooterConstants.flywheelFollowerId);

  SparkMax hoodSparkMax = new SparkMax(ShooterConstants.hoodId, MotorType.kBrushless);

  SparkClosedLoopController hoodController;
  RelativeEncoder hoodEncoder;

  VoltageOut flywheelVoltageRequest = new VoltageOut(0);
  VelocityVoltage flywheelVelocityRequest = new VelocityVoltage(0);

  public ShooterIOReal() {
    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();

    flywheelConfig
        .MotorOutput
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Coast);
    flywheelConfig.CurrentLimits.withSupplyCurrentLimit(70).withSupplyCurrentLimitEnable(true);
    flywheelConfig
        .Slot0
        .withKS(ShooterConstants.flywheelKs)
        .withKV(ShooterConstants.flywheelKv)
        .withKA(ShooterConstants.flywheelKa)
        .withKP(ShooterConstants.flywheelKp)
        .withKI(ShooterConstants.flywheelKi)
        .withKD(ShooterConstants.flywheelKd);

    flywheelConfig.Feedback.withSensorToMechanismRatio(ShooterConstants.flywheelGearRatio);

    flywheelLeader.getConfigurator().apply(flywheelConfig);
    flywheelFollower.getConfigurator().apply(flywheelConfig);

    flywheelFollower.setControl(
        new Follower(ShooterConstants.flywheelLeaderId, MotorAlignmentValue.Aligned));

    SparkMaxConfig hoodConfig = new SparkMaxConfig();

    hoodConfig.encoder.positionConversionFactor(ShooterConstants.hoodGearRatio);
    hoodConfig.encoder.velocityConversionFactor(ShooterConstants.hoodGearRatio);
    hoodConfig.idleMode(IdleMode.kCoast).inverted(true);

    hoodConfig
        .closedLoop
        .outputRange(
            -ShooterConstants.hoodMaxOutput, ShooterConstants.hoodMaxOutput, ClosedLoopSlot.kSlot0)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(
            ShooterConstants.hoodKp,
            ShooterConstants.hoodKi,
            ShooterConstants.hoodKd,
            ClosedLoopSlot.kSlot0);
    // .feedForward
    // .sva(
    //     IntakeConstants.hoodKs,
    //     IntakeConstants.hoodKv,
    //     IntakeConstants.hoodKa,
    //     ClosedLoopSlot.kSlot0);

    hoodSparkMax.configure(
        hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    hoodController = hoodSparkMax.getClosedLoopController();
    hoodEncoder = hoodSparkMax.getEncoder();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.flywheelPosition = Rotations.of(flywheelLeader.getPosition().getValueAsDouble());
    inputs.flywheelVelocity = RPM.of(flywheelLeader.getVelocity().getValueAsDouble());
    inputs.flywheelAppliedVoltage = Volts.of(flywheelLeader.getMotorVoltage().getValueAsDouble());
    inputs.flywheelStatorCurrent = Amps.of(flywheelLeader.getStatorCurrent().getValueAsDouble());
    inputs.flywheelSupplyCurrent = Amps.of(flywheelLeader.getSupplyCurrent().getValueAsDouble());

    inputs.hoodAngle = Rotations.of(hoodEncoder.getPosition());
    inputs.hoodAngularVelocity = RPM.of(hoodEncoder.getVelocity());
    inputs.hoodAppliedVoltage = Volts.of(hoodSparkMax.getAppliedOutput());
    inputs.hoodCurrent = Amps.of(hoodSparkMax.getOutputCurrent());
  }

  @Override
  public void runFlywheelOpenLoop(double output) {
    flywheelLeader.setControl(flywheelVoltageRequest.withOutput(output));
  }

  @Override
  public void setFlywheelAngularVelocity(AngularVelocity velocity) {
    flywheelLeader.setControl(
        flywheelVelocityRequest.withVelocity(velocity.in(RotationsPerSecond)));
  }

  @Override
  public void runHoodOpenLoop(double output) {
    hoodSparkMax.setVoltage(output);
  }

  @Override
  public void setHoodAngle(Angle angle) {
    hoodController.setSetpoint(angle.in(Rotations), ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }
}
