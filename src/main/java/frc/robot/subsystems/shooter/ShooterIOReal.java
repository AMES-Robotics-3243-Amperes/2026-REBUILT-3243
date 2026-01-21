// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
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
  TalonFX shooterLeader = new TalonFX(ShooterConstants.shooterLeaderId);
  TalonFX shooterFollower = new TalonFX(ShooterConstants.shooterFollowerId);

  SparkMax hoodSparkMax = new SparkMax(ShooterConstants.hoodId, MotorType.kBrushless);

  SparkClosedLoopController hoodController;
  RelativeEncoder hoodEncoder;

  VoltageOut shooterVoltageRequest = new VoltageOut(0);
  VelocityVoltage shooterVelocityRequest = new VelocityVoltage(0);

  public ShooterIOReal() {
    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

    shooterConfig
        .MotorOutput
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Coast);
    shooterConfig.CurrentLimits.withSupplyCurrentLimit(70).withSupplyCurrentLimitEnable(true);
    shooterConfig
        .Slot0
        .withKS(ShooterConstants.shooterKs)
        .withKV(ShooterConstants.shooterKv)
        .withKA(ShooterConstants.shooterKa)
        .withKP(ShooterConstants.shooterKp)
        .withKI(ShooterConstants.shooterKi)
        .withKD(ShooterConstants.shooterKd);

    shooterLeader.getConfigurator().apply(shooterConfig);
    shooterFollower.getConfigurator().apply(shooterConfig);

    shooterFollower.setControl(
        new Follower(ShooterConstants.shooterLeaderId, MotorAlignmentValue.Aligned));

    SparkMaxConfig hoodConfig = new SparkMaxConfig();

    hoodConfig.encoder.positionConversionFactor(ShooterConstants.hoodPositionConversionFactor);
    hoodConfig.encoder.velocityConversionFactor(ShooterConstants.hoodVelocityConversionFactor);
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
    inputs.shooterPosition = Rotations.of(shooterLeader.getPosition().getValueAsDouble());
    inputs.shooterVelocity = RPM.of(shooterLeader.getVelocity().getValueAsDouble() * 60);
    inputs.shooterAppliedVoltage = Volts.of(shooterLeader.getMotorVoltage().getValueAsDouble());
    inputs.shooterStatorCurrent = Amps.of(shooterLeader.getStatorCurrent().getValueAsDouble());
    inputs.shooterSupplyCurrent = Amps.of(shooterLeader.getSupplyCurrent().getValueAsDouble());

    inputs.hoodAngle = Degrees.of(hoodEncoder.getPosition());
    inputs.hoodAngularVelocity = DegreesPerSecond.of(hoodEncoder.getVelocity());
    inputs.hoodAppliedVoltage = Volts.of(hoodSparkMax.getAppliedOutput());
    inputs.hoodCurrent = Amps.of(hoodSparkMax.getOutputCurrent());
  }

  @Override
  public void runShooterOpenLoop(double output) {
    shooterLeader.setControl(shooterVoltageRequest.withOutput(output));
  }

  @Override
  public void setShooterAngularVelocity(AngularVelocity velocity) {
    shooterLeader.setControl(shooterVelocityRequest.withVelocity(velocity.in(RotationsPerSecond)));
  }

  @Override
  public void runHoodOpenLoop(double output) {
    hoodSparkMax.setVoltage(output);
  }

  @Override
  public void setHoodAngle(Angle angle) {
    hoodController.setSetpoint(angle.in(Degrees), ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }
}
