// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.ShooterConstants;
import frc.robot.util.ControlConstantsBuilder.ControlConstants;

/** Add your docs here. */
public class FlywheelIOTalonFX implements FlywheelIO {
  private final TalonFX flywheelLeader = new TalonFX(ShooterConstants.flywheelLeftId);
  private final TalonFX flywheelFollower = new TalonFX(ShooterConstants.flywheelRightId);

  private final VoltageOut flywheelVoltageRequest = new VoltageOut(0);
  private final VelocityVoltage flywheelVelocityRequest = new VelocityVoltage(0);

  public FlywheelIOTalonFX() {
    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();

    ControlConstants flywheelControl =
        ShooterConstants.flywheelControl.in(Rotations, Volts, Seconds);

    flywheelConfig
        .MotorOutput
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Coast);
    flywheelConfig
        .CurrentLimits
        .withSupplyCurrentLimit(ShooterConstants.flywheelSupplyCurrentLimit)
        .withSupplyCurrentLimitEnable(true);
    flywheelConfig
        .Slot0
        .withKS(flywheelControl.kS())
        .withKV(flywheelControl.kV())
        .withKA(flywheelControl.kA())
        .withKP(flywheelControl.kP())
        .withKI(flywheelControl.kI())
        .withKD(flywheelControl.kD());

    flywheelConfig.Feedback.withSensorToMechanismRatio(ShooterConstants.flywheelGearReduction);

    flywheelLeader.getConfigurator().apply(flywheelConfig);
    flywheelFollower.getConfigurator().apply(flywheelConfig);

    flywheelFollower.setControl(
        new Follower(
            ShooterConstants.flywheelLeftId,
            MotorAlignmentValue
                .Aligned)); // TODO: I think it should be inverted is wrong but check on the
    // physical robot
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.position = flywheelLeader.getPosition().getValue();
    inputs.velocity = flywheelLeader.getVelocity().getValue();
    inputs.appliedVoltage = flywheelLeader.getMotorVoltage().getValue();
  }

  @Override
  public void runOpenLoop(double output) {
    flywheelLeader.setControl(flywheelVoltageRequest.withOutput(output));
  }

  @Override
  public void setAngularVelocity(AngularVelocity velocity) {
    flywheelLeader.setControl(flywheelVelocityRequest.withVelocity(velocity));
  }
}
