// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
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
  protected final TalonFX leader = new TalonFX(ShooterConstants.flywheelLeftId);
  private final TalonFX follower = new TalonFX(ShooterConstants.flywheelRightId);

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private final CoastOut coastRequest = new CoastOut();

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

    flywheelConfig.Feedback.SensorToMechanismRatio = ShooterConstants.flywheelGearReduction;

    leader.getConfigurator().apply(flywheelConfig);
    follower.getConfigurator().apply(flywheelConfig);

    follower.setControl(
        new Follower(
            ShooterConstants.flywheelLeftId,
            MotorAlignmentValue
                .Opposed)); // TODO: I think it should be inverted is wrong but check on the
    // physical robot
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.position = leader.getPosition().getValue();
    inputs.velocity = leader.getVelocity().getValue();
    inputs.appliedVoltage = leader.getMotorVoltage().getValue();
  }

  @Override
  public void runOpenLoop(double output) {
    leader.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public void setAngularVelocity(AngularVelocity velocity) {
    leader.setControl(velocityRequest.withVelocity(velocity));
  }

  @Override
  public void coast() {
    leader.setControl(coastRequest);
  }
}
