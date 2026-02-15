// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.ShooterConstants;

/** Add your docs here. */
public class FlywheelIOReal implements FlywheelIO {
  protected final TalonFX leader = new TalonFX(ShooterConstants.flywheelLeaderId);
  private final TalonFX follower = new TalonFX(ShooterConstants.flywheelFollowerId);

  protected final StatusSignal<Angle> position;
  protected final StatusSignal<AngularVelocity> velocity;
  protected final StatusSignal<Voltage> voltage;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private final CoastOut coastRequest = new CoastOut();

  public FlywheelIOReal() {
    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();

    flywheelConfig
        .MotorOutput
        .withInverted(ShooterConstants.shooterFlywheelInverted)
        .withNeutralMode(NeutralModeValue.Coast);
    flywheelConfig
        .CurrentLimits
        .withSupplyCurrentLimit(ShooterConstants.flywheelSupplyCurrentLimit)
        .withSupplyCurrentLimitEnable(true);
    flywheelConfig
        .withSlot0(Slot0Configs.from(ShooterConstants.flywheelControl.talonFXConfigs().getFirst()))
        .withMotionMagic(ShooterConstants.flywheelControl.talonFXConfigs().getSecond());

    flywheelConfig.Feedback.SensorToMechanismRatio = ShooterConstants.flywheelGearReduction;

    leader.getConfigurator().apply(flywheelConfig);
    follower.getConfigurator().apply(flywheelConfig);

    follower.setControl(
        new Follower(ShooterConstants.flywheelLeaderId, MotorAlignmentValue.Opposed));

    position = leader.getPosition();
    velocity = leader.getVelocity();
    voltage = leader.getMotorVoltage();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, voltage);
    ParentDevice.optimizeBusUtilizationForAll(leader, follower);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    BaseStatusSignal.refreshAll(position, velocity, voltage);

    inputs.position = position.getValue();
    inputs.velocity = velocity.getValue();
    inputs.appliedVoltage = voltage.getValue();
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
