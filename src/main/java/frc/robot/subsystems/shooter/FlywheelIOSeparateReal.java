// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.ShooterConstants;
import java.util.function.Supplier;

/** Add your docs here. */
public class FlywheelIOSeparateReal implements FlywheelIO {
  private final TalonFX leader;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> torque;

  private final CoastOut coastRequest = new CoastOut();

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityVoltateRequest = new VelocityVoltage(0);

  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0);

  private boolean isRecoverEnabled = false;

  private static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      StatusCode error = command.get();
      if (error.isOK()) break;
    }
  }

  public FlywheelIOSeparateReal(int id, InvertedValue inverted) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    leader = new TalonFX(id);

    config.MotorOutput.withInverted(inverted).withNeutralMode(NeutralModeValue.Coast);
    config.CurrentLimits.withStatorCurrentLimit(ShooterConstants.flywheelStatorCurrentLimit)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(ShooterConstants.flywheelSupplyCurrentLimit)
        .withSupplyCurrentLimitEnable(true);
    config
        .withSlot0(Slot0Configs.from(ShooterConstants.flywheelControl.talonFXConfigs().getFirst()))
        .withSlot1(
            Slot1Configs.from(ShooterConstants.flywheelRecoverControl.talonFXConfigs().getFirst()))
        .withMotionMagic(ShooterConstants.flywheelControl.talonFXConfigs().getSecond());

    config.Feedback.SensorToMechanismRatio = ShooterConstants.flywheelGearReduction;

    tryUntilOk(5, () -> leader.getConfigurator().apply(config, 0.25));

    position = leader.getPosition();
    velocity = leader.getVelocity();
    voltage = leader.getMotorVoltage();
    torque = leader.getTorqueCurrent();

    // Configure periodic frames
    if (ShooterConstants.flywheelClosedLoopOutput == ClosedLoopOutputType.TorqueCurrentFOC)
      torque.setUpdateFrequency(50.0);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, voltage);
    ParentDevice.optimizeBusUtilizationForAll(leader);
  }

  private int controlSlot() {
    return isRecoverEnabled ? 1 : 0;
  }

  @Override
  public void enableRecoverControl(boolean enable) {
    isRecoverEnabled = enable;
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    if (ShooterConstants.flywheelClosedLoopOutput == ClosedLoopOutputType.TorqueCurrentFOC)
      torque.refresh();
    BaseStatusSignal.refreshAll(position, velocity, voltage);

    inputs.position = position.getValue();
    inputs.velocity = velocity.getValue();
    inputs.appliedVoltage = voltage.getValue();
  }

  @Override
  public void runOpenLoop(double output) {
    leader.setControl(
        switch (ShooterConstants.flywheelClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output).withEnableFOC(true);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  @Override
  public void setAngularVelocity(AngularVelocity velocity) {
    leader.setControl(
        switch (ShooterConstants.flywheelClosedLoopOutput) {
          case Voltage ->
              velocityVoltateRequest
                  .withVelocity(velocity)
                  .withEnableFOC(true)
                  .withSlot(controlSlot());
          case TorqueCurrentFOC ->
              velocityTorqueCurrentRequest.withVelocity(velocity).withSlot(controlSlot());
        });
  }

  @Override
  public void coast() {
    leader.setControl(coastRequest);
  }
}
