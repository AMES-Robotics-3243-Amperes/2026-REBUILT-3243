// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.ShooterConstants;
import frc.robot.util.ControlConstantsBuilder;
import java.util.function.Supplier;

/** Add your docs here. */
public class FlywheelIOJoinedShaftReal implements FlywheelIO {
  private final TalonFX leader = new TalonFX(ShooterConstants.flywheelLeftId, new CANBus("*"));
  private final TalonFX[] followers =
      new TalonFX[] {
        new TalonFX(ShooterConstants.flywheelRightId, new CANBus("*")),
        new TalonFX(ShooterConstants.flywheelMiddleId, new CANBus("*"))
      };

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

  private static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      StatusCode error = command.get();
      if (error.isOK()) break;
    }
  }

  public FlywheelIOJoinedShaftReal(ControlConstantsBuilder control) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.withInverted(ShooterConstants.shooterFlywheelInverted)
        .withNeutralMode(NeutralModeValue.Coast);
    config.CurrentLimits.withStatorCurrentLimit(ShooterConstants.flywheelStatorCurrentLimit)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(ShooterConstants.flywheelSupplyCurrentLimit)
        .withSupplyCurrentLimitEnable(true);
    config
        .withSlot0(Slot0Configs.from(control.talonFXConfigs().getFirst()))
        .withMotionMagic(control.talonFXConfigs().getSecond());

    config.Feedback.SensorToMechanismRatio = ShooterConstants.flywheelGearReduction;

    tryUntilOk(5, () -> leader.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> followers[0].getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> followers[1].getConfigurator().apply(config, 0.25));

    followers[0].setControl(
        new Follower(ShooterConstants.flywheelLeftId, MotorAlignmentValue.Aligned));
    followers[1].setControl(
        new Follower(ShooterConstants.flywheelLeftId, MotorAlignmentValue.Aligned));

    position = leader.getPosition();
    velocity = leader.getVelocity();
    voltage = leader.getMotorVoltage();
    torque = leader.getTorqueCurrent();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(100.0, position, velocity, voltage, torque);
    ParentDevice.optimizeBusUtilizationForAll(leader, followers[0], followers[1]);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    BaseStatusSignal.refreshAll(position, velocity, voltage, torque);

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
          case Voltage -> velocityVoltateRequest.withVelocity(velocity).withEnableFOC(true);
          case TorqueCurrentFOC -> velocityTorqueCurrentRequest.withVelocity(velocity);
        });
  }

  @Override
  public void coast() {
    leader.setControl(coastRequest);
  }
}
