// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
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
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.ShooterConstants;
import java.util.function.Supplier;

/** Add your docs here. */
public class FlywheelIOReal implements FlywheelIO {
  protected final TalonFX leader = new TalonFX(ShooterConstants.flywheelLeaderId);
  private final TalonFX follower = new TalonFX(ShooterConstants.flywheelFollowerId);

  protected final StatusSignal<Angle> position;
  protected final StatusSignal<AngularVelocity> velocity;
  protected final StatusSignal<Voltage> voltage;

  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final VelocityVoltage velocityVoltateRequest = new VelocityVoltage(0).withEnableFOC(true);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0);
  private final CoastOut coastRequest = new CoastOut();

  private static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      StatusCode error = command.get();
      if (error.isOK()) break;
    }
  }

  public FlywheelIOReal() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.withInverted(ShooterConstants.shooterFlywheelInverted)
        .withNeutralMode(NeutralModeValue.Coast);
    config.CurrentLimits.withSupplyCurrentLimit(ShooterConstants.flywheelSupplyCurrentLimit)
        .withSupplyCurrentLimitEnable(true);
    config
        .withSlot0(Slot0Configs.from(ShooterConstants.flywheelControl.talonFXConfigs().getFirst()))
        .withMotionMagic(ShooterConstants.flywheelControl.talonFXConfigs().getSecond());

    config.Feedback.SensorToMechanismRatio = ShooterConstants.flywheelGearReduction;

    tryUntilOk(5, () -> leader.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> leader.getConfigurator().apply(config, 0.25));

    follower.setControl(
        new Follower(ShooterConstants.flywheelLeaderId, MotorAlignmentValue.Opposed));

    position = leader.getPosition();
    velocity = leader.getVelocity();
    voltage = leader.getMotorVoltage();

    // Configure periodic frames
    if (ShooterConstants.flywheelClosedLoopOutput == ClosedLoopOutputType.TorqueCurrentFOC)
      leader.getTorqueCurrent().setUpdateFrequency(50.0);
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
    leader.setControl(
        switch (ShooterConstants.flywheelClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  @Override
  public void setAngularVelocity(AngularVelocity velocity) {
    leader.setControl(
        switch (ShooterConstants.flywheelClosedLoopOutput) {
          case Voltage -> velocityVoltateRequest.withVelocity(velocity);
          case TorqueCurrentFOC -> velocityTorqueCurrentRequest.withVelocity(velocity);
        });
  }

  @Override
  public void coast() {
    leader.setControl(coastRequest);
  }
}
