// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.IndexerConstants;

public class KickerIOReal implements KickerIO {
  SparkMax sparkMax = new SparkMax(IndexerConstants.kickerId, MotorType.kBrushless);

  SimpleMotorFeedforward feedforward = IndexerConstants.kickerControl.simpleFeedforward(Radians);
  PIDController feedback = IndexerConstants.kickerControl.pidController(Radians);
  RelativeEncoder encoder;

  AngularVelocity target = RadiansPerSecond.of(0);

  public KickerIOReal() {
    SparkMaxConfig config = new SparkMaxConfig();

    config.encoder.positionConversionFactor(1.0 / IndexerConstants.kickerReduction);
    config.encoder.velocityConversionFactor(1.0 / IndexerConstants.kickerReduction);
    config.idleMode(IdleMode.kCoast).inverted(true);

    config.smartCurrentLimit(IndexerConstants.kickerCurrentLimit);

    sparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = sparkMax.getEncoder();
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    inputs.position = Rotations.of(encoder.getPosition());
    inputs.velocity = RPM.of(encoder.getVelocity());
    inputs.appliedVoltage = Volts.of(sparkMax.getAppliedOutput() * sparkMax.getBusVoltage());
  }

  @Override
  public void runOpenLoop(double output) {
    sparkMax.setVoltage(output);
  }

  @Override
  public void setAngularVelocity(AngularVelocity velocity) {
    sparkMax.setVoltage(
        feedback.calculate(
                Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()),
                velocity.in(RadiansPerSecond))
            + feedforward.calculate(velocity.in(RadiansPerSecond)));
  }

  @Override
  public void coast() {
    sparkMax.set(0);
  }
}
