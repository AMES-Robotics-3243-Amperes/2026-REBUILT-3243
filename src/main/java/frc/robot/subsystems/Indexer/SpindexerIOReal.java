// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.IndexerConstants;

/** Add your docs here. */
public class SpindexerIOReal implements SpindexerIO {
  SparkMax sparkMax = new SparkMax(IndexerConstants.spindexerId, MotorType.kBrushless);

  SparkClosedLoopController closedLoopController;
  RelativeEncoder encoder;

  public SpindexerIOReal() {
    SparkMaxConfig config = new SparkMaxConfig();

    config.encoder.positionConversionFactor(1.0 / IndexerConstants.spindexerReduction);
    config.encoder.velocityConversionFactor(1.0 / IndexerConstants.spindexerReduction);
    config.idleMode(IdleMode.kCoast).inverted(true);

    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .apply(IndexerConstants.spindexerControl.revClosedLoopConfig());

    sparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    closedLoopController = sparkMax.getClosedLoopController();
    encoder = sparkMax.getEncoder();
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
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
    closedLoopController.setSetpoint(velocity.in(RPM), ControlType.kVelocity);
  }

  @Override
  public void coast() {
    sparkMax.set(0);
  }
}
