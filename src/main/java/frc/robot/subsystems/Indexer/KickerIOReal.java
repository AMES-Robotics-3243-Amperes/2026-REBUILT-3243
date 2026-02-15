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
import frc.robot.util.TunableControls;

/** Add your docs here. */
public class KickerIOReal implements KickerIO {
  SparkMax sparkMax = new SparkMax(IndexerConstants.kickerId, MotorType.kBrushless);

  SparkClosedLoopController closedLoopController;
  RelativeEncoder encoder;

  public KickerIOReal() {
    SparkMaxConfig config = new SparkMaxConfig();

    TunableControls.registerSparkMaxClosedLoopTuning(
        sparkMax, "Kicker/Indexer", IndexerConstants.kickerControl);

    config.encoder.positionConversionFactor(1.0 / IndexerConstants.kickerReduction);
    config.encoder.velocityConversionFactor(1.0 / IndexerConstants.kickerReduction);
    config.idleMode(IdleMode.kCoast).inverted(true);

    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .apply(IndexerConstants.kickerControl.revClosedLoopConfig());

    sparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    closedLoopController = sparkMax.getClosedLoopController();
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
    closedLoopController.setSetpoint(velocity.in(RPM), ControlType.kVelocity);
  }
}
