// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

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
import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.IntakeConstants;

/** Add your docs here. */
public class PivotIOReal implements PivotIO {
  SparkMax sparkMax = new SparkMax(IntakeConstants.pivotId, MotorType.kBrushless);

  SparkClosedLoopController closedLoopController;
  RelativeEncoder encoder;

  public PivotIOReal() {
    SparkMaxConfig config = new SparkMaxConfig();

    config.encoder.positionConversionFactor(1.0 / IntakeConstants.pivotReduction);
    config.encoder.velocityConversionFactor(1.0 / IntakeConstants.pivotReduction);
    config.idleMode(IdleMode.kCoast).inverted(true);

    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .apply(IntakeConstants.pivotControl.revClosedLoopConfig());

    sparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    closedLoopController = sparkMax.getClosedLoopController();
    encoder = sparkMax.getEncoder();
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.angle = Rotations.of(encoder.getPosition());
    inputs.angularVelocity = RPM.of(encoder.getVelocity());
    inputs.appliedVoltage = Volts.of(sparkMax.getAppliedOutput() * sparkMax.getBusVoltage());
  }

  @Override
  public void resetPosition(Angle angle) {
    encoder.setPosition(angle.in(Rotations));
  }

  @Override
  public void runOpenLoop(double output) {
    sparkMax.setVoltage(output);
  }

  @Override
  public void setAngle(Angle angle) {
    closedLoopController.setSetpoint(angle.in(Rotations), ControlType.kPosition);
  }

  @Override
  public void coast() {
    sparkMax.set(0);
  }
}
