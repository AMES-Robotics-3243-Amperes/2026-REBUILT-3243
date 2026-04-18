// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
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
  RelativeEncoder relativeEncoder;
  AbsoluteEncoder absoluteEncoder;

  public PivotIOReal() {
    SparkMaxConfig config = new SparkMaxConfig();

    config.encoder.positionConversionFactor(1.0 / IntakeConstants.pivotReduction);
    config.encoder.velocityConversionFactor(1.0 / IntakeConstants.pivotReduction);

    config.absoluteEncoder.positionConversionFactor(1.0);
    config.absoluteEncoder.velocityConversionFactor(1.0);
    config.absoluteEncoder.inverted(true);

    config.idleMode(IdleMode.kCoast).inverted(true);

    config.smartCurrentLimit(IntakeConstants.pivotCurrentLimit, IntakeConstants.pivotCurrentLimit);

    sparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    closedLoopController = sparkMax.getClosedLoopController();
    relativeEncoder = sparkMax.getEncoder();
    absoluteEncoder = sparkMax.getAbsoluteEncoder();
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    Angle absoluteEncoderPosition =
        Rotations.of(absoluteEncoder.getPosition())
            .plus(IntakeConstants.pivotAbsoluteEncoderZeroAngle);

    if (absoluteEncoderPosition.lt(IntakeConstants.pivotAngleWrap.minus(Degrees.of(360))))
      absoluteEncoderPosition = absoluteEncoderPosition.plus(Degrees.of(360));
    else if (absoluteEncoderPosition.gt(IntakeConstants.pivotAngleWrap))
      absoluteEncoderPosition = absoluteEncoderPosition.minus(Degrees.of(360));

    inputs.absoluteEncoderPosition = absoluteEncoderPosition;
    inputs.absoluteEncoderVelocity = RPM.of(absoluteEncoder.getVelocity());

    inputs.internalEncoderPosition = Rotations.of(relativeEncoder.getPosition());
    inputs.internalEncoderVelocity = RPM.of(relativeEncoder.getVelocity());

    inputs.appliedVoltage = Volts.of(sparkMax.getAppliedOutput() * sparkMax.getBusVoltage());
  }

  @Override
  public void runOpenLoop(double output) {
    sparkMax.setVoltage(output);
  }

  @Override
  public void coast() {
    sparkMax.set(0);
  }
}
