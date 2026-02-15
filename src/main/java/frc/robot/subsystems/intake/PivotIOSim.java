// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.IntakeConstants;

/** Add your docs here. */
public class PivotIOSim implements PivotIO {
  SlewRateLimiter rateLimiter = new SlewRateLimiter(5);
  Angle position = IntakeConstants.pivotMaxRotation;
  Angle target = IntakeConstants.pivotMaxRotation;

  public PivotIOSim() {}

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    position = Radians.of(rateLimiter.calculate(target.in(Radians)));

    inputs.angle = position;
    inputs.angularVelocity = RPM.of(0);
    inputs.appliedVoltage = Volts.of(0);
  }

  @Override
  public void resetPosition(Angle angle) {
    position = angle;
  }

  @Override
  public void setAngle(Angle angle) {
    target = angle;
  }
}
