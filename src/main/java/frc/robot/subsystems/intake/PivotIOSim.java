// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.IntakeConstants;
import org.ironmaple.simulation.IntakeSimulation;

/** Add your docs here. */
public class PivotIOSim implements PivotIO {
  private final SlewRateLimiter rateLimiter = new SlewRateLimiter(5);
  private Angle position = IntakeConstants.pivotMaxRotation;
  private Angle target = IntakeConstants.pivotMaxRotation;

  private final IntakeSimulation intakeSimulation;

  public PivotIOSim(IntakeSimulation intakeSimulation) {
    this.intakeSimulation = intakeSimulation;
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    position = Radians.of(rateLimiter.calculate(target.in(Radians)));

    if (position.isNear(IntakeConstants.pivotMinRotation, Degrees.of(2)))
      intakeSimulation.startIntake();
    else intakeSimulation.stopIntake();

    inputs.angle = position;
    inputs.angularVelocity = RadiansPerSecond.of(0);
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

  @Override
  public void coast() {
    target =
        position.gt(Degrees.of(90))
            ? IntakeConstants.pivotMaxRotation
            : IntakeConstants.pivotMinRotation;
  }
}
