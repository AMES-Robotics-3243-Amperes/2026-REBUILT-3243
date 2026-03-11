// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.IntakeConstants;
import org.ironmaple.simulation.IntakeSimulation;

/** Add your docs here. */
public class PivotIOSim implements PivotIO {
  private Angle position = IntakeConstants.pivotMaxRotation;
  private AngularVelocity velocity = RPM.of(0);
  private double output = 0;

  private final IntakeSimulation intakeSimulation;

  public PivotIOSim(IntakeSimulation intakeSimulation) {
    this.intakeSimulation = intakeSimulation;
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    if (position.isNear(IntakeConstants.pivotMinRotation, Degrees.of(2)))
      intakeSimulation.startIntake();
    else intakeSimulation.stopIntake();

    inputs.angle = position;
    inputs.angularVelocity = velocity;
    inputs.appliedVoltage = Volts.of(output);
  }

  @Override
  public void runOpenLoop(double output) {
    velocity = DegreesPerSecond.of(45).times(output);
    position = position.plus(velocity.times(Milliseconds.of(20)));
    Angle clampedPos =
        Radians.of(
            MathUtil.clamp(
                position.in(Radians),
                IntakeConstants.pivotMinRotation.in(Radians),
                IntakeConstants.pivotMaxRotation.in(Radians)));

    if (!position.isEquivalent(clampedPos)) {
      velocity = RPM.of(0);
      this.output = 0;

      if (output < 0) intakeSimulation.startIntake();
    } else if (output > 0) {
      this.output = output;
      this.intakeSimulation.stopIntake();
    } else {
      this.output = output;
    }

    position = clampedPos;
  }

  @Override
  public void coast() {
    velocity = RPM.of(0);
  }
}
