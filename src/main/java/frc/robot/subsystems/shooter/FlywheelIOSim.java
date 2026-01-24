package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.ShooterConstants;

public class FlywheelIOSim implements FlywheelIO {
  Angle position = Radians.of(0);
  AngularVelocity velocity = RadiansPerSecond.of(0);

  public FlywheelIOSim() {}

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    position = position.plus(velocity.times(Milliseconds.of(20)));

    inputs.position = position;
    inputs.velocity = velocity;
    inputs.appliedVoltage =
        Volts.of(
            ShooterConstants.flywheelControl.kV.in(
                    PerUnit.combine(Volts, PerUnit.combine(Radians, Second)))
                * velocity.in(RadiansPerSecond));
  }

  @Override
  public void runOpenLoop(double output) {
    velocity =
        RadiansPerSecond.of(
            output
                / ShooterConstants.flywheelControl.kV.in(
                    PerUnit.combine(Volts, PerUnit.combine(Radians, Second))));
  }

  @Override
  public void setAngularVelocity(AngularVelocity velocity) {
    this.velocity = velocity;
  }

  @Override
  public void coast() {
    this.velocity = RadiansPerSecond.of(0);
  }
}
