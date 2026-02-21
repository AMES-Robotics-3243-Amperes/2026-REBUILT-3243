package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
  @AutoLog
  class RollerIOInputs {
    Angle position = Radians.of(0);
    AngularVelocity velocity = RadiansPerSecond.of(0);
    Voltage appliedVoltage = Volts.of(0);
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(RollerIOInputs inputs) {}

  /** Run the motors at the specified open loop value. */
  default void runOpenLoop(double output) {}

  /** Run the motors at the specified velocity. */
  default void setAngularVelocity(AngularVelocity velocity) {}

  /** Coasts the roller. */
  default void coast() {}
}
