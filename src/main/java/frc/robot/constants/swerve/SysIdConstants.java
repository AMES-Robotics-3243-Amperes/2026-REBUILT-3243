package frc.robot.constants.swerve;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.drivetrain.SwerveSubsystem.SwerveSysIdRoutine;

public class SysIdConstants {
  public static final SwerveSysIdRoutine activeRoutine =
      SwerveSysIdRoutine.DRIVE_LINEAR_FEEDFORWARD;

  public static final Velocity<VoltageUnit> sysIdRampRate(SwerveSysIdRoutine routine) {
    switch (routine) {
      case DRIVE_LINEAR_FEEDFORWARD:
      case AZIMUTH_FEEDFORWARD:
        return Volts.per(Second).of(0.4);
    }

    return null;
  }

  public static final Voltage sysIdStepVoltage(SwerveSysIdRoutine routine) {
    switch (routine) {
      case DRIVE_LINEAR_FEEDFORWARD:
      case AZIMUTH_FEEDFORWARD:
        return Volts.of(2.5);
    }

    return null;
  }

  public static final Time sysIdTimeout = Seconds.of(8);
}
