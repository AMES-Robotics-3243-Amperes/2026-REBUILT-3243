package frc.robot.commands;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SysIdCommand {
  public static Command sysIdCommand(
      SysIdRoutine routine, Trigger advanceRoutine, Runnable pause, Subsystem requirement) {
    Function<Double, Command> waitCommand =
        waitSeconds ->
            Commands.parallel(
                Commands.waitUntil(advanceRoutine.negate()),
                Commands.waitSeconds(waitSeconds),
                requirement.runOnce(pause));

    return Commands.sequence(
        waitCommand.apply(0.5),
        routine.dynamic(SysIdRoutine.Direction.kForward).until(advanceRoutine),
        waitCommand.apply(1.2),
        routine.dynamic(SysIdRoutine.Direction.kReverse).until(advanceRoutine),
        waitCommand.apply(1.2),
        routine.quasistatic(SysIdRoutine.Direction.kForward).until(advanceRoutine),
        waitCommand.apply(1.2),
        routine.quasistatic(SysIdRoutine.Direction.kReverse).until(advanceRoutine));
  }

  public static Command sysIdCommand(
      Trigger advanceRoutine,
      String logKey,
      Velocity<VoltageUnit> rampRate,
      Voltage stepVoltage,
      Time timeout,
      Consumer<Voltage> output,
      Supplier<Angle> position,
      Supplier<AngularVelocity> velocity,
      Supplier<Voltage> voltage,
      Subsystem requirement) {
    SysIdRoutine routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                rampRate,
                stepVoltage,
                timeout,
                (state) -> {
                  Logger.recordOutput(logKey + "/State", state.toString());

                  Logger.recordOutput(logKey + "/PositionRadians", position.get().in(Radians));
                  Logger.recordOutput(
                      logKey + "/VelocityRadiansPerSecond", velocity.get().in(RadiansPerSecond));
                  Logger.recordOutput(logKey + "/AppliedVolts", voltage.get().in(Volts));
                }),
            new SysIdRoutine.Mechanism(output, null, requirement));

    return sysIdCommand(routine, advanceRoutine, () -> output.accept(Volts.of(0)), requirement);
  }

  public static Command sysIdCommand(
      Trigger advanceRoutine,
      String logKey,
      Consumer<Voltage> output,
      Supplier<Angle> position,
      Supplier<AngularVelocity> velocity,
      Supplier<Voltage> voltage,
      Subsystem requirement) {
    return sysIdCommand(
        advanceRoutine, logKey, null, null, null, output, position, velocity, voltage, requirement);
  }
}
