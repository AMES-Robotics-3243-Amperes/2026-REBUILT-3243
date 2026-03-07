package frc.robot.commands;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.Container;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class GeneralPurposeCharacterization {
  public static Command sysIdCommand(
      SysIdRoutine routine, Trigger advanceRoutine, Runnable pause, Subsystem requirement) {
    Function<Double, Command> waitCommand =
        waitSeconds ->
            Commands.parallel(
                Commands.waitUntil(advanceRoutine.negate()),
                Commands.waitSeconds(waitSeconds),
                requirement.runOnce(pause));

    Command command =
        Commands.sequence(
            waitCommand.apply(0.5),
            routine.dynamic(SysIdRoutine.Direction.kForward).until(advanceRoutine),
            waitCommand.apply(2.0),
            routine.dynamic(SysIdRoutine.Direction.kReverse).until(advanceRoutine),
            waitCommand.apply(2.0),
            routine.quasistatic(SysIdRoutine.Direction.kForward).until(advanceRoutine),
            waitCommand.apply(2.0),
            routine.quasistatic(SysIdRoutine.Direction.kReverse).until(advanceRoutine));
    command.addRequirements(requirement);
    return command;
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

  public static Command torqueCurrentKsCharacterization(
      Current initialGuess,
      Supplier<AngularVelocity> velocity,
      Consumer<Current> applyCurrent,
      Runnable stop,
      String name,
      Subsystem requirement) {
    Container<Current> upperBound = new Container<Current>(initialGuess.times(2));
    Container<Current> lowerBound = new Container<Current>(Amps.of(0));

    return Commands.sequence(
            requirement.run(stop).until(() -> velocity.get().isEquivalent(RPM.of(0))),
            Commands.waitSeconds(0.5),
            requirement
                .run(() -> applyCurrent.accept(upperBound.inner.plus(lowerBound.inner).div(2)))
                .withTimeout(0.5),
            requirement
                .run(() -> applyCurrent.accept(upperBound.inner.plus(lowerBound.inner).div(2)))
                .until(() -> velocity.get().gt(RPM.of(2e-2)))
                .withTimeout(1.5),
            requirement.runOnce(
                () -> {
                  boolean isRunning = !velocity.get().isNear(RPM.of(0), RPM.of(1e-2));

                  if (isRunning) {
                    upperBound.inner = upperBound.inner.plus(lowerBound.inner).div(2);
                  } else {
                    lowerBound.inner = upperBound.inner.plus(lowerBound.inner).div(2);
                  }
                }))
        .repeatedly()
        .until(() -> upperBound.inner.minus(lowerBound.inner).lt(Amps.of(5e-2)))
        .beforeStarting(
            () -> {
              upperBound.inner = initialGuess.times(2);
              lowerBound.inner = Amps.of(0);
            })
        .finallyDo(
            () -> {
              System.out.println("====================");
              System.out.println(
                  name + " kS: " + upperBound.inner.plus(lowerBound.inner).div(2).toLongString());
              System.out.println("====================");
              stop.run();
            });
  }
}
