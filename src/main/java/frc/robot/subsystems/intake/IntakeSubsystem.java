// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SysIdCommand;
import frc.robot.constants.IntakeConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  private final RollerIO rollerIO;
  private final RollerIOInputsAutoLogged rollerInputs = new RollerIOInputsAutoLogged();

  public final PivotIO pivotIO;
  private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(RollerIO rollerIO, PivotIO pivotIO) {
    this.rollerIO = rollerIO;
    this.pivotIO = pivotIO;

    pivotIO.resetPosition(IntakeConstants.pivotMaxRotation);
  }

  @Override
  public void periodic() {
    rollerIO.updateInputs(rollerInputs);
    Logger.processInputs("Intake/Roller", rollerInputs);

    pivotIO.updateInputs(pivotInputs);
    Logger.processInputs("Intake/Pivot", pivotInputs);
  }

  /** Finds the angle clamped to physical limits, sends it to the pivot, and returns it. */
  private Angle setPivotAngle(Angle angle) {
    Angle clampedAngle =
        Degrees.of(
            MathUtil.clamp(
                angle.in(Degrees),
                IntakeConstants.pivotMinRotation.in(Degrees),
                IntakeConstants.pivotMaxRotation.in(Degrees)));

    pivotIO.setAngle(clampedAngle);
    Logger.recordOutput("Intake/Pivot/SetpointAngle", clampedAngle);

    return clampedAngle;
  }

  private void setRollerVelocity(AngularVelocity velocity) {
    rollerIO.setAngularVelocity(velocity);
    Logger.recordOutput("Intake/Roller/SetpointSpeed", velocity);
  }

  private void coastRoller() {
    rollerIO.coast();
    Logger.recordOutput("Intake/Roller/SetpointSpeed", RadiansPerSecond.of(0));
  }

  public Command intakeAtSpeedCommand(AngularVelocity velocity) {
    return runEnd(
        () -> {
          setPivotAngle(IntakeConstants.pivotMinRotation);

          // TODO: if (pivotInputs.angle.isNear(IntakeConstants.pivotMinRotation, Radians.of(1)))
          setRollerVelocity(velocity);
        },
        () -> {
          coastRoller();
          setPivotAngle(IntakeConstants.pivotMaxRotation);
        });
  }

  public Command intakeAtGroundSpeedCommand(Supplier<LinearVelocity> drivetrainVelocity) {
    return runEnd(
        () -> {
          AngularVelocity offsetDriveVelocity =
              RadiansPerSecond.of(
                  drivetrainVelocity
                      .get()
                      .div(IntakeConstants.rollerRadius)
                      .in(MetersPerSecond.per(Meters)));
          AngularVelocity velocity = offsetDriveVelocity.plus(IntakeConstants.rollerAbsoluteSpeed);

          setPivotAngle(IntakeConstants.pivotMinRotation);

          if (pivotInputs.angle.isNear(
              IntakeConstants.pivotMinRotation, IntakeConstants.pivotToleranceBeforeRollersEngage))
            setRollerVelocity(velocity);
        },
        () -> {
          coastRoller();
          setPivotAngle(IntakeConstants.pivotMaxRotation);
        });
  }

  public Command holdIntakeDownCommand() {
    return runEnd(
        () -> setPivotAngle(IntakeConstants.pivotMinRotation),
        () -> setPivotAngle(IntakeConstants.pivotMaxRotation));
  }

  public Angle getPivotAngle() {
    return pivotInputs.angle;
  }

  //
  // SysId
  //

  public Command rollerSysIdCommand(Trigger advanceRoutine) {
    return SysIdCommand.sysIdCommand(
        advanceRoutine,
        "Indexer/Intake/Roller",
        voltage -> rollerIO.runOpenLoop(voltage.in(Volts)),
        () -> rollerInputs.position,
        () -> rollerInputs.velocity,
        () -> rollerInputs.appliedVoltage,
        this);
  }

  public Command pivotSysIdCommand(Trigger advanceRoutine) {
    return SysIdCommand.sysIdCommand(
        advanceRoutine,
        "Indexer/Intake/Pivot",
        voltage -> pivotIO.runOpenLoop(voltage.in(Volts)),
        () -> pivotInputs.angle,
        () -> pivotInputs.angularVelocity,
        () -> pivotInputs.appliedVoltage,
        this);
  }
}
