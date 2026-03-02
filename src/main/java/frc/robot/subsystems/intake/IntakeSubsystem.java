// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SysIdCommand;
import frc.robot.constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  private final RollerIO rollerIO;
  private final RollerIOInputsAutoLogged rollerInputs = new RollerIOInputsAutoLogged();

  public final PivotIO pivotIO;
  private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();

  private Angle pivotTarget = null;

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

    // reset pivot encoder
    if (pivotInputs.angle.gt(IntakeConstants.pivotMaxRotation))
      pivotIO.resetPosition(IntakeConstants.pivotMaxRotation);
    if (pivotInputs.angle.lt(IntakeConstants.pivotMinRotation))
      pivotIO.resetPosition(IntakeConstants.pivotMinRotation);

    // TODO: this early return is here for safety but should probably be removed later
    if (pivotTarget == null) return;
    if (pivotInputs.angle.isNear(pivotTarget, IntakeConstants.pivotToleranceBeforeCoast))
      pivotIO.coast();
    else pivotIO.setAngle(pivotTarget);
  }

  /** Finds the angle clamped to physical limits, sends it to the pivot, and returns it. */
  private Angle setPivotAngle(Angle angle) {
    Angle clampedAngle =
        Degrees.of(
            MathUtil.clamp(
                angle.in(Degrees),
                IntakeConstants.pivotMinRotation.in(Degrees),
                IntakeConstants.pivotMaxRotation.in(Degrees)));
    pivotTarget = clampedAngle;
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

          if (pivotInputs.angle.isNear(
              IntakeConstants.pivotMinRotation, IntakeConstants.pivotToleranceBeforeCoast))
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
        "Intake/SysId/Roller",
        voltage -> rollerIO.runOpenLoop(voltage.in(Volts)),
        () -> rollerInputs.position,
        () -> rollerInputs.velocity,
        () -> rollerInputs.appliedVoltage,
        this);
  }

  public Command pivotSysIdCommand(Trigger advanceRoutine) {
    return SysIdCommand.sysIdCommand(
        advanceRoutine,
        "Intake/SysId/Pivot",
        Volts.of(0.4).per(Second),
        Volts.of(1.2),
        Seconds.of(8),
        voltage -> pivotIO.runOpenLoop(voltage.in(Volts)),
        () -> pivotInputs.angle,
        () -> pivotInputs.angularVelocity,
        () -> pivotInputs.appliedVoltage,
        this);
  }
}
