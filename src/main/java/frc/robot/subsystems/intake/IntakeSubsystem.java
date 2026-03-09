// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.GeneralPurposeCharacterization;
import frc.robot.constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  private final RollerIO rollerIO;
  private final RollerIOInputsAutoLogged rollerInputs = new RollerIOInputsAutoLogged();

  public final PivotIO pivotIO;
  private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();

  private boolean atBottom = false;
  private boolean atTop = false;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(RollerIO rollerIO, PivotIO pivotIO) {
    this.rollerIO = rollerIO;
    this.pivotIO = pivotIO;
  }

  @Override
  public void periodic() {
    rollerIO.updateInputs(rollerInputs);
    Logger.processInputs("Intake/Roller", rollerInputs);

    pivotIO.updateInputs(pivotInputs);
    Logger.processInputs("Intake/Pivot", pivotInputs);
  }

  private void setRollerVelocity(AngularVelocity velocity) {
    rollerIO.setAngularVelocity(velocity);
    Logger.recordOutput("Intake/Roller/SetpointSpeed", velocity);
  }

  private void coastRoller() {
    rollerIO.coast();
    Logger.recordOutput("Intake/Roller/SetpointSpeed", RadiansPerSecond.of(0));
  }

  private void runPivotOpenLoop(double output) {
    atBottom = false;
    atTop = false;
    pivotIO.runOpenLoop(output);
  }

  private void coastPivot() {
    runPivotOpenLoop(0);
  }

  private void lowerPivot() {
    if (atBottom) {
      pivotIO.runOpenLoop(0);
      return;
    }

    pivotIO.runOpenLoop(IntakeConstants.pivotAutomaticVolts.unaryMinus().in(Volts));
    atBottom |=
        (pivotInputs.appliedVoltage.abs(Volts) > IntakeConstants.pivotAutomaticVolts.abs(Volts)
            && pivotInputs.angularVelocity.abs(RadiansPerSecond)
                < IntakeConstants.pivotVelocityToleranceBeforeStop.abs(RadiansPerSecond));
  }

  private void raisePivot() {
    if (atTop) {
      pivotIO.runOpenLoop(0);
      return;
    }

    pivotIO.runOpenLoop(IntakeConstants.pivotAutomaticVolts.in(Volts));
    atTop |=
        (pivotInputs.appliedVoltage.abs(Volts) > IntakeConstants.pivotAutomaticVolts.abs(Volts)
            && pivotInputs.angularVelocity.abs(RadiansPerSecond)
                < IntakeConstants.pivotVelocityToleranceBeforeStop.abs(RadiansPerSecond));
  }

  public Command intakeCommand() {
    return runEnd(
        () -> {
          lowerPivot();
          setRollerVelocity(IntakeConstants.rollerIntakeSpeed);
        },
        () -> {
          coastPivot();
          coastRoller();
        });
  }

  public Command agitateCommand() {
    return runEnd(() -> setRollerVelocity(IntakeConstants.rollerIntakeSpeed), this::coastRoller);
  }

  public Command raisePivotCommand() {
    return runEnd(this::raisePivot, this::coastPivot);
  }

  public Command runPivotOpenLoopCommand(Voltage output) {
    return runEnd(() -> runPivotOpenLoop(output.in(Volts)), () -> runPivotOpenLoop(0));
  }

  public Angle getPivotAngle() {
    return pivotInputs.angle;
  }

  //
  // SysId
  //
  public Command rollerSysIdCommand(Trigger advanceRoutine) {
    return GeneralPurposeCharacterization.sysIdCommand(
        advanceRoutine,
        "Intake/SysId/Roller",
        voltage -> rollerIO.runOpenLoop(voltage.in(Volts)),
        () -> rollerInputs.position,
        () -> rollerInputs.velocity,
        () -> rollerInputs.appliedVoltage,
        this);
  }

  public Command pivotSysIdCommand(Trigger advanceRoutine) {
    return GeneralPurposeCharacterization.sysIdCommand(
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
