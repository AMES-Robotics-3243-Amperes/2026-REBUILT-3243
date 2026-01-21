// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(ShooterIO shooterIO) {
    this.shooterIO = shooterIO;
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(shooterInputs);
    Logger.processInputs("Shooter", shooterInputs);
  }

  public Command runFlywheelAtSpeedCommand(AngularVelocity velocity) {
    return runEnd(
        () -> {
          shooterIO.setFlywheelAngularVelocity(velocity);
          Logger.recordOutput("Shooter/SetpointVelocity", velocity);
        },
        () -> {
          shooterIO.setFlywheelAngularVelocity(RPM.of(0));
          Logger.recordOutput("Shooter/SetpointVelocity", RPM.of(0));
        });
  }

  public Command runFlywheelOpenLoopCommand(Voltage volts) {
    return runEnd(
        () -> {
          shooterIO.runFlywheelOpenLoop(volts.in(Volts));
          Logger.recordOutput("Shooter/FlywheelOpenLoopVolts", volts);
        },
        () -> {
          shooterIO.runFlywheelOpenLoop(0);
          Logger.recordOutput("Shooter/FlywheelOpenLoopVolts", Volts.of(0));
        });
  }

  public Command setHoodAngleCommand(Angle angle) {
    return runEnd(
        () -> {
          shooterIO.setHoodAngle(angle);
          Logger.recordOutput("Shooter/HoodSetpointAngle", angle);
        },
        () -> {
          shooterIO.runHoodOpenLoop(0);
          Logger.recordOutput("Shooter/HoodSetpointAngle", angle);
        });
  }

  public Command runHoodOpenLoopCommand(Voltage volts) {
    return runEnd(
        () -> {
          shooterIO.runHoodOpenLoop(volts.in(Volts));
          Logger.recordOutput("Shooter/HoodOpenLoopVolts", volts);
        },
        () -> {
          shooterIO.runHoodOpenLoop(0);
          Logger.recordOutput("Shooter/HoodOpenLoopVolts", Volts.of(0));
        });
  }

  public Angle getHoodAngle() {
    return shooterInputs.hoodAngle;
  }

  public AngularVelocity getFlywheelVelocity() {
    return shooterInputs.flywheelVelocity;
  }
}
