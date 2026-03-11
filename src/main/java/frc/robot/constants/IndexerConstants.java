// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.util.ControlConstantsBuilder;

/** Add your docs here. */
public class IndexerConstants {
  public static final Time idleTimeBeforeIndexing = Milliseconds.of(150);

  // spindexer
  public static final int spindexerId = 1;
  public static final double spindexerReduction = 4.0;

  public static final AngularVelocity spindexerIndexingSpeed = RPM.of(800);

  public static final int spindexerCurrentLimit = 90;

  public static final ControlConstantsBuilder spindexerControl =
      ControlConstantsBuilder.fromRadiansAndSeconds()
          .pid(0.016377, 0, 0)
          .sva(0.21938, 0.068485, 0.0065446);

  // kicker
  public static final int kickerId = 4;
  public static final double kickerReduction = 2.0;

  public static final AngularVelocity maxKickerSpeed = RPM.of(6000).div(kickerReduction);
  public static final AngularVelocity kickerSpinUpTolerance = RPM.of(40);

  public static final int kickerCurrentLimit = 90;

  public static final Distance kickerWheelRadius = Inches.of(1.125);

  public static final ControlConstantsBuilder kickerControl =
      ControlConstantsBuilder.fromRadiansAndSeconds()
          .pid(0.025, 0, 0)
          .sva(0.39863, 0.035224, 0.0017345);
}
