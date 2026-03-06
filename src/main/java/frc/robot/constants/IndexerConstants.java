// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.ControlConstantsBuilder;

/** Add your docs here. */
public class IndexerConstants {
  // spindexer
  public static final int spindexerId = 1;
  public static final double spindexerReduction = 4.0;

  public static final AngularVelocity spindexerIndexingSpeed = RPM.of(1200);

  public static final int spindexerCurrentLimit = 80;

  public static final ControlConstantsBuilder spindexerControl =
      ControlConstantsBuilder.fromRadiansAndSeconds()
          .pid(0, 0, 0)
          .sva(0.18119, 0.025646, 0.0026016);

  // kicker
  public static final int kickerId = 4;
  public static final double kickerReduction = 2.0;

  public static final AngularVelocity kickerShootingSpeed = RPM.of(2200);

  public static final int kickerCurrentLimit = 90;

  public static final Distance kickerWheelRadius = Inches.of(1.25);

  public static final ControlConstantsBuilder kickerControl =
      ControlConstantsBuilder.fromRadiansAndSeconds()
          .pid(0, 0, 0)
          .sva(0.41898, 0.035302, 0.0025373);
}
