package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.ShooterConstants;

public class HoodIOSim implements HoodIO {
  final double maxVelocityDegreesPerSecond = 90;

  Angle position = Degrees.of(0);
  Angle goal = Degrees.of(0);
  SlewRateLimiter rateLimiterDegrees = new SlewRateLimiter(maxVelocityDegreesPerSecond);

  public HoodIOSim() {
    position = ShooterConstants.hoodMinRotation;
    goal = ShooterConstants.hoodMaxRotation;
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    position = Degrees.of(rateLimiterDegrees.calculate(goal.in(Degrees)));

    inputs.angle = position;
    inputs.angularVelocity =
        position.isEquivalent(goal)
            ? DegreesPerSecond.of(0)
            : DegreesPerSecond.of(maxVelocityDegreesPerSecond)
                .times(Math.signum(goal.minus(position).in(Degrees)));
    inputs.appliedVoltage = position.isEquivalent(goal) ? Volts.of(0) : Volts.of(1);
  }

  @Override
  public void resetPosition(Angle angle) {
    position = angle;
  }

  @Override
  public void setAngle(Angle angle) {
    goal = angle;
  }
}
