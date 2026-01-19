package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.IntakeConstants;

public class RollerIOReal implements RollerIO {
  SparkMax leaderSparkMax = new SparkMax(IntakeConstants.leftRollerId, MotorType.kBrushless);
  SparkMax followerSparkMax = new SparkMax(IntakeConstants.rightRollerId, MotorType.kBrushless);

  SparkClosedLoopController leaderController;
  RelativeEncoder leaderEncoder;

  public RollerIOReal() {
    SparkMaxConfig config = new SparkMaxConfig();

    config.encoder.positionConversionFactor(IntakeConstants.rollerGearRatio);
    config.encoder.velocityConversionFactor(IntakeConstants.rollerGearRatio);
    config.idleMode(IdleMode.kCoast).inverted(true);

    config
        .closedLoop
        .outputRange(
            -IntakeConstants.rollerMaxOutput,
            IntakeConstants.rollerMaxOutput,
            ClosedLoopSlot.kSlot0)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(
            IntakeConstants.rollerKp,
            IntakeConstants.rollerKi,
            IntakeConstants.rollerKd,
            ClosedLoopSlot.kSlot0);
    // .feedForward
    // .sva(
    //     IntakeConstants.rollerKs,
    //     IntakeConstants.rollerKv,
    //     IntakeConstants.rollerKa,
    //     ClosedLoopSlot.kSlot0);

    leaderSparkMax.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.follow(leaderSparkMax, true);
    followerSparkMax.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leaderController = leaderSparkMax.getClosedLoopController();
    leaderEncoder = leaderSparkMax.getEncoder();
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.position = Rotations.of(leaderEncoder.getPosition());
    inputs.velocity = RPM.of(leaderEncoder.getVelocity());
    inputs.appliedVoltage = Volts.of(leaderSparkMax.getAppliedOutput());
  }

  @Override
  public void runOpenLoop(double output) {
    leaderSparkMax.setVoltage(output);
  }

  @Override
  public void setAngularVelocity(AngularVelocity velocity) {
    leaderController.setSetpoint(velocity.in(RPM), ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }
}
