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
  SparkMax rollerSparkMax = new SparkMax(IntakeConstants.RollerId, MotorType.kBrushless);

  SparkClosedLoopController rollerController;
  RelativeEncoder rollerEncoder;

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

    rollerSparkMax.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    
    rollerController = rollerSparkMax.getClosedLoopController();
    rollerEncoder = rollerSparkMax.getEncoder();
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.position = Rotations.of(rollerEncoder.getPosition());
    inputs.velocity = RPM.of(rollerEncoder.getVelocity());
    inputs.appliedVoltage = Volts.of(rollerSparkMax.getAppliedOutput());
  }

  @Override
  public void runOpenLoop(double output) {
    rollerSparkMax.setVoltage(output);
  }

  @Override
  public void setAngularVelocity(AngularVelocity velocity) {
    rollerController.setSetpoint(velocity.in(RPM), ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }
}
