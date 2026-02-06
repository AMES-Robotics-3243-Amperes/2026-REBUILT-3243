package frc.robot.state;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.state.RobotState.OperatorControlled;

public class StateMachine {
  private RobotState state;
  private Command activeCommand;
  private RobotContainer container;

  public StateMachine(RobotContainer robotContainer) {
    this.container = robotContainer;
  }

  private void switchState(RobotState newState) {
    if (activeCommand != null) activeCommand.cancel();

    state = newState;
    activeCommand = state.getCommand(container);
    CommandScheduler.getInstance().schedule(activeCommand);
  }

  private void reset() {
    activeCommand.cancel();
    state = null;
  }

  private void updateState() {
    if (state == null) switchState(new OperatorControlled());
    state.nextState(container, activeCommand).ifPresent(newState -> switchState(newState));
  }

  public Command stateCommand() {
    return Commands.runEnd(this::updateState, this::reset);
  }
}
