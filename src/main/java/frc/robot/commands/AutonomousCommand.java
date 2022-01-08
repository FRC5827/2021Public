
package frc.robot.commands;

import java.util.List;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * A complex auto command sequence.
 */
public class AutonomousCommand extends SequentialCommandGroup {
  /**
   * Creates a new AutonomousCommand.
   *
   * @param commandList The ordered list of commands to execute
   */
  public AutonomousCommand(List<Command> commandList) {
      for (Command command : commandList) {
//      System.out.println(command.toString());
      addCommands(command);
    }
  }

  public void Cancel() {
      this.cancel();
  }

}