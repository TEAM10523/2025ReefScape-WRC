package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.VerticalIntakeCommands;
import java.util.Set;

public class Mid103 extends SequentialCommandGroup {

  public Mid103(RobotContainer _RobotContainer) {
    addCommands(
        new InstantCommand(
            () -> {
              if (DriverStation.getAlliance().get() == Alliance.Blue)
                _RobotContainer.drive.setPose(
                    _RobotContainer.drive
                        .generatePath("Mid103-1")
                        .getStartingHolonomicPose()
                        .get());
              else
                _RobotContainer.drive.setPose(
                    _RobotContainer.drive
                        .generatePath("Mid103-1")
                        .flipPath()
                        .getStartingHolonomicPose()
                        .get());
            }));
    addCommands(_RobotContainer.drive.followPathCommand("Mid103-1"));
    addCommands(
        Commands.defer(
            () -> ScoreCommand.getClosestReefCommand(_RobotContainer, 1, 4).withTimeout(2),
            Set.of(_RobotContainer.m_SuperStructure, _RobotContainer.drive)));
    addCommands(
        _RobotContainer.drive
            .followPathCommand("Mid103-2")
            .raceWith(new VerticalIntakeCommands(_RobotContainer)));
    addCommands(_RobotContainer.drive.followPathCommand("Mid103-3"));
    addCommands(
        Commands.defer(
            () -> ScoreCommand.getClosestReefCommand(_RobotContainer, 1, 3).withTimeout(2),
            Set.of(_RobotContainer.m_SuperStructure, _RobotContainer.drive)));
    addCommands(
        _RobotContainer.drive
            .followPathCommand("Mid103-4")
            .raceWith(new VerticalIntakeCommands(_RobotContainer)));
    addCommands(
        Commands.defer(
            () -> ScoreCommand.getClosestReefCommand(_RobotContainer, 2, 4).withTimeout(2),
            Set.of(_RobotContainer.m_SuperStructure, _RobotContainer.drive)));
    addCommands(
        _RobotContainer.drive
            .followPathCommand("Mid103-6")
            .raceWith(new VerticalIntakeCommands(_RobotContainer)));
    addCommands(_RobotContainer.drive.followPathCommand("Mid103-7"));
    addCommands(
        Commands.defer(
            () -> ScoreCommand.getClosestReefCommand(_RobotContainer, 2, 3).withTimeout(3.5),
            Set.of(_RobotContainer.m_SuperStructure, _RobotContainer.drive)));
  }
}
