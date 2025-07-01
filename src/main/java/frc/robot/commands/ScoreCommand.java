package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ScoreCommand extends Command {
  RobotContainer m_RobotContainer;

  enum State {
    Aligning,
    Scoring,
    End
  }

  HolonomicDriveController m_HolonomicDriveController;

  public ScoreCommand() {
    m_HolonomicDriveController =
        new HolonomicDriveController(
            new PIDController(5.5, 0.01, 0.04), // TODO add those to driveConstants
            new PIDController(5.5, 0.01, 0.04),
            new ProfiledPIDController(
                8, // 6036
                0,
                0.2,
                new TrapezoidProfile.Constraints(3 * Math.PI, Math.PI * 4)));
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {

    return false;
  }
}
