package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SuperStructureConstants;
import frc.robot.RobotContainer;

public class IntakeCommands extends Command {
  RobotContainer m_RobotContainer;

  public IntakeCommands(RobotContainer _RobotContainer) {
    m_RobotContainer = _RobotContainer;
    // addRequirements(m_RobotContainer.drive);
    addRequirements(m_RobotContainer.m_SuperStructure);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_RobotContainer.m_SuperStructure.SetMotionMagic(
        SuperStructureConstants.UpperStructureState.GroundIntake);
    m_RobotContainer.m_SuperStructure.SetIntakeSpeed(80);
    ;
    // m_RobotContainer.m_SuperStructure.
  }

  @Override
  public void end(boolean interrupted) {
    // TODO Auto-generated method stub
    m_RobotContainer.m_SuperStructure.SetMotionMagic(
        SuperStructureConstants.UpperStructureState.Rest);
    m_RobotContainer.m_SuperStructure.SetIntakeVoltage(0);
  }

  @Override
  public boolean isFinished() {

    return false;
  }
}
