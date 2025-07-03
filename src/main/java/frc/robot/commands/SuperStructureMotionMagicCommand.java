package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SuperStructureConstants;
import frc.robot.RobotContainer;

public class SuperStructureMotionMagicCommand extends Command {

  double m_TargetHeight;
  double m_TargetArmAngle;
  double m_WristAngle;
  RobotContainer m_RobotContainer;

  public SuperStructureMotionMagicCommand(
      double _TargetHeight,
      double _TargetArmAngle,
      double _WristAngle,
      RobotContainer _RobotContainer) {
    m_TargetHeight = _TargetHeight;
    m_TargetArmAngle = _TargetArmAngle;
    m_WristAngle = _WristAngle;
    m_RobotContainer = _RobotContainer;
    addRequirements(m_RobotContainer.m_SuperStructure);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    m_RobotContainer.m_SuperStructure.SetMotionMagic(
        m_TargetHeight, m_TargetArmAngle, m_WristAngle);
  }

  @Override
  public void end(boolean interrupted) {
    m_RobotContainer.m_SuperStructure.SetMotionMagic(
        SuperStructureConstants.UpperStructureState.Rest);
  }

  @Override
  public boolean isFinished() {

    return false;
  }
}
