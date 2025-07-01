package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.util.BlackholePlanner.Setpoint2d;
import frc.robot.util.BlackholePlanner.Trajectory2d;

public class SuperStructureFollowPathCommand extends Command {
  Trajectory2d m_Trajectory2d;
  double m_WristAngle;
  RobotContainer m_RobotContainer;
  double StartTimeStamps;

  public SuperStructureFollowPathCommand(
      Trajectory2d _Trajectory2d, RobotContainer _RobotContainer, double _WristAngle) {
    m_Trajectory2d = _Trajectory2d;
    StartTimeStamps = 0.;
    m_RobotContainer = _RobotContainer;
    m_WristAngle = _WristAngle;
    addRequirements(m_RobotContainer.m_SuperStructure);
  }

  public SuperStructureFollowPathCommand(
      String _Trajectory2d, RobotContainer _RobotContainer, double _WristAngle) {

    m_Trajectory2d = new Trajectory2d(_Trajectory2d, 1);
    StartTimeStamps = 0.;
    m_RobotContainer = _RobotContainer;
    m_WristAngle = _WristAngle;
    addRequirements(m_RobotContainer.m_SuperStructure);
  }

  @Override
  public void initialize() {
    StartTimeStamps = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    double deltaTime = Timer.getFPGATimestamp() - StartTimeStamps;
    Setpoint2d _SetPoint2d = m_Trajectory2d.getSetpoint(deltaTime);
    m_RobotContainer.m_SuperStructure.SetSetpoint2d(_SetPoint2d, m_WristAngle);
  }

  @Override
  public void end(boolean interrupted) {
    m_RobotContainer.m_SuperStructure.SetMotionMagic(0, Math.PI / 2, 0);
  }

  @Override
  public boolean isFinished() {

    return false;
  }
}
