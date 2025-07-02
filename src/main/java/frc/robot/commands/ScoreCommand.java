package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.RobotContainer;
import frc.robot.util.BlackholePlanner.Trajectory2d;
import frc.robot.util.math.MUtils;

public class ScoreCommand extends Command {
  RobotContainer m_RobotContainer;

  enum State {
    Aligning,
    Scoring,
    End
  }

  State m_State;
  int m_ReefId;
  Pose2d m_Pose2d;
  int m_Level;
  PIDController m_PoseXController = new PIDController(6, 0, 0.0);
  PIDController m_PoseYController = new PIDController(6, 0, 0.0);
  PIDController m_RotationController = new PIDController(3, 0, 0.0);
  double StartTimeStamps;
  Trajectory2d m_Trajectory2d;
  boolean m_isInverted;

  public ScoreCommand(
      RobotContainer _RobotContainer, Pose2d _Pose2d, int _Level, boolean _isinverted) {
    m_RobotContainer = _RobotContainer;
    m_Pose2d = _Pose2d;
    addRequirements(m_RobotContainer.m_SuperStructure);
    addRequirements(m_RobotContainer.drive);
    m_RotationController.enableContinuousInput(-Math.PI, Math.PI);
    m_PoseXController.setSetpoint(m_Pose2d.getX());
    m_PoseYController.setSetpoint(m_Pose2d.getY());
    if (_isinverted) {
      m_RotationController.setSetpoint(m_Pose2d.getRotation().getRadians() + Math.PI);
    } else {
      m_RotationController.setSetpoint(m_Pose2d.getRotation().getRadians());
    }
    m_Level = _Level;
    m_Trajectory2d = new Trajectory2d("Rest2L" + _Level, 1);
    m_isInverted = _isinverted;
  }

  @Override
  public void initialize() {
    m_State = State.Aligning;
    StartTimeStamps = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    switch (m_State) {
      case Aligning:
        Align();
        break;
      case Scoring:
        Score();
        break;
      default:
        break;
    }
  }

  void Align() {
    ChassisSpeeds _ChassisSpeeds = new ChassisSpeeds();
    _ChassisSpeeds.vxMetersPerSecond =
        MUtils.numberLimit(
            -Constants.AutoAlignConstants.kMaxVelocity,
            Constants.AutoAlignConstants.kMaxVelocity,
            m_PoseXController.calculate(m_RobotContainer.drive.getPose().getX()));
    _ChassisSpeeds.vyMetersPerSecond =
        MUtils.numberLimit(
            -Constants.AutoAlignConstants.kMaxVelocity,
            Constants.AutoAlignConstants.kMaxVelocity,
            m_PoseYController.calculate(m_RobotContainer.drive.getPose().getY()));
    _ChassisSpeeds.omegaRadiansPerSecond =
        MUtils.numberLimit(
            -Constants.AutoAlignConstants.kMaxVelocity,
            Constants.AutoAlignConstants.kMaxVelocity,
            m_RotationController.calculate(
                m_RobotContainer.drive.getPose().getRotation().getRadians()));

    m_RobotContainer.m_SuperStructure.SetMotionMagic(
        m_Trajectory2d.getStartlpoint().a0, m_Trajectory2d.getStartlpoint().a1, 0);
    m_RobotContainer.drive.runVelocity(_ChassisSpeeds);
    if (m_PoseXController.getError() < AutoAlignConstants.kPositionTolerance
        && m_PoseYController.getError() < AutoAlignConstants.kPositionTolerance
        && m_RotationController.getPositionError() < AutoAlignConstants.kAngleTolerance) {
      m_State = State.Scoring;
      StartTimeStamps = Timer.getFPGATimestamp();
    }
  }

  void Score() {
    double deltaTime = Timer.getFPGATimestamp() - StartTimeStamps;
    m_RobotContainer.m_SuperStructure.SetSetpoint2d(m_Trajectory2d.getSetpoint(deltaTime), 0);
  }

  @Override
  public void end(boolean interrupted) {
    m_RobotContainer.m_SuperStructure.SetMotionMagic(
        Constants.SuperStructureConstants.UpperStructureState.Rest.elevator_height,
        m_isInverted
            ? -Constants.SuperStructureConstants.UpperStructureState.Rest.arm_theta
            : Constants.SuperStructureConstants.UpperStructureState.Rest.arm_theta,
        Constants.SuperStructureConstants.UpperStructureState.Rest.wrist_theta);
  }

  @Override
  public boolean isFinished() {

    return false;
  }
}
