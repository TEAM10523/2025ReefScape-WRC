package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.util.BlackholePlanner.Trajectory2d;
import frc.robot.util.math.MUtils;

public class ScoreCommand extends Command {
  RobotContainer m_RobotContainer;

  enum State {
    Aligning,
    Rising,
    Scoring,
    End
  }

  State m_State;
  int m_ReefId;
  Pose2d m_Pose2d;
  int m_Level;
  PIDController m_PoseXController = new PIDController(3, 0, 0.3);
  PIDController m_PoseYController = new PIDController(4, 0, 0.3);
  PIDController m_RotationController = new PIDController(2, 0, 0.0);
  double StartTimeStamps;
  Trajectory2d m_Trajectory2d;
  Trajectory2d m_RisingTrajectory2d;
  boolean m_isInverted;

  public ScoreCommand(
      RobotContainer _RobotContainer, Pose2d _Pose2d, int _Level, boolean _isinverted) {
    m_RobotContainer = _RobotContainer;
    m_Pose2d = _Pose2d;
    addRequirements(m_RobotContainer.m_SuperStructure);
    addRequirements(m_RobotContainer.drive);
    m_RotationController.enableContinuousInput(-Math.PI, Math.PI);

    m_PoseXController.setSetpoint(0);
    m_PoseYController.setSetpoint(0);
    if (_isinverted) {
      m_RotationController.setSetpoint(m_Pose2d.getRotation().getRadians() + Math.PI);
    } else {
      m_RotationController.setSetpoint(m_Pose2d.getRotation().getRadians());
    }
    m_Level = _Level;
    m_Trajectory2d = new Trajectory2d("ScoreL" + _Level, 1.);
    m_RisingTrajectory2d = new Trajectory2d("Rest2L" + _Level, 1.3);
    m_isInverted = _isinverted;
  }

  public static Command getClosestReefCommand(
      RobotContainer _RobotContainer, int _LRindex, int level) {
    return new ScoreCommand(
        _RobotContainer,
        FieldConstants.getClosestReefPose(
            _RobotContainer.drive.getPose().getTranslation(),
            _LRindex,
            DriverStation.getAlliance().get()),
        level,
        false);
  }

  @Override
  public void initialize() {
    m_State = State.Aligning;
    StartTimeStamps = Timer.getFPGATimestamp();
    m_PoseXController.setTolerance(AutoAlignConstants.kPositionTolerance);
    m_PoseYController.setTolerance(AutoAlignConstants.kPositionTolerance);
    m_RotationController.setTolerance(AutoAlignConstants.kAngleTolerance);
    m_PoseXController.reset();
    m_PoseYController.reset();
    m_RotationController.reset();
  }

  @Override
  public void execute() {
    switch (m_State) {
      case Aligning:
        Align();
        break;
      case Rising:
        Rise();
        break;
      case Scoring:
        Score();
        break;
      default:
        break;
    }
  }

  void driveToTarget() {
    ChassisSpeeds _ChassisSpeeds = new ChassisSpeeds();
    Translation2d TargetToRobotVector =
        new Translation2d(
            m_RobotContainer.drive.getPose().getX() - m_Pose2d.getX(),
            m_RobotContainer.drive.getPose().getY() - m_Pose2d.getY());
    Translation2d TargetBasedVector =
        TargetToRobotVector.rotateBy(m_Pose2d.getRotation().unaryMinus());
    Translation2d _OutputTranslation2d =
        new Translation2d(
            MUtils.numberLimit(
                -Constants.AutoAlignConstants.kMaxVelocity,
                Constants.AutoAlignConstants.kMaxVelocity,
                m_PoseXController.calculate(TargetBasedVector.getX())),
            MUtils.numberLimit(
                -Constants.AutoAlignConstants.kMaxVelocity,
                Constants.AutoAlignConstants.kMaxVelocity,
                m_PoseYController.calculate(TargetBasedVector.getY())));
    _OutputTranslation2d = _OutputTranslation2d.rotateBy(m_Pose2d.getRotation());

    _ChassisSpeeds.vxMetersPerSecond = _OutputTranslation2d.getX();

    _ChassisSpeeds.vyMetersPerSecond = _OutputTranslation2d.getY();
    _ChassisSpeeds.omegaRadiansPerSecond =
        MUtils.numberLimit(
            -Constants.AutoAlignConstants.kMaxAngularvelocity,
            Constants.AutoAlignConstants.kMaxAngularvelocity,
            m_RotationController.calculate(
                m_RobotContainer.drive.getPose().getRotation().getRadians()));
    m_RobotContainer.drive.runVelocityFieldRelative(_ChassisSpeeds);
  }

  void Align() {
    Translation2d TargetToRobotVector =
        new Translation2d(
            m_RobotContainer.drive.getPose().getX() - m_Pose2d.getX(),
            m_RobotContainer.drive.getPose().getY() - m_Pose2d.getY());
    Translation2d TargetBasedVector =
        TargetToRobotVector.rotateBy(m_Pose2d.getRotation().unaryMinus());
    if (TargetBasedVector.getNorm() < AutoAlignConstants.PlacementThreshold) {
      m_State = State.Rising;
      StartTimeStamps = Timer.getFPGATimestamp();
    }
    driveToTarget();
  }

  void Rise() {
    Translation2d TargetToRobotVector =
        new Translation2d(
            m_RobotContainer.drive.getPose().getX() - m_Pose2d.getX(),
            m_RobotContainer.drive.getPose().getY() - m_Pose2d.getY());
    Translation2d TargetBasedVector =
        TargetToRobotVector.rotateBy(m_Pose2d.getRotation().unaryMinus());

    double deltaTime = Timer.getFPGATimestamp() - StartTimeStamps;
    m_RobotContainer.m_SuperStructure.SetSetpoint2d(
        m_RisingTrajectory2d.getSetpoint(deltaTime), 1.8);

    if (
    // TargetBasedVector.getNorm() < AutoAlignConstants.ScoreThresholdDistance
    //   && Math.abs(m_RotationController.getError()) < AutoAlignConstants.ScoreThresholdDirection
    // && m_RobotContainer.m_SuperStructure.atGoal(UpperStructureState.valueOf("ScoreL" + m_Level))
    m_RobotContainer.controller.getHID().getR2Button()) {
      m_State = State.Scoring;
      StartTimeStamps = Timer.getFPGATimestamp();
    }
    driveToTarget();
  }

  void Score() {
    double deltaTime = Timer.getFPGATimestamp() - StartTimeStamps;
    m_RobotContainer.m_SuperStructure.SetSetpoint2d(m_Trajectory2d.getSetpoint(deltaTime), 1.8);
    m_RobotContainer.drive.stop();
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
