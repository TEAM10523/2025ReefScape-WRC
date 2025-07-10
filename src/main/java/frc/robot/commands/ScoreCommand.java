package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.SuperStructureConstants.UpperStructureState;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.util.BlackholePlanner.Trajectory2d;
import frc.robot.util.math.MUtils;
import org.littletonrobotics.junction.Logger;

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
  Pose2d m_PushPose2d;
  int m_Level;
  PIDController m_PoseXController = new PIDController(3.2, 0, 0.05);
  PIDController m_PoseYController = new PIDController(5, 0, 0.6);
  PIDController m_RotationController = new PIDController(4, 0, 0.0);
  double StartTimeStamps;
  Trajectory2d m_Trajectory2d;
  Trajectory2d m_RisingTrajectory2d;
  boolean m_isInverted;
  boolean m_isLocked;

  public ScoreCommand(
      RobotContainer _RobotContainer, Pose2d _Pose2d, int _Level, boolean _isinverted) {
    m_RobotContainer = _RobotContainer;
    m_Pose2d = _Pose2d;
    m_PushPose2d = _Pose2d;
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
    m_RisingTrajectory2d = new Trajectory2d("Rest2L" + _Level, 1.);
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
    m_isLocked = false;
  }

  @Override
  public void execute() {
    Logger.recordOutput("TargetPose", m_Pose2d);
    Logger.recordOutput("IsAtDistance", IsAtDistance());
    Logger.recordOutput("IsAtRotation", IsAtRotation());
    Logger.recordOutput("IsAtSuper", IsAtSuper());
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

  void setState(State state) {
    m_State = state;
  }

  Translation2d getDeltaTranslation() {
    return new Translation2d(
        m_RobotContainer.drive.getPose().getX() - m_Pose2d.getX(),
        m_RobotContainer.drive.getPose().getY() - m_Pose2d.getY());
  }

  double getDistance2Target() {
    return getDeltaTranslation().getNorm();
  }

  void driveToTarget() {
    m_Pose2d =
        m_PushPose2d.plus(
            new Transform2d(
                new Translation2d(
                    Math.abs(m_PoseYController.getError()) > 0.15
                        ? AutoAlignConstants.pushDistance
                        : 0,
                    0),
                new Rotation2d()));
    ChassisSpeeds _ChassisSpeeds = new ChassisSpeeds();
    Translation2d TargetToRobotVector = getDeltaTranslation();
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
    if (!m_isLocked) {
      _ChassisSpeeds.vyMetersPerSecond = _OutputTranslation2d.getY();
      _ChassisSpeeds.omegaRadiansPerSecond =
          MUtils.numberLimit(
              -Constants.AutoAlignConstants.kMaxAngularvelocity,
              Constants.AutoAlignConstants.kMaxAngularvelocity,
              m_RotationController.calculate(
                  m_RobotContainer.drive.getPose().getRotation().getRadians()));
    } else {
      _ChassisSpeeds.vyMetersPerSecond = 0;
      _ChassisSpeeds.omegaRadiansPerSecond = 0;
    }

    if (TargetBasedVector.getNorm() < 0.03 && Math.abs(m_RotationController.getError()) < 0.05)
      m_isLocked = true;
    m_RobotContainer.drive.runVelocityFieldRelative(_ChassisSpeeds);
  }

  void Align() {
    if (getDistance2Target() < AutoAlignConstants.PlacementThreshold) {
      setState(State.Rising);
      StartTimeStamps = Timer.getFPGATimestamp();
    }
    driveToTarget();
  }

  void Rise() {
    double deltaTime = Timer.getFPGATimestamp() - StartTimeStamps;
    m_RobotContainer.m_SuperStructure.SetSetpoint2d(
        m_RisingTrajectory2d.getSetpoint(deltaTime), 1.8);

    if (getDistance2Target() < AutoAlignConstants.ScoreThresholdDistance
        && Math.abs(m_RotationController.getError()) < AutoAlignConstants.ScoreThresholdDirection
        && m_RobotContainer.m_SuperStructure.atGoal(UpperStructureState.valueOf("ScoreL" + m_Level))
    // && m_RobotContainer.controller.getHID().getR2Button()
    ) {
      m_State = State.Scoring;
      StartTimeStamps = Timer.getFPGATimestamp();
    }
    driveToTarget();
  }

  boolean IsAtDistance() {
    return getDistance2Target() < AutoAlignConstants.ScoreThresholdDistance;
  }

  boolean IsAtRotation() {
    return Math.abs(m_RotationController.getError()) < AutoAlignConstants.ScoreThresholdDirection;
  }

  boolean IsAtSuper() {
    return m_RobotContainer.m_SuperStructure.atGoal(
        UpperStructureState.valueOf("ScoreL" + m_Level));
  }

  void Score() {
    double deltaTime = Timer.getFPGATimestamp() - StartTimeStamps;
    m_RobotContainer.m_SuperStructure.SetSetpoint2d(m_Trajectory2d.getSetpoint(deltaTime), 1.8);
    m_RobotContainer.drive.stop();
    if (m_RobotContainer.m_SuperStructure.atGoal(UpperStructureState.Rest)) m_State = State.End;
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

    return m_State == State.End;
  }
}
