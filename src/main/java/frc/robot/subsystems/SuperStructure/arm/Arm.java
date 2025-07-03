package frc.robot.subsystems.SuperStructure.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SuperStructure.arm.ArmIO.ArmIOInputs;
import org.littletonrobotics.junction.AutoLogOutput;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputs inputs = new ArmIOInputs();
  @AutoLogOutput public double goal = 0.0;

  public Arm(ArmIO io) {
    this.io = io;
    io.setPID(
        ArmConstants.kp,
        ArmConstants.Ki,
        ArmConstants.Kd,
        ArmConstants.Ks,
        ArmConstants.Kv,
        ArmConstants.Ka,
        ArmConstants.acceleration,
        ArmConstants.velocity);
  }

  // These variables are used for logging and telementry, they don't actually participate in the
  // calculation and the running of the robot
  @AutoLogOutput private double armTargetPositionRadian = 0;
  @AutoLogOutput private double armTargetOmegaRadiansPerSec = 0;
  @AutoLogOutput private double armFeedForwardTorque = 0;
  @AutoLogOutput private double armTargetAlphaRadiansPerSec2 = 0;

  public enum ArmState {
    FollowingPath,
    MotionMagicToPosition,
    Volt,
    Stop
  }

  @AutoLogOutput ArmState m_ArmState = ArmState.Stop;

  public static ArmState armState = ArmState.Stop;

  public void runSetPoint(
      double angleRads, double omegaRadPerSec, double alphaRadsPerSecSquared, double torque) {
    io.runPositionSetpoint(angleRads, omegaRadPerSec, alphaRadsPerSecSquared, torque);
    armTargetPositionRadian = angleRads;
    armTargetOmegaRadiansPerSec = omegaRadPerSec;
    armTargetAlphaRadiansPerSec2 = alphaRadsPerSecSquared;
    armFeedForwardTorque = torque;
    m_ArmState = ArmState.FollowingPath;
  }

  public void runMotionMagicPosition(double angleRads, double omegaRadPerSec, double torque) {
    goal = MathUtil.clamp(angleRads, ArmConstants.armMinimumAngle, ArmConstants.armMaximumAngle);
    io.runMotionMagicPosition(goal, omegaRadPerSec, torque);
    armTargetPositionRadian = angleRads;
    armTargetOmegaRadiansPerSec = omegaRadPerSec;
    armTargetAlphaRadiansPerSec2 = 0;
    armFeedForwardTorque = torque;
    m_ArmState = ArmState.MotionMagicToPosition;
  }

  public void runPath() {
    // Add implementation here if needed
  }

  public void runVolts(double volts) {
    io.runVolts(volts);
    m_ArmState = ArmState.Volt;
  }

  public void stop() {
    io.stop();
    m_ArmState = ArmState.Stop;
  }

  public double getOmegaRadPerSec() {
    return inputs.armOmegaRadPerSec;
  }

  public double getAlphaRadsPerSecSquared() {
    return inputs.armAlphaRadsPerSecSquared;
  }

  public double getAngleRads() {
    return inputs.armPositionRads;
  }

  public double getvolts() {
    return inputs.armAppliedVolts;
  }

  @AutoLogOutput
  public boolean atGoal() {
    return Math.abs(goal - getAngleRads()) < ArmConstants.armTolerance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }
}
