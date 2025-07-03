package frc.robot.subsystems.SuperStructure.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SuperStructure.wrist.WristIO.WristIOInputs;
import org.littletonrobotics.junction.AutoLogOutput;

public class Wrist extends SubsystemBase {
  private final WristIO io;
  private final WristIOInputs inputs = new WristIOInputs();

  public double goal = 0.0;

  public Wrist(WristIO io) {
    this.io = io;
    io.setPID(
        WristConstants.kp,
        WristConstants.Ki,
        WristConstants.Kd,
        WristConstants.Ks,
        WristConstants.Kv,
        WristConstants.Ka,
        WristConstants.acceleration,
        WristConstants.velocity);
  }

  public enum WristState {
    Stop,
    MotionMagicToPosition,
    FollowingPath
  }
  // These variables are used for logging and telementry, they don't actually participate in the
  // calculation and the running of the robot
  @AutoLogOutput public static WristState wristState = WristState.Stop;
  @AutoLogOutput private double wristTargetPositionRadian = 0;
  @AutoLogOutput private double wristTargetOmegaRadiansPerSec = 0;
  @AutoLogOutput private double wristFeedForwardTorque = 0;
  @AutoLogOutput private double wristTargetAlphaRadiansPerSec2 = 0;

  public void runSetPoint(
      double angleRads, double omegaRadPerSec, double alphaRadsPerSecSquared, double torque) {
    io.runPositionSetpoint(angleRads, omegaRadPerSec, alphaRadsPerSecSquared, torque);
    wristTargetPositionRadian = angleRads;
    wristTargetOmegaRadiansPerSec = omegaRadPerSec;
    wristTargetAlphaRadiansPerSec2 = alphaRadsPerSecSquared;
    wristFeedForwardTorque = torque;
    wristState = WristState.FollowingPath;
  }

  public void runMotionMagicPosition(double angleRads) {
    goal =
        MathUtil.clamp(
            angleRads, WristConstants.wristMinimumAngle, WristConstants.wristMaximumAngle);
    io.runMotionMagicPosition(goal);
    wristTargetPositionRadian = angleRads;
    wristState = WristState.MotionMagicToPosition;
  }

  public void runVolts(double volts) {
    io.runVolts(volts);
  }

  public void stop() {
    io.stop();
  }

  public double getOmegaRadPerSec() {
    return inputs.wristOmegaRadPerSec;
  }

  public double getAlphaRadsPerSecSquared() {
    return inputs.wristAlphaRadsPerSecSquared;
  }

  public double getAngleRads() {
    return inputs.wristPositionRads;
  }

  public double getvolts() {
    return inputs.wristAppliedVolts;
  }

  public boolean atGoal() {
    return Math.abs(goal - getAngleRads()) < WristConstants.wristTolerance;
  }

  public boolean atGoal(double _WristAngle) {
    return Math.abs(_WristAngle - getAngleRads()) < WristConstants.wristTolerance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // SmartDashboard.putNumber("WristOmegaRadPerSec()", getOmegaRadPerSec());
    // SmartDashboard.putBoolean("atGoal", atGoal());
    // SmartDashboard.putNumber("WristAngleRads", getAngleRads());
    // SmartDashboard.putNumber("WristWristAppliedVolts", getAlphaRadsPerSecSquared());
    // SmartDashboard.putString("WristState", wristState.toString());
  }
}
