package frc.robot.subsystems.SuperStructure.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SuperStructure.wrist.WristIO.WristIOInputs;

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
    Test,
    RunGoal
  }

  public static WristState wristState = WristState.Stop;

  public void runSetPoint(
      double angleRads, double omegaRadPerSec, double alphaRadsPerSecSquared, double torque) {
    io.runPositionSetpoint(angleRads, omegaRadPerSec, alphaRadsPerSecSquared, torque);
  }

  public void runMotionMagicPosition(double angleRads) {
    goal =
        MathUtil.clamp(
            angleRads, WristConstants.wristMinimumAngle, WristConstants.wristMaximumAngle);
    io.runMotionMagicPosition(goal);
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

  public void setState(WristState state) {
    wristState = state;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    switch (wristState) {
      case RunGoal -> {
        // runMotionMagicPosition(WristConstants.wristIntakeAngle);
      }
      case Test -> {
        runVolts(WristConstants.testVolts);
      }
      case Stop -> {
        stop();
      }
      default -> {}
    }

    // SmartDashboard.putNumber("WristOmegaRadPerSec()", getOmegaRadPerSec());
    // SmartDashboard.putBoolean("atGoal", atGoal());
    SmartDashboard.putNumber("WristAngleRads", getAngleRads());
    // SmartDashboard.putNumber("WristWristAppliedVolts", getAlphaRadsPerSecSquared());
    SmartDashboard.putString("WristState", wristState.toString());
  }
}
