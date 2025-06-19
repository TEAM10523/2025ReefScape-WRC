package frc.robot.subsystems.SuperStructure.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SuperStructure.arm.ArmIO.ArmIOInputs;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputs inputs = new ArmIOInputs();

  public double goal = 0.0;

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

  public enum ArmState{
    Stop,
    Test_UP,
    Test_DOWN,
    RunGoal
  }

  public static ArmState armState = ArmState.Stop;

  public void runSetPoint(double angleRads, double omegaRadPerSec, double alphaRadsPerSecSquared, double torque) {
    io.runPositionSetpoint(angleRads, omegaRadPerSec, alphaRadsPerSecSquared, torque);
  }

  public void runMotionMagicPosition(double angleRads, double omegaRadPerSec, double torque) {
    goal = MathUtil.clamp(angleRads, ArmConstants.armMinimumAngle, ArmConstants.armMaximumAngle);
    io.runMotionMagicPosition(goal, omegaRadPerSec, torque);
  }

  public void runPath() {
    // Add implementation here if needed
  }

  public void runVolts(double volts) {
    io.runVolts(volts);
  }

  public void stop() {
    io.stop();
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

  public boolean atGoal(){
    return Math.abs(goal-getAngleRads())<ArmConstants.armTolerance;
  }

  public void setState(ArmState state){
      armState=state;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
     switch (armState) {
      case RunGoal -> {
        
      }
      case Test_UP -> {
        // runVolts(ArmConstants.testUpVolts);
      }
      case Test_DOWN -> {
        // runVolts(ArmConstants.testDownVolts);
      }
      case Stop -> {
        stop();
      }
      default -> {}
    }

    SmartDashboard.putNumber("armOmegaRadPerSec()", getOmegaRadPerSec());
    SmartDashboard.putBoolean("atGoal", atGoal());
    SmartDashboard.putNumber("armAngleRads", getAngleRads());
    SmartDashboard.putNumber("armarmAppliedVolts", getAlphaRadsPerSecSquared());
    SmartDashboard.putString("ArmState", armState.toString());
  }
}
