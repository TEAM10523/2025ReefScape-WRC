package frc.robot.subsystems.SuperStructure.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SuperStructure.elevator.ElevatorIO.ElevatorIOInputs;
import org.littletonrobotics.junction.AutoLogOutput;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputs inputs = new ElevatorIOInputs();

  public double goal = 0.0;

  public Elevator(ElevatorIO io) {
    this.io = io;
    io.setPID(
        ElevatorConstants.kp,
        ElevatorConstants.Ki,
        ElevatorConstants.Kd,
        ElevatorConstants.Ks,
        ElevatorConstants.Kv,
        ElevatorConstants.Ka,
        ElevatorConstants.Kg,
        ElevatorConstants.acceleration,
        ElevatorConstants.velocity);
  }

  public enum ElevatorState {
    FollowingPath,
    MotionMagicToPosition,
    Volt,
    Stop
  }

  @AutoLogOutput public ElevatorState elevatorState = ElevatorState.Stop;
  @AutoLogOutput private double elevatorTargetHeight = 0;
  @AutoLogOutput private double elevatorTargetVelocity = 0;
  @AutoLogOutput private double elevatorFeedForwardTorque = 0;
  @AutoLogOutput private double elevatorTargetAcceleration = 0;

  public void runSetPoint(
      double positionMeters, double velocityMetersPerSec, double acceleration, double torque) {
    elevatorTargetHeight = positionMeters;
    elevatorTargetVelocity = velocityMetersPerSec;
    elevatorTargetAcceleration = acceleration;
    elevatorFeedForwardTorque = torque;
    elevatorState = ElevatorState.FollowingPath;
    io.runPositionSetpoint(positionMeters, velocityMetersPerSec, acceleration, torque);
  }

  public void runMotionMagicPosition(
      double positionMeters, double velocityMetersPerSec, double torque) {
    goal =
        MathUtil.clamp(
            positionMeters,
            ElevatorConstants.elevatorMinimumPositin,
            ElevatorConstants.elevatorMaximumPositin);
    elevatorState = ElevatorState.MotionMagicToPosition;
    io.runMotionMagicPosition(goal, velocityMetersPerSec, torque);
    elevatorTargetHeight = positionMeters;
    elevatorTargetVelocity = velocityMetersPerSec;
    elevatorFeedForwardTorque = torque;
    elevatorState = ElevatorState.MotionMagicToPosition;
  }

  public void resetPosition(double x) {
    io.resetPosition(x);
  }

  public void runVolts(double volts) {
    elevatorState = ElevatorState.Volt;
    io.runVolts(volts);
  }

  public void stop() {
    io.stop();
  }

  public double getVelocityMetersPerSec() {
    return inputs.elevatorVelocityMetersPerSec;
  }

  public double getAcceleration() {
    return inputs.elevatorAcceleration;
  }

  public double getPositionMeters() {
    return inputs.elevatorPositionMeters;
  }

  public double getvolts() {
    return inputs.elevatorAppliedVolts;
  }

  @AutoLogOutput
  public boolean atGoal() {
    return Math.abs(goal - getPositionMeters()) < ElevatorConstants.elevatorTolerance;
  }

  public boolean atGoal(double _ElevatorHeight) {
    return Math.abs(_ElevatorHeight - getPositionMeters()) < ElevatorConstants.elevatorTolerance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // SmartDashboard.putNumber("ElevatorVegetVelocityMetersPerSec()", getVelocityMetersPerSec());
    // SmartDashboard.putBoolean("atGoal", atGoal());
    SmartDashboard.putNumber("ElevatorPsigetPositionMeters", getPositionMeters());
    SmartDashboard.putNumber("Elevatoracceleration", getAcceleration());
    SmartDashboard.putString("ElevatorState", elevatorState.toString());
  }
}
