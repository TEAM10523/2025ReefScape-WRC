package frc.robot.subsystems.SuperStructure.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SuperStructure.elevator.ElevatorIO.ElevatorIOInputs;

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
    Stop,
    Test_UP,
    Test_DOWN,
    RunGoal
  }

  public ElevatorState elevatorState = ElevatorState.Stop;

  public void runSetPoint(
      double positionMeters, double velocityMetersPerSec, double acceleration, double torque) {
    io.runPositionSetpoint(positionMeters, velocityMetersPerSec, acceleration, torque);
  }

  public void runMotionMagicPosition(
      double positionMeters, double velocityMetersPerSec, double torque) {
    goal =
        MathUtil.clamp(
            positionMeters,
            ElevatorConstants.elevatorMinimumPositin,
            ElevatorConstants.elevatorMaximumPositin);
    io.runMotionMagicPosition(goal, velocityMetersPerSec, torque);
  }

  public void resetPosition(double x) {
    io.resetPosition(x);
  }

  public void runVolts(double volts) {
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

  public boolean atGoal() {
    return Math.abs(goal - getPositionMeters()) < ElevatorConstants.elevatorTolerance;
  }

  public void setState(ElevatorState state) {
    elevatorState = state;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    switch (elevatorState) {
      case RunGoal -> {
        // runMotionMagicPosition(ElevatorConstants.elevatorIntakePositin, 0, 0);//TODO
      }
      case Test_UP -> {
        // runVolts(ElevatorConstants.testUpVolts);
      }
      case Test_DOWN -> {
        // runVolts(ElevatorConstants.testDownVolts);
      }
      case Stop -> {
        stop();
      }
      default -> {}
    }

    // SmartDashboard.putNumber("ElevatorVegetVelocityMetersPerSec()", getVelocityMetersPerSec());
    // SmartDashboard.putBoolean("atGoal", atGoal());
    SmartDashboard.putNumber("ElevatorPsigetPositionMeters", getPositionMeters());
    SmartDashboard.putNumber("Elevatoracceleration", getAcceleration());
    SmartDashboard.putString("ElevatorState", elevatorState.toString());
  }
}
