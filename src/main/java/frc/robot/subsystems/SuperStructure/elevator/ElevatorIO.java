package frc.robot.subsystems.SuperStructure.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    public boolean elevatorMotorConnected = true;

    public double elevatorPositionMeters = 0.0;
    public double elevatorVelocityMetersPerSec = 0.0;
    public double elevatorAcceleration = 0.0;
    public double elevatorAppliedVolts = 0.0;
    public double elevatorSupplyCurrentAmps = 0.0;
    public double elevatorTorqueCurrentAmps = 0.0;
    public double elevatorTempCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(ElevatorIOInputs inputs) {}

  /** Run elevator motor at volts */
  default void runVolts(double volts) {}

  /** Run to elevator positionsetpoint with feedforward */
  default void runPositionSetpoint(
      double positionMeters, double velocityMetersPerSec, double acceleration, double torque) {}

  /** Run to elevator positionsetpoint with motionmagic */
  default void runMotionMagicPosition(
      double positionMeters, double elevatorVelocityMetersPerSec, double torque) {}
  /** Configure elevator PID */
  default void setPID(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double kG,
      double MotionMagicAcceleration,
      double MotionMagicCruiseVelocity) {}

  /** resetPosition */
  default void resetPosition(double angleRads) {}

  /** Disable output to all motors */
  default void stop() {}
}
