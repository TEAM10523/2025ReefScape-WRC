package frc.robot.subsystems.SuperStructure.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SuperStructure.intake.IntakeIO.IntakeIOInputs;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputs inputs = new IntakeIOInputs();

  public Intake(IntakeIO io) {
    this.io = io;
    io.setPID(
      IntakeConstants.Kp,
      IntakeConstants.Ki,
      IntakeConstants.Kd,
      IntakeConstants.Ks,
      IntakeConstants.Kv,
      IntakeConstants.Ka);
  }

  public enum IntakeState{
    Intake,
    Eject,
    Score,
    Rest,
    Stop
  }

  public static IntakeState intakeState = IntakeState.Stop;


  public void runSetPoint(double velocityRadsPerSec, double acceleration) {
    io.runVelocitySetpoint(velocityRadsPerSec, acceleration);
  }

  public void runVolts(double volts) {
    io.runVolts(volts);
  }

   /** Stops motors. */
  public void stop() {
    io.stop();
  }

    /** Get velocity of rolley in m/s. */
  public double getVelocityMetersPerSec() {
      return inputs.intakeVelocityMetersPerSec;
  }

  public double getvolts(){
      return inputs.intakeAppliedVolts;
  }

  
  public void setState(IntakeState state){
      intakeState=state;
  }

  public boolean hasCoral(){
    return Math.abs(inputs.intakeVelocityMetersPerSec) < IntakeConstants.tolerance && 
      inputs.intakeTorqueCurrentAmps > IntakeConstants.minSupplyCurrent;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    switch (intakeState) {
      case Intake -> {
        runSetPoint(80, 0);
      }
      case Eject -> {
        runSetPoint(-1, 0);
      }
      case Score -> {
        // runVolts(IntakeConstants.scoreVolts);
      }
      case Rest -> {
        runSetPoint(0.1, 0);
      }
      case Stop -> {
        stop();
      }
      default -> {}
    }
    SmartDashboard.putString("IntakeState", intakeState.toString());

    // SmartDashboard.putNumber("velocity", getVelocityMetersPerSec());
    // SmartDashboard.putNumber("volts", getvolts());
    // SmartDashboard.putBoolean("sensor", sensorDetected());
  }
}
