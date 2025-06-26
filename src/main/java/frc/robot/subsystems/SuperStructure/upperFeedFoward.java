package frc.robot.subsystems.SuperStructure;

import frc.robot.subsystems.SuperStructure.arm.ArmConfig;
import frc.robot.subsystems.SuperStructure.elevator.ElevatorConfig;
import frc.robot.subsystems.SuperStructure.intake.intakeConfig;

public class upperFeedFoward {
  private final double g = 9.8;
  private ArmConfig arm;
  private ElevatorConfig elevator;
  private intakeConfig intake;

  public upperFeedFoward(ArmConfig arm, ElevatorConfig elevator, intakeConfig intake) {
    this.arm = arm;
    this.elevator = elevator;
    this.intake = intake;
  }

  public double[] toMotorTorque(
      double armCurrentAngle,
      double elevatorCurrentHeight,
      double elevatorAcc,
      double armAngularVelocity,
      double armAngularAcc) {
    double[] position = {
      Math.cos(armCurrentAngle), Math.sin(armCurrentAngle) + elevatorCurrentHeight
    };
    double armMotorTorque =
        ((arm.armMass + intake.intakeMass)
                * (g + elevatorAcc)
                * (position[0] * arm.armCOMRadius / arm.armLength)
            + (arm.armMass / 3 + intake.intakeMass)
                * (arm.armLength * arm.armLength)
                * armAngularAcc);
    double liftMotorTorque =
        ((elevator.elevatorMass * (g + elevatorAcc)
                + (arm.armMass * arm.armCOMRadius * armAngularVelocity * armAngularVelocity
                        + intake.intakeMass
                            * arm.armLength
                            * (armAngularVelocity * armAngularVelocity))
                    * (position[1] - elevator.elevatorHeight)
                    / arm.armLength)
            * elevator.elevatorPulleyRadius);
    double[] toTorque = {liftMotorTorque, armMotorTorque};
    return toTorque;
  }
}
