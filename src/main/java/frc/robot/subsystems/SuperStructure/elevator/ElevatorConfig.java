package frc.robot.subsystems.SuperStructure.elevator;

public class ElevatorConfig {
  public double elevatorMass;
  public double elevatorHeight;
  public double elevatorPulleyRadius;

  public ElevatorConfig(double elevatorMass, double elevatorHeight, double elevatorPulleyRadius) {
    this.elevatorMass = elevatorMass;
    this.elevatorHeight = elevatorHeight;
    this.elevatorPulleyRadius = elevatorPulleyRadius;
  }
}
