package frc.robot.subsystems.SuperStructure.arm;

public class ArmConfig {
  public double armMass;
  public double armLength;
  public double armMOI;
  public double armCOMRadius;

  public ArmConfig(double armMass, double armLength, double armMOI, double armCOMRadius) {
    this.armMass = armMass;
    this.armLength = armLength;
    this.armMOI = armMOI;
    this.armCOMRadius = armCOMRadius;
  }
}
