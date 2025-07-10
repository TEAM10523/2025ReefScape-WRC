package frc.robot.subsystems.SuperStructure.elevator;

public class ElevatorConstants {
  public static final int elevatorId = 13;

  public static final double sprocketRadius = 0.0222377;
  public static final double elevatorGearRatio = 9 / 2; // TODO

  public static final double kp = 200;
  public static final double Ki = 0.0;
  public static final double Kd = 20; // 0.0;
  public static final double Kg = 0;
  public static final double Ks = 15;
  public static final double Kv = 0;
  public static final double Ka = 0.;
  public static final double Kf = 95;
  public static final double acceleration = 12;
  public static final double velocity = 7;

  public static final double elevatorMinimumPositin = 0.0; // meters
  public static final double elevatorMaximumPositin = 1.47; // meters
  public static final double elevatorInitialPositin = -0.15;
  public static final double elevatorIntakePositin = 0.05;

  public static final double elevatorTolerance = 0.02;

  public static final double testUpVolts = 5.0;
  public static final double testDownVolts = -5.0;

  // Sim
  public static final double jkMetersSquared = 1.06328;
  public static final double armLength = 0.5;
  public static final double loopPeriodSecs = 0.02;
  public static final double drumRadiusMeters = 0.494;
  public static final ElevatorConfig _elevatorConfig =
      new ElevatorConfig(3, elevatorMaximumPositin, sprocketRadius);
  // TODO
}
