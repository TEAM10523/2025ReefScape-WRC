package frc.robot.subsystems.SuperStructure.arm;

import edu.wpi.first.math.util.Units;

public class ArmConstants {
  public static final int id = 14;

  public static final double armRadius = 0.494;
  public static final double armGearRatio = 80 * 62 * 60 / (18 * 18 * 10);

  public static final double kp = 600;
  public static final double Ki = 0.0;
  public static final double Kd = 80; // 0.0;
  public static final double Kg = 0;
  public static final double Ks = 5;
  public static final double Kv = 0;
  public static final double Ka = 0;
  public static final double Kf = 1; // converting current to torque
  public static final double acceleration = 3;
  public static final double velocity = 1;

  public static final double armMinimumAngle = Math.toRadians(-5); // radius
  public static final double armMaximumAngle = Math.toRadians(180); // radius
  public static final double armInitialAngle = Math.toRadians(90);
  public static final double armIntakeAngle = Math.toRadians(20);

  public static final double armTolerance = Math.toRadians(30);

  public static final double testUpVolts = 5.0;
  public static final double testDownVolts = -5.0;

  // Sim
  public static final double autoStartAngle = Units.degreesToRadians(0.0);
  public static final double jkMetersSquared = 1.06328;
  public static final double armLength = 0.5;
  public static final double loopPeriodSecs = 0.02;
  public static final ArmConfig _armConfig = new ArmConfig(3, armLength, 0.25, 0.25);
  // TODO
}
