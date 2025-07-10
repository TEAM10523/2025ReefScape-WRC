package frc.robot.subsystems.SuperStructure.wrist;

import edu.wpi.first.math.util.Units;

public class WristConstants {
  public static final int id = 15;

  public static final double wristRadius = 0.494;
  public static final double wristGearRatio = 10.0;

  public static final double kp = 1000;
  public static final double Ki = 0.0;
  public static final double Kd = 100; // 0.0;
  public static final double Kg = 0;
  public static final double Ks = 2;
  public static final double Kv = 0;
  public static final double Ka = 0;
  public static final double acceleration = 10;
  public static final double velocity = 3;

  public static final double wristMinimumAngle = Math.toRadians(-90); // radius
  public static final double wristMaximumAngle = Math.toRadians(150); // radius
  public static final double wristInitialAngle = Math.toRadians(0);
  public static final double wristIntakeAngle = Math.toRadians(20);

  public static final double wristScoringPosition = 1.8;
  public static final double wristIntakingPosition = 0;

  public static final double wristTolerance = Math.toRadians(10);

  public static final double testVolts = 5.0;

  // Sim
  public static final double autoStartAngle = Units.degreesToRadians(0.0);
  public static final double jkMetersSquared = 1.06328;
  public static final double wristLength = 0.5;
  public static final double loopPeriodSecs = 0.02;
}
