package frc.robot.subsystems.SuperStructure.intake;


public class IntakeConstants {
    public static final int id = 16;
    public static final int intakePhotoSensorChannel = 1;

    public static final double intakeGearRatio = 3.0;//Gear Ratios
    public static final double wheelRadius = 0.05;

    public static final double Kp = 30;
    public static final double Ki = 0;
    public static final double Kd = 0;
    public static final double Ks = 0;
    public static final double Kv = 0;
    public static final double Ka = 0;
    
    public static final double intakeVolts = 10.0;
    public static final double scoreVolts  = 11.0;
    public static final double ejectVolts = -10.0;
    public static final intakeConfig _intakeConfig = new intakeConfig(2);

    public static final double tolerance = 0.03;
    public static final double minSupplyCurrent = 1;

    //TODO
}
