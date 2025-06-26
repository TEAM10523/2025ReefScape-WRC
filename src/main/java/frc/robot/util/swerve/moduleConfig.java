package frc.robot.util.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public record moduleConfig(
    int driveID,
    int turnID,
    int absoluteEncoderID,
    double driveGearRatio,
    double turnGearRatio,
    Rotation2d absoluteEncoderOffset,
    boolean turnMotorInverted) {}
