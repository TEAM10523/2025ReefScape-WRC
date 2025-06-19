package frc.robot.util.field2025;

import edu.wpi.first.math.geometry.Pose2d;

public record pairReef(
    Pose2d pairReefPose,
    Pose2d[] reefsPose,
    int algaeLevel
) {}
