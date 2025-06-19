package frc.robot.util.swerve;

import frc.robot.util.math.vector;

public record massDistribution(
    vector[] measuredDetectPoint,
    double[] detectPointMass
) {}
