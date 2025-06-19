// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.field2025;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class PoseTargets {
    
    public static pairReef getNearestPairReef(Pose2d robotPose){
        pairReef nearestPairReef = new pairReef(null, null, 2);
        double shortestDistance = 1e+120;
        for(pairReef reef: FieldConstants.pairReefs){
            pairReef inversedReed = FieldPoseConversions.inversePairReefUsingAlliance(reef, Alliance.Blue);
            double distance = robotPose.getTranslation().getDistance(inversedReed.pairReefPose().getTranslation());
            if(distance < shortestDistance){
                nearestPairReef = inversedReed;
                shortestDistance = distance;
            }
        }
        return nearestPairReef;
    }

}
