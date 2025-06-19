// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.field2025;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

/** Add your docs here. */
public class FieldPoseConversions {

    public static Pose2d inversePose2dUsingAlliance(Pose2d pose, DriverStation.Alliance allianceColor){
        if(allianceColor != getAlliance()){
            return new Pose2d(FieldConstants.fieldLength - pose.getX(), FieldConstants.filedWidth - pose.getY(),
            new Rotation2d(Math.PI).plus(pose.getRotation()));
        }
        return pose;
    }

    public static Pose2d YFlipPose2d(Pose2d pose){
        return new Pose2d(pose.getX(), FieldConstants.filedWidth - pose.getY(), new Rotation2d(0).minus(pose.getRotation()));
    }

    public static pairReef inversePairReefUsingAlliance(pairReef pairReef, DriverStation.Alliance allianceColor){
        Pose2d pairReefPose = inversePose2dUsingAlliance(pairReef.pairReefPose(), allianceColor);
        Pose2d[] reefsPose = pairReef.reefsPose().clone();
        for(int i = 0; i < reefsPose.length; i++){
            reefsPose[i] = inversePose2dUsingAlliance(reefsPose[i], allianceColor);
        }
        return new pairReef(pairReefPose, reefsPose, pairReef.algaeLevel());
    }

    public static DriverStation.Alliance getAlliance(){
        try{
            return DriverStation.getAlliance().get();
        }catch(Exception e){
            return DriverStation.Alliance.Blue;
        }
    }

}
