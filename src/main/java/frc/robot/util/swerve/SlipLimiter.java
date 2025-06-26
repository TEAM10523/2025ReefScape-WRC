// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.math.vector;

/** Add your docs here. */
public class SlipLimiter {

    public SlipLimiter() {
    }

    public ChassisSpeeds getChassisSpeeds(
        final double maxVStep,
        final double maxV,
        final ChassisSpeeds preSpeeds,
        ChassisSpeeds desireState) {
        
        vector preV = new vector(new double[]{preSpeeds.vxMetersPerSecond, preSpeeds.vyMetersPerSecond});
        vector desireV = new vector(new double[]{desireState.vxMetersPerSecond, desireState.vyMetersPerSecond});

        vector deltaV = desireV.add(preV.times(-1));
        double deltaVNorm = deltaV.norm();
        
        if(deltaVNorm > maxVStep){
            desireV = preV.add(deltaV.times(maxVStep/deltaVNorm));
        }

        desireState = new ChassisSpeeds(desireV.vectorArray[0], desireV.vectorArray[1], desireState.omegaRadiansPerSecond);

        return desireState;
    }

}
