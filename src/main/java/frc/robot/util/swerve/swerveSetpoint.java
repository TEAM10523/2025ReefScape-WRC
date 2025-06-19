package frc.robot.util.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class swerveSetpoint {
    public ChassisSpeeds cuChassisSpeeds;
    public moduleState[] cuModuleState;

    public swerveSetpoint(ChassisSpeeds chassisSpeeds, moduleState[] initialStates){
        cuChassisSpeeds = chassisSpeeds;
        cuModuleState = initialStates;
    }
    @Override
    public String toString() {
        String ret = cuChassisSpeeds.toString() + "\n";
        for (int i = 0; i < cuModuleState.length; ++i ) {
            ret += "  " + cuModuleState[i].toString() + "\n";
        }
        return ret;
    }
}
