package frc.robot.util.swerve;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.math.vector;


public class swerveSetpointGenerator {
    private SwerveDriveKinematics cuKinematic;
    private vector[] modulePositions;
    private double g = 9.8;

    public swerveSetpointGenerator(SwerveDriveKinematics cuKinematic){
        this.cuKinematic = cuKinematic;
        Translation2d[] moduleTranslations = cuKinematic.getModules();
        modulePositions = new vector[moduleTranslations.length];
        for(int i =0; i < moduleTranslations.length; i++){
            modulePositions[i] = new vector(moduleTranslations[i]);
        }
    } 

    public vector getAxisRadius(vector v, double omega){
        return new vector(new double[]{
            (-v.vectorArray[1])/(omega), 
            (v.vectorArray[0])/(omega)}
            );
    }
    public vector[] toRotationCenter(vector rotationCenter, vector[] vectors){
        vector[] vectorsToCenter = new vector[vectors.length];
        vector inv_rotationCenter = rotationCenter.times(-1);
        for(int i = 0; i < vectors.length; i++){
            vectorsToCenter[i] = vectors[i].add(inv_rotationCenter);
        }
        return vectorsToCenter;
    }
    public double getInertia(vector rotationCenter, massDistribution pointMasses){
        vector[] measuredDetectPoint = pointMasses.measuredDetectPoint();
        vector[] pointsToCenter = toRotationCenter(rotationCenter, measuredDetectPoint);
        double[] detectPointMass = pointMasses.detectPointMass();
        double inertia = 0;
        for(int i = 0; i < measuredDetectPoint.length; i++){
            inertia += detectPointMass[i] * Math.pow(pointsToCenter[i].norm(), 2);
        }
        return inertia;
    }
    public double getOmega(vector r, vector v){
        return new vector(new double[]{
            -r.vectorArray[1],
            r.vectorArray[0]
        }).dot(v);
    }

    private double getModuleAccForce(double v, double vmax, double StallForce, double ForceWithoutSlip, double ForceVoltageLim){
        double tempF = Math.min((StallForce*(1-(v/vmax))),
                                ForceWithoutSlip);
        return Math.min(tempF, ForceVoltageLim);
    }
    private double getModuleDecForce(double v, double vmax, double breakingForce, double ForceWithoutSlip){
        double tempF = Math.min((breakingForce*(v/vmax)),
                                ForceWithoutSlip);
        return tempF;
    }
    
    private boolean flipHeading(Rotation2d prevToGoal) {
        return Math.abs(prevToGoal.getRadians()) > Math.PI / 2.0;
    }

    private double getRobotTorque(vector[] modulesForce, vector[] modulePositions){
        double torque = 0;
        for(int i = 0; i < modulesForce.length; i++){
            torque += modulesForce[i].dot(new vector(new double[]{
                -modulePositions[i].vectorArray[1],
                modulePositions[i].vectorArray[0]
            }));
        }
        return torque;
    }
    private double getRobotForce(vector[] modulesForce, vector preV){
        double f = 0;
        vector direction = preV.unit();
        for(int i = 0; i < modulesForce.length; i++){
            f += direction.dot(modulesForce[i]);
        }
        return f;
    }

    public swerveSetpoint generateSetpoint(final ChassisLimits LimitConstants, final swerveSetpoint prevSetpoint, ChassisSpeeds desireState, double dt) {
        //convert
        moduleState[] preModuleStates = prevSetpoint.cuModuleState;
        int num_modules = preModuleStates.length;

        ChassisSpeeds currentChassisSpeeds = prevSetpoint.cuChassisSpeeds;
        vector preVelocity = new vector(new double[]{
            currentChassisSpeeds.vxMetersPerSecond,
            currentChassisSpeeds.vyMetersPerSecond
        });
        double preOmega = currentChassisSpeeds.omegaRadiansPerSecond;

        vector[] modulesV = new vector[num_modules];
        for(int i = 0; i< num_modules; i++){
            modulesV[i] = new vector(preModuleStates[i].getVector());
        }

        SwerveModuleState[] wpiSwerveModuleStates = cuKinematic.toSwerveModuleStates(
            desireState
        );
            
        moduleState[] desireStates = new moduleState[num_modules];
        double maxDesireSpeed = 0;
        for(int i = 0; i < num_modules; i++){
            desireStates[i] = new moduleState(wpiSwerveModuleStates[i]);
            maxDesireSpeed = Math.max(maxDesireSpeed, Math.abs(wpiSwerveModuleStates[i].speedMetersPerSecond));
        }
        
        double desaturateScale = maxDesireSpeed > LimitConstants.MaxDriveVelocity? LimitConstants.MaxDriveVelocity / maxDesireSpeed: 1;
        vector[] desireVs = new vector[num_modules];
        for(int i = 0; i< num_modules; i++){
            desireVs[i] = new vector(desireStates[i].getVector()).times(desaturateScale);
        }

        vector desireVelocity = new vector(new double[]{
            desireState.vxMetersPerSecond,
            desireState.vyMetersPerSecond
        }).times(desaturateScale);

        double desireOmega = desireState.omegaRadiansPerSecond * desaturateScale;

        //check if module need acceleration
        boolean[] isAcc = new boolean[num_modules];
        for (int i = 0; i < num_modules; ++i) {
            isAcc[i] = modulesV[i].dot(desireVs[i]) > 0 && desireVs[i].norm() > modulesV[i].norm();
        }
        // to modules' force
        double maxAWithoutSlipping = g * LimitConstants.frictionCoeff;
        double maxForceWithoutSlipping = LimitConstants.robotMass * maxAWithoutSlipping / num_modules;
        vector[] modulesDv = new vector[num_modules];
        double minDvF = 1e+12;
        for (int i = 0; i < num_modules; ++i) {
            // double f = isAcc[i]? 
            //     getModuleAccForce(modulesV[i].norm(), LimitConstants.MaxDriveVelocity, LimitConstants.Fstall, maxForceWithoutSlipping, LimitConstants.FmaxWithVoltagelimit) : 
            //     getModuleDecForce(modulesV[i].norm(), LimitConstants.MaxDriveVelocity, LimitConstants.Fbreaking, maxForceWithoutSlipping);
            // modulesDv[i] = desireVs[i].add(modulesV[i].times(-1));
            // double DvF = f / (modulesDv[i].norm() + Util.kEpsilon);
            double f = isAcc[i]? 
                getModuleAccForce(modulesV[i].norm(), LimitConstants.MaxDriveVelocity, LimitConstants.Fstall, maxForceWithoutSlipping, LimitConstants.FmaxWithVoltagelimit) : 
                getModuleDecForce(modulesV[i].norm(), LimitConstants.MaxDriveVelocity, LimitConstants.Fbreaking, maxForceWithoutSlipping);
            modulesDv[i] = desireVs[i].add(modulesV[i].times(-1));
            double DvF = f / (Math.abs(modulesDv[i].dot(modulesV[i].unit())) + Util.kEpsilon);
            double DvFWithoutSlipping = LimitConstants.Fbreaking / (modulesDv[i].norm() + Util.kEpsilon);
            DvF = Math.min(DvF, DvFWithoutSlipping);
            minDvF = Math.min(minDvF, DvF);
        }
        vector[] modulesForce = new vector[num_modules];
        for (int i = 0; i < num_modules; ++i) {
            modulesForce[i] = modulesDv[i].times(minDvF);
        }


        //calculating steps
        SmartDashboard.putNumber("robotTorque", getRobotTorque(modulesForce, modulePositions));
        double maxOmegaStep = dt * Math.abs(getRobotTorque(modulesForce, modulePositions)) / LimitConstants.robotMIO;
        double maxVelocityUpStep = dt * Math.abs(getRobotForce(modulesForce, preVelocity)) / LimitConstants.robotMass;
        double maxVelocitySideStep = dt * maxAWithoutSlipping;

        vector deltaV = desireVelocity.add(preVelocity.times(-1));

        //break down to components
        vector preDirection = preVelocity.unit();
        double deltaVUp = Math.abs(preDirection.dot(deltaV)) + Util.kEpsilon;
        double deltaVSide = Math.abs(new vector(new double[]{
            -preDirection.vectorArray[1],
            preDirection.vectorArray[0]}).dot(deltaV)) + Util.kEpsilon;
        double deltaOmega = Math.abs(preOmega - desireOmega) + Util.kEpsilon;

        double s = 1;

        if(deltaOmega > LimitConstants.speedDeadband){
            s = Math.min(s, maxOmegaStep / deltaOmega);
        }
        if(deltaVUp > LimitConstants.speedDeadband){
            s = Math.min(s, maxVelocityUpStep / deltaVUp);
        }
        if(deltaVSide > LimitConstants.speedDeadband){
            s = Math.min(s, maxVelocitySideStep / deltaVSide);
        }
        // SmartDashboard.putNumberArray("forces", new double[]{
        //     new Translation2d(modulesForce[0].vectorArray[0], modulesForce[0].vectorArray[1]).getAngle().getRadians(), modulesForce[0].norm(), 
        //     new Translation2d(modulesForce[1].vectorArray[0], modulesForce[1].vectorArray[1]).getAngle().getRadians(), modulesForce[1].norm(), 
        //     new Translation2d(modulesForce[2].vectorArray[0], modulesForce[2].vectorArray[1]).getAngle().getRadians(), modulesForce[2].norm(), 
        //     new Translation2d(modulesForce[3].vectorArray[0], modulesForce[3].vectorArray[1]).getAngle().getRadians(), modulesForce[3].norm(), 

        // });

        SmartDashboard.putNumber("deltaup", deltaVUp);
        SmartDashboard.putNumberArray("currentSpeed", new double[]{
            preVelocity.vectorArray[0],
            preVelocity.vectorArray[1],
            preOmega

        });

        SmartDashboard.putNumber("maxOmegaStep", Math.min(1, maxOmegaStep / deltaOmega));
        SmartDashboard.putNumber("maxVelocityUpStep", Math.min(1, maxVelocityUpStep / deltaVUp));
        SmartDashboard.putNumber("maxVelocitySideStep", Math.min(1, maxVelocitySideStep / deltaVSide));
        SmartDashboard.putNumber("maxForce", maxVelocityUpStep);
        SmartDashboard.putNumber("omegaStep", maxOmegaStep);


        s = Math.max(s, Util.kEpsilon);
        SmartDashboard.putNumber("s", s);

        desireOmega = preOmega + s * (desireOmega - preOmega);
        desireVelocity = preVelocity.add(desireVelocity.add(preVelocity.times(-1)).times(s));

        ChassisSpeeds desireChassisSpeeds = new ChassisSpeeds(desireVelocity.vectorArray[0], desireVelocity.vectorArray[1], desireOmega);
        wpiSwerveModuleStates = cuKinematic.toSwerveModuleStates(
            desireChassisSpeeds
            );
            
        moduleState[] retStates = new moduleState[num_modules];
        for(int i = 0; i < num_modules; i++){
            retStates[i] = new moduleState(wpiSwerveModuleStates[i]);
            retStates[i].driveForce = modulesForce[i].norm();
        }

        for (int i = 0; i < num_modules; ++i) {
            final var deltaRotation = prevSetpoint.cuModuleState[i].turnPosition.unaryMinus().rotateBy(retStates[i].turnPosition);
            boolean flip = flipHeading(deltaRotation);
            if (flip) {
                retStates[i].turnPosition = retStates[i].turnPosition.rotateBy(new Rotation2d(Math.PI));
                retStates[i].driveVelocity *= -1.0;
                retStates[i].driveForce *= -1.0;
            }
            retStates[i].turnVelocity = -prevSetpoint.cuModuleState[i].turnPosition.minus(
                retStates[i].turnPosition
                ).getRadians() / dt;
            retStates[i].driveAcceleration = (retStates[i].driveVelocity - 
                prevSetpoint.cuModuleState[i].driveVelocity) / dt;

            
        }

        return new swerveSetpoint(desireChassisSpeeds, retStates);
    }   

}
