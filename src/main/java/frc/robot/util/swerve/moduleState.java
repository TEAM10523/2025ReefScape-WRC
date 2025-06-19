package frc.robot.util.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class moduleState {

    public double driveVelocity = 0;
    public double drivePosition = 0;
    public double driveAcceleration = 0;
    public double driveForce = 0;
    public double turnVelocity = 0;
    public Rotation2d turnPosition = new Rotation2d(0);
    
    public moduleState(double driveVelocity, double drivePosition, double driveAcceleration, double driveForce,
        double turnVelocity, Rotation2d turnPosition){

        this.driveVelocity = driveVelocity;
        this.drivePosition = drivePosition;
        this.driveAcceleration = driveAcceleration;
        this.driveForce = driveForce;

        this.turnVelocity = turnVelocity;
        this.turnPosition = turnPosition;
    }

    public moduleState(double driveVelocity, double driveAcceleration,
        double turnVelocity, Rotation2d turnPosition){

        this.driveVelocity = driveVelocity;
        this.driveAcceleration = driveAcceleration;

        this.turnVelocity = turnVelocity;
        this.turnPosition = turnPosition;
    }

    public moduleState(double driveVelocity,
        double turnVelocity, Rotation2d turnPosition){

        this.driveVelocity = driveVelocity;

        this.turnVelocity = turnVelocity;
        this.turnPosition = turnPosition;
    }

    public moduleState(double driveVelocity,
        Rotation2d turnPosition){
        this.driveVelocity = driveVelocity;

        this.turnPosition = turnPosition;
    }

    public moduleState(SwerveModuleState wpiModuleState){
        this.driveVelocity = wpiModuleState.speedMetersPerSecond;
        this.turnPosition = wpiModuleState.angle;
    }

    @Override
    public String toString() {
        return String.format(
                "SwerveModuleState(Speed: %.2f m/s, Angle: %s)", driveVelocity, turnPosition.getDegrees());
    }

    public void updateState(moduleState measuredState, double measurementWeight, double estimateWeight){
        double totalWeight = measurementWeight + estimateWeight;
        this.driveVelocity =
            (this.driveVelocity * estimateWeight + measuredState.driveVelocity * measurementWeight) / totalWeight;
        this.drivePosition = 
            (this.drivePosition * estimateWeight + measuredState.drivePosition * measurementWeight) / totalWeight;
        this.driveAcceleration = 
            (this.driveAcceleration * estimateWeight + measuredState.driveAcceleration * measurementWeight) / totalWeight;
        this.turnVelocity = 
            (this.turnVelocity * estimateWeight + measuredState.turnVelocity * measurementWeight) / totalWeight;
        this.turnPosition = new Rotation2d(
            (this.turnPosition.getRadians() * estimateWeight + measuredState.turnPosition.getRadians() * measurementWeight) / totalWeight);
    }
    public double[] getVector(){
        return new double[]{
            driveVelocity * turnPosition.getCos(),
            driveVelocity * turnPosition.getSin(),
        };
    }

    public SwerveModuleState getWPISwerveModuleState(){
        return new SwerveModuleState(driveVelocity, turnPosition);
    }
    public SwerveModulePosition getWPISwerveModulePosition(){
        return new SwerveModulePosition(drivePosition, turnPosition);
    }

}
