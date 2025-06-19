package frc.robot.util.swerve;

public class ChassisLimits {
    public double MaxDriveVelocity;
    public double MaxTurnVelocity;
    public double frictionCoeff;
    public double FmaxWithVoltagelimit;
    public double Fstall;
    public double Fbreaking;
    public double robotMass;
    public double robotMIO;
    public double speedDeadband;

    public ChassisLimits(
        double MaxDriveVelocity,
        double MaxTurnVelocity,
        double frictionCoeff,
        double FmaxWithVoltagelimit,
        double Fstall,
        double Fbreaking,
        double robotMass,
        double robotMOI,
        double speedDeadband){
            this.MaxDriveVelocity = MaxDriveVelocity;
            this.MaxTurnVelocity = MaxTurnVelocity;
            this.frictionCoeff = frictionCoeff;
            this.FmaxWithVoltagelimit = FmaxWithVoltagelimit;
            this.Fstall = Fstall;
            this.Fbreaking = Fbreaking;
            this.robotMass = robotMass;
            this.robotMIO = robotMOI;
            this.speedDeadband = speedDeadband;
    }

    // @Override
    // public String toString(){
    //     return "MaxDriveVelocity{"+ MaxDriveVelocity + 
    //     "}, MaxTurnVelocity{" + MaxTurnVelocity +
    //     "}, driveStallAcceleration{" + driveStallAcceleration +
    //     "}, maxAccAcceleration{" + maxAccAcceleration +
    //     "}, maxBreakingAcceleration{" + maxBreakingAcceleration +
    //     "maxAccWithoutSlipping{" + maxAccWithoutSlipping;
    // }
}
