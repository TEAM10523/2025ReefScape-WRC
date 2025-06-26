package frc.robot.util.BlackholePlanner;

public class Setpoint2d {
  public double x0;
  public double x1;
  public double v0;
  public double v1;
  public double a0;
  public double a1;

  public Setpoint2d(double x0, double x1, double v0, double v1, double a0, double a1) {
    this.x0 = x0;
    this.x1 = x1;
    this.v0 = v0;
    this.v1 = v1;
    this.a0 = a0;
    this.a1 = a1;
  }
}
