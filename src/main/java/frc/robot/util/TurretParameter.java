package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class TurretParameter {
    public Rotation2d hoodRad = new Rotation2d();
    public Rotation2d swivelRad = new Rotation2d();
    public double shotRPS = 0.0;

    public TurretParameter() {
    }

    public TurretParameter(Rotation2d hoodRad, Rotation2d swivelRad, double shotRPS) {
        this.hoodRad = hoodRad;
        this.swivelRad = swivelRad;
        this.shotRPS = shotRPS;
    }
}