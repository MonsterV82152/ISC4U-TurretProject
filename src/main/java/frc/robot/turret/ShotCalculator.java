package frc.robot.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ShotCalculator {
    private static final double GRAVITY = -9.81;
    private static final double MIN_Y_VELOCITY = 2;
    private static final double SLIP_FACTOR = 1.0; // Adjust this based on empirical testing

    private static final double FLYWHEEL_RADIUS = Units.inchesToMeters(1); // 1 inches in meters

    private static Translation3d target = new Translation3d(0, 2, 5); // Example target position (x, y, z)

    public static void setTarget(Translation3d newTarget) {
        Logger.recordOutput("Target", newTarget);
        target = newTarget;
    }

    public static double calculateFlywheelRPS(double shotVelocity) {
        if (Constants.IS_SIM) {
            return shotVelocity / (2 * Math.PI * FLYWHEEL_RADIUS) / 2;
        }
        return shotVelocity / (2 * Math.PI * FLYWHEEL_RADIUS) / 2 * SLIP_FACTOR;
    }

    public static TurretParameter calculateShot() {
        Translation2d target2D = new Translation2d(Math.hypot(target.getX(), target.getZ()), target.getY());
        Rotation2d yaw = new Rotation2d(target.getX(), target.getZ());

        if (Constants.IS_SIM) {
            double Ty = target2D.getY();
            double Tx = target2D.getX();

            double yApex = Ty - (MIN_Y_VELOCITY * MIN_Y_VELOCITY) / (2.0 * GRAVITY);
            double sqrtTerm = -2.0 * yApex / GRAVITY;
            double vy0 = -Math.sqrt(sqrtTerm) * GRAVITY;

            double discriminant = vy0 * vy0 - 4.0 * (0.5 * GRAVITY) * (-Ty);
            double xt = (-vy0 - Math.sqrt(discriminant)) / GRAVITY;

            double angle = Math.atan(vy0 * xt / Tx);

            return new TurretParameter(new Rotation2d(angle), yaw, calculateFlywheelRPS(Math.hypot(Tx / xt, vy0)));
        }

        // Placeholder Math

        return new TurretParameter();
    }
}

class TurretParameter {
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