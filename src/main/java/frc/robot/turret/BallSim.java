package frc.robot.turret;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.Constants.BallSimConstants;

public class BallSim {
    private static BallSim instance;
    private final List<SimulatedBall> balls = new ArrayList<>();
    private FlywheelSim flywheelSim;

    private BallSim() {
    }

    public static BallSim getInstance() {
        if (instance == null) {
            instance = new BallSim();
        }
        return instance;
    }

    /**
     * Sets the flywheel simulation reference so ball shots can affect flywheel velocity
     */
    public void setFlywheelSim(FlywheelSim flywheelSim) {
        this.flywheelSim = flywheelSim;
    }

    /**
     * Shoots a ball from the turret with the given parameters
     * 
     * @param hoodAngle    The hood angle (pitch) in radians
     * @param swivelAngle  The swivel angle (yaw) in radians
     * @param flywheelRPS  The flywheel rotations per second
     */
    public void shootBall(Rotation2d hoodAngle, Rotation2d swivelAngle, double flywheelRPS) {
        if (balls.size() >= BallSimConstants.MAX_BALLS) {
            // Remove the oldest ball if at capacity
            balls.remove(0);
        }

        // Calculate initial velocity from flywheel RPS
        double shotVelocity = flywheelRPS * Math.PI * Constants.TurretConstants.FLYWHEEL_RADIUS_METERS;

        // Calculate velocity components based on hood and swivel angles
        // Coordinate system: X = Left(+)/Right(-), Y = Forward(+)/Backward(-), Z = Up(+)/Down(-)
        double vHorizontal = shotVelocity * hoodAngle.getCos(); // Horizontal component based on hood angle
        double vVertical = shotVelocity * hoodAngle.getSin();   // Vertical component based on hood angle

        double vx = vHorizontal * swivelAngle.getSin();  // Left/Right
        double vy = vHorizontal * swivelAngle.getCos();  // Forward/Backward
        double vz = vVertical;                            // Up/Down

        // Spawn position at turret location
        Translation3d spawnPosition = new Translation3d(
                BallSimConstants.TURRET_FORWARD_OFFSET_METERS * swivelAngle.getSin(),  // X: Left/Right
                BallSimConstants.TURRET_FORWARD_OFFSET_METERS * swivelAngle.getCos(),  // Y: Forward
                BallSimConstants.TURRET_HEIGHT_METERS                                                 // Z: Height
        );

        Translation3d initialVelocity = new Translation3d(vx, vy, vz);

        balls.add(new SimulatedBall(spawnPosition, initialVelocity));

        // Apply reaction force to flywheel (conservation of momentum)
        if (flywheelSim != null) {
            applyFlywheelReaction(shotVelocity);
        }
    }

    /**
     * Applies the reaction force from shooting a ball to the flywheel
     */
    private void applyFlywheelReaction(double shotVelocity) {
        // Calculate momentum transfer: p = mv
        double ballMomentum = BallSimConstants.BALL_MASS_KG * shotVelocity;

        // Convert to angular momentum loss on flywheel
        // L = p * r, where r is the flywheel radius
        double angularMomentumLoss = ballMomentum * Constants.TurretConstants.FLYWHEEL_RADIUS_METERS;

        // Calculate velocity loss: ΔL = I * Δω
        double flywheelInertia = Constants.TurretConstants.FLYWHEEL_MOMENT_OF_INERTIA;
        double angularVelocityLossRadPerSec = angularMomentumLoss / flywheelInertia;

        // Convert to RPM loss and apply (FlywheelSim uses RPM internally)
        double rpmLoss = (angularVelocityLossRadPerSec / (2 * Math.PI)) * 60;
        double currentRPM = flywheelSim.getAngularVelocityRPM();
        double newRPM = Math.max(0, currentRPM - rpmLoss);
        flywheelSim.setAngularVelocity(Units.rotationsPerMinuteToRadiansPerSecond(newRPM));

        Logger.recordOutput("BallSim/FlywheelRPMLoss", rpmLoss);
        Logger.recordOutput("BallSim/ExpectedNewRPM", newRPM);
    }

    /**
     * Updates all ball simulations - call this every robot periodic
     */
    public void periodic() {
        double dt = Constants.LOOP_PERIOD;

        Iterator<SimulatedBall> iterator = balls.iterator();
        while (iterator.hasNext()) {
            SimulatedBall ball = iterator.next();
            ball.update(dt);

            // Remove balls that have expired or fallen below ground (Z < 0)
            if (ball.isExpired() || ball.getPosition().getZ() < -1.0) {
                iterator.remove();
            }
        }

        // Log ball positions to AdvantageScope
        logBalls();
    }

    /**
     * Logs all ball positions to AdvantageScope for visualization
     */
    private void logBalls() {
        Translation3d[] ballPoses = new Translation3d[balls.size()];
        for (int i = 0; i < balls.size(); i++) {
            SimulatedBall ball = balls.get(i);
            ballPoses[i] = ball.getPosition();
        }

        Logger.recordOutput("BallSim/Balls", ballPoses);
        Logger.recordOutput("BallSim/BallCount", balls.size());
    }

    /**
     * Gets the number of active balls in simulation
     */
    public int getBallCount() {
        return balls.size();
    }

    /**
     * Clears all balls from simulation
     */
    public void clearBalls() {
        balls.clear();
    }
}

/**
 * Represents a single simulated ball with position and velocity
 */
class SimulatedBall {
    private Translation3d position;
    private Translation3d velocity;
    private double lifetime;

    public SimulatedBall(Translation3d initialPosition, Translation3d initialVelocity) {
        this.position = initialPosition;
        this.velocity = initialVelocity;
        this.lifetime = 0.0;
    }

    /**
     * Updates the ball position using basic projectile motion (no air resistance)
     * Coordinate system: X = Left(+)/Right(-), Y = Forward(+)/Backward(-), Z = Up(+)/Down(-)
     * 
     * @param dt Time step in seconds
     */
    public void update(double dt) {
        // Update position: p = p0 + v*dt
        position = new Translation3d(
                position.getX() + velocity.getX() * dt,
                position.getY() + velocity.getY() * dt,
                position.getZ() + velocity.getZ() * dt);

        // Update velocity (only gravity affects Z component - up/down)
        velocity = new Translation3d(
                velocity.getX(),
                velocity.getY(),
                velocity.getZ() + BallSimConstants.GRAVITY * dt);

        lifetime += dt;
    }

    public Translation3d getPosition() {
        return position;
    }

    public Translation3d getVelocity() {
        return velocity;
    }

    public boolean isExpired() {
        return lifetime > BallSimConstants.BALL_LIFETIME_SECONDS;
    }

    public double getLifetime() {
        return lifetime;
    }
}
