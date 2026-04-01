package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.turret.Turret;

public class RobotContainer {
    // Subsystems
    private final Turret turret;

    // Controller
    private final XboxController controller = new XboxController(0);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        turret = Turret.getInstance();
    }

    public void teleopPeriodic() {
        if (controller.getAButton()) {
            turret.shootBall();
        }
    }

    public void autonomousInit() {
    }

    public void teleopInit() {
    }

    public void periodic() {
        turret.periodic();
    }
    // Zissis was here (lead prog)
}