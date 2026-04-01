package frc.robot.util;

import frc.robot.Constants;

public class VelocityPIDController {
    private double kP;
    private double kI;
    private double kD;
    private double kS;
    private double kV;

    private double setpoint;
    private double previousError;
    private double integral;

    public VelocityPIDController(double kP, double kI, double kD, double kS, double kV) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kS = kS;
        this.kV = kV;
    }

    public void setGains(double kP, double kI, double kD, double kS, double kV) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kS = kS;
        this.kV = kV;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public double calculate(double measurement) {
        double error = setpoint - measurement;
        integral += error * Constants.LOOP_PERIOD;
        double derivative = (error - previousError) / Constants.LOOP_PERIOD;
        previousError = error;
        return kS * Math.signum(setpoint) + kV * setpoint + kP * error + kI * integral + kD * derivative;
    }

}
