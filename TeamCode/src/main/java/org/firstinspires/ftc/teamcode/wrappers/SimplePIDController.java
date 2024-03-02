package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.robotcore.util.ElapsedTime;

public class SimplePIDController {


    public SimplePIDController(double inKp, double inKi, double inKd) {
        Kp = inKp;
        Ki = inKi;
        Kd = inKd;
    }
    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private double intergralSum = 0;

    private double Kp = 0.04;
    private double Ki = 0.000008;
    private double Kd = 0.0004;
    private double targetPosition = 0;
    private double currentPosition = 0;

    private  boolean enabled = false;

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
        enabled = true;
    }

    public void setCurrentPosition(double inCurrentPosition) {
        this.currentPosition = inCurrentPosition;
    }

    public double getCurrentPosition() {
        return this.currentPosition;
    }

    public void clearTarget() {
        enabled = false;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public double getLastError() {
        return lastError;
    }

    public double getTargetPosition() {
        return  targetPosition;
    }

    public double getPower(double state) {

        double error = targetPosition - state;
        double seconds = timer.seconds();
        intergralSum += error * seconds;

        double errorDiff = error - lastError;
        double derivative = errorDiff / timer.seconds();

        derivative = (error - lastError) / seconds;

        lastError = error;
        timer.reset();
        double output = (error*Kp) + (derivative*Kd) + (intergralSum*Ki);

        if(output>1) {
            output = 1;
        }
        return  output;
    }

    public double getPowerWithStates(double inKp, double inKi, double inKd) {

        double error = targetPosition - currentPosition;
        double seconds = timer.seconds();
        intergralSum += error * seconds;

        double errorDiff = error - lastError;
        double derivative = errorDiff / timer.seconds();

        derivative = (error - lastError) / seconds;

        lastError = error;
        timer.reset();
        double output = (error*inKp) + (derivative*inKd) + (intergralSum*inKi);

        if(output>1) {
            output = 1;
        }
        return  output;
    }

}
