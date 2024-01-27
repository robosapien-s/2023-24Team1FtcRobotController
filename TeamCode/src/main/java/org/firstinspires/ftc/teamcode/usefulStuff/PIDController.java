package org.firstinspires.ftc.teamcode.usefulStuff;


import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double Kp; // Proportional gain
    private double Ki; // Integral gain
    private double Kd; // Derivative gain
    //private double maxIntegralSum;
    //private double a;
    private double prevError;
    private double integral;

    private double errorChange;
    private double currentFilterEstimate;
    private double prevFilterEstimate;

    public PIDController(double Kp, double Ki, double Kd, double maxIntegralSum, double a) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        //this.maxIntegralSum = maxIntegralSum;
        //this.a = a;
        this.prevError = 0.0;
        this.integral = 0.0;
    }

    public double calculate(double setpoint, double current, ElapsedTime dt) {
        double error = setpoint - current;
        integral += error*dt.seconds();
        errorChange = error - prevError;
        //currentFilterEstimate = (a * prevFilterEstimate) + (1-a) * errorChange;
        prevFilterEstimate = currentFilterEstimate;

        double derivative = currentFilterEstimate / dt.seconds();

        // Limit the integral term to prevent windup
       /* if (integral > maxIntegralSum) {
            integral = maxIntegralSum;
        } else if (integral < -maxIntegralSum) {
            integral = -maxIntegralSum;
        }*/

        double output = (Kp * error) + (Ki * integral) + (Kd * derivative);

        prevError = error;

        return output;
    }

    public double calculatewitherror(double error, ElapsedTime dt) {
        integral += error*dt.seconds();
        errorChange = error - prevError;
        //currentFilterEstimate = (a * prevFilterEstimate) + (1-a) * errorChange;
        prevFilterEstimate = currentFilterEstimate;

        double derivative = currentFilterEstimate / dt.seconds();

        // Limit the integral term to prevent windup
        /*if (integral > maxIntegralSum) {
            integral = maxIntegralSum;
        } else if (integral < -maxIntegralSum) {
            integral = -maxIntegralSum;
        }*/

        double output = (Kp * error) + (Ki * integral) + (Kd * derivative);

        prevError = error;

        return output;
    }
}