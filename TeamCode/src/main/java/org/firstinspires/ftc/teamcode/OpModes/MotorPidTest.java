package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
@Disabled
public class MotorPidTest extends LinearOpMode {

    DcMotorEx motor;
    DcMotorEx motor2;
    ElapsedTime timer = new ElapsedTime();

    private double lastError = 0;
    private double intergralSum = 0;

    public static double Kp = 0.0;
    public static double Ki = 0.0;
    public static double Kd = 0.0;

    public static double targetPosition = 5000;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {

        TelemetryPacket packet  = new TelemetryPacket();


        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {


            double power = returnPower(targetPosition, motor.getCurrentPosition());

            packet.put("power", power);
            packet.put("position", motor.getCurrentPosition());
            packet.put("error", lastError);

            motor.setPower(power);
            motor2.setPower(power);

            dashboard.sendTelemetryPacket(packet);


        }


    }

    public double returnPower(double reference, double state) {
        double error = reference - state;
        intergralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        timer.reset();
        double output = (error*Kp) + (derivative*Kd) + (intergralSum*Ki);
        return  output;
    }

}
