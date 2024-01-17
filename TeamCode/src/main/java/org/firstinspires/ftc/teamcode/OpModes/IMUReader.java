package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp
public class IMUReader extends LinearOpMode {

    IMU imu;

    double pitch;
    double roll;
    double yaw;

    double PGain = .03;

    double targetHeading;

    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;


    @Override
    public void runOpMode() throws InterruptedException {
        //0
        frontLeftMotor =  hardwareMap.dcMotor.get("fL");
        //1
        frontRightMotor = hardwareMap.dcMotor.get("fR");
        //2
        backLeftMotor = hardwareMap.dcMotor.get("bL");
        //3
        backRightMotor = hardwareMap.dcMotor.get("bR");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        waitForStart();
        while (!isStopRequested()) {
 
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

            pitch = orientation.getPitch(AngleUnit.DEGREES);
            roll = orientation.getRoll(AngleUnit.DEGREES);
            yaw = orientation.getYaw(AngleUnit.DEGREES);

            if (length(gamepad1.left_stick_x, gamepad1.left_stick_y) > .5) {
                targetHeading = Math.toDegrees(Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x));
            }  // Save for telemetry

            // Determine the heading current error
            double headingError = targetHeading - yaw;

            // Normalize the error to be within +/- 180 degrees
            while (headingError > 180) headingError -= 360;
            while (headingError <= -180) headingError += 360;

            telemetry.addData("Joystick Left Angle", targetHeading);
            telemetry.addData("Joystick Right Angle", Math.toDegrees(Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x)));
            telemetry.addData("Pitch", pitch);
            telemetry.addData("Roll", roll);
            telemetry.addData("Yaw", yaw);
            telemetry.addData("Error", headingError);
            telemetry.addData("Range",Range.clip(headingError * PGain, -1, 1));

            MoveMecanum( 0,0,Range.clip(headingError * PGain, -1, 1));

            telemetry.update();



        }

    }

    double length(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }

    void MoveMecanum(double x,double y,double rx){
        frontLeftMotor.setPower(y + x + rx);
        backLeftMotor.setPower(y - x + rx);
        frontRightMotor.setPower(y - x - rx);
        backRightMotor.setPower(y + x - rx);
    }
}
