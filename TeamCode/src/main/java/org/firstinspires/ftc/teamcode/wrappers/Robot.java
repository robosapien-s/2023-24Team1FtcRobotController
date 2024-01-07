package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    double speed = 1;
    double rotSpeed = 1;

    JoystickWrapper joystickWrapper;
    DrivingWrapper drivingWrapper;
    ArmWrapper armWrapper;

    public Robot(HardwareMap inHardwareMap, Telemetry inTelemetry) {
        hardwareMap = inHardwareMap;
        telemetry = inTelemetry;
    }

    public void doStuff() {
        drivingWrapper.Drive(joystickWrapper, speed, rotSpeed);
        armWrapper.PPArmMove(joystickWrapper);




        /**
         * Add inputs below as needed
         * For example:
         * public void gamepad1a() {
         *      servo.setPower(.5);
         * }
         */
    }

}
