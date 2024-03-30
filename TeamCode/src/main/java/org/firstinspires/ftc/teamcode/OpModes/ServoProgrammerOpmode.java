package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.ui.GamepadUser;
import org.firstinspires.ftc.teamcode.controllers.IRobotTask;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskSeries;
import org.firstinspires.ftc.teamcode.controllers.ServoTask;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;
import org.firstinspires.ftc.teamcode.wrappers.NeoArmWrapper;

import java.util.ArrayList;

@TeleOp
public class ServoProgrammerOpmode extends LinearOpMode {
    ServoProgrammer wrapper;

    JoystickWrapper joystickWrapper;

    @Override
    public void runOpMode() throws InterruptedException {
        wrapper = new ServoProgrammer();
        joystickWrapper=new JoystickWrapper(gamepad1,gamepad2);
        waitForStart();

        wrapper.AddServos(hardwareMap.get(Servo.class,"armServo0"),"armServo0");
        wrapper.AddServos(hardwareMap.get(Servo.class,"armServo1"),"armServo1");
        wrapper.AddServos(hardwareMap.get(Servo.class,"wristServo"),"wristServo");


        wrapper.AddServos(hardwareMap.get(Servo.class,"armWrist"),"armWrist");
        wrapper.AddServos(hardwareMap.get(Servo.class,"armChain"),"armChain");
        wrapper.AddServos(hardwareMap.get(Servo.class,"armLeftRight"),"armLeftRight");
        wrapper.AddServos(hardwareMap.get(Servo.class,"armPixelRot"),"armPixelRot");
        

        while(!isStopRequested()){
            wrapper.Update(joystickWrapper,telemetry);

        }

    }
}
