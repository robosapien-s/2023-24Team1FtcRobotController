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

        wrapper.AddServos(hardwareMap.get(Servo.class,"armServo"),"armServo");
        wrapper.AddServos(hardwareMap.get(Servo.class,"wristServo"),"wristServo");
        

        while(!isStopRequested()){
            wrapper.Update(joystickWrapper,telemetry);

        }

    }
}
