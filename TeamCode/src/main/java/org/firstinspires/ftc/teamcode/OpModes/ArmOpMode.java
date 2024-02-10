package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

@TeleOp
public class ArmOpMode extends LinearOpMode {

    JoystickWrapper joystick;
    Servo leftClaw;
    Servo rightClaw;
    Servo clawVerticalWrist;
    boolean open;
    boolean open2;

    DcMotor intakeMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        joystick = new JoystickWrapper(gamepad1,gamepad2);

        leftClaw = hardwareMap.servo.get("leftClaw");
        rightClaw = hardwareMap.servo.get("rightClaw");
        clawVerticalWrist = hardwareMap.servo.get("clawVerticalWrist");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");


        waitForStart();
        while (!isStopRequested()) {
            /*if(joystick.gamepad1GetA()){
                if(open) {
                    leftClaw.setPosition(.5);
                    rightClaw.setPosition(.5);
                    open = false;
                }else {
                    leftClaw.setPosition(0);
                    rightClaw.setPosition(0);
                    open = true;
                }
            }
            if(joystick.gamepad1GetDDown()){
                clawVerticalWrist.setPosition(clawVerticalWrist.getPosition()-.1);
            }
            if(joystick.gamepad1GetDUp()){
                clawVerticalWrist.setPosition(clawVerticalWrist.getPosition()+.1);
            }*/


            if (joystick.gamepad1GetDDown()) {
                if (open) {
                    rightClaw.setPosition(.9);
                    leftClaw.setPosition(.35);
                    open = false;
                } else {
                    rightClaw.setPosition(.6);
                    leftClaw.setPosition(.7);
                    open = true;
                }
            }
            if (joystick.gamepad1GetDUp()) {
                if (open2) {
                    clawVerticalWrist.setPosition(0);
                    open2 = false;
                } else {
                    clawVerticalWrist.setPosition(.3);
                    open2 = true;
                }
            }

            telemetry.addData("Right Claw",rightClaw.getPosition());
            telemetry.addData("Left Claw",leftClaw.getPosition());
            telemetry.addData("Wrist",clawVerticalWrist.getPosition());
            telemetry.update();

            intakeMotor.setPower(gamepad1.right_trigger-gamepad1.left_trigger);


        }
        //wrist down = 0, wrist up = .3,
        //Right claw closed = .9, right claw open = .6
        //Left claw closed = .7, left claw open = .35
    }
}
