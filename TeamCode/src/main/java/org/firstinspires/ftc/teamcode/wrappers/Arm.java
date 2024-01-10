package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    boolean open;

    Servo rearLiftServo;

    Servo rearWristVerticalServo;
    Servo rearClawServo;

    Servo frontLiftServo;
    Servo frontWristVerticalServo;
    Servo frontWristHorizontalServo;
    Servo frontClawServo;

    JoystickWrapper joystickWrapper;

    public Arm(Telemetry inTelemetry, HardwareMap inHardware, JoystickWrapper wrapper){
        this.telemetry = inTelemetry;
        hardwareMap = inHardware;
        rearLiftServo = hardwareMap.get(Servo.class, "rearLiftServo");
        rearWristVerticalServo = hardwareMap.get(Servo.class, "rearWristVerticalServo");
        rearClawServo = hardwareMap.get(Servo.class, "rearClaw");
        joystickWrapper = wrapper;

        telemetry.addData("in2","");
    }
    public void update(Gamepad gamepad){
        telemetry.addData("in","");
        if (joystickWrapper.gamepad2GetA()){
            frontWristVerticalServo.setPosition(.8);
            frontWristHorizontalServo.setPosition(.5);
            telemetry.addData("A","");

        }
        if (joystickWrapper.gamepad2GetB()) {
            frontWristVerticalServo.setPosition(.8);
            frontWristHorizontalServo.setPosition(.5);
        }
        if (joystickWrapper.gamepad2GetX()) {

        }
        if (joystickWrapper.gamepad2GetY()) {

        }
        if (joystickWrapper.gamepad2GetDUp()){
            openClaw(true);
            //vertical wrist = .7, lift = .6 or .5 depending on height of the stack
            rearWristVerticalServo.setPosition(.7);
            rearLiftServo.setPosition(.5);
            //PICKUP HIGH
        }
        if(joystickWrapper.gamepad2GetDRight()){
            openClaw(true);
            rearLiftServo.setPosition(.7);
            rearWristVerticalServo.setPosition(.5);
            //vertical wrist =.5, lift = .7
        }
        if (joystickWrapper.gamepad2GetDLeft()){
            openClaw(true);
            rearLiftServo.setPosition(.7);
            rearWristVerticalServo.setPosition(.5);
            //vertical wrist =.5, lift = .7
        }
        if (joystickWrapper.gamepad2GetDDown()){
            openClaw(true);
            rearWristVerticalServo.setPosition(.4);
            rearLiftServo.setPosition(1);
            //verticalwrist = .4, lift = 1
        }

        if(joystickWrapper.gamepad2GetLeftBumperDown()){
            if(open){
                rearClawServo.setPosition(.3);
                open=false;
            }else {
                rearClawServo.setPosition(0);
                open=true;
            }
        }
    }
    public void openClaw(boolean _open){
        if(!_open){
            rearClawServo.setPosition(.3);
            open=false;
        }else {
            rearClawServo.setPosition(0);
            open=true;
        }
    }


}
