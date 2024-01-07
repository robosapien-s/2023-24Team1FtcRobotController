package org.firstinspires.ftc.teamcode.wrappers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PIDController;

public class ArmWrapper {
    ElapsedTime dt;

    int actualPosition;
    HardwareMap hardwareMap;
    Telemetry telemetry;

    Servo rearLiftServo;

    Servo rearWristVerticalServo;
    Servo rearClawServo;

    Servo frontLiftServo;
    Servo frontWristVerticalServo;
    Servo frontWristHorizontalServo;
    Servo frontClawServo;

    DcMotorEx slideMotor;
    public int slidePos = 0;
    int Ratio = 28;
    final int slideEncoderFactor = 10;

    boolean limit = true;
    double MotorTicks = ((((1+(46/17))) * (1+(46/11))) * 28);



    public ArmWrapper(HardwareMap inHardwareMap, Telemetry inTelemetry) {
        hardwareMap = inHardwareMap;
        telemetry = inTelemetry;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slideMotor  = hardwareMap.get(DcMotorEx.class, "slideMotor");
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setTargetPosition(0);
    }

    public void PPArmMove(JoystickWrapper joystickWrapper) {


        slidePos = slideMotor.getTargetPosition() + (int)((joystickWrapper.gamepad1GetRightTrigger()-joystickWrapper.gamepad1GetLeftTrigger())*slideEncoderFactor);
        if (joystickWrapper.gamepad1GetRightBumperDown()){
            if(limit){
                limit=false;
            }else {
                limit=true;
            }
        }

       /* if (joystickWrapper.gamepad2GetDDown()) {
            clawBase.setPower(-.5);
            servoPos = clawBase.getPosition() - .01;
        }
        if(joystickWrapper.gamepad2GetDUp()) {
            servoPos = clawBase.getPosition();

        }*/

/*
        if (joystickWrapper.gamepad1GetA()) {
            slidePos = 70;
        }else if (joystickWrapper.gamepad1GetX()) {
            slidePos = 220;
        }
        else if (joystickWrapper.gamepad1GetY()) {
            slidePos = 390;
        }else if (joystickWrapper.gamepad1GetB()) {
            slidePos = 550;
        }
        if (joystickWrapper.gamepad1GetDDown()) {
            slidePos = 5;
        }else if (joystickWrapper.gamepad1GetDLeft()) {
            slidePos = 1670;
        }
        else if (joystickWrapper.gamepad1GetDUp()) {
            slidePos = 2900;
        }else if (joystickWrapper.gamepad1GetDRight()) {
            slidePos = 4000;
        }
*/

        if (slidePos<5 && limit) {
            slidePos = 10;
        }
        if (slidePos>3000 && limit) {
            slidePos = 3000;
        }




        slideMotor.setPower(1);
        slideMotor.setTargetPosition(slidePos);

        slideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        telemetry.addData("CurrentPosition:slide", slideMotor.getCurrentPosition());
        // telemetry.addData("CurrentPosition:servo", clawServo.getPosition());
        telemetry.addData("TargetPosition", slideMotor.getTargetPosition());
        // telemetry.addData("ClawBase", clawBase.getPower());
        telemetry.addData("Limit?", limit);
        telemetry.update();
        dt.reset();
        //topMotor.setPower(-joystickWrapper.gamepad2GetLeftStickY());


        /*if(joystickWrapper.gamepad1GetLeftBumperDown()) {
            if (open) {
                clawServo.setPosition(.5);
                open = false;
            } else {
                clawServo.setPosition(.3);
                open = true;
            }
        }*/
    }

    /*public double GetCurrentRotation(){
        double n = (360*(bottomMotor.getCurrentPosition()/(MotorTicks*Ratio)))%360;
        if(n>180){
            return n-360;
        }else return n;
    }*/

    /*public void RotateArm(double angle){
        if(0>angle - GetCurrentRotation()){
            bottomMotor.setPower(-1 * 0.25);
            telemetry.addData("Input", "Negative");
        }else if (0<angle - GetCurrentRotation()){
            bottomMotor.setPower(1 * 0.25);
            telemetry.addData("Input", "Positive");
        }
        telemetry.update();
    }*/

}





/*
init 192.168.43.1:5555 at ip ADB
push REABD library -- =: 17
POP REABD library -- =: 17(grade.strip().split(1::2))
 */


