package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class NeoArmWrapper {

    Telemetry telemetry;
    HardwareMap hardwareMap;
    Gamepad gamepad1;
    Gamepad gamepad2;

    public DcMotorEx IntakeMotorEx;
    public DcMotorEx ActuatorMotorEx;
    public DcMotorEx ExtensionMotorEx1;
    public DcMotorEx ExtensionMotorEx2;

    float maxExtensionLength;
    float minExtensionLength;

    public Servo armServo0;
    public Servo armServo1;
    public Servo wristServo;
    public CRServo armWheel;

    public NeoArmWrapper(Telemetry inTelemetry, HardwareMap inHardwareMap, Gamepad inGamepad1, Gamepad inGamepad2){
        //Set Values
        telemetry = inTelemetry;
        hardwareMap = inHardwareMap;
        gamepad1 = inGamepad1;
        gamepad2 = inGamepad2;
        ExtensionMotorEx1 = hardwareMap.get(DcMotorEx.class,"ExtensionMotorEx0");
        ExtensionMotorEx2 = hardwareMap.get(DcMotorEx.class, "ExtensionMotorEx1");
        ActuatorMotorEx = hardwareMap.get(DcMotorEx.class,"ActuatorMotor");
        IntakeMotorEx = hardwareMap.get(DcMotorEx.class, "IntakeMotor");

        armServo0 = hardwareMap.get(Servo.class, "armServo0");
        armServo1 = hardwareMap.get(Servo.class, "armServo1");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        armWheel = hardwareMap.get(CRServo.class, "armWheel");

    }

    public enum ePosition{
        BOTTOM,
        LOW,
        MEDIUM,
        HIGH
    }

    public void UpdateSlidePosition(ePosition position){
        switch(position){
            case BOTTOM:

            case LOW:

            case MEDIUM:

            case HIGH:

        }
    }
    public void UpdateActuatorPosition(ePosition position){
        switch(position){
            case BOTTOM:

            case LOW:

            case MEDIUM:

            case HIGH:

        }
    }
    public void UpdateIntakePower(float power){
        IntakeMotorEx.setPower(power);
    }



    public void MoveMotorWithTelemetry(int Move){
        telemetry.addData("current",ActuatorMotorEx.getCurrent(CurrentUnit.AMPS));
        ActuatorMotorEx.setTargetPosition(ActuatorMotorEx.getCurrentPosition()+Move);
        telemetry.update();
    }

    public void OpenPos(){
        armServo0.setPosition(.65);
        armServo1.setPosition(.7);
    }
    public void ClosePos(){
        armServo0.setPosition(.95);
        armServo1.setPosition(.35);
    }
    public void WristDown(){
        wristServo.setPosition(0);
    }
    public void WristUp(){
        wristServo.setPosition(.25);
    }

    public void ManualExtention(JoystickWrapper joystickWrapper, int slideEncoderFactor){

        boolean limit = true;
        int slidePos=ExtensionMotorEx1.getCurrentPosition() + (int)(-joystickWrapper.gamepad2GetRightStickY()*slideEncoderFactor);

       // if (joystickWrapper.gamepad2GetRightBumperDown()){
        //    limit = !limit;
       // }

        if (slidePos<5 && limit) {
            slidePos = 10;
        }
        if (slidePos>3000 && limit) {
            slidePos = 3000;
        }
        //ExtensionMotorEx1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //ExtensionMotorEx2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        ExtensionMotorEx1.setTargetPosition(slidePos);
        ExtensionMotorEx2.setTargetPosition(slidePos);
        ExtensionMotorEx1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ExtensionMotorEx2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ExtensionMotorEx1.setPower(joystickWrapper.gamepad2GetRightStickY());
        ExtensionMotorEx2.setPower(joystickWrapper.gamepad2GetRightStickY());
        telemetry.addData("Slide Pos:", slidePos);
    }



    public void setOuttake() {

        ActuatorMotorEx.setTargetPosition(1000);
        ExtensionMotorEx1.setTargetPosition(2000);
        ExtensionMotorEx2.setTargetPosition(2000);

        ActuatorMotorEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ExtensionMotorEx1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ExtensionMotorEx2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ActuatorMotorEx.setPower(1);
        ExtensionMotorEx1.setPower(1);
        ExtensionMotorEx2.setPower(1);

    }


    public void setIntake() {

        ActuatorMotorEx.setTargetPosition(0);
        ExtensionMotorEx1.setTargetPosition(0);
        ExtensionMotorEx2.setTargetPosition(0);

        ActuatorMotorEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ExtensionMotorEx1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ExtensionMotorEx2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ActuatorMotorEx.setPower(1);
        ExtensionMotorEx1.setPower(.5);
        ExtensionMotorEx2.setPower(.5);

    }

    public void MoveExtensionMotors(int position) {
        //ExtensionMotorEx1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //ExtensionMotorEx2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        ActuatorMotorEx.setTargetPosition(position);
        //ExtensionMotorEx1.setTargetPosition(position);
        //ExtensionMotorEx2.setTargetPosition(position);

        ActuatorMotorEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //ExtensionMotorEx1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //ExtensionMotorEx2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ActuatorMotorEx.setPower(.5);
        //ExtensionMotorEx1.setPower(.1);
        //ExtensionMotorEx2.setPower(.1);
    }

    public void MoveActuatorMotor(int pos){
        ActuatorMotorEx.setPower(1);
        ActuatorMotorEx.setTargetPosition(pos);
        ActuatorMotorEx.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void ResetMotorPositions(){
        ActuatorMotorEx.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        ExtensionMotorEx1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        ExtensionMotorEx2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        ActuatorMotorEx.setDirection(DcMotorEx.Direction.FORWARD);
        ExtensionMotorEx1.setDirection(DcMotorEx.Direction.FORWARD);
        ExtensionMotorEx2.setDirection(DcMotorEx.Direction.REVERSE);

        ActuatorMotorEx.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ExtensionMotorEx1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ExtensionMotorEx2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //ActuatorMotorEx.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //ExtensionMotorEx1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //ExtensionMotorEx2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        /*ActuatorMotorEx.setTargetPosition(0);
        ExtensionMotorEx1.setTargetPosition(0);
        ExtensionMotorEx2.setTargetPosition(0);*/
    }
    public void SetWheelSpin(double Power){
        armWheel.setPower(Power);
    }


}
