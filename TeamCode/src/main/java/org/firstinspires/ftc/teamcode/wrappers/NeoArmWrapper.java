package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    DcMotorEx IntakeMotorEx;
    DcMotorEx ActuatorMotorEx;
    DcMotorEx ExtensionMotorEx1;
    DcMotorEx ExtensionMotorEx2;

    float maxExtensionLength;
    float minExtensionLength;

    public Servo armServo0;
    public Servo armServo1;
    public Servo wristServo;

    public NeoArmWrapper(Telemetry inTelemetry, HardwareMap inHardwareMap, Gamepad inGamepad1, Gamepad inGamepad2){
        //Set Values
        telemetry = inTelemetry;
        hardwareMap = inHardwareMap;
        gamepad1 = inGamepad1;
        gamepad2 = inGamepad2;
        ExtensionMotorEx1 = hardwareMap.get(DcMotorEx.class,"");
        ExtensionMotorEx2 = hardwareMap.get(DcMotorEx.class, " ");
        ActuatorMotorEx = hardwareMap.get(DcMotorEx.class,"ActuatorMotor");
        IntakeMotorEx = hardwareMap.get(DcMotorEx.class, "IntakeMotor");

        armServo0 = hardwareMap.get(Servo.class, "armServo0");
        armServo1 = hardwareMap.get(Servo.class, "armServo1");
        wristServo = hardwareMap.get(Servo.class, "wristServo");

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

    public void MoveExtensionMotors(int pos){
        ExtensionMotorEx1.setPower(1);
        ExtensionMotorEx2.setPower(1);
        ExtensionMotorEx1.setTargetPosition(pos);
        ExtensionMotorEx2.setTargetPosition(pos);
        ExtensionMotorEx1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        ExtensionMotorEx2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void MoveActuatorMotor(int pos){
        ActuatorMotorEx.setPower(1);
        ActuatorMotorEx.setTargetPosition(pos);
        ActuatorMotorEx.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

}
