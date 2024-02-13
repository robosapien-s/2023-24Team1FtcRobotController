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

    public Servo armServo;
    public Servo wristServo;

    public NeoArmWrapper(Telemetry inTelemetry, HardwareMap inHardwareMap, Gamepad inGamepad1, Gamepad inGamepad2){
        //Set Values
        telemetry = inTelemetry;
        hardwareMap = inHardwareMap;
        gamepad1 = inGamepad1;
        gamepad2 = inGamepad2;
        ActuatorMotorEx = hardwareMap.get(DcMotorEx.class,"ActuatorMotor");
        IntakeMotorEx = hardwareMap.get(DcMotorEx.class, "IntakeMotor");

        armServo = hardwareMap.get(Servo.class, "armServo");
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

    public void IntakePos(){
        armServo.setPosition(.25);
        wristServo.setPosition(.2);
    }
    public void DropPos(){
        armServo.setPosition(.25);
        wristServo.setPosition(.2);
    }
}