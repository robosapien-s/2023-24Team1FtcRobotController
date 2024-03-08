package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.sql.Time;
import java.util.Date;
import java.util.Timer;
import java.util.TimerTask;

public class NeoArmWrapper {

    Servo planeServo;

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

    public Servo armWristServo;
    public CRServo armWheel;

    //public TouchSensor armTouch;
    public DigitalChannel armTouch;

    boolean limit = true;



    ElapsedTime ext_timer = new ElapsedTime();
    private double ext_lastError = 0;
    private double ext_intergralSum = 0;

    private double ext_Kp = 0.004;
    private double ext_Ki = 0.0000001;
    private double ext_Kd = 0.00004;
    private double ext_targetPosition = 0;


    public static double arm_wrist_floor = .3;
    public static double arm_wrist_ceiling = 0;
    public static  double  arm_wrist_intake_pos = .65;

    private  double  arm_wrist_tagetPosition = arm_wrist_intake_pos;

    Timer loopTimer = null;



    ElapsedTime act_timer = new ElapsedTime();
    private double act_lastError = 0;
    private double act_intergralSum = 0;

    private double act_Kp = 0.008;
    private double act_Ki = 0.000001;
    private double act_Kd = 0.0004;
    private double act_targetPosition = 0;
    boolean isAuto = false;


    private long wrist_servo_close_time = -1;
    private  boolean isIntakeMode = false;

    public NeoArmWrapper(Telemetry inTelemetry, HardwareMap inHardwareMap, Gamepad inGamepad1, Gamepad inGamepad2, Boolean inIsAuto){
        //Set Values
        telemetry = inTelemetry;
        hardwareMap = inHardwareMap;
        gamepad1 = inGamepad1;
        gamepad2 = inGamepad2;
        ExtensionMotorEx1 = hardwareMap.get(DcMotorEx.class,"ExtensionMotorEx0");
        ExtensionMotorEx2 = hardwareMap.get(DcMotorEx.class, "ExtensionMotorEx1");
        ActuatorMotorEx = hardwareMap.get(DcMotorEx.class,"ActuatorMotor");
        IntakeMotorEx = hardwareMap.get(DcMotorEx.class, "IntakeMotor");

        planeServo = hardwareMap.get(Servo.class, "planeServo");

        armServo0 = hardwareMap.get(Servo.class, "armServo0");
        armServo1 = hardwareMap.get(Servo.class, "armServo1");
        wristServo = hardwareMap.get(Servo.class, "wristServo");

        armWristServo = hardwareMap.get(Servo.class, "armWrist");
        armWheel = hardwareMap.get(CRServo.class, "armWheel");

        //armTouch = hardwareMap.get(DigitalChannel.class, "armTouch");

        isAuto = inIsAuto;
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

    public void setIsAuto(boolean inIsAuto) {
        isAuto = inIsAuto;
    }

    public void ClosePos(){
        armServo0.setPosition(.75);
        armServo1.setPosition(.57);
    }
    public void OpenPos(){
        armServo0.setPosition(.95);
        armServo1.setPosition(.35);
        telemetry.update();
    }
    public void WristUp(){
        wristServo.setPosition(.15);
    }
    public void WristDown(){
        wristServo.setPosition(.78);
    }

    public void UpdateExtensionPlusInput(JoystickWrapper joystickWrapper, int slideEncoderFactor, int actuatorEncoderFactor){


        double actuatorLimit = 6500;
        double actuatorTransitionPoint = 300;
        double actuatorDelayTime = 500;

        double newExtTargetPositionRequest = ext_targetPosition;
        double newActTargetPositionRequest = act_targetPosition;

        if (joystickWrapper != null && Math.abs(joystickWrapper.gamepad2GetRightStickY()) > .5) {
            newExtTargetPositionRequest = ExtensionMotorEx1.getCurrentPosition() + (int) (-joystickWrapper.gamepad2GetRightStickY() * slideEncoderFactor);
        }

        if (joystickWrapper != null && Math.abs(joystickWrapper.gamepad2GetLeftStickY()) > .5) {
            newActTargetPositionRequest = ActuatorMotorEx.getCurrentPosition() + (int) (-joystickWrapper.gamepad2GetLeftStickY() * actuatorEncoderFactor);
        }

        ext_targetPosition = newExtTargetPositionRequest;
        act_targetPosition = newActTargetPositionRequest;

        // if (joystickWrapper.gamepad2GetRightBumperDown()){
        //    limit = !limit;
        // }

        if (ext_targetPosition < 0 && limit) {
            ext_targetPosition = 0;
        }
        if (ext_targetPosition > 2335 && limit) {
            ext_targetPosition = 2335;
        }

        if (act_targetPosition < 0 && limit) {
            act_targetPosition = 0;
        }
        if (act_targetPosition > actuatorLimit && limit) {
            act_targetPosition = actuatorLimit;
        }

        double requestedExt = ext_targetPosition;
       double requestedAct = act_targetPosition;



        if(ExtensionMotorEx1.getCurrentPosition() <= actuatorTransitionPoint &&  (requestedExt < ExtensionMotorEx1.getCurrentPosition() && ActuatorMotorEx.getCurrentPosition()>10)) {
            requestedExt = ExtensionMotorEx1.getCurrentPosition();
        }

        double servoTimeDiff = System.currentTimeMillis() - wrist_servo_close_time;
        if(ExtensionMotorEx1.getCurrentPosition() <= actuatorTransitionPoint &&  servoTimeDiff < actuatorDelayTime) {
            requestedExt = ExtensionMotorEx1.getCurrentPosition();
        }





//        if(ExtensionMotorEx1.getCurrentPosition() <= actuatorTransitionPoint &&  requestedExt < ExtensionMotorEx1.getCurrentPosition() && armWristServo.getPosition()>.29) {
//            requestedExt = ExtensionMotorEx1.getCurrentPosition();
//        }




        if(ExtensionMotorEx1.getCurrentPosition() <= actuatorTransitionPoint) {
            requestedAct = 0;

            if(ext_targetPosition < ExtensionMotorEx1.getCurrentPosition() && ExtensionMotorEx1.getCurrentPosition() < 100 ) {
                act_targetPosition = 0;
            }
        }

        if(true/*armTouch.getState() || ext_targetPosition < ExtensionMotorEx1.getCurrentPosition()*/) {

            double requestPower = returnPower(requestedExt, ExtensionMotorEx1.getCurrentPosition());
            double power = requestPower;

            if(ExtensionMotorEx1.getCurrentPosition() < 600 && power<0) {
                power *= .5;
            }

            if(isIntakeMode && wrist_servo_close_time>actuatorDelayTime) {
                power = requestPower;
            }

            ExtensionMotorEx1.setPower(power);
            ExtensionMotorEx2.setPower(power);

            telemetry.addData("Slide Pos:", ext_targetPosition);
            telemetry.addData("Slide power:", power);


        } else {
            ext_targetPosition = ExtensionMotorEx1.getCurrentPosition();
            ExtensionMotorEx1.setPower(0);
            ExtensionMotorEx2.setPower(0);
        }


        double act_power = act_returnPower(requestedAct, ActuatorMotorEx.getCurrentPosition());
        ActuatorMotorEx.setPower(act_power);


        /*
        if(armTouch.getState() || act_targetPosition > ActuatorMotorEx.getCurrentPosition()) {

            double act_power = act_returnPower(requestedAct, ActuatorMotorEx.getCurrentPosition());
            ActuatorMotorEx.setPower(act_power);


        } else {
            act_targetPosition = ActuatorMotorEx.getCurrentPosition();
            ActuatorMotorEx.setPower(0);
        }
        */



//        if(!armTouch.isPressed()) {
//
//            double power = returnPower(ext_targetPosition, ExtensionMotorEx1.getCurrentPosition());
//            ExtensionMotorEx1.setPower(power);
//            ExtensionMotorEx2.setPower(power);
//
//            telemetry.addData("Slide Pos:", ext_targetPosition);
//            telemetry.addData("Slide power:", power);
//
//            double act_power = act_returnPower(act_targetPosition, ActuatorMotorEx.getCurrentPosition());
//            ActuatorMotorEx.setPower(act_power);
//            //ActuatorMotorEx.setPower(act_power);
//
//
//        } else {
//
//
//
//            act_targetPosition = ActuatorMotorEx.getCurrentPosition();
//            ActuatorMotorEx.setPower(0);
//
//        }

        if(ExtensionMotorEx1.getCurrentPosition() > actuatorTransitionPoint) {
            double fudgeFactor = .1;
            double percentActuator =  (ActuatorMotorEx.getCurrentPosition() / actuatorLimit);
            double armWristSeverSpread = arm_wrist_floor - arm_wrist_ceiling;
            double newArmServoPos = arm_wrist_floor - (armWristSeverSpread * percentActuator) - fudgeFactor;


            if(newArmServoPos < 0) {
                newArmServoPos = 0;
            } else if (newArmServoPos > .3) {
                newArmServoPos = .3;
            }


            if(!isAuto) {
                armWristServo.setPosition(newArmServoPos);
            }
            isIntakeMode = false;
        } else {

            if(!isIntakeMode) {
                isIntakeMode = true;
                wrist_servo_close_time = System.currentTimeMillis();
            }

            if(!isAuto) {
                armWristServo.setPosition(arm_wrist_intake_pos);
            }
        }



    }


    public double returnPower(double reference, double state) {
        double error = reference - state;
        double seconds = ext_timer.seconds();
        ext_intergralSum += error * seconds;

        double errorDiff = error - ext_lastError;
        double derivative = errorDiff / ext_timer.seconds();

        derivative = (error - ext_lastError) / seconds;

        ext_lastError = error;
        ext_timer.reset();
        double output = (error*ext_Kp) + (derivative*ext_Kd) + (ext_intergralSum*ext_Ki);

        if(output>1) {
            output = 1;
        }
        return  output;
    }

    public double act_returnPower(double reference, double state) {
        double error = reference - state;
        double seconds = act_timer.seconds();
        act_intergralSum += error * seconds;

        double errorDiff = error - act_lastError;
        double derivative = errorDiff / ext_timer.seconds();

        derivative = (error - act_lastError) / seconds;

        act_lastError = error;
        act_timer.reset();
        double output = (error*act_Kp) + (derivative*act_Kd) + (act_intergralSum*ext_Ki);

        if(output>1) {
            output = 1;
        }
        return  output;
    }


    public void setArmPositions(double acutator, double extension) {
        act_lastError = 0;
        act_targetPosition = acutator;

        ext_lastError = 0;
        ext_targetPosition = extension;
    }

    public void setOuttake() {

        act_lastError = 0;
        act_targetPosition = 2400;

        ext_lastError = 0;
        ext_targetPosition = 2335;

    }


    public void setIntake() {

        act_lastError = 0;
        act_targetPosition = 0;


        ext_lastError = 0;
        ext_targetPosition = 0;

    }

    public void SetLinearExtensionPos(int pos) {
        ext_lastError = 0;
        ext_targetPosition = pos;
    }
    public void SetLinearActuator(int pos){
        act_lastError = 0;
        act_targetPosition = pos;
    }

    public void MoveExtensionMotors(int position) {
        //ExtensionMotorEx1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //ExtensionMotorEx2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        //ActuatorMotorEx.setTargetPosition(position);
        ExtensionMotorEx1.setTargetPosition(position);
        //ExtensionMotorEx2.setTargetPosition(position);

        //ActuatorMotorEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ExtensionMotorEx1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //ExtensionMotorEx2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //ActuatorMotorEx.setPower(.5);
        ExtensionMotorEx1.setPower(1);
        //ExtensionMotorEx2.setPower(.1);
    }

    public void MoveActuatorMotor(int pos){
        ActuatorMotorEx.setPower(1);
        ActuatorMotorEx.setTargetPosition(pos);
        ActuatorMotorEx.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void ResetMotorPositions(){

        ActuatorMotorEx.setDirection(DcMotorEx.Direction.FORWARD);
        ActuatorMotorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ActuatorMotorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ActuatorMotorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ExtensionMotorEx1.setDirection(DcMotorEx.Direction.REVERSE);
        ExtensionMotorEx1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ExtensionMotorEx1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ExtensionMotorEx1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ExtensionMotorEx2.setDirection(DcMotorEx.Direction.FORWARD);
        ExtensionMotorEx2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ExtensionMotorEx2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ExtensionMotorEx2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //ActuatorMotorEx.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //ExtensionMotorEx1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //ExtensionMotorEx2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        /*ActuatorMotorEx.setTargetPosition(0);
        ExtensionMotorEx1.setTargetPosition(0);
        ExtensionMotorEx2.setTargetPosition(0);*/
    }
    public void SetWheelSpin(double Power){
        armWheel.setPower(Power);
        telemetry.addData("Wheel Power", Power);
    }
    public void ActivateLoop(){
        if(loopTimer == null) {
            loopTimer = new Timer();
        }
        loopTimer.schedule(new TimerTask() {
            @Override
            public void run() {
                telemetry.addData("aaaa", System.currentTimeMillis());
                UpdateExtensionPlusInput(null, 200, 200);
                telemetry.update();
            }
        }, 100);

    }

    public void DeactivateLoop(){

        if(loopTimer != null) {
            loopTimer.cancel();
            loopTimer = null;
        }
    }

    public void setPlaneServo(double position) {
        planeServo.setPosition(position);
    }


    public void setHangPos() {
        SetLinearActuator(5653);
        SetLinearExtensionPos(1417);
    }




}
