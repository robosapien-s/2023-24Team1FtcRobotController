package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.OpModes.IMUWrapper;
import org.firstinspires.ftc.teamcode.OpModes.InitActuatorPos;
import org.firstinspires.ftc.teamcode.OpModes.NewDrive;
import org.firstinspires.ftc.teamcode.OpModes.RedOrBlue;
import org.firstinspires.ftc.teamcode.controllers.CallBackTask;
import org.firstinspires.ftc.teamcode.controllers.IRobotTask;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskParallel;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskSeries;
import org.firstinspires.ftc.teamcode.controllers.ServoTask;

import java.sql.Time;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.Timer;
import java.util.TimerTask;

public class NeoArmWrapper {

    double angle;

    boolean isWalking = false;


    int lastActuator;
    int lastExtension;



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
    //public CRServo armWheel;

    public CRServo intakeServoFront;
    public CRServo intakeServo;


    public Servo armWristServo;
    public Servo armLeftRight;
    public Servo armChain;
    public Servo armPixelRot;

    public Servo rightPixelHolder;
    public Servo leftPixelHolder;


    //public TouchSensor armTouch;
    public DigitalChannel armTouch;

    public boolean limit = true;



    ElapsedTime ext_timer = new ElapsedTime();
    private double ext_lastError = 0;
    private double ext_intergralSum = 0;

    private double ext_Kp = 0.004;
    private double ext_Ki = 0.0000001;
    private double ext_Kd = 0.00004;
    private double ext_targetPosition = 0;


    private double ext_targetPosition_delay_until = -1;


    public static double arm_wrist_floor = .3;
    public static double arm_wrist_ceiling = 0;
    public static  double  arm_wrist_intake_pos = .65;

    private  double  arm_wrist_tagetPosition = arm_wrist_intake_pos;

    Timer loopTimer = null;
    Timer intakeTimer = null;



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

    ArrayList<IRobotTask> tasks = new ArrayList<IRobotTask>();

    EIntakeOuttakeMode intakeOuttakeMode = EIntakeOuttakeMode.INTAKE;

    //EPixelHolderLocation pixelHolderLocation = EPixelHolderLocation.SINGLE;

    int pixelHolderIndex = 0;//EPixelHolderLocation.SINGLE;
    ArrayList<EPixelHolderLocation> pixelHolderList = new ArrayList<EPixelHolderLocation>();

    ArrayList<Double> pixelHolderListValues = new ArrayList<Double>();

    boolean isWristUp = true;

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

        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        intakeServoFront = hardwareMap.get(CRServo.class, "intakeServoFront");

        planeServo = hardwareMap.get(Servo.class, "planeServo");

        armServo0 = hardwareMap.get(Servo.class, "armServo0");
        armServo1 = hardwareMap.get(Servo.class, "armServo1");
        wristServo = hardwareMap.get(Servo.class, "wristServo");

        //armWheel = hardwareMap.get(CRServo.class, "armWheel");

        armWristServo = hardwareMap.get(Servo.class, "armWrist");
        armLeftRight = hardwareMap.get(Servo.class, "armLeftRight");
        armChain = hardwareMap.get(Servo.class, "armChain");
        armPixelRot = hardwareMap.get(Servo.class,"armPixelRot");


        rightPixelHolder = hardwareMap.get(Servo.class,"rightPixelHolder");
        leftPixelHolder = hardwareMap.get(Servo.class,"leftPixelHolder");


        pixelHolderList.add(EPixelHolderLocation.SINGLE);
        pixelHolderList.add(EPixelHolderLocation.DOUBLE);
        pixelHolderList.add(EPixelHolderLocation.SINGLE_UPSIDE_DOWN);
        pixelHolderList.add(EPixelHolderLocation.DOUBLE_UPSIDE_DOWN);

        pixelHolderListValues.add(0.63);
        pixelHolderListValues.add(0.3);
        pixelHolderListValues.add(0.0);
        pixelHolderListValues.add(1.0);

        //armTouch = hardwareMap.get(DigitalChannel.class, "armTouch");

        isAuto = inIsAuto;
    }

    public enum ePosition{
        BOTTOM,
        LOW,
        MEDIUM,
        HIGH
    }

    public enum EIntakeOuttakeMode {
        INTAKE,
        OUTTAKE
    }

    public enum EPixelHolderLocation {
        DOUBLE,
        DOUBLE_UPSIDE_DOWN,
        SINGLE,
        SINGLE_UPSIDE_DOWN
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

    public void openRightPixelHolder() {
        rightPixelHolder.setPosition(.55);
    }

    public void closeRightPixelHolder() {
        rightPixelHolder.setPosition(.38);
    }

    public void openLeftPixelHolder() {
        leftPixelHolder.setPosition(.5);
    }

    public void closeLeftPixelHolder() {
        leftPixelHolder.setPosition(.38);
    }


    public void UpdateIntakePower(float power, JoystickWrapper joystickWrapper){

        if(Math.abs(power)>0) {
            rightPixelHolder.setPosition(.55);
            leftPixelHolder.setPosition(.5);
        } else {

            EPixelHolderLocation pixelHolderEnum = getCurrentPixelRotEnum();
            if(joystickWrapper != null && joystickWrapper.gamepad2GetRightTriggerPressed()) {


                switch (pixelHolderEnum){
                    case SINGLE_UPSIDE_DOWN:
                    case DOUBLE:
                        leftPixelHolder.setPosition(.5);
                        break;
                    case SINGLE:
                    case DOUBLE_UPSIDE_DOWN:
                        rightPixelHolder.setPosition(.55);
                        break;
                }
            } else {
                switch (pixelHolderEnum){
                    case SINGLE:
                        rightPixelHolder.setPosition(.38);
                        break;
                    case DOUBLE:
                        leftPixelHolder.setPosition(.4);
                        break;
                    case SINGLE_UPSIDE_DOWN:
                        leftPixelHolder.setPosition(.38);
                        break;
                    case DOUBLE_UPSIDE_DOWN:
                        rightPixelHolder.setPosition(.4);
                        break;
                }
            }


            if(joystickWrapper != null && joystickWrapper.gamepad2GetLeftTriggerPressed()) {
                switch (pixelHolderEnum){
                    case SINGLE:
                        leftPixelHolder.setPosition(.55);
                        break;
                    case DOUBLE:
                        rightPixelHolder.setPosition(.5);
                        break;
                    case SINGLE_UPSIDE_DOWN:
                        rightPixelHolder.setPosition(.55);
                        break;
                    case DOUBLE_UPSIDE_DOWN:
                        leftPixelHolder.setPosition(.5);
                        break;
                }
            } else {
                switch (pixelHolderEnum){
                    case SINGLE:
                        leftPixelHolder.setPosition(.4);
                        break;
                    case DOUBLE:
                        rightPixelHolder.setPosition(.38);
                        break;
                    case SINGLE_UPSIDE_DOWN:
                        rightPixelHolder.setPosition(.4);
                        break;
                    case DOUBLE_UPSIDE_DOWN:
                        leftPixelHolder.setPosition(.38);
                        break;
                }
            }
        }


        IntakeMotorEx.setPower(power);
        intakeServo.setPower(-power);
        intakeServoFront.setPower(-power);
    }



    public void MoveMotorWithTelemetry(int Move){
        telemetry.addData("current",ActuatorMotorEx.getCurrent(CurrentUnit.AMPS));
        ActuatorMotorEx.setTargetPosition(ActuatorMotorEx.getCurrentPosition()+Move);
        telemetry.update();
    }

    public void ClosePos(){
        //armServo0.setPosition(.75);
        //armServo1.setPosition(.57);
    }
    public void OpenPos(){
        //armServo0.setPosition(.95);
        //armServo1.setPosition(.35);
        telemetry.update();
    }
    public void WristUp(){
        isWristUp = true;
        wristServo.setPosition(.1);

    }
    public void WristDown(){
        isWristUp = false;
        wristServo.setPosition(.78);
    }

    public void UpdateExtensionPlusInput(JoystickWrapper joystickWrapper, int slideEncoderFactor, int actuatorEncoderFactor, IMUWrapper imuWrapper, IMU imu){

        if (isWristUp) {
            if (ActuatorMotorEx.getCurrentPosition() < 1500) {
                wristServo.setPosition(.1);
            } else {
                wristServo.setPosition(.4);
            }
        }


        double actuatorLimit = 6500;
        double actuatorTransitionPoint = 0;//300;
        double actuatorDelayTime = 0;//500;

        double newExtTargetPositionRequest = ext_targetPosition;
        double newActTargetPositionRequest = act_targetPosition;



//        if (joystickWrapper != null && Math.abs(joystickWrapper.gamepad2GetLeftStickY()) > .5) {
//            newActTargetPositionRequest = ActuatorMotorEx.getCurrentPosition() + (int) (-joystickWrapper.gamepad2GetLeftStickY() * actuatorEncoderFactor);
//
//
//
//            /*
//            double k = 1.35980056; //TODO: Solve for this, coefficient of an imperfect angle measurement -- SOLVED hypothetically, come back to this if not working
//
//            newExtTargetPositionRequest = inchesToExtension(  //x_2 =
//                    (extensionToInches(ExtensionMotorEx1.getCurrentPosition())  *   Math.sin(Math.toDegrees(60-(k*angle)))) // x_1 * sin(60˚-theta_1)
//                    /(Math.sin(Math.toDegrees(60-(k*getAngleGivenActuator((int) newActTargetPositionRequest))))) //    /sin(60˚-theta_2)
//            );
//*/
//
//            /**TODO: possible errors if doesn't work:
//             *      k is wrong
//             *      extensionToInches and inchesToExtension are imperfect
//             *      typed massive formula wrong -- I hate parentheses now
//             */
//
//            //angle = 13.26, slide = 752
//            //angle = 36.56, slide = 2319
//
//
//            //actuator = 9, slide = 491
//            //actuator = 2946, slide = 2305
//
//            //double extensionActuatorRatio = 0.6176370446;
//
//           // newExtTargetPositionRequest = ExtensionMotorEx1.getCurrentPosition() + (int) (extensionActuatorRatio*(newActTargetPositionRequest-ActuatorMotorEx.getCurrentPosition()));
//
//            //double extensionAngleRatio = 67.2532188841; //(1567.0)/(23.3)
//            //newExtTargetPositionRequest = ExtensionMotorEx1.getCurrentPosition() + (int) (extensionAngleRatio*(getAngleGivenActuator((int) newActTargetPositionRequest)-angle));
//
//
//
//        }

        if (joystickWrapper != null && Math.abs(joystickWrapper.gamepad2GetLeftStickY()) > .5) {
            if (!joystickWrapper.gamepad2GetLeftStickDown()) {
                if (!isWalking) {
                    lastActuator = ActuatorMotorEx.getCurrentPosition();
                    lastExtension = ExtensionMotorEx1.getCurrentPosition();
                }
                isWalking = true;
            } else {
                isWalking = false;
            }


            newActTargetPositionRequest = ActuatorMotorEx.getCurrentPosition() + (int) (-joystickWrapper.gamepad2GetLeftStickY() * actuatorEncoderFactor);

            //newExtTargetPositionRequest = (0.6834 * newActTargetPositionRequest) + 112.1;
            if (!joystickWrapper.gamepad2GetLeftStickDown()) {
                newExtTargetPositionRequest = (0.65 * (newActTargetPositionRequest - lastActuator)) + lastExtension;
            }
        }

        if (joystickWrapper != null && Math.abs(joystickWrapper.gamepad2GetRightStickY()) > .5) {

            isWalking = false;
            newExtTargetPositionRequest = ExtensionMotorEx1.getCurrentPosition() + (int) (-joystickWrapper.gamepad2GetRightStickY() * slideEncoderFactor);
        }



        if(joystickWrapper != null) {
            if(joystickWrapper.gamepad1GetRightBumperDown()){
                /*
                long timer = 450;

                if(intakeTimer == null) {
                    timer = 600;
                    intakeTimer = new Timer();
                } else {
                    intakeTimer.cancel();
                    intakeTimer = new Timer();
                }

                armServo0.setPower(.8);
                armServo1.setPower(-.8);

                intakeTimer.schedule(new TimerTask() {
                    @Override
                    public void run() {
                        armServo0.setPower(0);
                        armServo1.setPower(0);
                        intakeTimer.cancel();
                    }
                }, timer);


                */


                //armServo0.setPower(.5);
                //armServo1.setPower(-.5);
            }/* else {
                armServo0.setPower(0);
                armServo1.setPower(0);
            }*/
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

            if(System.currentTimeMillis() >= ext_targetPosition_delay_until) {
                ExtensionMotorEx1.setPower(power);
                ExtensionMotorEx2.setPower(power);

                telemetry.addData("Slide Pos:", ext_targetPosition);
                telemetry.addData("Slide power:", power);
            }

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

        executeTasks();

        if(imuWrapper != null) {
            double headingOffset;
            if (!RedOrBlue.isRed) {
                headingOffset = imuWrapper.getNormalizedHeadingError() - 90;
            } else {
                headingOffset = imuWrapper.getNormalizedHeadingError() + 90;
            }


            if (headingOffset > 180) {
                headingOffset-= 360;
            } else if (headingOffset < -180) {
                headingOffset +=360;
            }

            setLeftRight(headingOffset);
        } else if(imu != null) {

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

            double normalizedHeadingError = -orientation.getYaw(AngleUnit.DEGREES);

            double headingOffset;
            if (RedOrBlue.isRedAuto) { //Opposite of TeleOp because starting orientation is flipped
                headingOffset = normalizedHeadingError - 90;
            } else {
                headingOffset = normalizedHeadingError + 90;
            }

            if (headingOffset > 180) {
                headingOffset-= 360;
            } else if (headingOffset < -180) {
                headingOffset +=360;
            }

            setLeftRight(headingOffset);

        }

        setArmChain(ActuatorMotorEx.getCurrentPosition());

        //TODO need to auto do wrise in outtake mode??
//        if(ExtensionMotorEx1.getCurrentPosition() > actuatorTransitionPoint) {
//            double fudgeFactor = .1;
//            double percentActuator =  (ActuatorMotorEx.getCurrentPosition() / actuatorLimit);
//            double armWristSeverSpread = arm_wrist_floor - arm_wrist_ceiling;
//            double newArmServoPos = arm_wrist_floor - (armWristSeverSpread * percentActuator) - fudgeFactor;
//
//
//            if(newArmServoPos < 0) {
//                newArmServoPos = 0;
//            } else if (newArmServoPos > .3) {
//                newArmServoPos = .3;
//            }
//
//
//            if(!isAuto) {
//                armWristServo.setPosition(newArmServoPos);
//            }
//            isIntakeMode = false;
//        } else {
//
//            if(!isIntakeMode) {
//                isIntakeMode = true;
//                wrist_servo_close_time = System.currentTimeMillis();
//            }
//
//            if(!isAuto) {
//                armWristServo.setPosition(arm_wrist_intake_pos);
//            }
//        }


        telemetry.addData("Actuator Pos:", ActuatorMotorEx.getCurrentPosition());

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
        double output = (error*act_Kp) + (derivative*act_Kd) + (act_intergralSum*act_Ki);
        //double output = (error* NewDrive.act_Kp) + (derivative*NewDrive.act_Kd) + (act_intergralSum*NewDrive.act_Ki);

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

    public void setActuatorPosition(double acutator) {
        act_lastError = 0;
        act_targetPosition = acutator;
    }

    public void setExtPosition(double extension) {
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

    public void SetLinearActuatorTask(int pos){

        tasks.add(
                new CallBackTask(new CallBackTask.CallBackListener() {
                    @Override
                    public void setPosition(double value) {
                        act_targetPosition = value;
                    }

                    @Override
                    public double getPosition() {
                        return ActuatorMotorEx.getCurrentPosition();
                    }
                }, 300, 300, "ActuatorMotorEx", true)
        );
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

    public void ResetMotorPositionsWithoutZero(){

        ActuatorMotorEx.setDirection(DcMotorEx.Direction.FORWARD);
        //ActuatorMotorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ActuatorMotorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ActuatorMotorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ExtensionMotorEx1.setDirection(DcMotorEx.Direction.REVERSE);
        //ExtensionMotorEx1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ExtensionMotorEx1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ExtensionMotorEx1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ExtensionMotorEx2.setDirection(DcMotorEx.Direction.FORWARD);
        //ExtensionMotorEx2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ExtensionMotorEx2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ExtensionMotorEx2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

    public void DropRightBottomPixel() {

    }

    public void DropLeftTopPixel() {

    }

    public void SetWheelSpin(double Power){
        //armWheel.setPower(Power);
        telemetry.addData("Wheel Power", Power);
    }
    public void ActivateLoop(){

        /* TODO verify that this isn't being used
        if(loopTimer == null) {
            loopTimer = new Timer();
        }
        loopTimer.schedule(new TimerTask() {
            @Override
            public void run() {
                telemetry.addData("aaaa", System.currentTimeMillis());
                UpdateExtensionPlusInput(null, 200, 200, null, null);
                telemetry.update();
            }
        }, 100);
        */
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


    public void setHangPos(boolean hanged) {
        if (!hanged) {
            SetLinearActuator(5653);
            SetLinearExtensionPos(1900);

            armWristServo.setPosition(.8);
            armChain.setPosition(0.06);
        } else {
            SetLinearActuator(5653);
            SetLinearExtensionPos(1260);
        }
    }



    public void setArmChain(int actuatorPos) {
        double a = 13;
        double b = 5.5;
        double actuatorToInches = (4.75-1.125)/6500; //constant conversion rate of actuator position to additional inches
        double c = 7.75/*INCLUDES BASE EXTRA 1.125*/ + (double) actuatorPos*actuatorToInches; // the extra part based on current actuator length




        angle = Math.toDegrees(Math.acos((a*a+b*b-(c*c))/(2*a*b)));
        telemetry.addData("Actuator Angle", angle);
        if (intakeOuttakeMode == EIntakeOuttakeMode.OUTTAKE) {
                //TODO: lower: angle:13.35,servo:0.693333333333333...
                //TODO: upper: angle:42.535,servo:.8133333333
            armChain.setPosition((.12/29.185)*(angle-42.535)+.813333333333);

            telemetry.addData("armChainTarget",(.12/29.185)*(angle-42.535)+.813333333333);

        }
    }

    public double getAngleGivenActuator(int actuatorPos) {
        double a = 13;
        double b = 5.5;
        double actuatorToInches = (4.75 - 1.125) / 6500; //constant conversion rate of actuator position to additional inches
        double c = 7.75/*INCLUDES BASE EXTRA 1.125*/ + (double) actuatorPos * actuatorToInches; // the extra part based on current actuator length


        return Math.toDegrees(Math.acos((a * a + b * b - (c * c)) / (2 * a * b)));
    }


    public void setLeftRight(double headingError) {


        double upperBound = .69;
        double lowerBound = .27;
        double range = (upperBound - lowerBound) * 300;
        double midPoint = .49;
        double degreeBoundUpper = (upperBound-midPoint)*300;
        double degreeBoundLower = (lowerBound-midPoint)*300;


        if(intakeOuttakeMode == EIntakeOuttakeMode.OUTTAKE) {//TODO:  need to also make sure that the armWrist is greater that a certain spot

        /*if(wristServo.getPosition<[threshold]) {
            armLeftRight.setPosition(midPoint)
         } else {
         }
         */
            if (headingError > degreeBoundUpper) {
                headingError = degreeBoundUpper;
            } else if (headingError < degreeBoundLower) {
                headingError = degreeBoundLower;
            }

            if(headingError<0) {
                headingError *= 1.05; //TODO - ROBOT RIGHT, WRIST TURNED LEFT
            } else {
                headingError *= 1.22; //TODO - ROBOT RIGHT, WRIST TURNED LEFT
            }

            double newPosition = midPoint + (headingError / 300.0);

            telemetry.addData("armLeftRight", newPosition);
            telemetry.update();
            armLeftRight.setPosition(newPosition);
        } else {
            telemetry.addData("not calling", "not calling");
            telemetry.update();
            //armLeftRight.setPosition(midPoint);
        }

    }

    public void setNextRotServoEnum() {

        pixelHolderIndex++;
        if(pixelHolderIndex >= pixelHolderList.size() ) {
            pixelHolderIndex = 0;
        }

        updatePixelRotServo();
    }

    public void setPrevRotServoEnum() {

        pixelHolderIndex--;
        if(pixelHolderIndex < 0) {
            pixelHolderIndex =  pixelHolderList.size()-1;
        }

        updatePixelRotServo();
    }

    public int getPixelHolderIndexByEnum(EPixelHolderLocation inRotHolderLocation) {

        int index = 0;
        for(int i = 0; i < pixelHolderList.size(); i++) {

            if(pixelHolderList.get(i) == inRotHolderLocation) {
                index = i;
                break;
            }
        }

        return  index;
    }

    public void setRotServoEnum(EPixelHolderLocation inRotHolderLocation) {
        pixelHolderIndex = getPixelHolderIndexByEnum(inRotHolderLocation);
    }

    public double getPixelRotServoValue() {
       return pixelHolderListValues.get(pixelHolderIndex);
    }

    public double getPixelRotServoValueByEnum(EPixelHolderLocation ePixelHolderLocation) {
        return pixelHolderListValues.get(getPixelHolderIndexByEnum(ePixelHolderLocation));
    }

    public EPixelHolderLocation getCurrentPixelRotEnum() {
        return pixelHolderList.get(pixelHolderIndex);
    }

    public void setIntakeOuttakeMode(EIntakeOuttakeMode mode) {
        intakeOuttakeMode = mode;
    }

    public void updatePixelRotServo() {
        if(intakeOuttakeMode == EIntakeOuttakeMode.OUTTAKE) {
            armPixelRot.setPosition(getPixelRotServoValue());
        }
    }

    public void setLeftRightServo(double position) {
        armLeftRight.setPosition(position);
    }

    public void setArmPixelRotServo(EPixelHolderLocation location) {

        armPixelRot.setPosition(getPixelRotServoValueByEnum(location));
    }

    public void setArmChainServo(double position) {
        armChain.setPosition(position);
    }

    public void setArmWristServo(double position) {
        armWristServo.setPosition(position);
    }

    public void setIntakeNew() {

        act_lastError = 0;
        act_targetPosition = 0;

        ext_lastError = 0;
        //ext_targetPosition_delay_until = System.currentTimeMillis() + 10000;
        //ext_targetPosition = 0;

        clearTasks();

        intakeOuttakeMode = EIntakeOuttakeMode.INTAKE;
        RobotTaskSeries series = new RobotTaskSeries();

        RobotTaskParallel pullBackArm = new RobotTaskParallel();
        pullBackArm.add(new ServoTask(armLeftRight, 0.49, 500, "armLeftRight", true));
        pullBackArm.add(new ServoTask(armPixelRot, getPixelRotServoValueByEnum(EPixelHolderLocation.SINGLE), 250, "armPixelRot", true));
        double extensionCoef = (ExtensionMotorEx1.getCurrentPosition()-59)/(2316-59);
        double actuatorCoef = (ActuatorMotorEx.getCurrentPosition()-5)/(1854-5);
        double timeCoef;
        if (extensionCoef>actuatorCoef) {
            timeCoef=extensionCoef;
        } else {
            timeCoef=actuatorCoef;
        }
        pullBackArm.add(
                new CallBackTask(new CallBackTask.CallBackListener() {
                    @Override
                    public void setPosition(double value) {
                        ext_targetPosition = value;
                    }

                    @Override
                    public double getPosition() {
                        return ExtensionMotorEx1.getCurrentPosition();
                    }
                }, 300, (int) (1000*timeCoef), "ExtensionMotorEx1", true)
        );
        pullBackArm.add(
                new CallBackTask(new CallBackTask.CallBackListener() {
                    @Override
                    public void setPosition(double value) {
                        act_targetPosition = value;
                    }

                    @Override
                    public double getPosition() {
                        return ActuatorMotorEx.getCurrentPosition();
                    }
                }, 300, (int) (2000*timeCoef), "ActuatorMotorEx", true)
        );

        series.add(pullBackArm);

        RobotTaskParallel parallel = new RobotTaskParallel();
        parallel.add(new ServoTask(armWristServo, .7, 500, "armWristServo", true));
        parallel.add(new ServoTask(armChain, 0, 500, "armChain", true));
        parallel.add(new ServoTask(armPixelRot, getPixelRotServoValueByEnum(EPixelHolderLocation.SINGLE), 500, "armPixelRot", true));
        parallel.add( new CallBackTask(new CallBackTask.CallBackListener() {
            @Override
            public void setPosition(double value) {
                ext_targetPosition = value;
            }

            @Override
            public double getPosition() {
                return ExtensionMotorEx1.getCurrentPosition();
            }
        }, 300, 500, "lineralExt", true));

/*
        parallel.add(new ServoTask(armWristServo, .7, 1250, "armWristServo", true));
        parallel.add(new ServoTask(armChain, 0.25, 750, "armChain", true));
        parallel.add(new ServoTask(armLeftRight, 0.49, 1250, "armLeftRight", true));
        parallel.add(new ServoTask(armPixelRot, 0.63, 500, "armPixelRot", true));
 */

        series.add(parallel);


        RobotTaskParallel parallel2 = new RobotTaskParallel();
        parallel2.add(new ServoTask(armWristServo, .8, 600, "armWristServo", true));
        parallel2.add(new ServoTask(armChain, 0.063, 600, "armChain", true));
        parallel2.add( new CallBackTask(new CallBackTask.CallBackListener() {
            @Override
            public void setPosition(double value) {
                ext_targetPosition = value;
            }

            @Override
            public double getPosition() {
                return ExtensionMotorEx1.getCurrentPosition();
            }
        }, 0, 500, "lineralExt", true));

        parallel2.add(
                new CallBackTask(new CallBackTask.CallBackListener() {
                    @Override
                    public void setPosition(double value) {
                        act_targetPosition = value;
                    }

                    @Override
                    public double getPosition() {
                        return ActuatorMotorEx.getCurrentPosition();
                    }
                }, 0, 500, "ActuatorMotorEx", true)
        );

        series.add(parallel2);


        tasks.add(series);

    }

    public void setOuttakeNew(EPixelHolderLocation pixelLocation) {
        setOuttakeNew(pixelLocation, 500, 500);
    }
    public void setOuttakeNewWithActAndExt(EPixelHolderLocation pixelLocation, int actuatorPosition, int extPosition) {
        setOuttakeNew(pixelLocation, actuatorPosition, extPosition);
    }

    public void setOuttakeNewWithAct(EPixelHolderLocation pixelLocation, int actuatorPosition) {
        setOuttakeNew(pixelLocation, actuatorPosition, 500);
    }
    public void setOuttakeNew(EPixelHolderLocation pixelLocation, int actuatorPosition, int extPosition) {
        act_lastError = 0;
        act_targetPosition = actuatorPosition;

        ext_lastError = 0;
        //ext_targetPosition_delay_until = System.currentTimeMillis() + 500;
        ext_targetPosition = extPosition;

        clearTasks();

        RobotTaskSeries series = new RobotTaskSeries();

        series.add(new ServoTask(armWristServo, .5, 500, "armWristServo", true));

        RobotTaskParallel parallel = new RobotTaskParallel();
        parallel.add(new ServoTask(armWristServo, .3, 600, "armWristServo", true));
        parallel.add(new ServoTask(armChain, .7, 600, "armChain", true));

        parallel.add(new ServoTask(armPixelRot, getPixelRotServoValueByEnum(pixelLocation), 600, "armPixelRot", true));

        //parallel.add(new ServoTask(armLeftRight, 0.3, 10000, "armLeftRight", true));
        //parallel.add(new ServoTask(armPixelRot, 0.3, 600, "armPixelRot", true));



        series.add(parallel);
        series.add(new CallBackTask(new CallBackTask.CallBackListener() {
            @Override
            public void setPosition(double value) {
                intakeOuttakeMode = EIntakeOuttakeMode.OUTTAKE;
            }

            @Override
            public double getPosition() {
                return 1;
            }
        }, 1, 1, "", true));
        //series.add(new ServoTask(armPixelRot, 0.3, 1250, "armPixelRot", true));



        tasks.add(series);

        setRotServoEnum(EPixelHolderLocation.DOUBLE);

    }


    public void clearTasks() {

        for (IRobotTask task :
                tasks) {
            task.stopTask();
        }

        tasks.clear();

    }


    public void primeArm() {
        clearTasks();
        RobotTaskSeries series = new RobotTaskSeries();

        RobotTaskParallel parallel = new RobotTaskParallel();
        parallel.add(new CallBackTask(new CallBackTask.CallBackListener() {
            @Override
            public void setPosition(double value) {
                setIntakeNew();
            }

            @Override
            public double getPosition() {
                return ExtensionMotorEx1.getCurrentPosition();
            }
        }, 0, (1000), "ExtensionMotorEx1", true)
        );
        series.add(parallel);
        series.add(parallel);

        RobotTaskParallel parallel2 = new RobotTaskParallel();
        parallel2.add(new CallBackTask(new CallBackTask.CallBackListener() {
            @Override
            public void setPosition(double value) {
                act_targetPosition = value;
            }

            @Override
            public double getPosition() {
                return ExtensionMotorEx1.getCurrentPosition();
            }
        }, InitActuatorPos.actuatorPos, (1000), "ActuatorMotorEx", true));

        tasks.add(series);
    }
    public void executeTasks() {

        if(tasks.size()>0) {

            boolean isStarted = tasks.get(0).hasStarted();
            boolean isRunning = tasks.get(0).isRunning();
            boolean isComplete = tasks.get(0).isComplete();

//            if(showTaskTelemetry) {
//                telemetry.addData("isStarted", isStarted);
//                telemetry.addData("isRunning", isRunning);
//                telemetry.addData("isComplete", isComplete);
//            }

            tasks.get(0).execute(telemetry);




            if(isComplete){
                tasks.remove(0);

//                if(showTaskTelemetry) {
//                    telemetry.addData("taskSize completed", tasks.size());
//                }
            }
            //

            //telemetry.update();

        }



    }


    public void PickupStack(){
        /*
        long timer = 450;

        if(intakeTimer == null) {
            timer = 600;
        } else {
            intakeTimer.cancel();
        }
        intakeTimer = new Timer();

        armServo0.setPower(.8);
        armServo1.setPower(-.8);

        intakeTimer.schedule(new TimerTask() {
            @Override
            public void run() {
                armServo0.setPower(0);
                armServo1.setPower(0);
                intakeTimer.cancel();
            }
        }, timer);





        //armServo0.setPower(.5);
        //armServo1.setPower(-.5);
        */

    }

    public void PickupStack5Turn(float amount){



        armServo1.setPosition(amount*.2);
        armServo0.setPosition(1-(amount*.2));


        //armServo0.setPower(.5);
        //armServo1.setPower(-.5);
    }

    public double extensionToInches(int extensionPos) {
        double extToInRatio = 0.008797495682; //(35.875-15.5)/(2316-0)
        return extensionPos*extToInRatio+15.5;
    }

    public int inchesToExtension(double inches) {
        double extToInRatio = 0.008797495682; //(35.875-15.5)/(2316-0)
        return (int) ((inches-15.5)/extToInRatio);
    }



}
