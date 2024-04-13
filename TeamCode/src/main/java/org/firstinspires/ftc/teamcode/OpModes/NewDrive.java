package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.controllers.IRobotTask;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskParallel;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskSeries;
import org.firstinspires.ftc.teamcode.controllers.ServoTask;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.wrappers.ArmWrapper;
import org.firstinspires.ftc.teamcode.wrappers.AutoDropOffController;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;
import org.firstinspires.ftc.teamcode.wrappers.NeoArmWrapper;
import org.firstinspires.ftc.teamcode.wrappers.SimplePIDController;

import java.util.ArrayList;

@Config
@TeleOp
public class NewDrive extends LinearOpMode {

    IMUWrapper wrapper;

    boolean hanged = false;

    NeoArmWrapper armWrapper;

    JoystickWrapper joystickWrapper;

    //AutoDropOffController autoDropOffController;

    boolean planeShot = false;

    boolean isDown;
    boolean isOpen;

    boolean showArmTelemetry = false;
    boolean showTaskTelemetry = false;

    DcMotorEx ActuatorMotorEx;


    ArrayList<IRobotTask> tasks = new ArrayList<IRobotTask>();



    public static double act_Kp = 0.008;
    public static double act_Ki = 0.000001;
    public static double act_Kd = 0.0004;
    public static double act_targetPosition = 0;



    public static double Kp_x = 0.00046;
    public static double Ki_x = 0.00000008;
    public static double Kd_x = .00006;
    public static double targetPosition = 10;

    public static double Kp_y = 0.00035;
    public static double Ki_y = 0.0;
    public static double Kd_y = .00006;
    public static double targetPosition_dist = 10;


    public static boolean tune_y = true;
    public static boolean tune_x = true;



    //drop location tuning values
    public static double dropLocation0 = 1.9;
    public static double dropLocation1 = 3;
    public static double dropLocation2 = 4.5;
    public static double dropLocation3 = 6;
    public static double dropLocation4 = 7.5;
    public static double dropLocation5 = 9;
    public static double dropLocation6 = 10.5;
    public static double dropLocation7 = 12;
    public static double dropLocation8 = 13.5;
    public static double dropLocation9 = 15;
    public static double dropLocation10 = 16.5;
    public static double dropLocation11 = 18;
    public static double dropLocation12 = 19.5;
    public static double dropLocation_Kick = .15;


    //Drop height tuning values
    public static double height0_Distance = 7;
    public static int height0_Act = 800;
    public static int height0_Ext = 1200;


    public static double height1_Distance = 16.5;
    public static int height1_Act = 200;
    public static int height1_Ext = 200;


    public static double height2_Distance = 16.5;
    public static int height2_Act = 200;
    public static int height2_Ext = 200;

    public static double height3_Distance = 16.5;
    public static int height3_Act = 200;
    public static int height3_Ext = 200;

    public static double height4_Distance = 16.5;
    public static int height4_Act = 200;
    public static int height4_Ext = 200;



    public static boolean reset_imu = true;


    @Override
    public void runOpMode() throws InterruptedException {

        wrapper = new IMUWrapper();

        armWrapper = new NeoArmWrapper(telemetry,hardwareMap,gamepad2,gamepad2,false);
        armWrapper.ResetMotorPositions();

        if(reset_imu) {
            wrapper.InitializeResetImu(telemetry,hardwareMap,gamepad1, gamepad2,armWrapper);
        } else {
            wrapper.Initialize(telemetry,hardwareMap,gamepad1, gamepad2,armWrapper);
        }

        reset_imu = true;

        joystickWrapper = new JoystickWrapper(gamepad1,gamepad2);
        //autoDropOffController = new AutoDropOffController(wrapper, armWrapper);
        //autoDropOffController.initAprilTag(hardwareMap);
        waitForStart();



        while(!isStopRequested()){




            wrapper.Update();
            armWrapper.SetWheelSpin(gamepad2.left_trigger-gamepad2.right_trigger);
            armWrapper.UpdateIntakePower(gamepad1.right_trigger-gamepad1.left_trigger, joystickWrapper);
            armWrapper.UpdateExtensionPlusInput(joystickWrapper, 300, 300, wrapper, null);


            if(joystickWrapper.gamepad1GetX()) {
                //autoDropOffController.setDropLevel1();
            }

            if(joystickWrapper.gamepad1GetB()) {
                //autoDropOffController.setDropLevel2();
            }

            /*if(joystickWrapper.gamepad2GetDUp()) {
                autoDropOffController.setNextDropLevel();


            }

            if(joystickWrapper.gamepad2GetDDown()) {
                autoDropOffController.setPreviousDropLeve();
            }*/



            if(joystickWrapper.gamepad2GetLeftBumperDown()) {
                //autoDropOffController.setPreviousDropLeve();
                armWrapper.setPrevRotServoEnum();

            }

            if(joystickWrapper.gamepad2GetRightBumperDown()) {
                //autoDropOffController.setNextDropLevel();
                armWrapper.setNextRotServoEnum();
            }

            //autoDropOffController.telemetryAprilTag(telemetry, joystickWrapper, new Encoder(hardwareMap.get(DcMotorEx.class, "fL")),  new Encoder(hardwareMap.get(DcMotorEx.class, "bL")));

            if(joystickWrapper.gamepad2GetY()){
                //armWrapper.setOuttake();
                armWrapper.setOuttakeNew(NeoArmWrapper.EPixelHolderLocation.DOUBLE);
            }

            if(joystickWrapper.gamepad2GetB()){
                if (!planeShot) {
                    armWrapper.setPlaneServo(1);
                } else {
                    armWrapper.setPlaneServo(0);
                }

                planeShot = !planeShot;
            }

            if(joystickWrapper.gamepad2GetA()){
                //armWrapper.setIntake();
                armWrapper.setIntakeNew();
            }

            if(joystickWrapper.gamepad2GetX()) {
                armWrapper.setHangPos(hanged);
                hanged = !hanged;
            }


            if(joystickWrapper.gamepad2GetDDown()){
                /*autoDropOffController.currentDropLevel = 0;
                autoDropOffController.ledController.setCurrentIndex(autoDropOffController.currentDropLevel);*/
                armWrapper.armChain.setPosition(armWrapper.armChain.getPosition()-.05);
            }
            if(joystickWrapper.gamepad2GetDRight()){
                /*autoDropOffController.currentDropLevel = 1;
                autoDropOffController.ledController.setCurrentIndex(autoDropOffController.currentDropLevel);*/
                armWrapper.armChain.setPosition(armWrapper.armChain.getPosition()+.01);
            }
            if(joystickWrapper.gamepad2GetDLeft()){
                /*autoDropOffController.currentDropLevel = 3;
                autoDropOffController.ledController.setCurrentIndex(autoDropOffController.currentDropLevel);*/
                armWrapper.armChain.setPosition(armWrapper.armChain.getPosition()-.01);
            }
            if(joystickWrapper.gamepad2GetDUp()){
                /*autoDropOffController.currentDropLevel = 2;
                autoDropOffController.ledController.setCurrentIndex(autoDropOffController.currentDropLevel);*/
                armWrapper.armChain.setPosition(armWrapper.armChain.getPosition()+.05);
            }


//            if(joystickWrapper.gamepad1GetRightBumperDown()){
//                if(isOpen){
//                    armWrapper.ClosePos();
//                    isOpen = false;
//                }else {
//                    armWrapper.OpenPos();
//                    isOpen = true;
//                }
//            }

            if(joystickWrapper.gamepad1GetLeftBumperDown()){
                if(isDown){
                    armWrapper.WristUp();
                    isDown = false;
                }else {
                    armWrapper.WristDown();
                    isDown = true;
                }
            }

            telemetry.addData("Actuator Pos", armWrapper.ActuatorMotorEx.getCurrentPosition());
            telemetry.addData("Extension1 Pos", armWrapper.ExtensionMotorEx1.getCurrentPosition());
            telemetry.addData("armChainPos",armWrapper.armChain.getPosition());

            if(showArmTelemetry) {
                telemetry.addData("Actuator Pos", armWrapper.ActuatorMotorEx.getCurrentPosition());
                telemetry.addData("Extension1 Pos", armWrapper.ExtensionMotorEx1.getCurrentPosition());
                telemetry.addData("Extension2 Pos", armWrapper.ExtensionMotorEx2.getCurrentPosition());
                telemetry.addData("Actuator Target Pos", armWrapper.ActuatorMotorEx.getTargetPosition());
                telemetry.addData("Extension1 Target Pos", armWrapper.ExtensionMotorEx1.getTargetPosition());
                telemetry.addData("Extension2 Target Pos", armWrapper.ExtensionMotorEx2.getTargetPosition());
            }

            if(tasks.size()>0) {

                boolean isStarted = tasks.get(0).hasStarted();
                boolean isRunning = tasks.get(0).isRunning();
                boolean isComplete = tasks.get(0).isComplete();

                if(showTaskTelemetry) {
                    telemetry.addData("isStarted", isStarted);
                    telemetry.addData("isRunning", isRunning);
                    telemetry.addData("isComplete", isComplete);
                }

                tasks.get(0).execute(telemetry);




                if(isComplete){
                    tasks.remove(0);

                    if(showTaskTelemetry) {
                        telemetry.addData("taskSize completed", tasks.size());
                    }
                }
                //

                //telemetry.update();

            }
        }

        //autoDropOffController.close();
    }
}
