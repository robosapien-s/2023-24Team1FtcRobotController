package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    NeoArmWrapper armWrapper;

    JoystickWrapper joystickWrapper;

    AutoDropOffController autoDropOffController;

    boolean isDown;
    boolean isOpen;

    boolean showArmTelemetry = false;
    boolean showTaskTelemetry = false;

    DcMotorEx ActuatorMotorEx;


    ArrayList<IRobotTask> tasks = new ArrayList<IRobotTask>();




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




    @Override
    public void runOpMode() throws InterruptedException {
        wrapper = new IMUWrapper();
        wrapper.Initialize(telemetry,hardwareMap,gamepad1, gamepad2);
        joystickWrapper = new JoystickWrapper(gamepad1,gamepad2);
        armWrapper = new NeoArmWrapper(telemetry,hardwareMap,gamepad2,gamepad2);
        armWrapper.ResetMotorPositions();
        autoDropOffController = new AutoDropOffController(wrapper, armWrapper);
        autoDropOffController.initAprilTag(hardwareMap);
        waitForStart();



        while(!isStopRequested()){




            wrapper.Update();
            armWrapper.SetWheelSpin(gamepad2.left_trigger-gamepad2.right_trigger);
            armWrapper.UpdateIntakePower(gamepad1.right_trigger-gamepad1.left_trigger);
            armWrapper.UpdateExtensionPlusInput(joystickWrapper, 300, 300);


            if(joystickWrapper.gamepad1GetX()) {
                autoDropOffController.setDropLevel1();
            }

            if(joystickWrapper.gamepad1GetB()) {
                autoDropOffController.setDropLevel2();
            }

            if(joystickWrapper.gamepad2GetDUp()) {
                autoDropOffController.setNextDropLevel();
            }

            if(joystickWrapper.gamepad2GetDDown()) {
                autoDropOffController.setPreviousDropLeve();
            }


            if(joystickWrapper.gamepad2GetLeftBumperDown()) {
                autoDropOffController.setPreviousDropLocation();
            }

            if(joystickWrapper.gamepad2GetRightBumperDown()) {
                autoDropOffController.setNextDropLocation();
            }

            autoDropOffController.telemetryAprilTag(telemetry, joystickWrapper, new Encoder(hardwareMap.get(DcMotorEx.class, "fL")),  new Encoder(hardwareMap.get(DcMotorEx.class, "bL")));

            if(joystickWrapper.gamepad2GetY()){
                armWrapper.setOuttake();
            }
            /*
            if(joystickWrapper.gamepad2GetB()){
                armWrapper.MoveExtensionMotors(1000);
            }
*/
            if(joystickWrapper.gamepad2GetA()){
                armWrapper.setIntake();
            }

            /*
            if(joystickWrapper.gamepad2GetDDown()){
                armWrapper.MoveExtensionMotors(5);
            }
            if(joystickWrapper.gamepad2GetDRight()){
                armWrapper.MoveExtensionMotors(1500);
            }
            if(joystickWrapper.gamepad2GetDRight()){
                armWrapper.MoveExtensionMotors(2700);
            }
            if(joystickWrapper.gamepad2GetDLeft()){
                armWrapper.MoveExtensionMotors(1750);
            }*/
            if(joystickWrapper.gamepad1GetRightBumperDown()){
                if(isOpen){
                    armWrapper.ClosePos();
                    isOpen = false;
                }else {
                    armWrapper.OpenPos();
                    isOpen = true;
                }
            }
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

        autoDropOffController.close();
    }
}
