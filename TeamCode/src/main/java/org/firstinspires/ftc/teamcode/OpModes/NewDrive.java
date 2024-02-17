package org.firstinspires.ftc.teamcode.OpModes;

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
import org.firstinspires.ftc.teamcode.wrappers.ArmWrapper;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;
import org.firstinspires.ftc.teamcode.wrappers.NeoArmWrapper;

import java.util.ArrayList;

@TeleOp
public class NewDrive extends LinearOpMode {
    IMUWrapper wrapper;

    NeoArmWrapper armWrapper;

    JoystickWrapper joystickWrapper;

    boolean isDown;
    boolean isOpen;

    DcMotorEx ActuatorMotorEx;


    ArrayList<IRobotTask> tasks = new ArrayList<IRobotTask>();


    @Override
    public void runOpMode() throws InterruptedException {
        wrapper = new IMUWrapper();
        wrapper.Initialize(telemetry,hardwareMap,gamepad1, gamepad2);
        joystickWrapper = new JoystickWrapper(gamepad1,gamepad2);
        armWrapper = new NeoArmWrapper(telemetry,hardwareMap,gamepad2,gamepad2);
        armWrapper.ResetMotorPositions();
        waitForStart();





        while(!isStopRequested()){
            wrapper.Update();
            armWrapper.SetWheelSpin(gamepad2.left_trigger-gamepad2.right_trigger);
            armWrapper.UpdateIntakePower(gamepad1.right_trigger-gamepad1.left_trigger);
            armWrapper.UpdateExtensionPlusInput(joystickWrapper, 100);

            /*if(joystickWrapper.gamepad2GetX()){
                armWrapper.MoveExtensionMotors(1000);
            }
            */


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
            telemetry.addData("Extension2 Pos", armWrapper.ExtensionMotorEx2.getCurrentPosition());
            telemetry.addData("Actuator Target Pos", armWrapper.ActuatorMotorEx.getTargetPosition());
            telemetry.addData("Extension1 Target Pos", armWrapper.ExtensionMotorEx1.getTargetPosition());
            telemetry.addData("Extension2 Target Pos", armWrapper.ExtensionMotorEx2.getTargetPosition());
            if(tasks.size()>0) {

                boolean isStarted = tasks.get(0).hasStarted();
                boolean isRunning = tasks.get(0).isRunning();
                boolean isComplete = tasks.get(0).isComplete();

                telemetry.addData("isStarted", isStarted);
                telemetry.addData("isRunning", isRunning);
                telemetry.addData("isComplete", isComplete);

                tasks.get(0).execute(telemetry);




                if(isComplete){
                    tasks.remove(0);
                    telemetry.addData("taskSize completed", tasks.size());
                }
                //

                //telemetry.update();

            }
        }
    }
}
