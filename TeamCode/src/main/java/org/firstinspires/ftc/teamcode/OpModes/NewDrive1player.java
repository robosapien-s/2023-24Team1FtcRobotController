package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.controllers.IRobotTask;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;
import org.firstinspires.ftc.teamcode.wrappers.NeoArmWrapper;

import java.util.ArrayList;

@TeleOp
public class NewDrive1player extends LinearOpMode {
    IMUWrapper wrapper;

    NeoArmWrapper armWrapper;

    JoystickWrapper joystickWrapper;

    boolean isDown;
    boolean isOpen;
    float speed = 0;

    DcMotorEx ActuatorMotorEx;


    ArrayList<IRobotTask> tasks = new ArrayList<IRobotTask>();


    @Override
    public void runOpMode() throws InterruptedException {
        wrapper = new IMUWrapper();
        wrapper.Initialize(telemetry,hardwareMap,gamepad1, gamepad2);
        joystickWrapper = new JoystickWrapper(gamepad1,gamepad2);
        armWrapper = new NeoArmWrapper(telemetry,hardwareMap,gamepad2,gamepad2,false);
        armWrapper.ResetMotorPositions();
        waitForStart();





        while(!isStopRequested()){
            wrapper.Update();
            if(gamepad1.dpad_left){
                if(gamepad1.dpad_right){
                    speed = 0;
                }else {
                    speed = -1;
                }
            }else if (gamepad1.dpad_right){
                speed = 1;
            }
            armWrapper.SetWheelSpin(speed);
            armWrapper.UpdateIntakePower(gamepad1.right_trigger-gamepad1.left_trigger);
            armWrapper.UpdateExtensionPlusInput(joystickWrapper, 200, 200);



            if(joystickWrapper.gamepad1GetY()){
                armWrapper.setOuttake();
            }

            if(joystickWrapper.gamepad1GetA()){
                armWrapper.setIntake();
            }

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
