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

    boolean isDrop;

    DcMotorEx ActuatorMotorEx;


    ArrayList<IRobotTask> tasks = new ArrayList<IRobotTask>();


    @Override
    public void runOpMode() throws InterruptedException {
        wrapper = new IMUWrapper();
        wrapper.Initialize(telemetry,hardwareMap,gamepad1, gamepad2);

        waitForStart();



        joystickWrapper = new JoystickWrapper(gamepad1,gamepad2);

        armWrapper = new NeoArmWrapper(telemetry,hardwareMap,gamepad2,gamepad2);
        while(!isStopRequested()){
            wrapper.Update();
            armWrapper.ManualExtention(joystickWrapper,true,10);
            armWrapper.UpdateIntakePower(gamepad2.right_trigger-gamepad2.left_trigger);
            //armWrapper.MoveMotorWithTelemetry(Math.round((gamepad2.right_trigger-gamepad2.left_trigger)*100));
            armWrapper.ResetMotorPositions(); 
            if(joystickWrapper.gamepad1GetB()){
                armWrapper.MoveExtensionMotors(1000);

            }

            if(joystickWrapper.gamepad1GetA()){
                armWrapper.MoveExtensionMotors(0);
                armWrapper.MoveActuatorMotor(0);
            }
            if(joystickWrapper.gamepad1GetDDown()){
                armWrapper.MoveExtensionMotors(0);
            }
            if(joystickWrapper.gamepad1GetDRight()){
                armWrapper.MoveExtensionMotors(5250);
            }
            if(joystickWrapper.gamepad1GetDRight()){
                armWrapper.MoveExtensionMotors(3500);
            }
            if(joystickWrapper.gamepad1GetDLeft()){
                armWrapper.MoveExtensionMotors(1750);
            }

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
