package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
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

    DcMotorEx slideMotor;
    CRServo ArmWheel;

    ArrayList<IRobotTask> tasks = new ArrayList<IRobotTask>();

    IRobotTask intakePos(long delay) {
        RobotTaskSeries series = new RobotTaskSeries();
        series.add(new ServoTask(armWrapper.wristServo, .22, 200, ""));
        series.add(new ServoTask(armWrapper.armServo, .25, delay, ""));

        return series;
    }
    IRobotTask dropPos(long delay) {
        RobotTaskSeries series = new RobotTaskSeries();
        series.add(new ServoTask(armWrapper.wristServo, .3, 200, ""));
        series.add(new ServoTask(armWrapper.armServo, .5, 500, ""));
        series.add(new ServoTask(armWrapper.wristServo, .55, delay, ""));
        series.add(new ServoTask(armWrapper.armServo, .8, 500, ""));

        return series;
    }
    IRobotTask drop(){
        RobotTaskSeries series = new RobotTaskSeries();
        series.add(new ServoTask(armWrapper.wristServo, .4,200,""));
        return series;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        wrapper = new IMUWrapper();
        wrapper.Initialize(telemetry,hardwareMap,gamepad1, gamepad2);

        waitForStart();

        ArmWheel = hardwareMap.get(CRServo.class,"ArmWheel");
        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        joystickWrapper = new JoystickWrapper(gamepad1,gamepad2);

        armWrapper = new NeoArmWrapper(telemetry,hardwareMap,gamepad2,gamepad2);
        while(!isStopRequested()){
            wrapper.Update();
            armWrapper.UpdateIntakePower(gamepad2.right_trigger-gamepad2.left_trigger);
            armWrapper.MoveMotorWithTelemetry(Math.round((gamepad2.right_trigger-gamepad2.left_trigger)*100));

            if(gamepad2.dpad_right){
                ArmWheel.setPower(-1);

            }else {
                ArmWheel.setPower(0);
            }

            slideMotor.setPower(joystickWrapper.gamepad2GetRightStickY());



            if(joystickWrapper.gamepad2GetRightBumperDown()){
                if (isDrop){
                    tasks.add(intakePos(0));
                    isDrop = false;
                }else{
                    tasks.add(dropPos(0));
                    isDrop = true;
                }

            }
            if (joystickWrapper.gamepad2GetLeftBumperDown()){
                tasks.add(drop());
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
