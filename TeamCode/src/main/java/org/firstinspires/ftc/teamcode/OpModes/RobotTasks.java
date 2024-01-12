package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.controllers.IRobotTask;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskParallel;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskSeries;
import org.firstinspires.ftc.teamcode.controllers.ServoTask;
import org.firstinspires.ftc.teamcode.wrappers.DrivingWrapper;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

import java.util.ArrayList;

@TeleOp
public class RobotTasks extends LinearOpMode {

    JoystickWrapper joystickWrapper;
    Servo ServoRearLift;
    Servo ServoRearWristVertical;
    Servo ServoRearClaw;

    Servo ServoFrontLift;
    Servo ServoFrontWristVertical;
    Servo ServoFrontWristHorizontal;
    Servo ServoFrontClaw;




    ArrayList<IRobotTask> tasks = new ArrayList<IRobotTask>();

    IRobotTask bottomPickup(long delay) {
        RobotTaskParallel parallel = new RobotTaskParallel();
        parallel.add(new ServoTask(ServoRearLift, 1, delay, "rearLift"));
        parallel.add(new ServoTask(ServoRearWristVertical, .4, delay, "rearLift"));
        parallel.add(new ServoTask(ServoRearClaw, .45, delay, "rearLift"));
        return parallel;
    }


    IRobotTask ReadyDropOff(Long delay){
        RobotTaskParallel parallel = new RobotTaskParallel();

        parallel.add(new ServoTask(ServoFrontLift, .55, delay, "rearLift"));
        parallel.add(new ServoTask(ServoFrontWristVertical, .7, delay, "rearLift"));
        parallel.add(new ServoTask(ServoFrontWristHorizontal, .45, delay, "rearLift"));

        return parallel;
    }
    IRobotTask DropOff(Long delay){
        RobotTaskSeries series = new RobotTaskSeries();

        series.add(new ServoTask(ServoFrontClaw, 0, 1000, "rearLift"));

        RobotTaskParallel parallel = new RobotTaskParallel();
        parallel.add(new ServoTask(ServoRearClaw, .65, 500, ""));
        parallel.add(new ServoTask(ServoFrontLift, 0, 600, ""));
        parallel.add(new ServoTask(ServoFrontWristVertical, 0.84, 600, ""));
        parallel.add(new ServoTask(ServoFrontWristHorizontal, 0.45, 600, ""));
        parallel.add(new ServoTask(ServoFrontClaw, 1, 600, ""));

        series.add(parallel);

        return series;
    }

    IRobotTask mediumPickup(long delay) {
        RobotTaskParallel parallel = new RobotTaskParallel();
        parallel.add(new ServoTask(ServoRearLift, .77, delay, "rearLift"));
        parallel.add(new ServoTask(ServoRearWristVertical, .55, delay, "rearLift"));
        parallel.add(new ServoTask(ServoRearClaw, .45, delay, "rearLift"));
        return parallel;
    }

    IRobotTask highPickup(long delay) {
        RobotTaskParallel parallel = new RobotTaskParallel();
        parallel.add(new ServoTask(ServoRearLift, .74, delay, ""));
        parallel.add(new ServoTask(ServoRearWristVertical, .55, delay, ""));
        parallel.add(new ServoTask(ServoRearClaw, .45, delay, ""));
        return parallel;
    }

    IRobotTask transfer() {
        RobotTaskSeries series = new RobotTaskSeries();

        //series.add(new ServoTask(ServoRearClaw, .65, 500, ""));

        RobotTaskParallel parallelFront = new RobotTaskParallel();
        parallelFront.add(new ServoTask(ServoRearClaw, .65, 500, ""));
        parallelFront.add(new ServoTask(ServoFrontLift, 0, 600, ""));
        parallelFront.add(new ServoTask(ServoFrontWristVertical, 0.84, 600, ""));
        parallelFront.add(new ServoTask(ServoFrontWristHorizontal, 0.45, 600, ""));
        parallelFront.add(new ServoTask(ServoFrontClaw, 0, 600, ""));
        series.add(parallelFront);

        RobotTaskParallel parallel = new RobotTaskParallel();
        parallel.add(new ServoTask(ServoRearLift, 0, 2000, ""));
        parallel.add(new ServoTask(ServoRearWristVertical, .65, 2000, ""));

        series.add(parallel);

        series.add(new ServoTask(ServoRearClaw, .45, 2000, ""));
        series.add(new ServoTask(ServoFrontClaw, 1, 300,""));
        series.add(new ServoTask(ServoRearLift, .25, 200, ""));

        return series;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        ServoRearLift = hardwareMap.servo.get("rearLift");
        ServoRearWristVertical = hardwareMap.servo.get("rearWristVertical");
        ServoRearClaw = hardwareMap.servo.get("rearClaw");

        ServoFrontLift = hardwareMap.servo.get("frontLift");
        ServoFrontWristVertical = hardwareMap.servo.get("frontWristVertical");
        ServoFrontWristHorizontal = hardwareMap.servo.get("frontWristHorizontal");
        ServoFrontClaw = hardwareMap.servo.get("frontClaw");

        joystickWrapper = new JoystickWrapper(gamepad1, gamepad2);

        DrivingWrapper drivingWrapper = new DrivingWrapper(hardwareMap, telemetry);
        int servoCounter = 0;

        ArrayList<Servo> servoArray = new ArrayList<Servo>();
        ArrayList<String> servoStringArray = new ArrayList<String>();
        servoArray.add(ServoRearLift);
        servoArray.add(ServoRearWristVertical);
        servoArray.add(ServoRearClaw);
        servoArray.add(ServoFrontLift);
        servoArray.add(ServoFrontWristVertical);
        servoArray.add(ServoFrontWristHorizontal);
        servoArray.add(ServoFrontClaw);
        servoStringArray.add("rearLift");
        servoStringArray.add("rearWristVertical");
        servoStringArray.add("rearClaw");
        servoStringArray.add("frontLift");
        servoStringArray.add("frontWristVertical");
        servoStringArray.add("frontWristHorizontal");
        servoStringArray.add("frontClaw");




        waitForStart();
        while (!isStopRequested()) {

            drivingWrapper.Drive(joystickWrapper, 1, 1);
            if(joystickWrapper.gamepad2GetDUp()){

            } else if (joystickWrapper.gamepad1GetDDown()) {

            }

            if(joystickWrapper.gamepad1GetDRight()) {

            }
            if (joystickWrapper.gamepad1GetLeftStickDown()){

            }
            telemetry.addData("Current Servo: ",servoStringArray.get(servoCounter));
            telemetry.addData("position",  servoArray.get(servoCounter).getPosition());

            if(joystickWrapper.gamepad1GetDLeft()) {
                tasks.add(ReadyDropOff(300L));
            }

            if(joystickWrapper.gamepad1GetRightBumperDown()) {
                servoArray.get(servoCounter).setPosition(servoArray.get(servoCounter).getPosition()+.05);

                telemetry.addData("position",  servoArray.get(servoCounter).getPosition());
                telemetry.update();
            }

            if(joystickWrapper.gamepad1GetLeftBumperDown()) {
                tasks.add(DropOff(1L));
            }



            if(joystickWrapper.gamepad1GetX()) {
                tasks.add(highPickup(1000));
            }

            if(joystickWrapper.gamepad1GetB()) {
                tasks.add(mediumPickup(1000));
            }


            if(joystickWrapper.gamepad1GetA()) {
                tasks.add(bottomPickup(1000));
            }

            if(joystickWrapper.gamepad1GetY()) {
                tasks.add(transfer());
            }

            //telemetry.addData("taskSize", tasks.size());

            if(tasks.size()>0) {

                boolean isStarted =  tasks.get(0).hasStarted();
                boolean isRunning =  tasks.get(0).isRunning();
                boolean isComplete =  tasks.get(0).isComplete();

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

            telemetry.update();
        }

    }
}
