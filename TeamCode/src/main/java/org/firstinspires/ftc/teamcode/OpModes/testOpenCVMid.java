package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.wrappers.OpenCvDetection;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class testOpenCVMid extends LinearOpMode {


    OpenCvWebcam webcam;
    Point loc;


    int barcodeInt;
    Mat cvtMat = new Mat();
    Mat mask = new Mat();
    Mat hierarchy = new Mat();

    int cameraHeight = 600;
    int cameraWidth = 800;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new TestPipline());
        webcam.setMillisecondsPermissionTimeout(5000);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the camera to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Also, we specify the rotation that the camera is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(cameraWidth, cameraHeight, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("Error", "openCameraDeviceAsync");
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        waitForStart();
        while (!isStopRequested()) {
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();
        }
    }
    public class TestPipline extends OpenCvPipeline
    {
        boolean viewportPaused = false;

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */


/*
            Scalar minValues = new Scalar(22, 93, 0);
            Scalar maxValues = new Scalar(45, 255, 255);
*/



            //Scalar minValues = new Scalar(0,0,0);
            //Scalar maxValues = new Scalar(255,255,164);
            Imgproc.cvtColor(input, cvtMat, Imgproc.COLOR_RGB2HSV);
            Imgproc.GaussianBlur(cvtMat,cvtMat,new Size(3,3),0);


            Scalar minValues = new Scalar(23, 50, 70);
            Scalar maxValues = new Scalar(32, 255, 255);



            Core.inRange(cvtMat, minValues, maxValues, mask);


            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);


            //MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
            //Rect[] boundRect = new Rect[contours.size()];
            //Point[] centers = new Point[contours.size()];
            //float[][] radius = new float[contours.size()][1];
            for (int i = 0; i < contours.size(); i++) {
                MatOfPoint2f contoursPoly = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly, 3, true);
                Rect boundRect = Imgproc.boundingRect(new MatOfPoint(contoursPoly.toArray()));
                Point center = new Point();
                float[] radius = new float[1];
                Imgproc.minEnclosingCircle(contoursPoly, center, radius);
                if(boundRect.size().width>50.0){
                    loc = center;
                    if (loc.x<(cameraWidth/3)){
                        barcodeInt = 1;
                    }else if (loc.x<((cameraWidth*2)/3)){
                        barcodeInt = 2;
                    }else {
                        barcodeInt = 3;
                    }
                    telemetry.addData("Test Barcode:", barcodeInt);
                    telemetry.addData("Location", loc);
                    telemetry.addData("Size",boundRect.size());
                    telemetry.update();

                    contoursPoly.release();


                    break;
                }
            }

            //cvtMat.release();
            //input.release();
            //hierarchy.release();

            return mask;


            //return input;
        }


        public int GetBarcodeInt(){
            return barcodeInt;
        }


        @Override
        public void onViewportTapped()
        {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
    }
    public void StopCameraStream(){
        webcam.stopStreaming();
    }
}





