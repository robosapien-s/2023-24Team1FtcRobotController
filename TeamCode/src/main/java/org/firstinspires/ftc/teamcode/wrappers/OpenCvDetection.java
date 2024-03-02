package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
//import org.outoftheboxrobotics.tensorflowapi.ImageClassification.TensorImageClassifier;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;


// This is a comment LOL ¯\_(ツ)_/¯

/* this is also a comment


and also this ¯\_(ツ)_/¯
 */
public class OpenCvDetection {

    String[] position = {"LEFT","MIDDLE","RIGHT"};

    OpenCvWebcam webcam;
    Point loc;

    HardwareMap hardwareMap;
    Telemetry telemetry;
    boolean red;

    public int barcodeInt;

    public OpenCvDetection(Telemetry inTelemetry, HardwareMap inHardwareMap){
        telemetry = inTelemetry;
        hardwareMap = inHardwareMap;
    }

    Mat cvtMat = new Mat();
    Mat mask = new Mat();
    Mat mask1 = new Mat();
    Mat mask2 = new Mat();
    Mat hierarchy = new Mat();

    int cameraHeight = 600;
    int cameraWidth = 800;

    public void initColor(boolean red) {
        this.red = red;
    }

    public void init() {

        int cameraMonitorViewId =  hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        webcam.setPipeline(new OpenCvPl());

        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
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
    }

    public class OpenCvPl extends OpenCvPipeline
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


            Scalar minValuesRed1 = new Scalar(159, 70, 50);
            Scalar minValuesRed2 = new Scalar(0, 70, 50);
            Scalar maxValuesRed1 = new Scalar(180, 255, 255);
            Scalar maxValuesRed2 = new Scalar(10, 255, 255);
            Scalar minValuesBlue = new Scalar(85, 40, 60);
            Scalar maxValuesBlue = new Scalar(135, 255, 255);

            Imgproc.cvtColor(input, cvtMat, Imgproc.COLOR_RGB2HSV);
            Imgproc.GaussianBlur(cvtMat,cvtMat,new Size(3,3),0);
            if (red) {
                Core.inRange(cvtMat, minValuesRed1, maxValuesRed1, mask1);
                Core.inRange(cvtMat, minValuesRed2, maxValuesRed2, mask2);
                Core.bitwise_or(mask1,mask2,mask);
                telemetry.addData("Color: ", "Red");
            } else {
                Core.inRange(cvtMat, minValuesBlue, maxValuesBlue, mask);
                telemetry.addData("Color: ", "Blue");
            }




            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);


            //MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
            //Rect[] boundRect = new Rect[contours.size()];
            //Point[] centers = new Point[contours.size()];
            //float[][] radius = new float[contours.size()][1];
            Rect bigBoundRect = new Rect(0,0,0,0);
            ArrayList<Rect> rects = new ArrayList<>();

            for (int i = 0; i < contours.size(); i++) {
                MatOfPoint2f contoursPoly = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly, 3, true);
                Rect boundRect = Imgproc.boundingRect(new MatOfPoint(contoursPoly.toArray()));
                rects.add(boundRect);
                Point center = new Point();
                float[] radius = new float[1];
                Imgproc.minEnclosingCircle(contoursPoly, center, radius);
                if (boundRect.area()> bigBoundRect.area()) {
                    bigBoundRect = boundRect;
                }

            }
            rects.sort(new Comparator<Rect>() {
                @Override
                public int compare(Rect rect, Rect t1) {
                    return Double.compare(rect.area(), t1.area());
                }
            });

            loc = new Point(bigBoundRect.x+(bigBoundRect.width)/2, bigBoundRect.y - (bigBoundRect.height/2));

            if (loc.x<(cameraWidth/3)){
                barcodeInt = 1;
            }else if (loc.x<((cameraWidth*2)/3)){
                barcodeInt = 2;
            }else {
                barcodeInt = 3;
            }
            telemetry.addData("Test Barcode:", barcodeInt);
            telemetry.addData("Position", position[barcodeInt-1]);
            telemetry.addData("Location", loc);
            telemetry.addData("Size",bigBoundRect.size());
            telemetry.update();




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

    public void switchColor() {
        red = !red;
    }

    public void StopCameraStream(){
        webcam.stopStreaming();
    }
}