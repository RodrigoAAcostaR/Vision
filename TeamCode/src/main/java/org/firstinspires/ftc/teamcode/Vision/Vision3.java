package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class Vision3 extends OpMode {
    OpenCvCamera camera1 = null;

    List<MatOfPoint> contours;
    MatOfPoint2f approxCurve;
    Scalar lowerBound, upperBound;

    @Override
    public void init() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "camara1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("CameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        camera1.setPipeline(new ExamplePipeline());

        camera1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera1.startStreaming(720, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }

    @Override
    public void loop() {
        // No es necesario implementar el loop para este caso
    }

    // ...

    class ExamplePipeline extends OpenCvPipeline {
        Scalar rectColor = new Scalar(230.0, 10.0, 10.0);
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        Mat middleCrop;
        double leftavgfin;
        double rightavgfin;
        double middleavgfin;
        Mat outPut = new Mat();

        @Override
        public Mat processFrame(Mat input) {
           Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
           telemetry.addLine("pipeline running");

           Rect leftRect = new Rect(1, 1, 240, 439);
           Rect rightRect = new Rect(240, 1, 240, 439);
           Rect middleRect = new Rect(480, 1, 239, 439);

           input.copyTo(outPut);
           Imgproc.rectangle(outPut, leftRect, rectColor, 2);
           Imgproc.rectangle(outPut, rightRect, rectColor, 2);
           Imgproc.rectangle(outPut, middleRect, rectColor, 2);

           leftCrop = YCbCr.submat(leftRect);
           rightCrop = YCbCr.submat(rightRect);
           middleCrop = YCbCr.submat(middleRect);

           Core.extractChannel(leftCrop, leftCrop, 2);
           Core.extractChannel(rightCrop, rightCrop, 2);
           Core.extractChannel(middleCrop, middleCrop, 2);

           Scalar leftavg = Core.mean(leftCrop);
           Scalar rightavg = Core.mean(rightCrop);
           Scalar middleavg = Core.mean(middleCrop);

           leftavgfin = leftavg.val[0];
           rightavgfin = rightavg.val[0];
           middleavgfin = middleavg.val[0];


           if(leftavgfin > rightavgfin && leftavgfin > middleavgfin){
               telemetry.addLine("Left");
           }else if(rightavgfin > leftavgfin && rightavgfin > middleavgfin){
               telemetry.addLine("Right");
           }else{
               telemetry.addLine("Middle");
           }

            telemetry.addData("Middle", middleavgfin);
            telemetry.addData("Left", leftavgfin);
            telemetry.addData("Right", rightavgfin);

           return(outPut);
        }
    }
}