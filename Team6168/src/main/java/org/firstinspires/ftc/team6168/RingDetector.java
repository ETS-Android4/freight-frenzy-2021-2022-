package org.firstinspires.ftc.team6168;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class RingDetector {
    OpMode opMode;
    OpenCvCamera camera;

    AddBoxesPipeline pipeline;

    private final Point centerBox_topLeft    = new Point(130,60);
    private final Point centerBox_bottomRight    = new Point(180, 110);

    private final Point leftBox_topLeft    = new Point(0,60);
    private final Point leftBox_bottomRight    = new Point(30, 110);

    private final Point rightBox_topLeft    = new Point(280,60);
    private final Point rightBox_bottomRight    = new Point(320, 110);

    private Point CTL;
    private Point CBR;


    private Point LTL;
    private Point LBR;

    private Point RTL;
    private Point RBR;

    private RGBColor box;

    public RingDetector(OpMode op){

        opMode = op;

        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        pipeline = new AddBoxesPipeline();
        camera.openCameraDevice();
        camera.setPipeline(pipeline);
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        CTL = centerBox_topLeft;
        CBR = centerBox_bottomRight;

        LTL = leftBox_topLeft;
        LBR = leftBox_bottomRight;

        RTL = rightBox_topLeft;
        RBR = rightBox_bottomRight;
    }

    public void stopStreaming(){
        camera.stopStreaming();
    }


    public String getDecision(double leftBoxColor, double centerBoxColor, double rightBoxColor){
        opMode.telemetry.addData("left", String.valueOf(leftBoxColor), "center", centerBoxColor,"right", rightBoxColor);
        opMode.telemetry.update();
        if((leftBoxColor > centerBoxColor) && (leftBoxColor > rightBoxColor)) {
            return "left";
        } else if((leftBoxColor > centerBoxColor) && (leftBoxColor > rightBoxColor)){
            return "center";
            } else{
            return "right";
        }
    }

    //Add boxes to the image display
    class AddBoxesPipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input){

            int leftBoxColor = getAverageColor(input, leftBox_topLeft, leftBox_bottomRight);
            int centerBoxColor = getAverageColor(input, centerBox_topLeft, centerBox_bottomRight);
            int rightBoxColor = getAverageColor(input, rightBox_topLeft, rightBox_bottomRight);

            int thickness = 3;
            Scalar leftColor = new Scalar(255,0,0);
            Scalar centerColor = new Scalar(255,0,0);
            Scalar rightColor = new Scalar(255,0,0);

            String position = getDecision(leftBoxColor, centerBoxColor, rightBoxColor);
            if (position == "left"){
                leftColor = new Scalar(0,255,0);
            }
            else if (position == "center"){
                centerColor = new Scalar(0,255,0);
            } else {
                rightColor = new Scalar(0,255,0);
            }
//
//            Imgproc.rectangle(input, leftBox_topLeft, leftBox_bottomRight, leftColor, thickness);
//            Imgproc.rectangle(input, centerBox_topLeft, centerBox_bottomRight, centerColor, thickness);
//            Imgproc.rectangle(input, rightBox_topLeft, rightBox_bottomRight, rightColor, thickness);

            Imgproc.rectangle(input, LTL, LBR, leftColor, thickness);
            Imgproc.rectangle(input, CTL, CBR, centerColor, thickness);
            Imgproc.rectangle(input, RTL, RBR, rightColor, thickness);

            //sendTelemetry();

            return input;
        }

        private int getAverageColor(Mat mat, Point topLeft, Point bottomRight){
            int red = 0;
            int green = 0;
            int blue = 0;
            int yellow = 0;
            int total = 0;

            for (int x = (int)topLeft.x; x < bottomRight.x; x++){
                for (int y = (int)topLeft.y; y < bottomRight.y; y++){
                    red += mat.get(y,x)[0];
                    green += mat.get(y,x)[1];
                    blue += mat.get(y,x)[2];
                    total++;
                }
            }

            red /= total;
            green /= total;
            blue /= total;
            yellow = red + green;

            opMode.telemetry.addData("red", String.valueOf(red));
            opMode.telemetry.addData("green", String.valueOf(green));
            opMode.telemetry.addData("blue", String.valueOf(blue));
            opMode.telemetry.addData("yellow", String.valueOf(yellow));

            return (yellow);


        }



    }
}