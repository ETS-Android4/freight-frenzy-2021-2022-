package org.firstinspires.ftc.team6168;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team6168.RGBColor;
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

    CustomPipeline pipeline;

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

        pipeline = new CustomPipeline();
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

    public int getDecision(){
        int boxValue = box.getYellow();
        opMode.telemetry.addData("Value: ", boxValue);
        opMode.telemetry.update();
        boolean topRing = false;
        boolean bottomRing = false;
        if (boxValue < 200) {
            return 4;


        } else if (boxValue < 290){
            return 1;

        } else{
            return 0;

        }
    }

    class CustomPipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input){

            box = getAverageColor(input, CTL, CBR);
            int thickness = 3;
            //Scalar topColor = new Scalar(255,0,0);
            Scalar Color = new Scalar(255,0,0);
            int position = getDecision();
            if (position == 4){
                Color = new Scalar(0,255,0);
            }
            else if (position == 1){
                Color = new Scalar(0,255,0);
            }

            Imgproc.rectangle(input, CTL, CBR, Color, thickness);

            Imgproc.rectangle(input, LTL, LBR, Color, thickness);

            Imgproc.rectangle(input, RTL, RBR, Color, thickness);

            //sendTelemetry();

            return input;
        }

        private RGBColor getAverageColor(Mat mat, Point topLeft, Point bottomRight){
            int red = 0;
            int green = 0;
            int blue = 0;
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

            return new RGBColor(red, green, blue);
        }

        public void sendTelemetry(){
            opMode.telemetry.addData("Top :" + " R " + box.getRed() + " G " + box.getGreen() + " B " + box.getBlue() + "Y" + box.getYellow() + "Decision: ", getDecision() );
            opMode.telemetry.update();
        }

    }
}