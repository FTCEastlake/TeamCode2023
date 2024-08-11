package org.firstinspires.ftc.teamcode;




import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name="Near Blue")


public class NearBlue extends LinearOpMode {
    private OpenCvCamera webcam;


    private static final int CAMERA_WIDTH  = 1280; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution




    public static double borderLeftX    = 0.0;   //fraction of pixels from the left side of the cam to skip
    public static double borderRightX   = 0.0;   //fraction of pixels from the right of the cam to skip
    public static double borderTopY     = 0.0;   //fraction of pixels from the top of the cam to skip
    public static double borderBottomY  = 0.0;   //fraction of pixels from the bottom of the cam to skip






    // Pink Range                                      Y      Cr     Cb
//    public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 160.0, 100.0);
//    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 255.0);


    // Yellow Range
    //--public static Scalar scalarLowerYCrCb = new Scalar(0.0, 100.0, 0.0);
//  public static Scalar scalarUpperYCrCb = new Scalar(255.0, 170.0, 120.0);


    // Blue Range
    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 0.0, 128.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 120.0, 255.0);


    // Red Range
    //public static Scalar scalarLowerYCrCb = new Scalar(0.0, 160.0, 100.0);
    //public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 255.0);


    // declare motors
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor intake;
    Servo clawHand;
    private final ElapsedTime runtime = new ElapsedTime();


    //measurements in inches
    double wheelDiam = 3.75;
    double robotDiam = 16.0;
    double robotCircumference = (robotDiam* Math.PI);




    //encoder resolution
    double ticks = 537.7;


    //inverse gear ratio of sprockets
    double reduction = 0.714285;
    double counts_per_inch = (ticks * reduction)/(wheelDiam * Math.PI);


    boolean notMoved = true;




    @Override
    public void runOpMode()
    {
        // setup motors
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        intake = hardwareMap.get(DcMotor.class, "intake");


        //servos
        clawHand = hardwareMap.get(Servo.class, "servo1");




        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);




        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Goofy Cam"), cameraMonitorViewId);
        //OpenCV Pipeline
        ContourPipeline myPipeline;
        webcam.setPipeline(myPipeline = new ContourPipeline(borderLeftX,borderRightX,borderTopY,borderBottomY));
        // Configuration of Pipeline
        myPipeline.configureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        myPipeline.configureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);
        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }


            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });


        telemetry.update();
        waitForStart();


        myPipeline.configureBorders(borderLeftX, borderRightX, borderTopY, borderBottomY);
        if(myPipeline.error){
            telemetry.addData("Exception: ", myPipeline.debug);
        }


        clawHand.setPosition(0.15);
        encoderDrive(0.1, 16, 16, 10);


        if (myPipeline.getRectHeight() >= 125 && notMoved) {
            AUTONOMOUS_B();
            notMoved = false;
            telemetry.addData("RectHeight: ", myPipeline.getRectHeight());
            telemetry.update();
        }


        if (notMoved) {
            encoderDrive(0.1, -robotCircumference / 8, robotCircumference / 8, 5);
            sleep(100);
        }


        if (myPipeline.getRectHeight() > 400 && notMoved) {
            AUTONOMOUS_C();
            notMoved = false;
            telemetry.addData("RectHeight: ", myPipeline.getRectHeight());
            telemetry.update();
        }


        if (notMoved) {
            encoderDrive(0.1, robotCircumference / 4, -robotCircumference / 4, 5);
            sleep(100);
        }


        if (notMoved) {
            notMoved = false;
            AUTONOMOUS_A();
        }

        //FOR NEAR USE ONLY, DRIVES INTO BACKSTAGE
        encoderDrive(0.1, 5, 5, 5);
        //left motor negative to turn left, switch left/right motor signs to turn right
        encoderDrive(0.1, -robotCircumference/4, robotCircumference/4, 5);
        encoderDrive(0.2, -40, -40, 10);
        clawHand.setPosition(0);
        sleep(500);
    }


    public void AUTONOMOUS_A(){
        telemetry.addData("dir", "right");
        telemetry.update();
        encoderDrive(0.1, 8, 8, 7);
        sleep(500);
        dropOffPixel();
        encoderDrive(0.1, -3, -3, 5);
        encoderDrive(0.1, -robotCircumference/8, robotCircumference/8, 5);
        encoderDrive(0.1, -16, -16, 5);
    }
    public void AUTONOMOUS_B(){
        telemetry.addData("dir", "center");
        telemetry.update();
        encoderDrive(0.1, 14, 14, 10);
        sleep(500);
        dropOffPixel();
        encoderDrive(0.1, -31, -31, 10);
    }
    public void AUTONOMOUS_C(){
        telemetry.addData("dir", "left");
        telemetry.update();
        encoderDrive(0.1, 8, 8, 5);
        sleep(500);
        dropOffPixel();
        encoderDrive(0.1, -3, -3, 5);
        encoderDrive(0.1, robotCircumference/8, -robotCircumference/8, 5);
        encoderDrive(0.1, -16, -16, 5);
    }


    public void dropOffPixel() {
        intake.setPower(0.25);
        encoderDrive(0.025, -5, -5, 5);
        sleep(500);
        intake.setPower(0);
    }


    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {




        int newLeftTarget;
        int newRightTarget;




        if (opModeIsActive()) {




            newLeftTarget = (leftDrive.getCurrentPosition() + (int)(leftInches * counts_per_inch));
            newRightTarget = (rightDrive.getCurrentPosition() + (int)(rightInches * counts_per_inch));
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);




            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);




            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));




            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {
                telemetry.update();
            }




            leftDrive.setPower(0);
            rightDrive.setPower(0);




            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




            sleep(250);
        }
    }
}
