package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "Drive")

public class drive extends OpMode {
    // Declaring the motors
    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor intake;
    DcMotor arm;

    // Declaring the servos
    Servo plane;
    Servo claw;



    

    @Override
    public void init() {
        // map motors to their names in the driver hub
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        arm = hardwareMap.get(DcMotor.class, "arm");
        plane = hardwareMap.get(Servo.class,"plane");
        claw = hardwareMap.get(Servo.class, "claw");

        // configure each motor to stop like a brake, and run using the encoder.
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }



    @Override
    public void loop() {

        // This is adding the power in the motors
        double intakePower = (gamepad1.left_trigger - gamepad1.right_trigger) / 1.01;
        double y = gamepad1.left_stick_y;
        double x = gamepad1.right_stick_x;
        double leftPower = (-y - x) / 1.1;
        double rightPower = (y - x) / 1.1;

        // This is showing the power in the screen
        telemetry.addData("LeftT Power:", gamepad1.left_trigger);
        telemetry.addData("RightT Power:", gamepad1.right_trigger);
        telemetry.addData("intake:", gamepad1.left_bumper);
        telemetry.addData("Intake Power:", intakePower);
        intake.setPower(intakePower);
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        // This is to utilize the arm
        if (gamepad2.circle) {
            arm.setPower(0.9);
        } else if (gamepad2.triangle) {
            arm.setPower(-0.9);
        } else {
            arm.setPower(0);
        }


        // This is to launch the plane
        if (gamepad2.dpad_up) {
            plane.setPosition(0.5);
        }

        if (gamepad2.dpad_down) {
            plane.setPosition(-0.5);
        }

        // This is to use the claw
        if (gamepad2.right_bumper)
            claw.setPosition(0.75);

        if (gamepad2.left_bumper)
            claw.setPosition(0.25);
    }
}
    