package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous (name = "auton")
public class auton extends LinearOpMode {
    DcMotor leftdrive;
    DcMotor rightdrive;
    DcMotor intake;




    @Override
    public void  runOpMode() {
        leftdrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightdrive = hardwareMap.get(DcMotor.class, "rightDrive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        waitForStart();
        moveForward();
        sleep(250);
        stopMotors();
        sleep(2000);
        turnRight();
        sleep(200);
        stopMotors();
        sleep(1000);
        moveForward();
        sleep(300);
        stopMotors();
        sleep(1000);
        Outtake();
        sleep(700);
        stopIntake();
        sleep(1000);
        moveBackward();
        sleep(330);
        stopMotors();
        sleep(1000);
        turnLeft();
        sleep(500);
        stopMotors();
        sleep(1000);
        moveForward();
        sleep(500);
        stopMotors();
        sleep(1000);
        Outtake();
        sleep(1000);
        moveBackward();
        sleep(250);
        stopMotors();
        sleep(1000);
        turnLeft();
        sleep(200);
        stopMotors();
        moveBackward();
        sleep(700);
        stopMotors();

    }
    public void Outtake() {
        intake.setPower(0.5);
    }
    public void stopIntake() {
        intake.setPower(0);
    }
    public void stopMotors() {
        leftdrive.setPower(0);
    }
    public void moveForward() {
        leftdrive.setPower(-0.6);
        rightdrive.setPower(0.5);
    }
    public void turnRight() {
        leftdrive.setPower(-0.6);
        rightdrive.setPower(-0.5);
    }
    public void turnLeft() {
        leftdrive.setPower(0.6);
        rightdrive.setPower(0.5);
    }
    public void moveBackward() {
        leftdrive.setPower(0.6);
        rightdrive.setPower(-0.5);
    }
    public void setMotorPowers(double left, double right){
        leftdrive.setPower(left);
        rightdrive.setPower(right);
    }
}