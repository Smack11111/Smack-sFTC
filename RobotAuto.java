package org.firstinspires.ftc23565;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class RobotAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    // 4WD motors
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    private DcMotor arm;  // Arm motor

    // Hardware for battery voltage
    private VoltageSensor batteryVoltageSensor;

    @Override
    public void runOpMode() {
        // Initialize motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        arm = hardwareMap.get(DcMotor.class, "arm");

        // Set motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Initialize battery sensor
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized, Waiting for Start");
        telemetry.update();
        waitForStart();
        runtime.reset();

        // Autonomous sequence
        moveForward(0.5, 2000); // Move forward at 50% power for 2 seconds
        turnRight(0.5, 1000);   // Turn right at 50% power for 1 second
        moveArm(0.5, 1500);      // Move arm up at 50% power for 1.5 seconds
        stopMotors();            // Stop all motors

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    // Method to move forward or backward
    private void moveForward(double power, long time) {
        drive4WD(power, power);
        sleep(time);
    }

    // Method to turn right
    private void turnRight(double power, long time) {
        drive4WD(power, -power);
        sleep(time);
    }

    // Method to move the arm
    private void moveArm(double power, long time) {
        arm.setPower(power);
        sleep(time);
        arm.setPower(0);  // Stop the arm
    }

    // Method to stop all motors
    private void stopMotors() {
        drive4WD(0, 0);
    }

    // Method to set power for all 4 motors
    private void drive4WD(double leftPower, double rightPower) {
        leftFrontDrive.setPower(leftPower);
        leftBackDrive.setPower(leftPower);
        rightFrontDrive.setPower(rightPower);
        rightBackDrive.setPower(rightPower);
    }
}
