package Random;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class RobotAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private double xPos;
    private double yPos;
    private double rotaion;

    private DcMotor arm;

    private VoltageSensor batteryVoltageSensor;

    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "LFD");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RFD");
        leftBackDrive = hardwareMap.get(DcMotor.class, "LBD");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RBD");
        arm = hardwareMap.get(DcMotor.class, "arm");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        telemetry.addData("Status", "Initialized, Waiting for Start");
        telemetry.update();
        waitForStart();
        runtime.reset();


        moveForward(0.5, 2000);
        turnRight(0.5, 1000);
        moveArm(0.5, 1500);
        stopMotors();           
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    private void moveForward(double power, long time) {
        drive4WD(power, power);
        sleep(time);
    }

    private void turnRight(double power, long time) {
        drive4WD(power, -power);
        sleep(time);
    }

    private void moveArm(double power, long time) {
        arm.setPower(power);
        sleep(time);
        arm.setPower(0);  
    }


    private void stopMotors() {
        drive4WD(0, 0);
    }


    private void drive4WD(double leftPower, double rightPower) {
        leftFrontDrive.setPower(leftPower);
        leftBackDrive.setPower(leftPower);
        rightFrontDrive.setPower(rightPower);
        rightBackDrive.setPower(rightPower);
    }
}
