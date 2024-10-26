package Main;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectionPipeline;
import java.util.List;

@Autonomous
public class RobotAutoWithPID extends LinearOpMode {
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private OpenCvCamera camera;
    private AprilTagDetectionPipeline aprilTagDetectionPipeline;
    private VoltageSensor batteryVoltageSensor;
    private AprilTagDetection tagOfInterest;

    // PID controllers
    private PIDController xPID = new PIDController(0.1, 0, 0);
    private PIDController yPID = new PIDController(0.1, 0, 0);
    private PIDController yawPID = new PIDController(0.05, 0, 0.01);

    private static final double FEET_PER_METER = 3.28084;
    private static final double tagsize = 0.1016;  // Tag size in meters (4 inches)
    private static final double fx = 578.272;     // Camera focal length x-axis
    private static final double fy = 578.272;     // Camera focal length y-axis
    private static final double cx = 402.145;     // Principal point x
    private static final double cy = 221.506;     // Principal point y
    private static final int TARGET_TAG_ID = 1;   // Target AprilTag ID

    @Override
    public void runOpMode() {
        initializeMotors();
        initializeCamera();

        telemetry.addData("Status", "Initialized, Waiting for Start");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            goToPosition(2.0, 2.0, 0);    // Example position (x, y, yaw)
            goToPosition(4.0, 2.0, 90);   // Example position (x, y, yaw)
            stopMotors();
        }
    }

    private void initializeMotors() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "LFD");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RFD");
        leftBackDrive = hardwareMap.get(DcMotor.class, "LBD");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RBD");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
    }

    private void initializeCamera() {
        // Get camera monitor view ID
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        
        // Create and initialize the webcam
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened(OpenCvCamera camera) {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(OpenCvCamera camera, int errorCode) {
                telemetry.addData("Camera Error", "Error code: " + errorCode);
                telemetry.update();
            }
        });
    }

    public void goToPosition(double targetX, double targetY, double targetYaw) {
        while (opModeIsActive()) {
            List<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (!currentDetections.isEmpty()) {
                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == TARGET_TAG_ID) {
                        tagOfInterest = tag;

                        double currentX = tag.pose.x * FEET_PER_METER;
                        double currentY = tag.pose.y * FEET_PER_METER;
                        double currentYaw = Math.toDegrees(tag.pose.yaw);

                        double xPower = xPID.calculate(targetX, currentX);
                        double yPower = yPID.calculate(targetY, currentY);
                        double yawPower = yawPID.calculate(targetYaw, currentYaw);

                        double leftPower = yPower + yawPower;
                        double rightPower = yPower - yawPower;

                        drive4WD(leftPower, rightPower);

                        telemetry.addData("Current X", currentX);
                        telemetry.addData("Current Y", currentY);
                        telemetry.addData("Current Yaw", currentYaw);
                        telemetry.addData("Target X", targetX);
                        telemetry.addData("Target Y", targetY);
                        telemetry.addData("Target Yaw", targetYaw);
                        telemetry.update();

                        if (Math.abs(targetX - currentX) < 0.1 && Math.abs(targetY - currentY) < 0.1 && Math.abs(targetYaw - currentYaw) < 2) {
                            stopMotors();
                            return;
                        }
                    }
                }
            } else {
                telemetry.addLine("Tag Not Detected");
                telemetry.update();
            }
        }
    }

    private void drive4WD(double leftPower, double rightPower) {
        leftFrontDrive.setPower(leftPower);
        leftBackDrive.setPower(leftPower);
        rightFrontDrive.setPower(rightPower);
        rightBackDrive.setPower(rightPower);
    }

    private void stopMotors() {
        drive4WD(0, 0);
    }
}

class PIDController {
    private double kP, kI, kD;
    private double integralSum = 0;
    private double previousError = 0;
    private long lastTime = System.currentTimeMillis();

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double calculate(double target, double current) {
        double error = target - current;
        long currentTime = System.currentTimeMillis();
        double deltaTime = (currentTime - lastTime) / 1000.0;

        integralSum += error * deltaTime;
        double derivative = (error - previousError) / deltaTime;

        double output = (kP * error) + (kI * integralSum) + (kD * derivative);

        previousError = error;
        lastTime = currentTime;

        return output;
    }

    public void reset() {
        integralSum = 0;
        previousError = 0;
    }
}
