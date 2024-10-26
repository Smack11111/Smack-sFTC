package Random;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="MecanumDrivetrainWithUI", group="TeleOp")
public class MecanumDrivetrainWithUI extends OpMode {

    // Declare motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    //D-Pad soft speed
    private double softSpeed = 0.3;

    @Override
    public void init() {
        // Initialize motors NOTE Change the motor names to you motor name for your DS
        frontLeft = hardwareMap.get(DcMotor.class, "front_left_motor");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_motor");
        backLeft = hardwareMap.get(DcMotor.class, "back_left_motor");
        backRight = hardwareMap.get(DcMotor.class, "back_right_motor");

        // Set motor directions
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        
        // Add telemetry
        telemetry.addData("Status", "Initialized. Ready to run.");
        telemetry.update();
    }

    @Override
    public void loop() {
        double drive = 0;
        double strafe = 0;
        double rotate = gamepad1.right_stick_x;

        String movement = "Stopped";  // Default movement state for telemetry

        // Fast forward/backward with triggers
        if (gamepad1.left_trigger > 0.1) {
            drive = 1.0; 
            movement = "Fast Forward (LT)";
        } else if (gamepad1.right_trigger > 0.1) {
            drive = -1.0;
            movement = "Fast Backward (RT)";
        }

        // D-pad soft movements and diagonal movement
        if (gamepad1.dpad_up && gamepad1.dpad_left) {
            drive = softSpeed; 
            strafe = -softSpeed;
            movement = "Diagonal Forward-Left (D-pad)";
        } else if (gamepad1.dpad_up && gamepad1.dpad_right) {
            drive = softSpeed; 
            strafe = softSpeed; 
            movement = "Diagonal Forward-Right (D-pad)";
        } else if (gamepad1.dpad_down && gamepad1.dpad_left) {
            drive = -softSpeed; 
            strafe = -softSpeed; 
            movement = "Diagonal Backward-Left (D-pad)";
        } else if (gamepad1.dpad_down && gamepad1.dpad_right) {
            drive = -softSpeed;
            strafe = softSpeed;
            movement = "Diagonal Backward-Right (D-pad)";
        } else if (gamepad1.dpad_up) {
            drive = softSpeed;
            movement = "Soft Forward (D-pad)";
        } else if (gamepad1.dpad_down) {
            drive = -softSpeed;
            movement = "Soft Backward (D-pad)";
        } else if (gamepad1.dpad_left) {
            strafe = -softSpeed;
            movement = "Soft Strafe Left (D-pad)";
        } else if (gamepad1.dpad_right) {
            strafe = softSpeed; 
            movement = "Soft Strafe Right (D-pad)";
        }

        // Calculate power for each wheel
        double frontLeftPower = drive + strafe + rotate;
        double frontRightPower = drive - strafe - rotate;
        double backLeftPower = drive - strafe + rotate;
        double backRightPower = drive + strafe - rotate;

        // Set power to motors
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        
        telemetry.addData("Movement", movement);
        telemetry.addData("Motor Power", "FL (%.2f), FR (%.2f), BL (%.2f), BR (%.2f)", 
                          frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        telemetry.update();
    }
}
