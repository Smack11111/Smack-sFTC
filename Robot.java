package org.firstinspires.ftc23565;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class Robot extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor arm;  // Arm motor

    @Override
    public void init() {
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        arm = hardwareMap.get(DcMotor.class, "arm");  // Initialize arm motor

        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    
    }

    @Override
    public void loop() {
        // Driving controls using triggers and right joystick
        double forward = gamepad1.right_trigger;  // RT for forward
        double backward = gamepad1.left_trigger;  // LT for backward
        double turn = gamepad1.right_stick_x;     // Right stick X for turning

        double drive = forward - backward;  // Positive for forward, negative for backward
        double leftPower = Range.clip(drive + turn, -1.0, 1.0);
        double rightPower = Range.clip(drive - turn, -1.0, 1.0);

        // Arm controls using D-pad up and down
        if (gamepad1.dpad_up) {
            arm.setPower(1.0);  // Move arm up
        } else if (gamepad1.dpad_down) {
            arm.setPower(-1.0);  // Move arm down
        } else {
            arm.setPower(0.0);  // Stop arm when no D-pad input
        }

        // Rotation using D-pad left and right
        if (gamepad1.dpad_left) {
            leftPower = -0.5;  // Rotate left
            rightPower = 0.5;
        } else if (gamepad1.dpad_right) {
            leftPower = 0.5;   // Rotate right
            rightPower = -0.5;
        }

        // Set motor powers for driving and rotation
        drive(leftPower,rightPower);

        // Telemetry data for debugging
        telemetry.addData("Forward (RT)", forward);
        telemetry.addData("Backward (LT)", backward);
        telemetry.addData("Turn", turn);
        telemetry.addData("Left Power", leftPower);
        telemetry.addData("Right Power", rightPower);
        telemetry.addData("Arm Power", arm.getPower());  
        telemetry.addData()
        
        // Show power data in telemetry
        telemetry.update();
    }

    public void drive(double leftPower,double rightPower)
    {
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

    }
}
