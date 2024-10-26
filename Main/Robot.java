package Main;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class Robot extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    
    // 4WD motors
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    
    private DcMotor arm;  // Arm motor
    private static final double DEAD_ZONE = 0.1;  // Dead zone for joystick

    // Hardware for battery voltage
    private VoltageSensor batteryVoltageSensor;

    @Override
    public void init() {
        // Initialize motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "LFD");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RFD");
        leftBackDrive = hardwareMap.get(DcMotor.class, "LBD");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RBD");
        arm = hardwareMap.get(DcMotor.class, "arm");

        // Set directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Initialize battery sensor
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Driving controls using triggers and right joystick
        double forward = gamepad1.right_trigger;  // RT for forward
        double backward = gamepad1.left_trigger;  // LT for backward
        double turn = Math.abs(gamepad1.right_stick_x) > DEAD_ZONE ? gamepad1.right_stick_x : 0;  // Right stick X for turning with dead zone

        double drive = forward - backward;  // Positive for forward, negative for backward
        double leftPower = Range.clip(drive + turn, -1.0, 1.0);
        double rightPower = Range.clip(drive - turn, -1.0, 1.0);

        // Arm controls with gradual adjustment using left stick Y
        double armPower = -gamepad1.left_stick_y;  // Arm control with variable speed
        arm.setPower(Range.clip(armPower, -1.0, 1.0));  // Clip arm power to prevent over-extension

        // Rotation using D-pad left and right
        if (gamepad1.dpad_left) {
            leftPower = -0.5;
            rightPower = 0.5;
        } else if (gamepad1.dpad_right) {
            leftPower = 0.5;
            rightPower = -0.5;
        }

        // Set motor powers for 4WD
        drive4WD(leftPower, rightPower);

        // Telemetry data for debugging
        telemetry.addData("== Control Inputs ==", "");
        telemetry.addData("Forward (RT)", forward);
        telemetry.addData("Backward (LT)", backward);
        telemetry.addData("Turn (Stick X)", turn);
        
        telemetry.addData("== Motor Powers ==", "");
        telemetry.addData("Left Power", "%.2f", leftPower);
        telemetry.addData("Right Power", "%.2f", rightPower);
        telemetry.addData("Arm Power", "%.2f", armPower);

        telemetry.addData("== Robot Status ==", "");
        telemetry.addData("Runtime", "%.1f seconds", runtime.seconds());
        telemetry.addData("Arm Position", arm.getCurrentPosition());
        
        // Battery telemetry
        telemetry.addData("Battery Voltage", "%.2f volts", batteryVoltageSensor.getVoltage());

        telemetry.update();
    }

    // Method to set power for all 4 motors
    public void drive4WD(double leftPower, double rightPower) {
        leftFrontDrive.setPower(leftPower);
        leftBackDrive.setPower(leftPower);
        rightFrontDrive.setPower(rightPower);
        rightBackDrive.setPower(rightPower);
    }
}
