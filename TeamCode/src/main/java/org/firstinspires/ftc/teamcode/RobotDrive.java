package org.firstinspires.ftc.teamcode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.BACK;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.START;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class RobotDrive {

    private final GamepadEx gamepad;
    private final RobotHardware robot;
    private ControlMode controlMode = ControlMode.FIELD_CENTRIC;
    private ElapsedTime debounceTimer = new ElapsedTime(); // Timer for debouncing
    private Telemetry telemetry;

    private boolean startPressed = false;
    private boolean backPressed = false;

    public RobotDrive(RobotHardware robot, GamepadEx gamepad, Telemetry telemetry) {
        this.robot = robot;
        this.gamepad = gamepad;
        this.telemetry = telemetry;
    }

    public void init() {
        // Initialize IMU from RobotHardware
        robot.initIMU();
    }

    public void driveLoop() {
        // Toggle control mode
        if (gamepad.getButton(START) && !startPressed) {
            toggleControlMode();
            debounceTimer.reset();
            startPressed = true;
        } else if (!gamepad.getButton(START)) {
            startPressed = false;
        }

        // Reset IMU heading using button back and reset odometry
        if (gamepad.getButton(BACK) && !backPressed) {
            robot.initIMU();
            robot.resetDriveEncoders();
            debounceTimer.reset();
            backPressed = true;
        } else if (!gamepad.getButton(BACK)) {
            backPressed = false;
        }

        // Set gamepad joystick power
        double drive = -gamepad.getRightY();
        double strafe = gamepad.getRightX();
        double rotate = gamepad.getLeftX();

        // Get robot's current heading
        double currentHeading = getRobotHeading();

        // Mecanum drive calculations
        setMecanumDrivePower(drive, strafe, rotate, currentHeading);

        // Show elapsed time in telemetry
        telemetry.addData("Status", "Run Time: " + debounceTimer.seconds());
        telemetry.addLine("-------------------");
        telemetry.addData("drive", drive);
        telemetry.addData("strafe", strafe);
        telemetry.addData("rotate", rotate);
        telemetry.addLine("-------------------");
        telemetry.addData("leftBackMotorPower", robot.frontRightMotor.getVelocity());
        telemetry.addData("RightFrontMotorPower", robot.backLeftMotor.getVelocity());
        telemetry.addData("RightBackMotorPower", robot.backRightMotor.getVelocity());
        telemetry.addLine("-------------------");
        telemetry.addData("leftOdometryEncoder", robot.leftodometry.getCurrentPosition());
        telemetry.addData("CenterOdometryEncoder", robot.rightodometry.getCurrentPosition());
        telemetry.addData("rightOdometryEncoder", robot.centerodometry.getCurrentPosition());
        telemetry.addLine("-------------------");
        telemetry.addData("EncoderCounts", String.valueOf(getEncoderCounts()[0]),getEncoderCounts()[1],getEncoderCounts()[2]);
        telemetry.addLine("-------------------");
        telemetry.addData("heading", currentHeading);
        telemetry.addLine("-------------------");
        telemetry.addData("control mode", controlMode.toString());
        telemetry.update();
    }

    private double getRobotHeading() {
        // Get the robot's heading from IMU
        // double heading = robot.imu().firstAngle;
        double heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        while (heading > 180.0) {
            heading -= 360.0;
        }
        while (heading < -180.0) {
            heading += 360.0;
        }
        return -heading;
    }

    private void toggleControlMode() {
        if (controlMode == ControlMode.FIELD_CENTRIC) {
            controlMode = ControlMode.ROBOT_CENTRIC;
        } else {
            controlMode = ControlMode.FIELD_CENTRIC;
        }
    }

    private void setMecanumDrivePower(double drive, double strafe, double rotate, double currentHeading) {
        // Determine the drive mode
        if (controlMode == ControlMode.FIELD_CENTRIC) {
            // Adjust for field-centric control using the gyro angle
            double headingRad = Math.toRadians(currentHeading);
            double temp = drive * Math.cos(headingRad) + strafe * Math.sin(headingRad);
            strafe = -drive * Math.sin(headingRad) + strafe * Math.cos(headingRad);
            drive = temp;
        }

        // Mecanum wheel drive formula
        double frontLeftPower = drive + strafe + rotate;
        double frontRightPower = drive - strafe - rotate;
        double backLeftPower = drive - strafe + rotate;
        double backRightPower = drive + strafe - rotate;

        // Constrain the power within +-1.0
        double maxPower = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backRightPower), Math.abs(backLeftPower))
        );

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backRightPower /= maxPower;
            backLeftPower /= maxPower;
        }

        // Set motor powers
        double powerFactor = 0.5;
        robot.frontLeftMotor.set(Range.clip(frontLeftPower * powerFactor, -1.0, 1.0));
        robot.frontRightMotor.set(Range.clip(frontRightPower * powerFactor, -1.0, 1.0));
        robot.backLeftMotor.set(Range.clip(backLeftPower * powerFactor, -1.0, 1.0));
        robot.backRightMotor.set(Range.clip(backRightPower * powerFactor, -1.0, 1.0));

    }
    // Method to get left encoder count
    public int [] getEncoderCounts() {
        int[] counts = new int[3];
        counts[0] = robot.leftodometry.getCurrentPosition();
        counts[1] = robot.rightodometry.getCurrentPosition();
        counts[2] = robot.centerodometry.getCurrentPosition();
        return counts;
    }

    public enum ControlMode {
        FIELD_CENTRIC,
        ROBOT_CENTRIC
    }
}
