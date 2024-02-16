package org.firstinspires.ftc.teamcode;


import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "BasicTeleOps_GW", group = "Linear Opmode")
public class BasicTeleops extends OpMode {
    public  RobotHardware robot;
    public GamepadEx gamepad;
    public RobotDrive robotDrive;
    private Telemetry telemetry;

    @Override
    public void init() {
        robot = new RobotHardware();
        robot.init(hardwareMap); // Initialize hardware in RobotHardware
        gamepad = new GamepadEx(gamepad2);
        robotDrive = new RobotDrive(robot, gamepad, telemetry); // Pass robot instance to RobotDrive
        robotDrive.init(); // Initialize RobotDrive
        telemetry.addLine("-------------------");
        telemetry.addData("Status"," initialized Motors and Encoder and IMU");
        telemetry.addLine("-------------------");
        telemetry.update();
    }

    @Override
    public void loop() {

        robotDrive.driveLoop(); // Use RobotDrive methods
    }
}
