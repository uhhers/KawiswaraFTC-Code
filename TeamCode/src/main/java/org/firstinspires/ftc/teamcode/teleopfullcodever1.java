package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="Tele-OP V.1", group="Linear OpMode")
public class teleopfullcodever1 extends LinearOpMode {

    final private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        DcMotor frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        DcMotor backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        DcMotor frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        DcMotor backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        boolean RobotCentricDriveMode = true;

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        runtime.reset();

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD //Set up with the robot placement, VERY IMPORTANT
                )
        );

        imu.initialize(parameters);

        while (opModeIsActive()) {
            double frontLeftPower;
            double frontRightPower;
            double backLeftPower;
            double backRightPower;

            if (RobotCentricDriveMode) {
                double turn = gamepad1.right_stick_x;
                double x = gamepad1.left_stick_x;
                double y = -gamepad1.left_stick_y;

                double theta = Math.atan2(y, x);
                double power = Math.hypot(x, y);

                double sin = Math.sin(theta - Math.PI / 4);
                double cos = Math.cos(theta - Math.PI / 4);
                double max = Math.max(Math.abs(sin), Math.abs(cos));

                frontLeftPower = power * cos / max + turn;
                frontRightPower = power * sin / max - turn;
                backLeftPower = power * sin / max + turn;
                backRightPower = power * cos / max - turn;

                if ((power + Math.abs(turn)) > 1) {
                    frontLeftPower /= power + turn;
                    frontRightPower /= power + turn;
                    backLeftPower /= power + turn;
                    backRightPower /= power + turn;
                }
            } else {
                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

                double heading = orientation.getYaw(AngleUnit.RADIANS);

                double turn = gamepad1.right_stick_x;
                double x = gamepad1.left_stick_x;
                double y = -gamepad1.left_stick_y;

                double rotatedX = x * Math.cos(-heading) - y * Math.sin(-heading);
                double rotatedY = x * Math.sin(-heading) + y * Math.cos(-heading);

                double theta = Math.atan2(rotatedY, rotatedX);
                double power = Math.hypot(rotatedX, rotatedY);

                double sin = Math.sin(theta - Math.PI / 4);
                double cos = Math.cos(theta - Math.PI / 4);
                double max = Math.max(Math.abs(sin), Math.abs(cos));

                frontLeftPower = power * cos / max + turn;
                frontRightPower = power * sin / max - turn;
                backLeftPower = power * sin / max + turn;
                backRightPower = power * cos / max - turn;
            }

            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);
        }
    }
}