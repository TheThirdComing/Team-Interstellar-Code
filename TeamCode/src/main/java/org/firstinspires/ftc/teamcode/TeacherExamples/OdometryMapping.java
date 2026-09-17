package org.firstinspires.ftc.teamcode.TeacherExamples;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Odometry Mapping", group = "LinearOpMode")
@Disabled
public class OdometryMapping extends LinearOpMode {

    GoBildaPinpointDriver pinpoint;

    public void configurePinpoint() {
        //This tells the computer where the odometry pods are on the robot;
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor FRONT_L = hardwareMap.get(DcMotor.class, "frontleft");
        DcMotor FRONT_R = hardwareMap.get(DcMotor.class, "frontright");
        DcMotor BACK_L = hardwareMap.get(DcMotor.class, "backleft");
        DcMotor BACK_R = hardwareMap.get(DcMotor.class, "backright");

        FRONT_L.setDirection(DcMotorSimple.Direction.FORWARD);
        FRONT_R.setDirection(DcMotorSimple.Direction.REVERSE);
        BACK_L.setDirection(DcMotorSimple.Direction.FORWARD);
        BACK_R.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection,usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        configurePinpoint();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, -20, -40, AngleUnit.DEGREES, -90));

        double heading = 0;

        double axial;
        double lateral;
        double yaw;

        double x;
        double y;

        double dx;
        double dy;
        double targetRotation;
        double turnAmount;

        waitForStart();
        imu.resetYaw();
        while (opModeIsActive()) {

            x = gamepad1.left_stick_y;
            yaw = gamepad1.right_stick_x;
            y = gamepad1.left_stick_x;

            pinpoint.update();
            Pose2D pose2D = pinpoint.getPosition();
            dx = pose2D.getX(DistanceUnit.INCH);
            dy = pose2D.getY(DistanceUnit.INCH);
            heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            axial = y*cos(-heading) - x * sin(-heading);
            lateral = y *sin(-heading) + x*cos(-heading);

            targetRotation = Math.atan2(dx, dy);
            turnAmount = targetRotation - heading;

            double frontRightPower = axial + yaw + lateral;
            double frontLeftPower = axial - yaw - lateral;
            double backRightPower = axial + yaw - lateral;
            double backLeftPower = axial - yaw + lateral;

            double max;
            //This makes sure the motors don't exceed 1.0/-1.0
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            FRONT_R.setPower(frontRightPower);
            FRONT_L.setPower(frontLeftPower);
            BACK_R.setPower(backRightPower);
            BACK_L.setPower(backLeftPower);

            telemetry.addData("Front Wheel Power Right/Left", "%4.2f, %4.2f", frontRightPower, frontLeftPower);
            telemetry.addData("Back Wheel Power Right/Left", "%4.2f, %4.2f", backRightPower, backLeftPower);
            telemetry.addData("Current Rotation (Degrees)", "%.2f", heading);
            telemetry.update();
        }
    }
}



