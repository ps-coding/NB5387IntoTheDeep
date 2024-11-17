package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ITDRobot {
    public double flDrivePower;
    public double frDrivePower;
    public double brDrivePower;
    public double blDrivePower;
    public double driveSlowdown = 2;
    public DcMotor flDrive;
    public DcMotor frDrive;
    public DcMotor blDrive;
    public DcMotor brDrive;

    public double linearSlidePower;
    public final double BUILD_ERROR_FACTOR = 1.2;
    public DcMotor leftLinearSlide;
    public DcMotor rightLinearSlide;

    public double armPower;
    public DcMotor arm;

    public double clawPower;
    public CRServo claw;

    public IMU imu;

    public Telemetry telemetry;

    public ITDRobot(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void init(HardwareMap hardwareMap) {
        // Initialize hardware map
        flDrive = hardwareMap.get(DcMotor.class, "flDrive");
        frDrive = hardwareMap.get(DcMotor.class, "frDrive");
        blDrive = hardwareMap.get(DcMotor.class, "blDrive");
        brDrive = hardwareMap.get(DcMotor.class, "brDrive");

        leftLinearSlide = hardwareMap.get(DcMotor.class, "leftLinearSlide");
        rightLinearSlide = hardwareMap.get(DcMotor.class, "rightLinearSlide");
        arm = hardwareMap.get(DcMotor.class, "arm");
        claw = hardwareMap.get(CRServo.class, "clawServo");

        // Set up motors
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw.setPower(0.0);

        // Correct directions
        flDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        blDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set up the IMU (gyro/angle sensor)
        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(imuParameters);

        // Init state
        be();

        // Init telemetry
        what();
    }

    public void be(int waitAfter) {
        be();

        // Wait (to allow the action done to be performed)
        try {Thread.sleep(waitAfter);} catch (InterruptedException ignored) {}
    }

    public void be() {
        flDrive.setPower(flDrivePower / driveSlowdown);
        frDrive.setPower(frDrivePower / driveSlowdown);
        blDrive.setPower(blDrivePower / driveSlowdown);
        brDrive.setPower(brDrivePower / driveSlowdown);

        leftLinearSlide.setPower(linearSlidePower / BUILD_ERROR_FACTOR);
        rightLinearSlide.setPower(linearSlidePower);
        arm.setPower(armPower);
        claw.setPower(clawPower);
    }

    public void what() {
        telemetry.addData("FL Power: ", flDrivePower);
        telemetry.addData("FR Power: ", frDrivePower);
        telemetry.addData("BL Power: ", blDrivePower);
        telemetry.addData("BR Power: ", brDrivePower);
        telemetry.addData("Drive Slowdown: ", driveSlowdown);
        telemetry.addData("Angle: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Linear Slide Power: ", linearSlidePower);
        telemetry.addData("Arm Power: ", armPower);
        telemetry.addData("Claw Power: ", clawPower);

        telemetry.update();
    }

    public void gamePadPower(Gamepad gp1, Gamepad gp2) {
        // Plugins
        mainDrive(gp1);
        linearSlideControl(gp2);
        armControl(gp2);
        clawControl(gp2);

        // Update state
        be();

        // Update telemetry
        what();
    }

    public void mainDrive(Gamepad gp) {
        double drive = -gp.left_stick_y;
        double turn = gp.right_stick_x;
        double strafe = -gp.left_stick_x;
        double slowDownFactor = gp.right_trigger;
        double speedUpFactor = gp.left_trigger;

        mainDrive(drive, turn, strafe, slowDownFactor, speedUpFactor);
    }

    public void mainDrive(double drive, double turn, double strafe, double slowDownFactor, double speedUpFactor) {
        flDrivePower = drive - strafe + turn;
        frDrivePower = drive - strafe - turn;
        blDrivePower = drive + strafe + turn;
        brDrivePower = drive + strafe - turn;

        if (slowDownFactor > 0.5 && speedUpFactor > 0.5) {
            driveSlowdown = 2;
        } else if (slowDownFactor > 0.5) {
            driveSlowdown = 4;
        } else if (speedUpFactor > 0.5) {
            driveSlowdown = 1;
        } else {
            driveSlowdown = 2;
        }
    }

    public void linearSlideControl(Gamepad gp) {
        double power = gp.left_stick_y;
        linearSlideControl(power);
    }

    public void linearSlideControl(double power) {
        linearSlidePower = power;
    }

    public void armControl(Gamepad gp) {
        double power = gp.right_stick_y;
        armControl(power);
    }

    public void armControl(double power) {
        armPower = power;
    }

    public void clawControl(Gamepad gp) {
        double power = 0.0;

        if (gp.left_trigger > 0.5) {
            power = 1.0;
        } else if (gp.right_trigger > 0.5) {
            power = -1.0;
        }

        clawControl(power);
    }

    public void clawControl(double power) {
        clawPower = power;
    }

    public void turnTo(double target) {
        this.imu.resetYaw();

        if (target == 0) return;

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentPosition = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = target - currentPosition;

        double kp = 0.5;

        final int DELAY = 50;

        while (Math.abs(error) > 3) {
            currentPosition = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            error = target - currentPosition;

            double proportional = error * kp;

            double turn = proportional / (180 * kp);

            flDrivePower = -turn;
            frDrivePower = turn;
            blDrivePower = -turn;
            brDrivePower = turn;

            flDrive.setPower(flDrivePower / 3);
            frDrive.setPower(frDrivePower / 3);
            blDrive.setPower(blDrivePower / 3);
            brDrive.setPower(brDrivePower / 3);

            try {Thread.sleep(DELAY);} catch (InterruptedException ignored) {}
        }

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        try {Thread.sleep(500);} catch (InterruptedException ignored) {}
    }

    public void driveTo(int pos) {
        if (pos == 0) return;

        this.imu.resetYaw();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        final int delay = 20;

        while (Math.abs(pos - flDrive.getCurrentPosition()) > 10 || Math.abs(pos - frDrive.getCurrentPosition()) > 10) {
            int flDistance = pos - flDrive.getCurrentPosition();
            int frDistance = pos - frDrive.getCurrentPosition();
            int blDistance = pos - blDrive.getCurrentPosition();
            int brDistance = pos - brDrive.getCurrentPosition();

            flDrivePower = (double)flDistance / (double)Math.abs(pos);
            blDrivePower = (double)blDistance / (double)Math.abs(pos);
            frDrivePower = (double)frDistance / (double)Math.abs(pos);
            brDrivePower = (double)brDistance / (double)Math.abs(pos);

            flDrive.setPower(flDrivePower / 3);
            frDrive.setPower(frDrivePower / 3);
            blDrive.setPower(blDrivePower / 3);
            brDrive.setPower(brDrivePower / 3);

            try { Thread.sleep(delay); } catch (InterruptedException ignored) {}
        }

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        try {Thread.sleep(500);} catch (InterruptedException ignored) {}
    }

    public void strafeTo(int frontPos) {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int backPos = -frontPos;

        while (Math.abs(frontPos - brDrive.getCurrentPosition()) > 15 || Math.abs(frontPos - flDrive.getCurrentPosition()) > 15) {
            int flDistance = frontPos - flDrive.getCurrentPosition();
            int frDistance = frontPos - frDrive.getCurrentPosition();
            int blDistance = backPos - blDrive.getCurrentPosition();
            int brDistance = backPos - brDrive.getCurrentPosition();

            flDrivePower = (double)flDistance / (double)Math.abs(frontPos);
            blDrivePower = (double)blDistance / (double)Math.abs(frontPos);
            frDrivePower = (double)frDistance / (double)Math.abs(backPos);
            brDrivePower = (double)brDistance / (double)Math.abs(backPos);

            flDrive.setPower(flDrivePower / 5);
            frDrive.setPower(frDrivePower / 5);
            blDrive.setPower(blDrivePower / 5);
            brDrive.setPower(brDrivePower / 5);
        }

        brake();

        try {Thread.sleep(500);} catch (InterruptedException ignored) {}
    }

    private void setDriveMode(DcMotor.RunMode mode) {
        flDrive.setMode(mode);
        frDrive.setMode(mode);
        blDrive.setMode(mode);
        brDrive.setMode(mode);
    }

    private void brake() {
        flDrive.setPower(0.0);
        blDrive.setPower(0.0);
        frDrive.setPower(0.0);
        brDrive.setPower(0.0);
    }
}