package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ITDRobot {
    public double flDrivePower;
    public double frDrivePower;
    public double brDrivePower;
    public double blDrivePower;
    public double SLOWDOWN = 4;
    public DcMotor flDrive;
    public DcMotor frDrive;
    public DcMotor blDrive;
    public DcMotor brDrive;

    public double linearSlidePower;
    public DcMotor leftLinearSlide;
    public DcMotor rightLinearSlide;

    public boolean barUp;
    private final ElapsedTime barDebounce = new ElapsedTime();
    public Servo leftBarServo;
    public Servo rightBarServo;

    public boolean extensionOut;
    private final ElapsedTime extensionDebounce = new ElapsedTime();
    public Servo extensionServo;

    public boolean clawClosed;
    private final ElapsedTime clawDebounce = new ElapsedTime();
    public Servo clawServo;

    public IMU imu;

    public Telemetry telemetry;

    public ITDRobot(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void init(final HardwareMap hardwareMap) {
        // Initialize hardware map
//        flDrive = hardwareMap.get(DcMotor.class, "flDrive");
//        frDrive = hardwareMap.get(DcMotor.class, "frDrive");
//        blDrive = hardwareMap.get(DcMotor.class, "blDrive");
//        brDrive = hardwareMap.get(DcMotor.class, "brDrive");
//
//        leftLinearSlide = hardwareMap.get(DcMotor.class, "leftLinearSlide");
//        rightLinearSlide = hardwareMap.get(DcMotor.class, "rightLinearSlide");

        leftBarServo = hardwareMap.get(Servo.class, "leftBarServo");
//        rightBarServo = hardwareMap.get(Servo.class, "rightBarServo");
        extensionServo = hardwareMap.get(Servo.class, "extensionServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        // Set up motors
//        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        flDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        blDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        brDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        leftLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        rightLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Update state
        be();

        // Set up the IMU (gyro/angle sensor)
        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(imuParameters);

        // Explain state
        what();
    }

    public void be() {
//        flDrive.setPower(flDrivePower / SLOWDOWN);
//        frDrive.setPower(frDrivePower / SLOWDOWN);
//        blDrive.setPower(blDrivePower / SLOWDOWN);
//        brDrive.setPower(brDrivePower / SLOWDOWN);
//
//        leftLinearSlide.setPower(linearSlidePower);
//        rightLinearSlide.setPower(linearSlidePower);

        if (barUp) {
            leftBarServo.setPosition(0.0);
//            rightBarServo.setPosition(1.0);
        } else {
            leftBarServo.setPosition(1.0);
//            rightBarServo.setPosition(0.0);
        }

        if (extensionOut) {
            extensionServo.setPosition(0.4);
        } else {
            extensionServo.setPosition(1.0);
        }

        if (clawClosed) {
            clawServo.setPosition(1.0);
        } else {
            clawServo.setPosition(0.6);
        }
    }

    public void what() {
//        telemetry.addData("FL Power: ", flDrivePower);
//        telemetry.addData("FR Power: ", frDrivePower);
//        telemetry.addData("BL Power: ", blDrivePower);
//        telemetry.addData("BR Power: ", brDrivePower);
        telemetry.addData("Angle: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Linear Slide Power: ", linearSlidePower);
        telemetry.addData("Bar Up? ", barUp);
        telemetry.addData("Extension Out? ", extensionOut);
        telemetry.addData("Claw Closed?", clawClosed);

        telemetry.update();
    }

    public void gamePadPower(Gamepad gp1, Gamepad gp2) {
        // Plugins
        mainDrive(gp1);
        linearSlideControl(gp2);
        barToggle(gp2);
        extensionToggle(gp2);
        clawToggle(gp2);

        // Update state
        be();

        // Explain state
        what();
    }

    public void mainDrive(Gamepad gp) {
        final double drive = (-gp.left_stick_y);
        final double turn = (gp.right_stick_x);
        final double strafe = (gp.left_stick_x);

        flDrivePower = (drive + strafe + turn);
        frDrivePower = (drive - strafe - turn);
        blDrivePower = (drive - strafe + turn);
        brDrivePower = (drive + strafe - turn);
    }

    public void linearSlideControl(Gamepad gp) {
        linearSlidePower = Math.abs(gp.left_stick_y);
    }

    public void barToggle(Gamepad gp) {
        if (gp.x && barDebounce.milliseconds() > 300) {
            barDebounce.reset();

            barUp = !barUp;
        }
    }

    public void extensionToggle(Gamepad gp) {
        if (gp.a && extensionDebounce.milliseconds() > 300) {
            extensionDebounce.reset();

            extensionOut = !extensionOut;
        }
    }

    public void clawToggle(Gamepad gp) {
        if (gp.b && clawDebounce.milliseconds() > 300) {
            clawDebounce.reset();

            clawClosed = !clawClosed;
        }
    }

    public void turn(double target) {
        this.imu.resetYaw();

        if (target == 0) return;

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentPosition = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = target - currentPosition;

        double kp = 0.5;

        final int DELAY = 50;

        while (Math.abs(error) > 2) {
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

            try {Thread.sleep(DELAY);} catch (InterruptedException e) {}
        }

        drive(0.0);

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        try {Thread.sleep(500);} catch (InterruptedException e) {}
    }

    public void driveToInches(final double inches) {
        driveTo((int) (inches * (100 / 11.75) * 1.5));
    }

    public void driveTo(final int pos) {
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

            try { Thread.sleep(delay); } catch (InterruptedException e) {}
        }

        drive(0.0);

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentPosition = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = 0 - currentPosition;

        double kp = 0.5;

        final int DELAY = 50;

        while (Math.abs(error) > 2) {
            currentPosition = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            error = 0 - currentPosition;

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

            try {Thread.sleep(DELAY);} catch (InterruptedException e) {}
        }

        drive(0.0);

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        try {Thread.sleep(500);} catch (InterruptedException e) {}
    }

    public void tinyStrafe(int pow) {
        strafe(48 * pow);
    }

    public void strafe(int pos) {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int flBrPos = pos;
        int frBlPos = -pos;

        while (Math.abs(flBrPos - brDrive.getCurrentPosition()) > 10 || Math.abs(flBrPos - flDrive.getCurrentPosition()) > 10) {
            int flDistance = flBrPos - flDrive.getCurrentPosition();
            int frDistance = frBlPos - frDrive.getCurrentPosition();
            int blDistance = frBlPos - blDrive.getCurrentPosition();
            int brDistance = flBrPos - brDrive.getCurrentPosition();

            flDrivePower = (double)flDistance / (double)Math.abs(flBrPos);
            blDrivePower = (double)blDistance / (double)Math.abs(frBlPos);
            frDrivePower = (double)frDistance / (double)Math.abs(frBlPos);
            brDrivePower = (double)brDistance / (double)Math.abs(flBrPos);

            flDrive.setPower(flDrivePower / 5);
            frDrive.setPower(frDrivePower / 5);
            blDrive.setPower(blDrivePower / 5);
            brDrive.setPower(brDrivePower / 5);
        }

        drive(0.0);

        try {Thread.sleep(500);} catch (InterruptedException e) {}
    }

    private void setDriveMode(final DcMotor.RunMode mode) {
        flDrive.setMode(mode);
        frDrive.setMode(mode);
        blDrive.setMode(mode);
        brDrive.setMode(mode);
    }

    private void drive(final double pow) {
        flDrive.setPower(pow);
        blDrive.setPower(pow);
        frDrive.setPower(pow);
        brDrive.setPower(pow);
    }
}