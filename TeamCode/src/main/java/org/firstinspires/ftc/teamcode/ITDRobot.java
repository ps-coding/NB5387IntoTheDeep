package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    public double driveSlowdown = 2;
    public DcMotor flDrive;
    public DcMotor frDrive;
    public DcMotor blDrive;
    public DcMotor brDrive;

    public double linearSlidePower;
    public DcMotor leftLinearSlide;
    public DcMotor rightLinearSlide;

    public double armPower;
    public DcMotor arm;

    public double clawPower;
    public CRServo claw;

    public ElapsedTime clawTurnDebounce = new ElapsedTime();
    public ClawTurnState clawTurnState = ClawTurnState.LEFT;
    public Servo clawTurn;

    public ElapsedTime lockHangDebounce = new ElapsedTime();
    public boolean lockHang = false;

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
        claw = hardwareMap.get(CRServo.class, "claw");
        clawTurn = hardwareMap.get(Servo.class, "clawTurn");

        // Set up motors
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        blDrive.setDirection(DcMotorSimple.Direction.REVERSE);

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

        // Set up the IMU (gyro/angle sensor)
        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        );
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(imuParameters);

        // Init state
        be();
        what();
    }

    public void be(int waitAfter) {
        be();

        // Wait (to allow the action done to be performed)
        sleep(waitAfter);
    }

    public void be() {
        flDrive.setPower(flDrivePower / driveSlowdown);
        frDrive.setPower(frDrivePower / driveSlowdown);
        blDrive.setPower(blDrivePower / driveSlowdown);
        brDrive.setPower(brDrivePower / driveSlowdown);

        if (lockHang) {
            leftLinearSlide.setPower(0.5);
            rightLinearSlide.setPower(0.5);
            arm.setPower(0.3);
        } else {
            leftLinearSlide.setPower(linearSlidePower);
            rightLinearSlide.setPower(linearSlidePower);
            arm.setPower(armPower);
        }

        claw.setPower(clawPower);

        if (clawTurnState == ClawTurnState.LEFT) {
            clawTurn.setPosition(0.0);
        } else if (clawTurnState == ClawTurnState.MIDDLE) {
            clawTurn.setPosition(0.4);
        } else {
            clawTurn.setPosition(0.75);
        }
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
        telemetry.addData("Claw Out: ", clawTurnState);

        telemetry.update();
    }

    public void gamePadPower(Gamepad gp1, Gamepad gp2) {
        // Plugins
        mainDrive(gp1);
        linearSlideControl(gp2);
        armControl(gp2);
        clawControl(gp2);
        setClaw(gp2);
        lockHang(gp2);

        // Update state
        be();

        // Update telemetry
        what();
    }

    public void mainDrive(Gamepad gp) {
        double drive = -gp.left_stick_y;
        double turn = gp.right_stick_x;
        double strafe = gp.left_stick_x;
        double slowDownFactor = gp.right_trigger;
        double speedUpFactor = gp.left_trigger;

        mainDrive(drive, turn, strafe, slowDownFactor, speedUpFactor);
    }

    public void mainDrive(double drive, double turn, double strafe, double slowDownFactor, double speedUpFactor) {
        flDrivePower = drive + strafe + turn;
        frDrivePower = drive - strafe - turn;
        blDrivePower = drive - strafe + turn;
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
        linearSlidePower = power - 0.2;
    }

    public void armControl(Gamepad gp) {
        double power = gp.right_stick_y;

        if (Math.abs(power) <= 0.1) {
            armControl(0.0);
        } else {
            armControl(power);
        }
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

    public void setClaw(Gamepad gp) {
        if (gp.a) {
            if (clawTurnDebounce.milliseconds() < 500) return;

            clawTurnDebounce.reset();

            if (clawTurnState == ClawTurnState.LEFT) {
                clawTurnState = ClawTurnState.RIGHT;
            } else if (clawTurnState == ClawTurnState.RIGHT) {
                clawTurnState = ClawTurnState.LEFT;
            }
        } else if (gp.y) {
            if (clawTurnDebounce.milliseconds() < 500) return;

            clawTurnDebounce.reset();

            if (clawTurnState == ClawTurnState.MIDDLE) {
                clawTurnState = ClawTurnState.RIGHT;
            } else if (clawTurnState == ClawTurnState.RIGHT) {
                clawTurnState = ClawTurnState.MIDDLE;
            }
        }
    }

    public void setClaw(ClawTurnState state) {
        clawTurnState = state;
    }

    public void lockHang(Gamepad gp) {
        if (gp.x) {
            if (lockHangDebounce.milliseconds() < 500) return;

            lockHangDebounce.reset();

            lockHang = !lockHang;
        }
    }

    public void armTo(int pos) {
        if (pos == 0) return;

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        final int DELAY = 20;

        while (Math.abs(pos - arm.getCurrentPosition()) > 5) {
            int armDistance = pos - arm.getCurrentPosition();

            armPower = (double)armDistance / (double)Math.abs(pos);

            armPower = (armPower / 1.5) + (Math.signum(armPower) * 0.1);

            be();

            sleep(DELAY);
        }

        armPower = 0.0;
        arm.setPower(0.0);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(500);
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

        while (Math.abs(error) > 1) {
            currentPosition = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            error = target - currentPosition;

            double proportional = error * kp;

            double turn = proportional / (180 * kp);

            flDrivePower = -turn;
            frDrivePower = turn;
            blDrivePower = -turn;
            brDrivePower = turn;

            flDrivePower = (flDrivePower / 1.5) + (Math.signum(flDrivePower) * 0.1);
            frDrivePower = (frDrivePower / 1.5) + (Math.signum(frDrivePower) * 0.1);
            blDrivePower = (blDrivePower / 1.5) + (Math.signum(blDrivePower) * 0.1);
            brDrivePower = (brDrivePower / 1.5) + (Math.signum(brDrivePower) * 0.1);

            flDrivePower = flDrivePower * driveSlowdown;
            frDrivePower = frDrivePower * driveSlowdown;
            blDrivePower = blDrivePower * driveSlowdown;
            brDrivePower = brDrivePower * driveSlowdown;

            be();

            sleep(DELAY);
        }

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(500);
    }

    public void driveTo(int pos) {
        if (pos == 0) return;

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        final int DELAY = 20;

        while (Math.abs(pos - flDrive.getCurrentPosition()) > 5 || Math.abs(pos - frDrive.getCurrentPosition()) > 5) {
            int flDistance = pos - flDrive.getCurrentPosition();
            int frDistance = pos - frDrive.getCurrentPosition();
            int blDistance = pos - blDrive.getCurrentPosition();
            int brDistance = pos - brDrive.getCurrentPosition();

            flDrivePower = (double)flDistance / (double)Math.abs(pos);
            blDrivePower = (double)blDistance / (double)Math.abs(pos);
            frDrivePower = (double)frDistance / (double)Math.abs(pos);
            brDrivePower = (double)brDistance / (double)Math.abs(pos);

            flDrivePower = (flDrivePower / 1.5) + (Math.signum(flDrivePower) * 0.1);
            frDrivePower = (frDrivePower / 1.5) + (Math.signum(frDrivePower) * 0.1);
            blDrivePower = (blDrivePower / 1.5) + (Math.signum(blDrivePower) * 0.1);
            brDrivePower = (brDrivePower / 1.5) + (Math.signum(brDrivePower) * 0.1);

            flDrivePower = flDrivePower * driveSlowdown;
            frDrivePower = frDrivePower * driveSlowdown;
            blDrivePower = blDrivePower * driveSlowdown;
            brDrivePower = brDrivePower * driveSlowdown;

            be();

            sleep(DELAY);
        }

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(500);
    }

    public void driveTo(int pos, double slowdown) {
        if (pos == 0) return;

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        final int DELAY = 20;

        while (Math.abs(pos - flDrive.getCurrentPosition()) > 5 || Math.abs(pos - frDrive.getCurrentPosition()) > 5) {
            int flDistance = pos - flDrive.getCurrentPosition();
            int frDistance = pos - frDrive.getCurrentPosition();
            int blDistance = pos - blDrive.getCurrentPosition();
            int brDistance = pos - brDrive.getCurrentPosition();

            flDrivePower = (double)flDistance / (double)Math.abs(pos);
            blDrivePower = (double)blDistance / (double)Math.abs(pos);
            frDrivePower = (double)frDistance / (double)Math.abs(pos);
            brDrivePower = (double)brDistance / (double)Math.abs(pos);

            flDrivePower = (flDrivePower / slowdown) + (Math.signum(flDrivePower) * 0.1);
            frDrivePower = (frDrivePower / slowdown) + (Math.signum(frDrivePower) * 0.1);
            blDrivePower = (blDrivePower / slowdown) + (Math.signum(blDrivePower) * 0.1);
            brDrivePower = (brDrivePower / slowdown) + (Math.signum(brDrivePower) * 0.1);

            flDrivePower = flDrivePower * driveSlowdown;
            frDrivePower = frDrivePower * driveSlowdown;
            blDrivePower = blDrivePower * driveSlowdown;
            brDrivePower = brDrivePower * driveSlowdown;

            be();

            sleep(DELAY);
        }

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(500);
    }

    public void strafeTo(int pos) {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        final int DELAY = 20;

        int flBrPos = pos;
        int frBlPos = -pos;

        while (Math.abs(flBrPos - brDrive.getCurrentPosition()) > 5 || Math.abs(flBrPos - flDrive.getCurrentPosition()) > 5) {
            int flDistance = flBrPos - flDrive.getCurrentPosition();
            int frDistance = frBlPos - frDrive.getCurrentPosition();
            int blDistance = frBlPos - blDrive.getCurrentPosition();
            int brDistance = flBrPos - brDrive.getCurrentPosition();

            flDrivePower = (double)flDistance / (double)Math.abs(flBrPos);
            blDrivePower = (double)blDistance / (double)Math.abs(frBlPos);
            frDrivePower = (double)frDistance / (double)Math.abs(frBlPos);
            brDrivePower = (double)brDistance / (double)Math.abs(flBrPos);

            flDrivePower = (flDrivePower) + (Math.signum(flDrivePower) * 0.1);
            frDrivePower = (frDrivePower) + (Math.signum(frDrivePower) * 0.1);
            blDrivePower = (blDrivePower) + (Math.signum(blDrivePower) * 0.1);
            brDrivePower = (brDrivePower) + (Math.signum(brDrivePower) * 0.1);

            flDrivePower = flDrivePower * driveSlowdown;
            frDrivePower = frDrivePower * driveSlowdown;
            blDrivePower = blDrivePower * driveSlowdown;
            brDrivePower = brDrivePower * driveSlowdown;

            be();

            sleep(DELAY);
        }

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(500);
    }

    private void setDriveMode(DcMotor.RunMode mode) {
        flDrive.setMode(mode);
        frDrive.setMode(mode);
        blDrive.setMode(mode);
        brDrive.setMode(mode);
    }

    public void brake() {
        flDrivePower = 0.0;
        blDrivePower = 0.0;
        frDrivePower = 0.0;
        brDrivePower = 0.0;

        flDrive.setPower(0.0);
        blDrive.setPower(0.0);
        frDrive.setPower(0.0);
        brDrive.setPower(0.0);
    }

    public void sleep(int milliseconds) {
        try {Thread.sleep(milliseconds);} catch (InterruptedException ignored) {}
    }

    public enum ClawTurnState {
        LEFT,
        MIDDLE,
        RIGHT
    }
}