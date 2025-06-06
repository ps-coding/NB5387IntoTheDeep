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

    public ElapsedTime driveModeDebounce = new ElapsedTime();
    public DriveMode driveMode = DriveMode.NORMAL;

    public double linearSlidePower;
    public DcMotor leftLinearSlide;
    public DcMotor rightLinearSlide;

    public double horizontalSlidePower;
    public DcMotor horizontalSlide;

    public double armPower;
    public DcMotor arm;

    public double intakePower;
    public CRServo intakeWheel;

    public ElapsedTime intakeTurnDebounce = new ElapsedTime();
    public IntakeTurnState intakeTurnState = IntakeTurnState.LEFT;
    public Servo intakeTurn;

    public ElapsedTime clawDebounce = new ElapsedTime();
    public ClawState clawState = ClawState.CLOSED;
    public Servo claw;

    public ElapsedTime clawTurnDebounce = new ElapsedTime();
    public ClawTurnState clawTurnState = ClawTurnState.IN;
    public Servo clawTurn;

    public ElapsedTime returnSequenceDebounce = new ElapsedTime();
    public boolean initiatedReturnSequence = false;

    public ElapsedTime lockDebounce = new ElapsedTime();
    public boolean lock = false;

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
        horizontalSlide = hardwareMap.get(DcMotor.class, "horizontalSlide");
        arm = hardwareMap.get(DcMotor.class, "arm");
        intakeWheel = hardwareMap.get(CRServo.class, "intakeWheel");
        intakeTurn = hardwareMap.get(Servo.class, "intakeTurn");
        claw = hardwareMap.get(Servo.class, "claw");
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

        horizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeWheel.setPower(0.0);

        // Set up the IMU (gyro/angle sensor)
        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        );
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(imuParameters);
        imu.resetYaw();

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
        if (lock) return;

        flDrive.setPower(flDrivePower / driveSlowdown);
        frDrive.setPower(frDrivePower / driveSlowdown);
        blDrive.setPower(blDrivePower / driveSlowdown);
        brDrive.setPower(brDrivePower / driveSlowdown);

        leftLinearSlide.setPower(linearSlidePower);
        rightLinearSlide.setPower(linearSlidePower);

        arm.setPower(armPower);

        horizontalSlide.setPower(horizontalSlidePower);

        intakeWheel.setPower(intakePower);

        if (intakeTurnState == IntakeTurnState.LEFT) {
            intakeTurn.setPosition(0.0);
        } else if (intakeTurnState == IntakeTurnState.MIDDLE) {
            intakeTurn.setPosition(0.4);
        } else {
            intakeTurn.setPosition(0.75);
        }

        if (clawState == ClawState.CLOSED) {
            claw.setPosition(1.0);
        } else {
            claw.setPosition(0.6);
        }

        if (clawTurnState == ClawTurnState.IN) {
            clawTurn.setPosition(1.0);
        } else {
            clawTurn.setPosition(0.0);
        }
    }

    public void what() {
        telemetry.addData("Arm Encoder: ", arm.getCurrentPosition());
        telemetry.addData("FL Power: ", flDrivePower);
        telemetry.addData("FR Power: ", frDrivePower);
        telemetry.addData("BL Power: ", blDrivePower);
        telemetry.addData("BR Power: ", brDrivePower);
        telemetry.addData("Drive Slowdown: ", driveSlowdown);
        telemetry.addData("Angle: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Linear Slide Power: ", linearSlidePower);
        telemetry.addData("Horizontal Slide Power: ", horizontalSlidePower);
        telemetry.addData("Arm Power: ", armPower);
        telemetry.addData("Intake Wheel Power: ", intakePower);
        telemetry.addData("Intake Turn: ", intakeTurnState);
        telemetry.addData("Claw: ", clawState);
        telemetry.addData("Claw Turn: ", clawTurnState);
        telemetry.addData("Horizontal Slide Position: ", horizontalSlide.getCurrentPosition());

        telemetry.update();
    }

    public void gamePadPower(Gamepad gp1, Gamepad gp2) {
        // Plugins
        setDrivingMode(gp1);

        if (driveMode == DriveMode.NORMAL) {
            mainDrive(gp1);
        } else {
            mainGyroDrive(gp1);
        }

        linearSlideControl(gp2);
        horizontalSlideControl(gp2);
        armControl(gp2);
        intakeControl(gp2);
        setIntakeTurn(gp2);
        setClaw(gp2);
        setClawTurn(gp2);
        returnSequence(gp2);
        setLock(gp1);

        // Update state
        be();

        // Update telemetry
        what();
    }

    public void setDrivingMode(Gamepad gp) {
        if (gp.x) {
            if (driveModeDebounce.milliseconds() < 500) return;

            driveModeDebounce.reset();

            if (driveMode == DriveMode.NORMAL) {
                setDrivingMode(DriveMode.GYRO);
            } else {
                setDrivingMode(DriveMode.NORMAL);
            }
        }
    }

    public void setDrivingMode(DriveMode mode) {
        driveMode = mode;
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

    public void mainGyroDrive(Gamepad gp) {
        double drive = -gp.left_stick_y;
        double turn = gp.right_stick_x;
        double strafe = gp.left_stick_x;
        double slowDownFactor = gp.right_trigger;
        double speedUpFactor = gp.left_trigger;

        if (gp.a) {
            imu.resetYaw();
        }

        mainGyroDrive(drive, turn, strafe, slowDownFactor, speedUpFactor);
    }

    public void mainGyroDrive(double drive, double turn, double strafe, double slowDownFactor, double speedUpFactor) {
        double imuPos = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double cosine = Math.cos(-imuPos);
        double sine = Math.sin(-imuPos);

        double driveCos = cosine * drive;
        double driveSin = sine * drive;
        double strafeCos = cosine * strafe;
        double strafeSin = sine * strafe;

        double actualDrive = (driveCos + strafeSin) / 1.5;
        double actualStrafe = strafeCos - driveSin;

        mainDrive(actualDrive, turn, actualStrafe, slowDownFactor, speedUpFactor);
    }

    public void linearSlideControl(Gamepad gp) {
        double power = gp.left_trigger - gp.right_trigger;

        if (leftLinearSlide.getCurrentPosition() > -2140 || power >= 0) {
            linearSlideControl(power);
        } else {
            linearSlideControl(0.0);
        }
    }

    public void linearSlideControl(double power) {
        linearSlidePower = power - 0.2;
    }

    public void horizontalSlideControl(Gamepad gp) {
        double power = gp.left_stick_y;

        if (horizontalSlide.getCurrentPosition() > -880 || power >= 0) {
            horizontalSlideControl(power);
        } else {
            horizontalSlideControl(0.0);
        }
    }

    public void horizontalSlideControl(double power) {
        horizontalSlidePower = power;
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

    public void intakeControl(Gamepad gp) {
        double power = 0.0;

        if (gp.left_bumper) {
            power = 1.0;
        } else if (gp.right_bumper) {
            power = -1.0;
        }

        intakeControl(power);
    }

    public void intakeControl(double power) {
        intakePower = power;
    }

    public void setIntakeTurn(Gamepad gp) {
        if (gp.dpad_down) {
            if (intakeTurnDebounce.milliseconds() < 500) return;

            intakeTurnDebounce.reset();

            if (intakeTurnState == IntakeTurnState.LEFT) {
                setIntakeTurn(IntakeTurnState.RIGHT);
            } else if (intakeTurnState == IntakeTurnState.RIGHT) {
                setIntakeTurn(IntakeTurnState.LEFT);
            }
        } else if (gp.dpad_left) {
            if (intakeTurnDebounce.milliseconds() < 500) return;

            intakeTurnDebounce.reset();

            if (intakeTurnState == IntakeTurnState.MIDDLE) {
                setIntakeTurn(IntakeTurnState.RIGHT);
            } else if (intakeTurnState == IntakeTurnState.RIGHT) {
                setIntakeTurn(IntakeTurnState.MIDDLE);
            }
        }
    }

    public void setIntakeTurn(IntakeTurnState state) {
        intakeTurnState = state;
    }

    public void setClaw(Gamepad gp) {
        if (gp.a) {
            if (clawDebounce.milliseconds() < 500) return;

            clawDebounce.reset();

            if (clawState == ClawState.OPEN) {
                setClaw(ClawState.CLOSED);
            } else {
                setClaw(ClawState.OPEN);
            }
        }
    }

    public void setClaw(ClawState state) {
        clawState = state;
    }

    public void setClawTurn(Gamepad gp) {
        if (gp.x) {
            if (clawTurnDebounce.milliseconds() < 500) return;

            clawTurnDebounce.reset();

            setClaw(ClawState.CLOSED);

            if (clawTurnState == ClawTurnState.IN) {
                setClawTurn(ClawTurnState.OUT);
            } else {
                setClawTurn(ClawTurnState.IN);
            }
        }
    }

    public void setClawTurn(ClawTurnState state) {
        clawTurnState = state;
    }

    public void returnSequence(Gamepad gp) {
        if (gp.dpad_up) {
            if (returnSequenceDebounce.milliseconds() < 500) return;

            returnSequenceDebounce.reset();

            if (!initiatedReturnSequence) {
                setClawTurn(ClawTurnState.IN);
            }

            initiatedReturnSequence = !initiatedReturnSequence;
        }

        returnSequence();
    }

    public void returnSequence() {
        if (initiatedReturnSequence) {
            if (horizontalSlide.getCurrentPosition() < -100) {
                horizontalSlideControl(1.0);
            } else if (horizontalSlide.getCurrentPosition() < -50) {
                horizontalSlideControl(0.3);
            } else {
                horizontalSlideControl(0.0);
                initiatedReturnSequence = false;
            }
        }
    }

    public void setLock(Gamepad gp) {
        if (gp.y) {
            if (lockDebounce.milliseconds() < 500) return;

            lockDebounce.reset();

            lock = !lock;
        }
    }

    public void driveTo(int pos) {
        if (pos == 0) return;

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        brake();

        final int DELAY = 20;

        while ((Math.abs(pos - flDrive.getCurrentPosition()) + Math.abs(pos - frDrive.getCurrentPosition()) / 2) > 20) {
            // Drive
            int flDistance = pos - flDrive.getCurrentPosition();
            int frDistance = pos - frDrive.getCurrentPosition();
            int blDistance = pos - blDrive.getCurrentPosition();
            int brDistance = pos - brDrive.getCurrentPosition();

            flDrivePower = (double)flDistance / (double)Math.abs(pos);
            blDrivePower = (double)blDistance / (double)Math.abs(pos);
            frDrivePower = (double)frDistance / (double)Math.abs(pos);
            brDrivePower = (double)brDistance / (double)Math.abs(pos);

            if (Math.abs(pos - flDrive.getCurrentPosition()) > 150 && Math.abs(pos - frDrive.getCurrentPosition()) > 150) {
                flDrivePower = Math.signum(flDrivePower) * 0.9;
                frDrivePower = Math.signum(frDrivePower) * 0.9;
                blDrivePower = Math.signum(blDrivePower) * 0.9;
                brDrivePower = Math.signum(brDrivePower) * 0.9;
            } else {
                flDrivePower = (flDrivePower) + (Math.signum(flDrivePower) * 0.1);
                frDrivePower = (frDrivePower) + (Math.signum(frDrivePower) * 0.1);
                blDrivePower = (blDrivePower) + (Math.signum(blDrivePower) * 0.1);
                brDrivePower = (brDrivePower) + (Math.signum(brDrivePower) * 0.1);
            }

            // Slowdown
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
    }

    public void driveTo(int pos, double slowdown) {
        if (pos == 0) return;

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        brake();

        final int DELAY = 20;

        while (((Math.abs(pos - flDrive.getCurrentPosition()) + Math.abs(pos - frDrive.getCurrentPosition())) / 2) > 20) {
            // Drive
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

            // Slowdown
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
    }

    public void strafeTo(int pos) {
        double targetAngle = 0;

        double currentAngle = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double errorAngle = targetAngle - currentAngle;

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        brake();

        final int DELAY = 20;

        int flBrPos = pos;
        int frBlPos = -pos;

        while (Math.abs((Math.abs(flBrPos - flDrive.getCurrentPosition()) + Math.abs(frBlPos - frDrive.getCurrentPosition())) / 2) > 20) {
            int flDistance = flBrPos - flDrive.getCurrentPosition();
            int frDistance = frBlPos - frDrive.getCurrentPosition();
            int blDistance = frBlPos - blDrive.getCurrentPosition();
            int brDistance = flBrPos - brDrive.getCurrentPosition();

            flDrivePower = (double)flDistance / (double)Math.abs(flBrPos);
            blDrivePower = (double)blDistance / (double)Math.abs(frBlPos);
            frDrivePower = (double)frDistance / (double)Math.abs(frBlPos);
            brDrivePower = (double)brDistance / (double)Math.abs(flBrPos);

            if (Math.abs(flBrPos - brDrive.getCurrentPosition()) > 100 || Math.abs(flBrPos - flDrive.getCurrentPosition()) > 100) {
                flDrivePower = Math.signum(flDrivePower);
                frDrivePower = Math.signum(frDrivePower);
                blDrivePower = Math.signum(blDrivePower);
                brDrivePower = Math.signum(brDrivePower);
            } else {
                flDrivePower = (flDrivePower) + (Math.signum(flDrivePower) * 0.15);
                frDrivePower = (frDrivePower) + (Math.signum(frDrivePower) * 0.15);
                blDrivePower = (blDrivePower) + (Math.signum(blDrivePower) * 0.15);
                brDrivePower = (brDrivePower) + (Math.signum(brDrivePower) * 0.15);
            }

            // Correct angle
            currentAngle = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            errorAngle = targetAngle - currentAngle;

            double kp = 1;

            double proportional = errorAngle * kp;

            double turn = proportional / (180 * kp);

            double flDrivePowerCorrection = -turn;
            double frDrivePowerCorrection = turn;
            double blDrivePowerCorrection = -turn;
            double brDrivePowerCorrection = turn;

            flDrivePowerCorrection = (flDrivePowerCorrection / 2) + (Math.signum(flDrivePowerCorrection) * 0.1);
            frDrivePowerCorrection = (frDrivePowerCorrection / 2) + (Math.signum(frDrivePowerCorrection) * 0.1);
            blDrivePowerCorrection = (blDrivePowerCorrection / 2) + (Math.signum(blDrivePowerCorrection) * 0.1);
            brDrivePowerCorrection = (brDrivePowerCorrection / 2) + (Math.signum(brDrivePowerCorrection) * 0.1);

            flDrivePower += flDrivePowerCorrection;
            frDrivePower += frDrivePowerCorrection;
            blDrivePower += blDrivePowerCorrection;
            brDrivePower += brDrivePowerCorrection;

            // Slowdown
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
    }

    public void driveAndStrafe(int drivePos, int strafePos) {
        double targetAngle = 0;

        double currentAngle = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double errorAngle = targetAngle - currentAngle;

        brake();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        brake();

        final int DELAY = 20;

        int flBrPos = (int)(1.5 * (double)strafePos + (double)drivePos);
        int frBlPos = (int)(-1.5 * (double)strafePos + (double)drivePos);

        while (Math.abs((Math.abs(flBrPos - flDrive.getCurrentPosition()) + Math.abs(frBlPos - frDrive.getCurrentPosition())) / 2) > 15) {
            int flDistance = flBrPos - flDrive.getCurrentPosition();
            int frDistance = frBlPos - frDrive.getCurrentPosition();
            int blDistance = frBlPos - blDrive.getCurrentPosition();
            int brDistance = flBrPos - brDrive.getCurrentPosition();

            flDrivePower = (double)flDistance / (double)Math.abs(flBrPos);
            blDrivePower = (double)blDistance / (double)Math.abs(frBlPos);
            frDrivePower = (double)frDistance / (double)Math.abs(frBlPos);
            brDrivePower = (double)brDistance / (double)Math.abs(flBrPos);

            flDrivePower = (flDrivePower) + (Math.signum(flDrivePower) * 0.15);
            frDrivePower = (frDrivePower) + (Math.signum(frDrivePower) * 0.15);
            blDrivePower = (blDrivePower) + (Math.signum(blDrivePower) * 0.15);
            brDrivePower = (brDrivePower) + (Math.signum(brDrivePower) * 0.15);

            // Correct angle
            currentAngle = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            errorAngle = targetAngle - currentAngle;

            double kp = 1;

            double proportional = errorAngle * kp;

            double turn = proportional / (180 * kp);

            double flDrivePowerCorrection = -turn;
            double frDrivePowerCorrection = turn;
            double blDrivePowerCorrection = -turn;
            double brDrivePowerCorrection = turn;

            flDrivePowerCorrection = (flDrivePowerCorrection / 2) + (Math.signum(flDrivePowerCorrection) * 0.1);
            frDrivePowerCorrection = (frDrivePowerCorrection / 2) + (Math.signum(frDrivePowerCorrection) * 0.1);
            blDrivePowerCorrection = (blDrivePowerCorrection / 2) + (Math.signum(blDrivePowerCorrection) * 0.1);
            brDrivePowerCorrection = (brDrivePowerCorrection / 2) + (Math.signum(brDrivePowerCorrection) * 0.1);

            flDrivePower += flDrivePowerCorrection;
            frDrivePower += frDrivePowerCorrection;
            blDrivePower += blDrivePowerCorrection;
            brDrivePower += brDrivePowerCorrection;

            // Slowdown
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
    }

    public void turnToFromHere(double target) {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentPosition = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = target - currentPosition;

        double kp = 1;

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
    }

    public void turnTo(double target) {
        this.imu.resetYaw();

        if (target == 0) return;

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentPosition = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = target - currentPosition;

        double kp = 1;

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

        this.imu.resetYaw();
    }

    public void armToFromHere(int pos) {
        if (pos == 0) return;

        arm.setPower(0.0);

        final int DELAY = 20;

        while (Math.abs(pos - arm.getCurrentPosition()) > 10) {
            // Drive
            int distance = pos - arm.getCurrentPosition();

            armPower = (double)distance / (double)Math.abs(pos);

            if (Math.abs(pos - arm.getCurrentPosition()) > 150) {
                armPower = Math.signum(armPower);
            } else {
                armPower = (armPower) + (Math.signum(armPower) * 0.1);
            }

            // Slowdown
            armPower = armPower * driveSlowdown;

            be();

            sleep(DELAY);
        }

        arm.setPower(0.0);
    }

    private void setDriveMode(DcMotor.RunMode mode) {
        flDrive.setMode(mode);
        frDrive.setMode(mode);
        blDrive.setMode(mode);
        brDrive.setMode(mode);
    }

    public void resetGyro() {
        this.imu.resetYaw();
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

    public enum DriveMode {
        NORMAL,
        GYRO
    }

    public enum IntakeTurnState {
        LEFT,
        MIDDLE,
        RIGHT
    }

    public enum ClawState {
        OPEN,
        CLOSED
    }

    public enum ClawTurnState {
        IN,
        OUT
    }
}