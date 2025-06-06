package org.firstinspires.ftc.teamcode.autonomous.place;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ITDRobot;

@Autonomous(name = "Left", group = "Standard")
public class Left extends LinearOpMode {
    ITDRobot robot = new ITDRobot(telemetry);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();

        // First high basket

        // Drive a bit forward for clearance
        robot.driveAndStrafe(75,150);

        // Claw right for specimen, arm up, claw spinning in
        robot.setIntakeTurn(ITDRobot.IntakeTurnState.RIGHT);
        robot.armControl(-1.0);
        robot.intakeControl(-1.0);

        // Keep going
        robot.driveTo(660);

        // Stop arm from going further up
        robot.armToFromHere(-2800);
        robot.armControl(0.05);
        robot.be();

        // Move to basket area
        robot.turnToFromHere(120);
        robot.resetGyro();

        // Extend linear slide
        robot.setIntakeTurn(ITDRobot.IntakeTurnState.MIDDLE);
        robot.linearSlideControl(-1.0);
        robot.be();
        while (robot.leftLinearSlide.getCurrentPosition() > -2140) {
            robot.sleep(50);
        }
        robot.linearSlideControl(-0.1);
        robot.be();

        // Move in front of basket
        robot.driveTo(840, 1.8);

        // Adjust
        robot.armToFromHere(-2900);
        robot.armControl(0.05);
        robot.be();

        // Claw spinning out
        robot.intakeControl(1.0);
        robot.be(1000);

        robot.armToFromHere(-2300);
        robot.armControl(0.05);
        robot.be();

        // Slide back
        robot.driveTo(-400);

        robot.linearSlideControl(1.0);
        robot.intakeControl(-1.0);
        robot.be();

        robot.strafeTo(-180);
        robot.turnTo(-90);
        robot.driveTo(-350);

        // Reset to initial position
        while (robot.leftLinearSlide.getCurrentPosition() < 0) {
            robot.sleep(50);
        }
        robot.linearSlideControl(0.0);
        robot.be();

        // Second high basket

        // Arm very low for intake
        robot.armControl(-1.0);
        robot.be(1700);

        // Stop arm
        robot.armControl(0.0);
        robot.be();
        robot.armToFromHere(-4600);
        robot.armControl(0.0);
        robot.be();

        // Move forward to intake
        robot.driveTo(800, 5);

        // Raise arm
        robot.armToFromHere(-2600);
        robot.armControl(0.0);
        robot.be();

        // Line up with basket
        robot.turnTo(90);
        robot.strafeTo(-200);
        robot.driveTo(200);

        // Extend linear slide
        robot.linearSlideControl(-0.8);
        robot.be(900);

        // Stop linear slide
        robot.linearSlideControl(-0.1);
        robot.be();

        // Move in front of basket
        robot.driveTo(860, 1.7);

        // Claw spinning out
        robot.intakeControl(1.0);
        robot.be(1000);

        // Reset to initial position
        robot.linearSlideControl(0.6);
        robot.armControl(0.7);
        robot.be(1500);
    }
}