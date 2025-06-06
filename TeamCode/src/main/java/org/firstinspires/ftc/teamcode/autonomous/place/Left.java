package org.firstinspires.ftc.teamcode.autonomous.place;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ITDRobot;

@Autonomous(name = "PowerLeft", group = "Power")
public class PowerLeft extends LinearOpMode {
    ITDRobot robot = new ITDRobot(telemetry);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();

        robot.sleep(2000);

        // High bar specimen

        // Drive a bit forward for clearance
        robot.driveTo(600);

        // Claw right for specimen, arm up, claw spinning in
        robot.setIntakeTurn(ITDRobot.IntakeTurnState.RIGHT);
        robot.armControl(-1.0);
        robot.intakeControl(-1.0);
        robot.be(950);

        // Stop arm from going further up
        robot.armControl(0.05);
        robot.be(100);

        // Move to bar
        robot.strafeTo(1300);
        robot.driveTo(360);

        // Lower the arm to place specimen
        robot.armControl(-0.8);
        robot.be(600);

        // Stop arm from lowering too far and breaking
        robot.armControl(-0.1);
        robot.be();

        // Drag specimen back
        robot.driveTo(-750);

        // Clear up holder

        // Claw spinning out, arm up
        robot.intakeControl(1.0);
        robot.armControl(0.3);
        robot.be(700);

        // High basket sample

        // Claw center, claw spinning in
        robot.setIntakeTurn(ITDRobot.IntakeTurnState.MIDDLE);
        robot.intakeControl(-1.0);
        robot.armControl(0.1);
        robot.be(500);

        // Align with sample
        robot.strafeTo(-1000);
        robot.turnTo(60);
        robot.strafeTo(985);

        // Arm very low for intake
        robot.armControl(-0.6);
        robot.be(300);

        // Stop arm
        robot.armControl(0.0);
        robot.be();

        // Move forward to intake
        robot.driveTo(700);

        // Raise arm
        robot.armControl(1.0);
        robot.be(850);

        // Stop arm from raising too far
        robot.armControl(0.0);
        robot.be();

        // Line up with basket
        robot.strafeTo(-300);
        robot.turnTo(70);
        robot.driveTo(325);

        // Extend linear slide
        robot.linearSlideControl(-0.8);
        robot.be(900);

        // Stop linear slide
        robot.linearSlideControl(-0.1);
        robot.be();

        // Move in front of basket
        robot.driveTo(430, 1.7);

        // Claw spinning out
        robot.intakeControl(1.0);
        robot.be(1000);

        // Reset to initial position
        robot.linearSlideControl(0.6);
        robot.armControl(0.7);
        robot.be(1500);
    }
}