package org.firstinspires.ftc.teamcode.autonomous.place;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ITDRobot;
//potato
@Autonomous(name = "Right", group = "Standard")
public class Right extends LinearOpMode {
    ITDRobot robot = new ITDRobot(telemetry);

    @Override
    public void runOpMode() {
        // Init
        robot.init(hardwareMap);
        waitForStart();

        // High bar specimen

        // Drive a bit forward for clearance
        robot.driveTo(50);

        // Claw right for specimen, arm up, claw spinning in
        robot.setIntakeTurn(ITDRobot.IntakeTurnState.RIGHT);
        robot.armControl(-1.0);
        robot.intakeControl(-1.0);

        // Keep going
        robot.driveTo(760);

        // Stop arm from going further up
        robot.armToFromHere(-2715);
        robot.armControl(0.05);
        robot.be();

        // Move to bar
        robot.driveAndStrafe(225, -400);

        // Lower the arm to place specimen
        robot.armControl(-0.7);
        robot.be();

        // Drag specimen back
        robot.driveTo(-680);
        robot.turnToFromHere(0);

        // Claw spinning out, arm up
        robot.intakeControl(1.0);
        robot.armControl(0.5);
        robot.be(500);

        // Bring sample back

        // Claw center, claw spinning in, arm up, slide out
        robot.setIntakeTurn(ITDRobot.IntakeTurnState.MIDDLE);
        robot.intakeControl(-1.0);
        robot.be();

        // Bring it back
        robot.strafeTo(1615);
        robot.turnToFromHere(0);
        robot.driveTo(2000);
        robot.strafeTo(340);
        robot.turnToFromHere(0);
        robot.armToFromHere(0);
        robot.armControl(0.05);
        robot.driveTo(-1800);

        // Second high bar specimen

        // Align with sample
        robot.armControl(-0.7);
        robot.be();
        robot.driveAndStrafe(925, 150);
        robot.turnToFromHere(172);
        robot.resetGyro();

        // Stop arm
        robot.armControl(0.0);
        robot.armToFromHere(-4500);
        robot.armControl(0.0);
        robot.be(1000);

        // Move forward to intake
        robot.driveTo(850, 3);

        // Raise arm
        robot.strafeTo(300);
        robot.armControl(0.6);
        robot.be(100);

        robot.turnTo(172);

        // Stop arm from raising too far and turn claw
        robot.armToFromHere(-2600);
        robot.armControl(0.0);
        robot.setIntakeTurn(ITDRobot.IntakeTurnState.RIGHT);
        robot.be(500);

        // Go to bar
        robot.driveAndStrafe(100, -1500);

        // Lower the arm to place specimen
        robot.armControl(-0.7);
        robot.be();

        // Drag specimen back
        robot.driveTo(-680);

        // Claw spinning out, arm up
        robot.intakeControl(1.0);
        robot.armControl(0.3);
        robot.be(1000);

        // Stop arm
        robot.armControl(0.0);
        robot.be();

        // Park
        robot.setIntakeTurn(ITDRobot.IntakeTurnState.LEFT);
        robot.strafeTo(2100);

//        // Again
//        robot.turnToFromHere(172);
//        robot.resetGyro();
//        robot.armControl(-0.7);
//        robot.be(400);
//
//        // Stop arm
//        robot.armControl(0.0);
//        robot.be(400);
//
//        // Move forward to intake
//        robot.driveTo(650, 3);
//
//        // Raise arm
//        robot.armControl(0.6);
//        robot.be();
//
//        robot.turnTo(172);
//
//        // Stop arm from raising too far and turn claw
//        robot.armControl(0.0);
//        robot.setIntakeTurn(ITDRobot.IntakeTurnState.RIGHT);
//        robot.be();
//
//        // Go to bar
//        robot.driveAndStrafe(-250, -1450);
//
//        // Lower the arm to place specimen
//        robot.armControl(-0.7);
//        robot.be();
//
//        // Drag specimen back
//        robot.driveTo(-680);
//
//        // Claw spinning out, arm up
//        robot.intakeControl(1.0);
//        robot.armControl(0.3);
//        robot.be(500);
//
//        // Stop arm
//        robot.armControl(0.0);
//        robot.be();
    }
}