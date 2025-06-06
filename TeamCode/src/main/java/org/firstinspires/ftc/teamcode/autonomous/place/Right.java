package org.firstinspires.ftc.teamcode.autonomous.place;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ITDRobot;

@Autonomous(name = "PowerRight", group = "Power")
public class PowerRight extends LinearOpMode {
    ITDRobot robot = new ITDRobot(telemetry);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();

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
        robot.driveAndStrafe(340, -400);

        // Lower the arm to place specimen
        robot.armControl(-0.8);
        robot.be(600);

        // Stop arm from lowering too far and breaking
        robot.armControl(-0.1);
        robot.be();

        // Drag specimen back
        robot.driveTo(-680);

        // Clear up holder

        // Claw spinning out, arm up
        robot.intakeControl(1.0);
        robot.armControl(0.3);
        robot.be(700);

        // Second high bar specimen

        // Claw center, claw spinning in
        robot.setIntakeTurn(ITDRobot.IntakeTurnState.MIDDLE);
        robot.intakeControl(-1.0);
        robot.armControl(0.1);
        robot.be(500);

        // Align with sample
        robot.driveAndStrafe(500, 700);
        robot.turnToFromHere(-120);

        // Arm very low for intake
        robot.armControl(-0.6);
        robot.be(300);

        // Stop arm
        robot.armControl(0.0);
        robot.be(1500);

        // Move forward to intake
        robot.driveTo(700);

        // Raise arm
        robot.armControl(1.0);
        robot.be(850);

        // Stop arm from raising too far
        robot.armControl(0.0);
        robot.be();


//        for (int i = 0; i < 1; i ++) {
//            // Extend horizontal slide
//            robot.horizontalSlideControl(-1.0);
//            robot.be();
//
//            while (robot.horizontalSlide.getCurrentPosition() > -1150) {
//                robot.sleep(50);
//            }
//
//            robot.horizontalSlideControl(0.0);
//            robot.be();
//
//            // Open
//            robot.setClawTurn(ITDRobot.ClawTurnState.OUT);
//            robot.setClaw(ITDRobot.ClawState.OPEN);
//            robot.be(800);
//
//            // Close and return
//            robot.setClaw(ITDRobot.ClawState.CLOSED);
//            robot.be(1000);
//            robot.setClawTurn(ITDRobot.ClawTurnState.IN);
//            robot.be(800);
//
//            robot.initiatedReturnSequence = true;
//
//            while (robot.initiatedReturnSequence) {
//                if (robot.horizontalSlide.getCurrentPosition() < -250) {
//                    robot.horizontalSlideControl(1.0);
//                } else {
//                    robot.horizontalSlideControl(0.0);
//                    robot.initiatedReturnSequence = false;
//                }
//
//                robot.be();
//            }
//
//            // Drop
//            robot.turnTo(85);
//            robot.turnTo(85);
//            robot.setClawTurn(ITDRobot.ClawTurnState.OUT);
//            robot.be(1000);
//            robot.setClaw(ITDRobot.ClawState.OPEN);
//            robot.be(800);
//
//            robot.setClaw(ITDRobot.ClawState.CLOSED);
//            robot.setClawTurn(ITDRobot.ClawTurnState.IN);
//            robot.be(800);
//        }
    }
}