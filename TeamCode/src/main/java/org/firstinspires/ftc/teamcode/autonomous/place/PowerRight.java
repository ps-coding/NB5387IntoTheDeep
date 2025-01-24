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
        robot.straighten();

        // Claw right for specimen, arm up, claw spinning in
        robot.setIntakeTurn(ITDRobot.IntakeTurnState.RIGHT);
        robot.armControl(-1.0);
        robot.intakeControl(-1.0);
        robot.be(950);

        // Stop arm from going further up
        robot.armControl(0.05);
        robot.be(100);

        // Move to bar
        robot.strafeTo(-400);
        robot.driveTo(340);
        robot.straighten();

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

        // Stop arm up
        robot.setIntakeTurn(ITDRobot.IntakeTurnState.LEFT);
        robot.armControl(1.0);
        robot.intakeControl(0.0);
        robot.be(1500);
        robot.armControl(0.1);
        robot.be();

        // Park
        robot.driveTo(175);
        robot.straighten();
        robot.strafeTo(2200);

        // Stop arm
        robot.armControl(0.0);
        robot.be();

        for (int i = 0; i < 2; i ++) {
            // Extend horizontal slide
            robot.horizontalSlideControl(-1.0);
            robot.be();

            while (robot.horizontalSlide.getCurrentPosition() > -1150) {
                robot.sleep(50);
            }

            robot.horizontalSlideControl(0.0);
            robot.be();

            // Open
            robot.setClawTurn(ITDRobot.ClawTurnState.OUT);
            robot.setClaw(ITDRobot.ClawState.OPEN);
            robot.be(800);

            // Close and return
            robot.setClaw(ITDRobot.ClawState.CLOSED);
            robot.be(1000);
            robot.setClawTurn(ITDRobot.ClawTurnState.IN);
            robot.be(800);

            robot.initiatedReturnSequence = true;

            while (robot.initiatedReturnSequence) {
                if (robot.horizontalSlide.getCurrentPosition() < -100) {
                    robot.horizontalSlideControl(1.0);
                } else if (robot.horizontalSlide.getCurrentPosition() < -50) {
                    robot.horizontalSlideControl(0.3);
                } else {
                    robot.horizontalSlideControl(0.0);
                    robot.initiatedReturnSequence = false;
                }

                robot.be();
            }

            // Drop
            robot.driveTo(-300);
            robot.setClawTurn(ITDRobot.ClawTurnState.OUT);
            robot.be(1000);
            robot.setClaw(ITDRobot.ClawState.OPEN);
            robot.be(800);

            robot.setClaw(ITDRobot.ClawState.CLOSED);
            robot.setClawTurn(ITDRobot.ClawTurnState.IN);
            robot.be(800);

            robot.strafeTo(250);
            robot.driveTo(300);
        }
    }
}