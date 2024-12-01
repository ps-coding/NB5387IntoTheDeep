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
        robot.setClaw(ITDRobot.ClawTurnState.RIGHT);
        robot.armControl(-1.0);
        robot.clawControl(-1.0);
        robot.be(950);

        // Stop arm from going further up
        robot.armControl(0.05);
        robot.be(100);

        // Move to bar
        robot.strafeTo(-400);
        robot.driveTo(340);

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
        robot.clawControl(1.0);
        robot.armControl(0.3);
        robot.be(700);

        // Park
        robot.driveTo(-210);
        robot.strafeTo(2000);
    }
}