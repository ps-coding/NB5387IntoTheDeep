package org.firstinspires.ftc.teamcode.autonomous.place;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ITDRobot;

@Autonomous(name = "Place", group = "Place")
public class Place extends LinearOpMode {
    ITDRobot robot = new ITDRobot(telemetry);

    @Override
    public void runOpMode() {
        // Init
        robot.init(hardwareMap);

        // Wait
        waitForStart();

        // Go up to basket
        robot.driveToInches(12);
        robot.tinyStrafe(-15);
        robot.turn(135);

        // Extend
        robot.barToggle();
        robot.extensionToggle();
        robot.be(700);
        robot.extensionToggle();
        robot.be(700);
        robot.extensionToggle();
        robot.be(700);
        robot.clawToggle();
        robot.be(700);
        robot.barToggle();
        robot.be(700);
    }
}