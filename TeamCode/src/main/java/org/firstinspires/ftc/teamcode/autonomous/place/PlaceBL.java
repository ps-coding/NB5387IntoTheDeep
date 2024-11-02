package org.firstinspires.ftc.teamcode.autonomous.place;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ITDRobot;

@Autonomous(name = "PlaceBL", group = "Place")
public class PlaceBL extends LinearOpMode {
    ITDRobot robot = new ITDRobot(telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.clawClosed = true;
        robot.be();

        waitForStart();

        robot.tinyStrafe(1);
        robot.driveToInches(94);
        robot.barUp = true;
        robot.extensionUp = true;
        robot.be();
        robot.linearSlideUp();

        robot.clawClosed = false;
        robot.be();

        try {Thread.sleep(700);} catch (InterruptedException e) {}

        robot.linearSlideDown();

        stop();
    }
}