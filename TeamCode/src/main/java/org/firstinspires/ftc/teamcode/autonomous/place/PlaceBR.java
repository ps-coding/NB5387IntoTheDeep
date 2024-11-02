package org.firstinspires.ftc.teamcode.autonomous.place;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ITDRobot;

@Autonomous(name = "PlaceBR", group = "Place")
public class PlaceBR extends LinearOpMode {
    ITDRobot robot = new ITDRobot(telemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        robot.tinyStrafe(-1);
        robot.driveToInches(94);
    }
}