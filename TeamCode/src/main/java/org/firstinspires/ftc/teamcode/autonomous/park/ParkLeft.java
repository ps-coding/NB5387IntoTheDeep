package org.firstinspires.ftc.teamcode.autonomous.park;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ITDRobot;

@Autonomous(name = "ParkLeft", group = "Park")
public class ParkLeft extends LinearOpMode {
    ITDRobot robot = new ITDRobot(telemetry);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();

        robot.driveTo(130);
        robot.strafeTo(-960);
    }
}