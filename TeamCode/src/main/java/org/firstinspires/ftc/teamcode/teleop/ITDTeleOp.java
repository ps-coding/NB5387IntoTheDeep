package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.ITDRobot;

@TeleOp(name = "ITDTeleOp", group = "TeleOp")
public class ITDTeleOp extends OpMode {
    ITDRobot robot = new ITDRobot(telemetry);

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        robot.gamePadPower(gamepad1, gamepad2);
    }
}