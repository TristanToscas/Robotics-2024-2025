package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(group = "Test")
public class pidTweak extends OpMode {
    private PIDController controller;
    //tune p first, then d, then lastly i, start small
    public static double p = 0, i = 0, d = 0;

    public static double f = 0;

    public static int target = 0;

    //40rpm = 560
    //60rpm = 840
    public final double ticks_in_degree = 560 / 180.0;

    DcMotorEx armMotor;
    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armMotor = hardwareMap.get(DcMotorEx.class, "aM");
    }

    @Override
    public void loop() {
        //Sets the controller pid to the variables p, i, and d
        controller.setPID(p, i, d);

        //Gets the positions of the motors
        int armPos = armMotor.getCurrentPosition();

        //Calculates how much to go based on the position to run to the target
        double pid = controller.calculate(armPos, target);

        //Calculating the Feed Forward so the robot adjusts to gravity
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        //Calculates power for all 4 motors
        double power = pid + ff;


        //Sets power to all the motors
        armMotor.setPower(power);


        //Adds data to the telemetry/driver hub
        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.update();
    }
}
