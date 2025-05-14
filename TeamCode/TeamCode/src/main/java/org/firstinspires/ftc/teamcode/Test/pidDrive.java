package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@Autonomous(group = "Test")
public class pidDrive extends LinearOpMode
{
    //Pid Controller
    private PIDController controller;

    //Proportional, Integral, Derivative,
    public static double p = 0, i = 0, d = 0;

    //Feed forward
    public static double f = 0;

    //40rpm = 560
    //60rpm = 840
    public final double ticks_in_degree = 560 / 180.0;

    DcMotorEx rf, lf, rb, lb;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //Get new controller and telemetry
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Hardware map for the motors
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rb = hardwareMap.get(DcMotorEx.class, "rb");

        //Reverses left side direction
        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);

        //Wait for starts
        waitForStart();

        //Drive forward with PID
        PIDdrive(5);

    }
    public void PIDdrive(int moveTimes) {

        int target = 1120 * moveTimes;

        while (opModeIsActive())
        {
            //Gets the positions of the motors
            int rfPos = rf.getCurrentPosition();
            int lfPos = lf.getCurrentPosition();
            int rbPos = rb.getCurrentPosition();
            int lbPos = lb.getCurrentPosition();

            //Sets the controller pid to the variables p, i, and d
            controller.setPID(p, i, d);

            //Calculates how much to go based on the position to run to the target
            double pidRf = controller.calculate(rfPos, target);
            double pidLf = controller.calculate(lfPos, target);
            double pidRb = controller.calculate(rbPos, target);
            double pidLb = controller.calculate(lbPos, target);

            //Calculating the Feed Forward so the robot adjusts to gravity
            double ff = Math.cos(Math.toRadians(target/ ticks_in_degree)) * f;

            //Calculates power for all 4 motors
            double powerRf = pidRf + ff;
            double powerLf = pidLf + ff;
            double powerRb = pidRb + ff;
            double powerLb = pidLb + ff;

            //Sets power to all the motors
            rf.setPower(powerRf);
            lf.setPower(powerLf);
            rb.setPower(powerRb);
            lb.setPower(powerLb);

            //Adds data to the telemetry/driver hub
            telemetry.addData("pos", rfPos);
            telemetry.addData("pos", lfPos);
            telemetry.addData("pos", rbPos);
            telemetry.addData("pos", lbPos);
            telemetry.addData("target", target);

            //Updates telemetry
            telemetry.update();
        }
    }
}