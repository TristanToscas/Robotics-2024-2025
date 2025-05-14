package org.firstinspires.ftc.teamcode.SubTeles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Class.HolonomicDrive;

@TeleOp (name = "MainTeleOp", group = "TeleOp")
public class HoloTeleOp extends LinearOpMode {
    HolonomicDrive holonomicDrive;

    DcMotor intake;
    ServoImplEx Sg;

    /*
    DcMotorEx spoolMotor, armMotor;

    private ElapsedTime runtime = new ElapsedTime();


    public static double Sp = 0.01, Si = 0.03, Sd = 0.0002;
    public static double Sf = 0.001;

    public static double Ap = 0.01, Ai = 0.004, Ad = 0.001;
    public static double Af = 0.04;

    public int SpoolTarget = 0;
    public int ArmTarget = 0;

    public final double SpoolTicks_In_Degree = 560 / 180.0;
    public final double ArmTicks_In_Degree = 840 / 180.0;
     */

    double INTAKE_SPEED = 1.0;
    double DRIVETRAIN_SPEED = 1.0;

    public static double Cpos = 0.59;
    public static double Opos = 0.2;

    //public int Scount = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        holonomicDrive = new HolonomicDrive(hardwareMap, telemetry);

        Sg = hardwareMap.get(ServoImplEx.class, "Gs");
        Sg.setPwmRange(new PwmControl.PwmRange(500, 2500));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intake = hardwareMap.dcMotor.get("intake");

        telemetry.addData("robot ready","");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            holonomicDrive.ActiveDriveRO(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, DRIVETRAIN_SPEED);
            DRIVETRAIN_SPEED = gamepad1.left_bumper ? 0.5 : 1;

            INTAKE_SPEED = gamepad1.right_bumper ? 0.5 : 1;

            if (gamepad1.dpad_up)
            {
                intake.setPower(INTAKE_SPEED);
            }
            else if (gamepad1.dpad_down)
            {
                intake.setPower(-INTAKE_SPEED);
            }
            else
            {
                intake.setPower(0);
            }

            if(gamepad1.b)
            {
                Sg.setPosition(Cpos);
            }
            if (gamepad1.a)
            {
                Sg.setPosition(Opos);
            }
        }
    }
}
