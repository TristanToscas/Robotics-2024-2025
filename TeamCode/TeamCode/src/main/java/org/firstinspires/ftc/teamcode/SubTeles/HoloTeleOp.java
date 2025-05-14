package org.firstinspires.ftc.teamcode.SubTeles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Class.HolonomicDrive;

@TeleOp (group = "TeleOp")
public class HoloTeleOp extends LinearOpMode {
    HolonomicDrive holonomicDrive;

    DcMotor intake;
    ServoImplEx Sg;

    DcMotorEx spoolMotor, armMotor;

    private ElapsedTime runtime = new ElapsedTime();

    private PIDController SpoolController;
    private PIDController ArmController;

    public static double Sp = 0.01, Si = 0.03, Sd = 0.0002;
    public static double Sf = 0.001;

    public static double Ap = 0.01, Ai = 0.004, Ad = 0.001;
    public static double Af = 0.04;

    public int SpoolTarget = 0;
    public int ArmTarget = 0;

    public final double SpoolTicks_In_Degree = 560 / 180.0;
    public final double ArmTicks_In_Degree = 840 / 180.0;

    double DRIVETRAIN_SPEED = 1.0;

    public static double Cpos = 0.55;
    public static double Opos = 0.2;

    public int Scount = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        holonomicDrive = new HolonomicDrive(hardwareMap, telemetry);

        Sg = hardwareMap.get(ServoImplEx.class, "Gs");
        Sg.setPwmRange(new PwmControl.PwmRange(500, 2500));

        SpoolController = new PIDController(Sp, Si, Sd);
        ArmController = new PIDController(Ap, Ai, Ad);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intake = hardwareMap.dcMotor.get("intake");

        telemetry.addData("robot ready","");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            holonomicDrive.ActiveDriveRO(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, DRIVETRAIN_SPEED);
            DRIVETRAIN_SPEED = gamepad1.left_bumper ? 0.5 : 1.0;

            if (gamepad1.dpad_up)
            {
                intake.setPower(1);
            }
            else if (gamepad1.dpad_down)
            {
                intake.setPower(-1);
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
    public void liftPID(int time, int target) {

        runtime.reset();
        while(runtime.seconds() >= 0 && runtime.seconds() <= time) {
            SpoolTarget = target;
            SpoolController.setPID(Sp, Si, Sd);

            //Gets the positions of the motors
            int spoolPos = spoolMotor.getCurrentPosition();

            //Calculates how much to go based on the position to run to the target
            double pid = SpoolController.calculate(spoolPos, target);

            //Calculating the Feed Forward so the robot adjusts to gravity
            double ff = Math.cos(Math.toRadians(target / SpoolTicks_In_Degree)) * Sf;

            double power = pid + ff;

            spoolMotor.setPower(power);

            telemetry.addData("pos", spoolPos);
            telemetry.addData("target", target);
            telemetry.update();
        }
    }

    public void armPID(int time, int target) {
        runtime.reset();
        while(runtime.seconds() >= 0 && runtime.seconds() <= time) {
            ArmTarget = target;
            ArmController.setPID(Ap, Ai, Ad);

            //Gets the positions of the motors
            int armPos = armMotor.getCurrentPosition();

            //Calculates how much to go based on the position to run to the target
            double pid = ArmController.calculate(armPos, target);

            //Calculating the Feed Forward so the robot adjusts to gravity
            double ff = Math.cos(Math.toRadians(target / ArmTicks_In_Degree)) * Af;

            double power = pid + ff;

            armMotor.setPower(power);

            telemetry.addData("pos", armPos);
            telemetry.addData("target", target);
            telemetry.update();
        }
    }
}
