package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp (name = "MotorTest", group = "Testing")
public class MotorTest extends OpMode {
    DcMotor intake;
    ServoImplEx Sg;

    public static double Cpos = 0.55;
    public static double Opos = 0;

    public int Scount = 0;
    @Override
    public void init() {
        intake = hardwareMap.dcMotor.get("intake");
        Sg = hardwareMap.get(ServoImplEx.class, "Gs");
        Sg.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_down)
        {
            intake.setPower(1);
        }
        else if (gamepad1.dpad_up)
        {
            intake.setPower(-1);
        }
        else
        {
            intake.setPower(0);
        }

        if(gamepad1.b && Scount == 0)
        {
            Sg.setPosition(Cpos);
            Scount++;
        } else if (gamepad1.b && Scount == 1) {
            Sg.setPosition(Opos);
           Scount--;
        }
    }
}
