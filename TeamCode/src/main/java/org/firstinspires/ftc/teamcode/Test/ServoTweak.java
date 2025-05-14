package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@TeleOp(name = "ServoTweak", group = "Testing")

public class ServoTweak extends OpMode
{
    ServoImplEx Sg;
    public static double Opos = 0.55;
    public static double Cpos = 0;
    int Scount = 0;
    @Override
    public void init()
    {
        Sg = hardwareMap.get(ServoImplEx.class, "Gs");
        Sg.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }
    @Override
    public void loop()
    {
        if(gamepad1.b && Scount == 0)
        {
            Sg.setPosition(Opos);
            Scount++;
        }
        else if (gamepad1.b && Scount == 1)
        {
            Sg.setPosition(Cpos);
            Scount--;
        }
    }
}
