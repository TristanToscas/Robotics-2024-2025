package org.firstinspires.ftc.teamcode.SubTeles;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "TeleOp")
public class SubTeleOp extends OpMode {

    //Declaring the DcMotor variable that stores the motor names
    DcMotor lf, rf, lb, rb;
    DcMotor spoolMotor, armMotor;
    Servo ls, rs;

    //Used to initiate all of the stuff you need before starting
    @Override
    public void init() {
        //Calling the motor names and applying the hardwareMap names to it so the driver hub knows what to power
        rf = hardwareMap.dcMotor.get("rf");
        lf = hardwareMap.dcMotor.get("lf");
        rb = hardwareMap.dcMotor.get("rb");
        lb = hardwareMap.dcMotor.get("lb");
        spoolMotor = hardwareMap.dcMotor.get("sm");
        armMotor = hardwareMap.dcMotor.get("am");
        ls = hardwareMap.servo.get("ls");
        rs = hardwareMap.servo.get("rs");

        //Reverses the motor direction of the left side
        lb.setDirection(DcMotor.Direction.REVERSE);
        lf.setDirection(DcMotor.Direction.REVERSE);
    }
    //Loops all the code that's here
    @Override
    public void loop() {
        //Left joystick moves the left side using if statement
        if(Math.abs(gamepad1.left_stick_y) > .2) {
            //Sets the power according to how far you move the joystick
            lf.setPower(-gamepad1.left_stick_y);
            lb.setPower(-gamepad1.left_stick_y);
        } else {
            //If you don't touch the joystick the power will be 0
            lf.setPower(0);
            lb.setPower(0);
        }

        //Right joystick moves the right side using if statement
        if(Math.abs(gamepad1.right_stick_y) > .2) {
            //Sets the power according to how far you move the joystick
            rf.setPower(-gamepad1.right_stick_y);
            rb.setPower(-gamepad1.right_stick_y);
        } else {
            //If you don't touch the joystick the power will be 0
            rf.setPower(0);
            rb.setPower(0);
        }

        //arm extender
        if (Math.abs(gamepad2.right_stick_y) >.2) {
            armMotor.setPower(1);
        } else {
            armMotor.setPower(0);
        }
        if(gamepad2.right_stick_y < -0.2) {
            armMotor.setPower(-1);
        } else {
            armMotor.setPower(0);
        }

        //Spins spool to extract linear slides
        if (gamepad2.dpad_up) {
            spoolMotor.setPower(1);
        } else {
            spoolMotor.setPower(0);
        }

        if (gamepad2.dpad_down) {
            spoolMotor.setPower(-1);
        } else {
            spoolMotor.setPower(0);
        }

        //Claw closing and opening
        if (gamepad2.a) {
            ls.setPosition(0.7);
            rs.setPosition(0.1);
        }
        if (gamepad2.b) {
            ls.setPosition(0.5);
            rs.setPosition(0.5);
        }
    }
}
