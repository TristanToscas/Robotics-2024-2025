package org.firstinspires.ftc.teamcode.SubTeles;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class TestTeleOp extends OpMode {
    DcMotor lf, rf, lb, rb;
    @Override
    public void init() {
        lf = hardwareMap.dcMotor.get("lf");
        rf = hardwareMap.dcMotor.get("rf");
        lb = hardwareMap.dcMotor.get("lb");
        rb = hardwareMap.dcMotor.get("rb");

        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        if(gamepad1.left_stick_x > 0.3 || gamepad1.left_stick_x < -0.3) {
            lf.setPower(-gamepad1.left_stick_x);
            rf.setPower(gamepad1.left_stick_x);
            lb.setPower(-gamepad1.left_stick_x);
            rb.setPower(gamepad1.left_stick_x);
        } else {
            lf.setPower(0);
            rf.setPower(0);
            lb.setPower(0);
            rb.setPower(0);
        }

        if(gamepad1.left_stick_x < 0.3 || gamepad1.left_stick_x > -0.3) {
            lf.setPower(-gamepad1.left_stick_x);
            rf.setPower(-gamepad1.left_stick_x);
            lb.setPower(gamepad1.left_stick_x);
            rb.setPower(gamepad1.left_stick_x);
        } else {
            lf.setPower(0);
            rf.setPower(0);
            lb.setPower(0);
            rb.setPower(0);
        }
    }
}
