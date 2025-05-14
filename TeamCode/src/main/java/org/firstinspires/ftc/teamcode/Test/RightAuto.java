package org.firstinspires.ftc.teamcode.Test;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous (name = "TestAuto1", group = "Testing")
public class RightAuto extends LinearOpMode {
    public static class Clamp
    {
        private ServoImplEx Sg;

        //Instantiates Clamp class when called on line 92
        public Clamp(HardwareMap hardwareMap)
        {
            Sg = hardwareMap.get(ServoImplEx.class, "Gs");
            Sg.setPwmRange(new PwmControl.PwmRange(500, 2500));
        }
        //Class that tells the servo to run to the position 0.2
        public class OpenClamp implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                Sg.setPosition(0.2);
                return false;
            }
        }
        //Actual action that implements the OpenClamp class
        public Action openClamp()
        {
            return new OpenClamp();
        }
        //Class that tells the servo to run to the position 0.59
        public class CloseClamp implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    Sg.setPosition(0.5);
                    return false;
            }
        }
        //Actual action that implements the CloseClamp class
        public Action closeClamp() {
            return new CloseClamp();
        }
    }

    public static class Intake
    {
        DcMotor intake;
        public Intake(HardwareMap hardwareMap)
        {
            intake = hardwareMap.dcMotor.get("intake");
        }
        public class On implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intake.setPower(0.85);
                return false;
            }
        }
        public class OnN implements Action
        {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket)
            {
                intake.setPower(-0.85);
                return false;
            }
        }

        public Action on()
        {
            return new On();
        }

        public Action negative()
        {
            return new OnN();
        }

        public class Off implements Action
        {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intake.setPower(0);
                return false;
            }
        }
        public Action off()
        {
            return new Off();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //Create clamp and intake objects so we can use later in line 117-126
        Clamp clamp = new Clamp(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        //Sets initial position so we can create MecanumDrive object
        Pose2d initialPose = new Pose2d(52, 14, Math.toRadians(180));
        Pose2d secondPose = new Pose2d(58, 0, Math.toRadians(180));
        Pose2d thirdPose = new Pose2d(28, 40, Math.toRadians(125));
        Pose2d fourthPose = new Pose2d(24, 40, Math.toRadians(125));
        Pose2d fifthPose = new Pose2d(60, 0, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        //Different tabs that allow different actions to be ran
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(90))
                .lineToY(0)
                .setTangent(Math.toRadians(0))
                .lineToX(60);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(secondPose)
                .lineToX(58)
                .strafeToLinearHeading(new Vector2d(28, 40), Math.toRadians(125));
        TrajectoryActionBuilder tab3 = drive.actionBuilder(thirdPose)
                .lineToX(24);
        TrajectoryActionBuilder tab4 = drive.actionBuilder(fourthPose)
                .strafeToLinearHeading(new Vector2d(58, 0), Math.toRadians(-180))
                .setTangent(0)
                .lineToX(60);
        TrajectoryActionBuilder tab5 = drive.actionBuilder(fifthPose)
                        .lineToX(48);
        waitForStart();

        Actions.runBlocking(

                new SequentialAction(
                        //Build first instructions to place donut on alliance stake
                        tab1.build(),
                        new SleepAction(0.1),
                        intake.on(),
                        new SleepAction(0.7),
                        intake.negative(),
                        //Builds second instructions to get a second donut
                        tab2.build(),
                        //Builds third instructions to score second donut on alliance stake
                        new ParallelAction(
                                tab3.build(),
                                intake.on()
                        ),
                        new SleepAction(0.3),
                        intake.off(),
                        tab4.build(),
                        new SleepAction(0.7),
                        intake.on(),
                        new SleepAction(0.7),
                        intake.negative(),
                        new SleepAction(0.5),
                        intake.off(),
                        tab5.build()
                )
        );
    }
}