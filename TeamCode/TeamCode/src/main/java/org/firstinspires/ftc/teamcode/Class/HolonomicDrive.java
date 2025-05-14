package org.firstinspires.ftc.teamcode.Class;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class HolonomicDrive
{
    ///// Create Motor Variables
    static DcMotor leftFront, leftBack, rightFront, rightBack;
    /////

    ///// Name of Drive Motors on Driver Hub
    private static final String leftFrontName  = "lf",
            leftBackName   = "lb",
            rightFrontName = "rf",
            rightBackName  = "rb";


    ///// Create IMU/gyro variables
    static BNO055IMU imu;
    Orientation angles = new Orientation();
    double initYaw;
    double adjustedYaw;

    ///// Create Motion Variables
    double accel = 0.5; // Determine how fast the robot should go to full speed

    // Actual motor power
    double  leftFront_power  = 0,
            leftBack_power   = 0,
            rightFront_power = 0,
            rightBack_power  = 0;

    // Intended motor power
    double  target_leftFront_power  = 0,
            target_leftBack_power   = 0,
            target_rightFront_power = 0,
            target_rightBack_power  = 0;
    /////

    ///// Create Input Variables
    double x, y, turn;
    /////

    static Telemetry telemetry;
    //cpr = 4 * PPR
    //40 = 4 * 280 = 1120
    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = 1;
    static final double WHEEL_DIAMETER_INCHES = 4.09449;

    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    private ElapsedTime runtime = new ElapsedTime();

    // This method is used to initialize the motors/drivetrain of the robot to holonomic/mecanum drive
    public HolonomicDrive(HardwareMap hardwareMap, Telemetry telemetry)
    {
        // Instantiate Motor Objects
        HolonomicDrive.leftFront  = hardwareMap.get(DcMotor.class, leftFrontName);
        HolonomicDrive.leftBack   = hardwareMap.get(DcMotor.class, leftBackName);
        HolonomicDrive.rightFront = hardwareMap.get(DcMotor.class, rightFrontName);
        HolonomicDrive.rightBack  = hardwareMap.get(DcMotor.class, rightBackName);

        // Instantiate IMU/gyro Objects
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        HolonomicDrive.imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        initYaw = angles.firstAngle;

        // Instantiate Telemetry
        HolonomicDrive.telemetry = telemetry;

        // If the joysticks aren't touched, the robot won't move (set to BRAKE)
        leftFront .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Display Message on Screen
        telemetry.addData("initializing", "motors");

        // Reverse Motor Directions for Positive Values
        rightBack .setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("reversing", "motors");
    }

    // This method is used to initialize the drive motors' modes
    public void InitAuto()
    {
        //Stops and resets the encoder on the motors
        leftFront .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack  .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Sets mode to run without the encoder (to maximize motor efficiency/power)
        leftFront .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack  .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("drive motors' mode set","");
    }

    // This method is used for robot-oriented driving in TeleOp
    public void ActiveDriveRO(double leftStickX, double leftStickY, double rightStickX, double DRIVETRAIN_SPEED)
    {
        double max; // Limit motor's powers to 100%

        // Cool vector math to calculate power to the drive motors
        x = -leftStickX;
        y = leftStickY;
        turn = -rightStickX;

        // Input variables in new version are power and theta
        double theta = Math.atan2(y,x);
        double power = Math.hypot(x,y);

        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        max = Math.max(Math.abs(sin), Math.abs(cos));

        // Combine variables to find power and set the intended power
        target_leftFront_power  = (power * cos / max + turn);
        target_leftBack_power   = (power * sin / max + turn);
        target_rightFront_power = (power * sin / max - turn);
        target_rightBack_power  = (power * cos / max - turn);

        if ((power + Math.abs(turn)) > 1.0) // Limit motor's powers to 100%
        {
            target_leftFront_power  /= power + Math.abs(turn);
            target_rightFront_power /= power + Math.abs(turn);
            target_leftBack_power   /= power + Math.abs(turn);
            target_rightBack_power  /= power + Math.abs(turn);
        }

        // Apply acceleration to the motor powers
        leftFront_power  += accel * (target_leftFront_power  - leftFront_power);
        leftBack_power   += accel * (target_leftBack_power   - leftBack_power);
        rightFront_power += accel * (target_rightFront_power - rightFront_power);
        rightBack_power  += accel * (target_rightBack_power  - rightBack_power);

        // Set motor powers to desired values
        leftFront.setPower(leftFront_power  * DRIVETRAIN_SPEED);
        leftBack.setPower(leftBack_power   * DRIVETRAIN_SPEED);
        rightFront.setPower(rightFront_power * DRIVETRAIN_SPEED);
        rightBack.setPower(rightBack_power  * DRIVETRAIN_SPEED);

        // Display motor power
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFront_power, rightFront_power);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBack_power, rightBack_power);
    }

    // This method is used for field-oriented driving in TeleOp
    public void ActiveDriveFO(double leftStickX, double leftStickY, double rightStickX, double DRIVETRAIN_SPEED)
    {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        adjustedYaw = angles.firstAngle - initYaw;

        // toggle field/normal
        double zeroedYaw = -initYaw + angles.firstAngle;

        // Cool vector math to calculate power to the drive motors
        x    = leftStickX;
        y    = -leftStickY;
        turn = rightStickX;

        double theta = Math.atan2(y,x)*(180/Math.PI); // aka angle

        double realTheta = (360 - zeroedYaw) + theta;

        double power = Math.hypot(x,y);

        double sin = Math.sin((realTheta * (Math.PI / 180)) - (Math.PI / 4));
        double cos = Math.cos((realTheta * (Math.PI / 180)) - (Math.PI / 4));
        double maxSinCos = Math.max(Math.abs(sin), Math.abs(cos));

        // Combine variables to find power and set the intended power
        target_leftFront_power  = (power * cos / maxSinCos + turn);
        target_leftBack_power   = (power * sin / maxSinCos + turn);
        target_rightFront_power = (power * sin / maxSinCos - turn);
        target_rightBack_power  = (power * cos / maxSinCos - turn);

        if ((power + Math.abs(turn)) > 1.0) // Limit motor's powers to 100%
        {
            target_leftFront_power  /= power + Math.abs(turn);
            target_rightFront_power /= power + Math.abs(turn);
            target_leftBack_power   /= power + Math.abs(turn);
            target_rightBack_power  /= power + Math.abs(turn);
        }

        // Apply acceleration to the motor powers
        leftFront_power  += accel * (target_leftFront_power  - leftFront_power);
        leftBack_power   += accel * (target_leftBack_power   - leftBack_power);
        rightFront_power += accel * (target_rightFront_power - rightFront_power);
        rightBack_power  += accel * (target_rightBack_power  - rightBack_power);

        // Set motor powers to desired values
        leftFront .setPower(leftFront_power  * DRIVETRAIN_SPEED);
        leftBack  .setPower(leftBack_power   * DRIVETRAIN_SPEED);
        rightFront.setPower(rightFront_power * DRIVETRAIN_SPEED);
        rightBack .setPower(rightBack_power  * DRIVETRAIN_SPEED);

        // Display motor power
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFront_power, rightFront_power);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBack_power, rightBack_power);
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS, boolean opMode) {
        int newFrontLeftTarget;
        int newBackLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;

        if (opMode) {
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            newFrontLeftTarget = leftFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newBackLeftTarget = leftBack.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = rightFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newBackRightTarget = rightBack.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            leftFront.setTargetPosition(newFrontLeftTarget);
            leftBack.setTargetPosition(newBackLeftTarget);
            rightFront.setTargetPosition(newFrontRightTarget);
            rightBack.setTargetPosition(newBackRightTarget);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

            leftFront.setPower(Math.abs(speed));
            leftBack.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            rightBack.setPower(Math.abs(speed));

            while (opMode &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())) {
                telemetry.addData("Path1", "Running to %7d :%7d : %7d: %7d ", newFrontLeftTarget, newBackLeftTarget, newFrontRightTarget, newBackRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d : %7d: %7d ",
                        leftFront.getCurrentPosition(),
                        leftBack.getCurrentPosition(),
                        rightFront.getCurrentPosition(),
                        rightBack.getCurrentPosition());
                telemetry.update();
            }
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);

            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
}