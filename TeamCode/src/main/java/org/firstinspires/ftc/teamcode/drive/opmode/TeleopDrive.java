package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Config
@TeleOp(group = "drive")
public class TeleopDrive extends LinearOpMode {
    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;



    private Servo rotateB;
    private Servo liftB;
    private Servo cB;


    private Servo clamp;

    private DcMotor leftIntake;
    private DcMotor rightIntake;
    private DcMotor linearLift;
    private DcMotor linearExtend;






    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rotateB= hardwareMap.get(Servo .class, "rotateb");
        liftB= hardwareMap.get(Servo.class, "liftb");
        cB= hardwareMap.get(Servo.class, "cb");



        clamp =hardwareMap.get(Servo.class, "clamp");

        leftIntake= hardwareMap.get(DcMotor.class, "leftEncoder");
        rightIntake = hardwareMap.get(DcMotor.class, "rightEncoder");
        linearLift = hardwareMap.get(DcMotor.class, "rl");
        linearExtend = hardwareMap.get(DcMotor.class, "ex");

        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        BNO055IMU imu;

        SampleMecanumDriveBNB drive = new SampleMecanumDriveBNB(hardwareMap);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        //byte AXIS_MAP_CONFIG_BYTE = 0b000110; // to swap x and z axes
        byte AXIS_MAP_CONFIG_BYTE = 0b011000; //to swap y and z

        byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis

//Need to be in CONFIG mode to write to registers
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);

        sleep(100); //Changing modes requires a delay before doing anything else

//Write to the AXIS_MAP_CONFIG register
        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG,AXIS_MAP_CONFIG_BYTE & 0b111111
        );

//Write to the AXIS_MAP_SIGN register
        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN,AXIS_MAP_SIGN_BYTE & 0b111);

//Need to change back into the IMU mode to use the gyro
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.IMU.bVal & 0b1111);

        sleep(100); //Changing modes again requires a delay

        imu.initialize(parameters);

        clamp.setPosition(0.1);


        waitForStart();

        while (!isStopRequested()) {
            Pose2d baseVel = new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            );

            Pose2d vel;
            if (Math.abs(baseVel.getX()) + Math.abs(baseVel.getY()) + Math.abs(baseVel.getHeading()) > 1) {
                // re-normalize the powers according to the weights
                double denom = VX_WEIGHT * Math.abs(baseVel.getX())
                    + VY_WEIGHT * Math.abs(baseVel.getY())
                    + OMEGA_WEIGHT * Math.abs(baseVel.getHeading());
                vel = new Pose2d(
                    VX_WEIGHT * baseVel.getX(),
                    VY_WEIGHT * baseVel.getY(),
                    OMEGA_WEIGHT * baseVel.getHeading()
                ).div(denom);
            } else {
                vel = baseVel;
            }

            drive.setDrivePower(vel);

            double extend, lift, intake;

            extend  = gamepad2.right_stick_x;
            lift = gamepad2.right_stick_y;
            intake = gamepad2.left_stick_y;

              linearExtend.setPower(extend);

            linearLift.setPower(lift);

            leftIntake.setPower(-intake);
            rightIntake.setPower(intake);

            if(gamepad2.x)
            {
                clamp.setPosition(0.9);
            }

            if(gamepad2.a)
            {
                clamp.setPosition(0.1);
            }

            







            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("heading2", imu.getAngularOrientation().firstAngle);;
            telemetry.update();
        }
    }
}
