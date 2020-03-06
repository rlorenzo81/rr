package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.opencv.core.Mat;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class AutoRedSkystone extends LinearOpMode {

    Servo rotateB;
    Servo liftB;
    Servo cB;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBNB drive = new SampleMecanumDriveBNB(hardwareMap);
drive.setLocalizer(new MyStandardTrackingWheelLocalizer(hardwareMap));

drive.setPoseEstimate(new Pose2d(-42.75,-53.5,Math.toRadians(90)));

        rotateB= hardwareMap.get(Servo .class, "rotateb");
        liftB= hardwareMap.get(Servo.class, "liftb");
        cB= hardwareMap.get(Servo.class, "cb");


        rotateB.setPosition(0.1);
        liftB.setPosition(0.5);
        cB.setPosition(0.4);

        waitForStart();

//run to right most stone
        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(new Pose2d(-42.75,-53.5, Math.toRadians(90)))
                .splineTo(new Pose2d(-33.5, -28.5, 0))

                .build();


        drive.followTrajectory(traj);



        sleep(1000);

        rotateB.setPosition(0.2);
        sleep(1000);


        liftB.setPosition(0.1);
        sleep(1000);

        cB.setPosition(0.9);
        sleep(1000);

        liftB.setPosition(0.9);
        sleep(10000);
/*
        drive.followTrajectory(
                drive.trajectoryBuilder(new Pose2d(30, 30, Math.toRadians(180)), 0)
                        .splineTo(new Pose2d(0, 0, Math.toRadians(180)))


                        .build()
        );

 */




/*
        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), 0)
                        .splineTo(new Pose2d(0, 0, 0))
                        .build()
        );

 */
    }
}
