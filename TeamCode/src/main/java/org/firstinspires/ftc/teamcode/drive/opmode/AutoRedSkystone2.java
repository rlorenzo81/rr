package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class AutoRedSkystone2 extends LinearOpMode {

    Servo liftF;
    Servo liftB;
    Servo cB;
    Servo cF;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBNB drive = new SampleMecanumDriveBNB(hardwareMap);
drive.setLocalizer(new MyStandardTrackingWheelLocalizer(hardwareMap));

drive.setPoseEstimate(new Pose2d(-42.75,-53.5,Math.toRadians(90)));

        liftF= hardwareMap.get(Servo .class, "lf");
        liftB= hardwareMap.get(Servo.class, "lb");
        cB= hardwareMap.get(Servo.class, "cb");
        cF= hardwareMap.get(Servo.class, "cf");



       liftB.setPosition(0.9);
        liftF.setPosition(0.3);
        cF.setPosition(0.8);
        cB.setPosition(0.8);


        waitForStart();

//run to right most stone
        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(new Pose2d(-42.75,-53.5, Math.toRadians(90)))
                .splineTo(new Pose2d(-36.5, -28, 0))
                .build();
        //Lines up back claw w/ the skystone in 2nd place


        drive.followTrajectory(traj);

        cB.setPosition(0.3);
        sleep(250);
        liftB.setPosition(0.1);
        sleep(500);
        cB.setPosition(0.9);
        sleep(500);
        cB.setPosition(0.9);
        liftB.setPosition(0.8);
        sleep(500);

        Trajectory tFnd = drive.trajectoryBuilder(new Pose2d(-28.5,-28, Math.toRadians(0)))
                .splineTo(new Pose2d(0, -30, 0))
                .splineTo(new Pose2d(48,-27,0))

                .build();

        drive.followTrajectory(tFnd);


        liftB.setPosition(0.1); //brings the lift downward, 0.1 lifts the arm
        sleep(100);
        cB.setPosition(0.1); //opens claw, 0.1 is the open claw number
        sleep(500);
        liftB.setPosition(0.8); //brings the lift upward, 0.9 lowers the arm
        sleep(500); //

        drive.followTrajectory(
                drive.trajectoryBuilder(new Pose2d(48, -25, Math.toRadians(180)), 0)
                        .splineTo(new Pose2d(0, -31, Math.toRadians(180)))
                        .splineTo(new Pose2d(-61,-29.5, Math.toRadians(180)))

                        .build()
        );

        cB.setPosition(0.3);
        sleep(250);
        liftB.setPosition(0.1);
        sleep(500);
        cB.setPosition(0.9);
        sleep(500);
        cB.setPosition(0.9);
        liftB.setPosition(0.8);
        sleep(500);

        drive.followTrajectory(
                drive.trajectoryBuilder(new Pose2d(-54, -29.5, 0))
                        .splineTo(new Pose2d(0, -31, 0))
                        .splineTo(new Pose2d(44,-27, 0))

                        .build()
        );




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
