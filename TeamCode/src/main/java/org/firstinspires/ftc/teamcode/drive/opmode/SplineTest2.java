package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import kotlin.Unit;
import kotlin.jvm.functions.Function1;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBNB drive = new SampleMecanumDriveBNB(hardwareMap);
drive.setLocalizer(new MyStandardTrackingWheelLocalizer(hardwareMap));

//drive.setPoseEstimate(new Pose2d(-,0,0));
        drive.setPoseEstimate(new Pose2d(-48,-24,0));

        waitForStart();

        if (isStopRequested()) return;
        Trajectory traj = drive.trajectoryBuilder(new Pose2d(-48,0-24,0))
                .splineTo(new Pose2d(-24, -32, 0))
                .splineTo(new Pose2d(0,-32,0))
                .splineTo(new Pose2d(48,-24,0) )

                .build();

        drive.followTrajectory(traj);

        sleep(1000);


        Trajectory returnStart = drive.trajectoryBuilder(new Pose2d(48,-24,0))
                .splineTo(new Pose2d(-24, -32, 0))
                .splineTo(new Pose2d(0,-32,0))
                .splineTo(new Pose2d(-48,-24,0) )
                .build();

        drive.followTrajectory(returnStart);

        sleep(1000);
      /*  Trajectory twotest = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Pose2d(-15, -33, 0))
                .build();

        drive.followTrajectory(twotest);

        sleep(1000);

        Trajectory threetest = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Pose2d(-0, -33, 0))
                .build();

        drive.followTrajectory(threetest);

        sleep(1000);

        Trajectory  fourtest= drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Pose2d(48, -24, 0))
                .build();

        drive.followTrajectory(fourtest);


       */
    }
}
