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
public class ServoTest extends LinearOpMode {
    Servo liftF;
    Servo liftB;
    Servo cB;
    Servo cF;



    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBNB drive = new SampleMecanumDriveBNB(hardwareMap);
drive.setLocalizer(new MyStandardTrackingWheelLocalizer(hardwareMap));
        liftF= hardwareMap.get(Servo .class, "lf");
        liftB= hardwareMap.get(Servo.class, "lb");
        cB= hardwareMap.get(Servo.class, "cb");
        cF= hardwareMap.get(Servo.class, "cf");
//drive.setPoseEstimate(new Pose2d(-42.75,-53.5,Math.toRadians(90)));



       // liftF.setPosition(0.1);
        // liftB.setPosition(0.1);
       //cF.setPosition (0.2);
        cB.setPosition(0.1);


        waitForStart();

        liftF.setPosition(0.9);
        sleep(3000);

        liftF.setPosition(0.1);
        sleep(1000);




        liftB.setPosition(0.1);
        sleep(1000);


        cF.setPosition(0.9); // Closes/clamps the claw down
        sleep(3000);

        cF.setPosition(0.1); // Opens the claw by a hair
        sleep(1000);

       cB.setPosition(0.9);
        sleep(3000);

        cB.setPosition(0.1);
        sleep(1000);












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
