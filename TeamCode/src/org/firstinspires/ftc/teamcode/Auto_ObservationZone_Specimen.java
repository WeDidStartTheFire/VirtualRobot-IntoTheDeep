package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Base.Dir.BACKWARD;
import static org.firstinspires.ftc.teamcode.Base.Dir.FORWARD;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "!Observation Zone Specimen")
public class Auto_ObservationZone_Specimen extends Base {

    volatile boolean tele = true;

    @Override
    public void runOpMode() throws InterruptedException {
//        setup(new Pose2d(0, 72 - ROBOT_LENGTH / 2, Math.toRadians(90)));
        setup(new Pose2d(0, 0, Math.toRadians(90)));
        Thread telemetryThread = new Thread(this::telemetryLoop);
        telemetryThread.start();
        drive(.001, BACKWARD);
        Trajectory trajectory = drive.trajectoryBuilder(currentPose)
                .splineTo(new Vector2d(36 - 12, 72 - ROBOT_LENGTH / 2 - 18), Math.toRadians(-90))
                .splineTo(new Vector2d(38, 72 - ROBOT_LENGTH / 2 - 36), Math.toRadians(-90))
                .splineTo(new Vector2d(36 + 12, 72 - ROBOT_LENGTH / 2 - 52), Math.toRadians(-90))
                .build();
        currentPose = trajectory.end();
        drive.followTrajectory(trajectory);
        drive(48, BACKWARD);
        Trajectory trajectory1 = drive.trajectoryBuilder(currentPose)
                .splineTo(new Vector2d(-36 - 12, 72 - ROBOT_LENGTH / 2 - 36), Math.toRadians(-90))
                .splineTo(new Vector2d(-36 - 20, 72 - ROBOT_LENGTH / 2 - 52), Math.toRadians(-90))
                .build();
        drive.followTrajectory(trajectory1);
        drive(48, BACKWARD);
    }

    public void telemetryLoop() {
        while (active() && tele) updateAll();
    }
}
