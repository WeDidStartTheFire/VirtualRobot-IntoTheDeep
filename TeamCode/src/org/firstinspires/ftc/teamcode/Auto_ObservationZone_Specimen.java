package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Base.Dir.BACKWARD;
import static org.firstinspires.ftc.teamcode.Base.Dir.FORWARD;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Observation Zone Specimen", group = "IntoTheDeep")
public class Auto_ObservationZone_Specimen extends Base {

    volatile boolean tele = true;

    @Override
    public void runOpMode() throws InterruptedException {
        setup(new Pose2d(0, 72 - ROBOT_LENGTH / 2, Math.toRadians(90)));
        Thread telemetryThread = new Thread(this::telemetryLoop);
        telemetryThread.start();
        drive(30, BACKWARD);
        sleep(1000);
        // Goes in a straight line to the observation zone
        Trajectory trajectory = drive.trajectoryBuilder(currentPose)
                .splineTo(new Vector2d(-36 + 12, 72 - ROBOT_LENGTH / 2 - 18), Math.toRadians(-90))
                .splineTo(new Vector2d(-36, 72 - ROBOT_LENGTH / 2 - 36), Math.toRadians(-90))
                .splineTo(new Vector2d(-36 - 12, 72 - ROBOT_LENGTH / 2 - 48), Math.toRadians(-90))
                .splineTo(new Vector2d(-36 - 12, 72 - ROBOT_LENGTH / 2), Math.toRadians(-90))
                .build();
        drive.followTrajectory(trajectory);
        currentPose = trajectory.end();
        tele = false;
        telemetryThread.join();
    }

    public void telemetryLoop() {
        while (active() && tele) updateAll();
    }
}
