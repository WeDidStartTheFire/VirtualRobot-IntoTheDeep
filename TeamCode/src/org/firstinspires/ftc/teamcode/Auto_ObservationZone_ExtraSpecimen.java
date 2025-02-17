package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Base.Dir.BACKWARD;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Observation Zone Extra Specimen", group = "IntoTheDeep")
public class Auto_ObservationZone_ExtraSpecimen extends Base {

    Runnable liftTask = () -> moveVerticalLift(V_LIFT_GOALS[3]);
    Runnable holdLiftTask = () -> holdVerticalLift(V_LIFT_GOALS[3]);

    @Override
    public void runOpMode() throws InterruptedException {
        setup(new Pose2d(-ROBOT_WIDTH / 2 - .5, 72 - ROBOT_LENGTH / 2, Math.toRadians(90)));

        auto = true;
        running = true;
//        Thread telemetryThread = new Thread(this::telemetryLoop);
//        telemetryThread.start();
        try {
            closeSpecimenServo();
            moveWrist(16);
            Thread driveThread = new Thread(() -> drive(30, BACKWARD));
            Thread liftThread = new Thread(liftTask);
            Thread holdLift = new Thread(holdLiftTask);
            // Start both threads
            driveThread.start();
            liftThread.start();
            // Wait for both threads to complete
            liftThread.join();
            holdLift.start();
            driveThread.join();
            hold = false;
            holdLift.join();
            moveVerticalLift(V_LIFT_GOALS[3] - 400);
            openSpecimenServo();
            s(.5);

            Trajectory trajectory = drive.trajectoryBuilder(currentPose)
                    .lineTo(new Vector2d(currentPose.getX() - 26, currentPose.getY() + 2))
                    .build();
            currentPose = trajectory.end();
            Trajectory trajectory_5 = drive.trajectoryBuilder(currentPose, true)
                    .splineTo(new Vector2d(-36 - 7, 8), toRadians(180))
                    .splineToConstantHeading(new Vector2d(-36 - 8, 72 - 20), toRadians(180))
                    .splineToConstantHeading(new Vector2d(-72 + ROBOT_WIDTH / 2, 72 - ROBOT_LENGTH / 2), toRadians(180))
                    .build();
            liftThread = new Thread(this::retractVerticalLift);
            liftThread.start();
            drive.followTrajectory(trajectory);
            drive.followTrajectory(trajectory_5);
            liftThread.join();
            closeSpecimenServo();
            s(.5);
            moveVerticalLift(100);

            Trajectory trajectory2 = drive.trajectoryBuilder(currentPose, true)
                    .lineToLinearHeading(new Pose2d(-ROBOT_WIDTH / 2 + 2, 72 - ROBOT_LENGTH / 2 - 29 + 14, toRadians(90)))
                    .splineToConstantHeading(new Vector2d(-ROBOT_WIDTH / 2 + 2, 72 - ROBOT_LENGTH / 2 - 29), toRadians(90))
                    .build();
            currentPose = trajectory2.end();
            driveThread = new Thread(() -> drive.followTrajectory(trajectory2));
            liftThread = new Thread(liftTask);
            holdLift = new Thread(holdLiftTask);
            driveThread.start();
            liftThread.start();
            // Wait for both threads to complete
            liftThread.join();
            holdLift.start();
            driveThread.join();
            hold = false;
            holdLift.join();
            moveVerticalLift(V_LIFT_GOALS[3] - 400);
            openSpecimenServo();
            s(.5);

            Trajectory trajectory4 = drive.trajectoryBuilder(currentPose)
                    .lineToLinearHeading(new Pose2d(-72 + ROBOT_WIDTH / 2, 72 - ROBOT_LENGTH / 2, toRadians(0)))
                    .build();
            currentPose = trajectory4.end();
            liftThread = new Thread(this::retractVerticalLift);
            liftThread.start();
            drive.followTrajectory(trajectory4);
            liftThread.join();
            closeSpecimenServo();
            s(.5);
            Trajectory trajectory5 = drive.trajectoryBuilder(currentPose, true)
                    .lineToLinearHeading(new Pose2d(-ROBOT_WIDTH / 2 + 4, 72 - ROBOT_LENGTH / 2 - 29, toRadians(90)))
                    .build();
            currentPose = trajectory5.end();
            driveThread = new Thread(() -> drive.followTrajectory(trajectory5));
            liftThread = new Thread(liftTask);
            holdLift = new Thread(holdLiftTask);
            moveVerticalLift(100);
            driveThread.start();
            liftThread.start();
            // Wait for both threads to complete
            liftThread.join();
            holdLift.start();
            driveThread.join();
            hold = false;
            holdLift.join();
            moveVerticalLift(V_LIFT_GOALS[3] - 400);
            openSpecimenServo();
            s(.5);

            Trajectory trajectory6 = drive.trajectoryBuilder(currentPose)
                    .lineToLinearHeading(new Pose2d(-36 - 11, 72 - ROBOT_LENGTH, toRadians(-90)))
                    .build();
            currentPose = trajectory6.end();
            drive.followTrajectory(trajectory6);
        } finally {
            running = false;
            loop = false;
//            telemetryThread.join();
        }
    }
}
