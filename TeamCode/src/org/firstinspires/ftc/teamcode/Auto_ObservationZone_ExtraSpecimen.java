package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Base.Dir.BACKWARD;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Observation Zone Extra Specimen", group="!")
public class Auto_ObservationZone_ExtraSpecimen extends Base {

    volatile boolean tele = true;
    Runnable liftTask = () -> moveVerticalLift(V_LIFT_GOALS[3]);
    Runnable holdLiftTask = () -> holdVerticalLift(V_LIFT_GOALS[3]);

    @Override
    public void runOpMode() throws InterruptedException {
        setup(new Pose2d(-ROBOT_WIDTH / 2 - .5, 72 - ROBOT_LENGTH / 2, toRadians(90)));
//        setup();
        Thread telemetryThread = new Thread(this::telemetryLoop);
        telemetryThread.start();
        closeSpecimenServo();
        Thread driveThread = new Thread(() -> drive(30, BACKWARD));
        Thread liftThread = new Thread(liftTask);
        Thread holdLift = new Thread(holdLiftTask);
        // Start both threads
        driveThread.start();
        liftThread.start();
        // Wait for both threads to complete
        try {
            liftThread.join();
            holdLift.start();
            driveThread.join();
        } catch (InterruptedException e) {
            except(e.getStackTrace());
        }
        hold = false;
        holdLift.join();
        moveVerticalLift(V_LIFT_GOALS[3] - 400);
        openSpecimenServo();
        s(.5);

        Trajectory trajectory = drive.trajectoryBuilder(currentPose)
                .lineTo(new Vector2d(currentPose.getX() - 24, currentPose.getY() + 2))
                .build();
        currentPose = trajectory.end();
        Trajectory trajectory_5 = drive.trajectoryBuilder(currentPose, true)
                .splineTo(new Vector2d(-36 - 7, 8), toRadians(180))
                .splineToConstantHeading(new Vector2d(-36 - 13, 72 - 20), toRadians(180))
                .splineToConstantHeading(new Vector2d(-72 + ROBOT_WIDTH / 2, 72 - ROBOT_LENGTH / 2), toRadians(180))
                .build();
        drive.followTrajectory(trajectory);
        drive.followTrajectory(trajectory_5);
        retractVerticalLift();
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
        try {
            liftThread.join();
            holdLift.start();
            driveThread.join();
        } catch (InterruptedException e) {
            except(e.getStackTrace());
        }
        hold = false;
        holdLift.join();
        moveVerticalLift(V_LIFT_GOALS[3] - 400);
        openSpecimenServo();
        s(.5);

        Trajectory trajectory4 = drive.trajectoryBuilder(currentPose)
                .lineToLinearHeading(new Pose2d(-72 + ROBOT_WIDTH / 2, 72 - ROBOT_LENGTH / 2, toRadians(0)))
                .build();
        currentPose = trajectory4.end();
        drive.followTrajectory(trajectory4);
        retractVerticalLift();
        closeSpecimenServo();
        s(.5);
        Trajectory trajectory5 = drive.trajectoryBuilder(currentPose, true)
                .lineToLinearHeading(new Pose2d(-ROBOT_WIDTH / 2 + 2, 72 - ROBOT_LENGTH / 2 - 29, toRadians(90)))
                .build();
        currentPose = trajectory5.end();
        driveThread = new Thread(() -> drive.followTrajectory(trajectory5));
        liftThread = new Thread(liftTask);
        holdLift = new Thread(holdLiftTask);
        moveVerticalLift(100);
        driveThread.start();
        liftThread.start();
        // Wait for both threads to complete
        try {
            liftThread.join();
            holdLift.start();
            driveThread.join();
        } catch (InterruptedException e) {
            except(e.getStackTrace());
        }
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


        tele = false;
        telemetryThread.join();
    }

    public void telemetryLoop() {
        while (active() && tele) updateAll();
    }

}
