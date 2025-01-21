package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Base.Dir.BACKWARD;
import static org.firstinspires.ftc.teamcode.Base.Dir.FORWARD;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "!Observation Zone Specimen")
public class Auto_ObservationZone_Specimen extends Base {

    volatile boolean tele = true;
    Runnable liftTask = () -> moveVerticalLift(V_LIFT_GOALS[3]);
    Runnable holdLiftTask = () -> holdVerticalLift(V_LIFT_GOALS[3]);

    @Override
    public void runOpMode() throws InterruptedException {
        setup(new Pose2d(0, 72 - ROBOT_LENGTH / 2, Math.toRadians(90)));
//        setup();
        Thread telemetryThread = new Thread(this::telemetryLoop);
        telemetryThread.start();
        closeSpecimenServo();
        Thread driveThread = new Thread(() -> drive(29, BACKWARD));
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
//                .splineTo(new Vector2d(-36 + 12, 72 - ROBOT_LENGTH / 2 - 18), Math.toRadians(-90))
                .splineTo(new Vector2d(-38, 72 - ROBOT_LENGTH / 2 - 40), Math.toRadians(-90))
                .splineTo(new Vector2d(-36 - 12, 72 - ROBOT_LENGTH / 2 - 52), Math.toRadians(-90))
                .build();
        drive.followTrajectory(trajectory);
        currentPose = trajectory.end();
        retractVerticalLift();
        drive(48, BACKWARD);
        Trajectory trajectory1 = drive.trajectoryBuilder(currentPose)
                .splineTo(new Vector2d(-36 - 12, 72 - ROBOT_LENGTH / 2 - 36), Math.toRadians(-90))
                .splineTo(new Vector2d(-36 - 20, 72 - ROBOT_LENGTH / 2 - 52), Math.toRadians(-90))
                .build();
        drive.followTrajectory(trajectory1);
        currentPose = trajectory1.end();
        drive(52, BACKWARD);
        Trajectory trajectory3 = drive.trajectoryBuilder(currentPose)
                .splineTo(new Vector2d(0, 72 - ROBOT_LENGTH / 2 - 29), Math.toRadians(90))
                .build();
        currentPose = trajectory3.end();
        driveThread = new Thread(() -> drive.followTrajectory(trajectory3));
        liftThread = new Thread(liftTask);
        holdLift = new Thread(holdLiftTask);
        closeSpecimenServo();
        s(.5);
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
        Trajectory trajectory4 = drive.trajectoryBuilder(currentPose)
                .splineTo(new Vector2d(-36 - 20, 72 - ROBOT_LENGTH / 2), Math.toRadians(-90))
                .build();
        currentPose = trajectory4.end();
        Trajectory trajectory5 = drive.trajectoryBuilder(currentPose)
                .splineTo(new Vector2d(0, 72 - ROBOT_LENGTH / 2 - 29), Math.toRadians(90))
                .build();
        currentPose = trajectory5.end();
        driveThread = new Thread(() -> drive.followTrajectory(trajectory5));
        liftThread = new Thread(liftTask);
        holdLift = new Thread(holdLiftTask);
        drive.followTrajectory(trajectory4);
        retractVerticalLift();
        closeSpecimenServo();
        s(.5);
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
    }

    public void telemetryLoop() {
        while (active() && tele) updateAll();
    }
}
