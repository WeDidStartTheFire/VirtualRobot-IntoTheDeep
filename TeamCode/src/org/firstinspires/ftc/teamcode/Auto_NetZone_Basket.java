package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Base.Dir.BACKWARD;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "!!!Net Zone Basket")
public class Auto_NetZone_Basket extends Base {

    volatile boolean tele = false;
    Runnable liftTask = () -> moveVerticalLift(V_LIFT_GOALS[4]);
    Runnable holdLiftTask = () -> holdVerticalLift(V_LIFT_GOALS[4]);

    @Override
    public void runOpMode() throws InterruptedException {
        setup(new Pose2d(48 - ROBOT_WIDTH / 2 - .5, 72 - .5 - ROBOT_LENGTH / 2, toRadians(180)));

        Thread telemetryThread = new Thread(this::telemetryLoop);
        telemetryThread.start();

        double v = 72 - 12 - (ROBOT_LENGTH / 2 / sqrt(2)) + 1;
        Trajectory trajectory = drive.trajectoryBuilder(currentPose, true)
                .splineTo(new Vector2d(v, v), toRadians(45))
                .build();
        currentPose = trajectory.end();
        Thread driveThread = new Thread(() -> drive.followTrajectory(trajectory));
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
        extendBasketServo();
        hold = false;
        holdLift.join();

        Trajectory trajectory1 = drive.trajectoryBuilder(currentPose)
                .splineTo(new Vector2d(36, 21), toRadians(-90))
                .splineTo(new Vector2d(48, 8), toRadians(-90))
                .build();
        currentPose = trajectory1.end();
        liftThread = new Thread(this::retractVerticalLift);
        liftThread.start();

        drive.followTrajectory(trajectory1);
        try {
            liftThread.join();
        } catch (InterruptedException e) {
            except(e.getStackTrace());
        }
        drive(52, BACKWARD);

        Trajectory trajectory2 = drive.trajectoryBuilder(currentPose)
                .splineTo(new Vector2d(48, 21), toRadians(-90))
                .splineTo(new Vector2d(56, 8), toRadians(-90))
                .build();
        currentPose = trajectory2.end();
        drive.followTrajectory(trajectory2);
        drive(48, BACKWARD);

        Trajectory trajectory3 = drive.trajectoryBuilder(currentPose)
                .splineTo(new Vector2d(54, 21), toRadians(-90))
                .splineTo(new Vector2d(72 - ROBOT_WIDTH / 2, 8), toRadians(-90))
                .build();
        currentPose = trajectory3.end();
        drive.followTrajectory(trajectory3);
        drive(42, BACKWARD);

        Trajectory trajectory4 = drive.trajectoryBuilder(currentPose)
                .splineTo(new Vector2d(40, 20 - ROBOT_WIDTH / 2), toRadians(0))
                .build();
        currentPose = trajectory4.end();
        Trajectory trajectory5 = drive.trajectoryBuilder(currentPose, true)
                .lineTo(new Vector2d(16 + ROBOT_WIDTH / 2, 20 - ROBOT_WIDTH / 2))
                .build();
        currentPose = trajectory5.end();
        drive.followTrajectory(trajectory4);
        drive.followTrajectory(trajectory5);

        tele = false;
        telemetryThread.join();
    }

    public void telemetryLoop() {
        tele = true;
        while (active() && tele) updateAll();
    }
}
