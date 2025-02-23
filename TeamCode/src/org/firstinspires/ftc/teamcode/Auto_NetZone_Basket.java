package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Base.Dir.LEFT;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Net Zone Basket", group="!Primary")
public class Auto_NetZone_Basket extends Base {

    volatile boolean tele = false;
    Runnable liftTask = () -> moveVerticalLift(V_LIFT_GOALS[4]);
    Runnable holdLiftTask = () -> holdVerticalLift(V_LIFT_GOALS[4]);

    @Override
    public void runOpMode() throws InterruptedException {
        setup(new Pose2d(48 - ROBOT_WIDTH / 2 - .5, 72 - ROBOT_LENGTH / 2, toRadians(180)));

        Thread telemetryThread = new Thread(this::telemetryLoop);
        telemetryThread.start();

        try {
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
            liftThread.join();
            holdLift.start();
            driveThread.join();
            extendBasketServo();
            s(.5);
            hold = false;
            holdLift.join();

            Trajectory trajectory1 = drive.trajectoryBuilder(currentPose)
                    .splineTo(new Vector2d(36, 21), toRadians(-90))
                    .splineTo(new Vector2d(48, 8), toRadians(0))
                    .build();
            currentPose = trajectory1.end();
            liftThread = new Thread(this::retractVerticalLift);
            liftThread.start();

            drive.followTrajectory(trajectory1);
            liftThread.join();
            returnBasketServo();
            strafe(52.5, LEFT);

            Trajectory trajectory2 = drive.trajectoryBuilder(currentPose)
                    .lineToConstantHeading(new Vector2d(currentPose.getX(), 21))
                    .splineToConstantHeading(new Vector2d(56, 8), toRadians(0))
                    .build();
            currentPose = trajectory2.end();
            drive.followTrajectory(trajectory2);
            strafe(48, LEFT);

            Trajectory trajectory3 = drive.trajectoryBuilder(currentPose)
                    .lineToConstantHeading(new Vector2d(currentPose.getX(), 21))
                    .splineToConstantHeading(new Vector2d(72 - ROBOT_WIDTH / 2, 8), toRadians(0))
                    .build();
            currentPose = trajectory3.end();
            drive.followTrajectory(trajectory3);
            strafe(42, LEFT);

            Trajectory trajectory5 = drive.trajectoryBuilder(currentPose)
                    .splineToConstantHeading(new Vector2d(40, 20 - ROBOT_WIDTH / 2), toRadians(0))
                    .splineToConstantHeading(new Vector2d(16 + ROBOT_WIDTH / 2, 20 - ROBOT_WIDTH / 2), toRadians(0))
                    .build();
            currentPose = trajectory5.end();
            drive.followTrajectory(trajectory5);

            tele = false;
            telemetryThread.join();
        } finally {
            drive.breakFollowing();
            tele = false;
            telemetryThread.interrupt();
            telemetryThread.join();
            drive.setMotorPowers(0, 0, 0, 0);
        }
    }

    public void telemetryLoop() {
        tele = true;
        while (!Thread.currentThread().isInterrupted() && active() && tele) updateAll();
    }
}
