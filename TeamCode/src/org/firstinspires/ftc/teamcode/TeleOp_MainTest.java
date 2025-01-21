package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.hypot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "Main Test", group = "Into The Deep")
public class TeleOp_MainTest extends Base {

    double axial,lateral, yaw;

    double leftFrontPower, rightFrontPower, leftBackPower, rightBackPower;

    double speedMultiplier;
    boolean touchSensorPressed = false;

    static final double SPEED_MULTIPLIER = 0.75;
    static final double BASE_TURN_SPEED = 2.5;
    static final double WRIST_MOTOR_POWER = 0.1;
    static final int[] WRIST_M_BOUNDS = {0, 140};
    double wristPos = 0.5;
    double newWristPos = 1;
    int vertA, vertB, vertAvg, vertGoal;
    boolean vertUp, vertDown, vertRunToPos = false;
    double power = 0;

    boolean vertStopped = false;
    boolean wasIntakeServoButtonPressed = false;
    boolean wasWristServoButtonPressed = false;
    boolean wasSpecimenServoButtonPressed = false;
    boolean wasBasketServoButtonPressed = false;
    int wristMotorTicksStopped = 0;
    int wristMotorPos = 0;
    int wristMotorStopPos = 0;
    int error;
    double difference, turnInput, xMove, yMove, angle, joystickAngle;

    static double[] speeds = {0.2, 0.6, 1};

    boolean wasDpu, isDpu, isDpd, wasDpd;

    int newGoal;

    @Override
    public void runOpMode() throws InterruptedException {
        setup();

        while (active()) {
            speedMultiplier = (gamepad1.left_bumper ? speeds[0] : (gamepad1.right_bumper ? speeds[2] : speeds[1]));

            angle = useOdometry ? -drive.getRawExternalHeading() : -imu.getRobotOrientation(INTRINSIC, ZYX, RADIANS).firstAngle;

            joystickAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);
            double moveAngle = joystickAngle - angle;
            double magnitude = Math.min(1, Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y));

            xMove = Math.cos(moveAngle) * magnitude;
            yMove = Math.sin(moveAngle) * magnitude;

            axial = yMove * SPEED_MULTIPLIER;
            lateral = -xMove * SPEED_MULTIPLIER;

            joystickAngle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x);
            difference = simplifyAngle(joystickAngle - angle + PI / 2, RADIANS);

            turnInput = (difference / Math.toRadians(20));
            turnInput = Math.max(-1.0, Math.min(1.0, turnInput));
            yaw = -turnInput * BASE_TURN_SPEED * hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);

            print("Angle", angle);
            print("Move Angle", moveAngle);
            print("X Move", xMove);
            print("Y Move", yMove);
            print("Yaw", yaw);

            leftFrontPower = axial + lateral + yaw;
            rightFrontPower = axial - lateral - yaw;
            leftBackPower = axial - lateral + yaw;
            rightBackPower = axial + lateral - yaw;

            double maxP = Math.max(
                    Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                    Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))
            );
            if (maxP > 1.0) {
                leftFrontPower  /= maxP;
                rightFrontPower /= maxP;
                leftBackPower   /= maxP;
                rightBackPower  /= maxP;
            }

            if (lf != null) {
                lf.setVelocity(leftFrontPower * 5000 * speedMultiplier);
                rf.setVelocity(rightFrontPower * 5000 * speedMultiplier);
                lb.setVelocity(leftBackPower * 5000 * speedMultiplier);
                rb.setVelocity(rightBackPower * 5000 * speedMultiplier);

                print("LF Power", leftFrontPower);
                print("RF Power", rightFrontPower);
                print("LB Power", leftBackPower);
                print("RB Power", rightBackPower);
            }

            // Logic for the wrist motor
            if (wristMotor != null) {
                wristMotorPos = wristMotor.getCurrentPosition();
                if (gamepad2.dpad_down && wristMotorPos < WRIST_M_BOUNDS[1]) {
                    power = WRIST_MOTOR_POWER;
                    wristMotorTicksStopped = 0;
                } else if (gamepad2.dpad_up && (wristMotorPos > WRIST_M_BOUNDS[0] || gamepad2.right_bumper)) {
                    power = -WRIST_MOTOR_POWER;
                    wristMotorTicksStopped = 0;
                    if (gamepad2.right_bumper) {
                        WRIST_M_BOUNDS[1] += wristMotorPos - WRIST_M_BOUNDS[0];
                        WRIST_M_BOUNDS[0] = wristMotorPos;
                    }
                } else {
                    error = wristMotorStopPos - wristMotorPos;
                    if (wristMotorTicksStopped < 5) wristMotorStopPos = wristMotorPos;
                    else power = abs(error) > 3 ? WRIST_MOTOR_POWER * error / 10.0 : 0;
                    wristMotorTicksStopped++;
                }
                wristMotor.setPower(power);
            }

            // Logic for the wrist servo
            if (wristServo != null && gamepad2.a && !wasWristServoButtonPressed) {
                if (wristPos == 0.0 || wristPos == 1.0) newWristPos = 0.5;
                else newWristPos = 1.0 - wristServo.getPosition();
                wristServo.setPosition(wristPos = newWristPos);
            }
            wasWristServoButtonPressed = gamepad2.a;

            // Logic for the intake servo
            if (intakeServo != null && gamepad2.b && !wasIntakeServoButtonPressed)
                intakeServo.setPosition(intakeServo.getPosition() == 0 ? 1 : 0);
            wasIntakeServoButtonPressed = gamepad2.b;

            // Logic to extend or retract the horizontal lift
            if (liftMotor != null) {
                power = 0;
                if (gamepad1.dpad_right ^ gamepad1.dpad_left) {
                    // If the touch sensor isn't connected, assume it isn't pressed
                    touchSensorPressed = verticalTouchSensor != null && verticalTouchSensor.getState();
                    if (gamepad1.dpad_right && !gamepad1.dpad_left)
                        power = liftMotor.getCurrentPosition() < LIFT_BOUNDARIES[1] ? speedMultiplier : 0;
                    else if (gamepad1.dpad_left && !gamepad1.dpad_right && !touchSensorPressed)
                        power = liftMotor.getCurrentPosition() > LIFT_BOUNDARIES[0] ? -speedMultiplier : 0;
                }
                liftMotor.setPower(power);
            }

            // Logic to raise or lower the vertical lift
            if (verticalMotorA != null) {
                vertA = verticalMotorA.getCurrentPosition();
                vertB = verticalMotorB.getCurrentPosition();
                vertAvg = (vertA + vertB) / 2;
                // Relies on one encoder if one seems disconnected
                if (vertB == 0 && vertA > 100) vertAvg = vertB = vertA;
                if (vertA == 0 && vertB > 100) vertAvg = vertA = vertB;
                if ((gamepad1.dpad_up || gamepad1.dpad_down) && !gamepad1.a) vertRunToPos = false;
                else {
                    if (gamepad1.a && (isDpu ^ isDpd)) {
                        vertGoal = vertRunToPos ? vertGoal : vertAvg;
                        if (isDpu) {
                            for (int goal : V_LIFT_GOALS) {
                                if (goal > vertGoal + 50) {
                                    vertGoal = goal;
                                    break;
                                }
                            }
                        } else { // isDpd is always true because of the conditions to enter the if
                            newGoal = vertGoal;
                            for (int goal : V_LIFT_GOALS) {
                                if (goal < vertGoal - 50) newGoal = goal;
                                else break;
                            }
                            vertGoal = newGoal;
                        }
                        vertRunToPos = true; // Ensure the flag is set
                    }
                    if (vertAvg - 20 < vertGoal && vertGoal < vertAvg + 20) {
                        vertRunToPos = false;
                        vertStopped = true;
                    }
                }
                if (vertRunToPos) {
                    if (vertAvg < vertGoal) {
                        vertUp = true;
                        if (vertAvg >= vertGoal - 50) speedMultiplier = speeds[0];
                    } else {
                        vertDown = true;
                        if (vertAvg < vertGoal + 50) speedMultiplier = speeds[0];
                    }
                }
                if (!gamepad1.a) {
                    vertUp = gamepad1.dpad_up || vertUp;
                    vertDown = gamepad1.dpad_down || vertDown;
                }
                if (vertUp == vertDown) {
                    if (!vertStopped && !gamepad1.a) {
                        vertStopped = true;
                        vertGoal = vertAvg;
                    }
                    power = vertAvg < vertGoal - 20 ? 0.1 : (vertGoal - vertAvg) / 20.0 * .1;
                } else {
                    vertStopped = false;
                    // If the touch sensor isn't connected, assume it isn't pressed
                    touchSensorPressed = verticalTouchSensor != null && verticalTouchSensor.getState();
                    if (touchSensorPressed) {
                        verticalMotorA.setMode(STOP_AND_RESET_ENCODER);
                        verticalMotorB.setMode(STOP_AND_RESET_ENCODER);
                        verticalMotorA.setMode(RUN_WITHOUT_ENCODER);
                        verticalMotorB.setMode(RUN_WITHOUT_ENCODER);
                    }
                    if (vertUp && !vertDown)
                        power = vertAvg < V_LIFT_BOUNDS[1] ? speedMultiplier == speeds[0] ? 0.7 : 1 : 0;
                    else if (vertDown && !vertUp && !touchSensorPressed)
                        power = vertAvg > V_LIFT_BOUNDS[0] ? speedMultiplier == speeds[0] ? -0.5 : -0.7 : 0;
                    vertUp = vertDown = false;
                }
                if (vertA > vertB + 5 && power != 0) {
                    verticalMotorA.setPower(power - .05);
                    verticalMotorB.setPower(power + .05);
                } else if (vertB > vertA + 5 && power != 0) {
                    verticalMotorA.setPower(power + .05);
                    verticalMotorB.setPower(power - .05);
                } else {
                    verticalMotorA.setPower(power);
                    verticalMotorB.setPower(power);
                }
                print("Vertical Lift Goal", vertGoal);
            } else vertAvg = 0; // So the basket servo won't go with the lift not connected

            // Logic to open and close basket servo
            if (basketServo != null && gamepad2.x && !wasBasketServoButtonPressed && vertAvg > 100)
                basketServo.setPosition(basketServo.getPosition() == 0 ? 1 : 0);
            wasBasketServoButtonPressed = gamepad2.x && vertAvg > 100;

            // Logic to open and close the specimen servo
            if (specimenServo != null && gamepad1.b && !wasSpecimenServoButtonPressed)
                specimenServo.setPosition(specimenServo.getPosition() == 0 ? 0.4 : 0);
            wasSpecimenServoButtonPressed = gamepad1.b;

            if (gamepad1.dpad_up && !isDpu && !wasDpu) isDpu = wasDpu = true;
            else if (gamepad1.dpad_up && isDpu && wasDpu) isDpu = false;
            else if (!gamepad1.dpad_up && wasDpu) wasDpu = false;
            else if (!gamepad1.dpad_down && isDpu) isDpu = wasDpu = false;

            if (gamepad1.dpad_down && !isDpd && !wasDpd) isDpd = wasDpd = true;
            else if (gamepad1.dpad_down && isDpd && wasDpd) isDpd = false;
            else if (!gamepad1.dpad_down && wasDpd) wasDpd = false;
            else if (!gamepad1.dpad_down && isDpd) isDpd = wasDpd = false;

            print("Speed Multiplier", speedMultiplier);
            updateAll();
        }
    }
}
