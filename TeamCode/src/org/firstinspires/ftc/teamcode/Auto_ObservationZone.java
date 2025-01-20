package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Base.Dir.BACKWARD;
import static org.firstinspires.ftc.teamcode.Base.Dir.FORWARD;
import static org.firstinspires.ftc.teamcode.Base.Dir.RIGHT;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Observation Zone", group = "IntoTheDeep")
public class Auto_ObservationZone extends Base {
    @Override
    public void runOpMode() throws InterruptedException {
        setup();
        strafe(20, RIGHT);
        drive(52, FORWARD);
        strafe(9, RIGHT);
        drive(44, BACKWARD);
        turn(0); // Re-align
        drive(44, FORWARD);
        strafe(12, RIGHT);
        drive(44, BACKWARD);
        turn(0); // Re-align
        drive(44, FORWARD);
        strafe(7, RIGHT);
        drive(48, BACKWARD);
    }
}
