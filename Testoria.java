package org.firstinspires.ftc.teamcode.opmodes12833;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Testoria", group = "Test")
@Disabled
public class Testoria extends LinearOpMode {
    private MM_Tote_Bot robot = new MM_Tote_Bot(this);

    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        robot.init();
        telemetry.addLine("Done with Init");
        telemetry.update();
        runtime.reset();

        waitForStart();

        while(opModeIsActive()) {
            robot.vuforiaNav.targetsAreVisible();
            robot.vuforiaNav.navTelemetry();
            telemetry.update();
        }

//        robot.moveToLocation(24, -24, 120, 0.0018, 0.0027, 0.025, 1, 15);
//        robot.moveToLocation(24, -24, 120, 0.0014, 0.0035, 0.025, 2, 15);
//        robot.moveToLocation(22, 39, 45, 2, 15); // get to sweet spot
//        robot.moveToLocation(22, 39, 45, .0015, .0040, .025, 2, 15); // get to sweet spot from @ 33, 32, 45
    }
}