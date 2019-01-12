package org.firstinspires.ftc.teamcode.opmodes12833;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Mineral All Four", group = "MM")
//@Disabled
public class MM_Mineral_All_Four extends LinearOpMode {
    private MM_Tote_Bot robot = new MM_Tote_Bot(this);

    private ElapsedTime runtime = new ElapsedTime();
    private String goldMineralLocation = "";

    @Override
    public void runOpMode() {
        telemetry.addLine("Waiting for Init");
        telemetry.update();
        robot.init();
        runtime.reset();

        waitForStart();

//        goldMineralLocation = robot.lift.deployAndDetect(robot); // Detect gold location while lowering from lander

        robot.drivetrain.brakesOn();
        robot.drivetrain.backward(1, 5, 5); // back up to release from lander latch
        robot.drivetrain.strafeRight(1, 5, 5);
        robot.drivetrain.gyroTurn(.6, 185);
        robot.strafeRightTillTarget(5);

//        robot.moveToLocation(0, 52, 180, .5, .75, 120);
//        robot.moveToLocation(19.6, 41.4, -140.0, 2, 15);

        while (opModeIsActive()) {
            robot.vuforiaNav.navTelemetry();
            telemetry.update();
        }
    }
}