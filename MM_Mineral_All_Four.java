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

        while (!isStarted()) {
            telemetry.addData("Heading", robot.drivetrain.getCurrentHeading());
            telemetry.addData(">", "Press Play");
            telemetry.update();
        }

//        goldMineralLocation = robot.lift.deployAndDetect(robot); // Detect gold location while lowering from lander

/*
        robot.drivetrain.backward(1, 5, 5); // back up to release from lander latch
        robot.drivetrain.strafeRight(1, 5, 5);
        robot.drivetrain.gyroTurn(.6, 185);
        robot.strafeRightTillTarget(5);
*/

//        robot.moveToLocation(12, 46, -150, 0.0014, 0.0035, 0.025, 2, 15);
//        robot.moveToLocation(12, 46, -150, 0.0014, 0.0025, 0.025, 2, 15);
//        robot.moveToLocation(17, 45, -150, 0.0008, 0.0035, 0.025, 1, 15);
//        robot.moveToLocation(17, 45, -150, 0.0008, 0.005, 0.007, 1, 15);
//        robot.moveToLocation(17, 45, -150, 0.0008, 0.002, 0.025, 1, 15);
//        robot.moveToLocation(17, 45, -150, 0.0008, 0.007, 0.025, 1, 15);
//        robot.moveToLocation(17, 45, -150, 0.002, 0.0033, 0.025, 2, 15); // position for right mineral
//        robot.moveToLocation(29.7, 28, -140, 0.002, 0.0033, 0.025, 2, 15); // position for center mineral
//        robot.moveToLocation(5.8, 44.7, -180, .0002, .0033, 0.025, 1, 15);

//        robot.moveToLocation(17, 45, -150, 1, .5, 15);

        while (opModeIsActive()) {
            robot.vuforiaNav.navTelemetry();
            telemetry.update();
        }
    }
}