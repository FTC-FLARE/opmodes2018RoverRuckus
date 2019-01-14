package org.firstinspires.ftc.teamcode.opmodes12833;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Start Crater Double Mineral", group = "MM")
//@Disabled
public class MM_Start_Crater_Double_Mineral extends LinearOpMode {
    private MM_Tote_Bot robot = new MM_Tote_Bot(this);

    private ElapsedTime runtime = new ElapsedTime();
    private String goldMineralLocation = "";

    @Override
    public void runOpMode() {
        telemetry.addLine("Waiting for Init");
        telemetry.update();
        robot.init();
        runtime.reset();

        telemetry.addLine("Press Play to start");
        telemetry.update();

        waitForStart();

        goldMineralLocation = robot.lift.deployAndDetect(robot); // Detect gold location while lowering from lander

        robot.drivetrain.brakesOn();
        robot.drivetrain.backward(1, 5, 5); // back up to release from lander latch
        robot.drivetrain.strafeRight(1, 14, 5);
        robot.drivetrain.gyroTurn(.6, -165);
        robot.driveFromCraterMineralAndDumpTeamMarkerVuforia(goldMineralLocation);

//        robot.strafeRightTillTarget(5);
//        robot.drivetrain.strafeRight(.9, 15, 8);

//        robot.moveToLocation(15.3, 44.7, -165.0, 1, 15);  // left mineral -150
//        robot.moveToLocation(28.7, 29.2, -140, 1, 15);  // mineral -150 center center
//        robot.drivetrain.backward(1, 13, 5); // only for right 10 for the others
//        robot.drivetrain.forward(1, 13, 5);

        while (opModeIsActive()) {
            robot.vuforiaNav.navTelemetry();
            telemetry.update();
        }
    }
}