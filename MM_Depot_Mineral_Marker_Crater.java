package org.firstinspires.ftc.teamcode.opmodes12833;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Depot Mineral Marker Crater", group = "MM")
//@Disabled
public class MM_Depot_Mineral_Marker_Crater extends LinearOpMode {
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
//        goldMineralLocation = "Center";

        robot.drivetrain.brakesOn();
        robot.drivetrain.backward(1, 5, 2); // back up to release from lander latch
        robot.drivetrain.strafeRight(1, 14, 3);
        robot.drivetrain.gyroTurn(.6, -165);  // face the target

//        robot.pauseSeconds(5);
        robot.driveFromDepotMineralAndDumpTeamMarkerVuforia(goldMineralLocation);

        while (opModeIsActive()) {
            robot.vuforiaNav.navTelemetry();
            telemetry.update();
        }
    }
}