package org.firstinspires.ftc.teamcode.opmodes12833;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Crater Mineral Marker Crater", group = "MM")
//@Disabled
public class MM_Crater_Mineral_Marker_Crater extends LinearOpMode {
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

        while (opModeIsActive()) {
            robot.vuforiaNav.navTelemetry();
            telemetry.update();
        }
    }
}