package org.firstinspires.ftc.teamcode.opmodes12833;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Depot", group = "MM")
//@Disabled
public class MM_Depot extends LinearOpMode {
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

        goldMineralLocation = robot.deployAndDetect(); // Detect gold location while lowering from lander

        robot.depotLeaveLander();

        if (goldMineralLocation == "Right") {
            robot.drivetrain.gyroTurn(.5, 105);
            robot.drivetrain.gyroDrive(1, -13, 105, 10);
        }
        else if (goldMineralLocation == "Left"){
            robot.drivetrain.gyroTurn(.5, 115);
            robot.moveToLocation(38, -21, 136, .0085, 7);
        }
        else {  // Center
            robot.drivetrain.gyroTurn(.5, 110);
            robot.moveToLocation(29, -29, 129, .0085, 7);
        }

        robot.hitMineral();
        robot.drivetrain.gyroDrive(.6, 11, 105, 10);
        robot.drivetrain.gyroTurn(.6, 120);
        robot.depotFindAndMoveToPic();

        robot.depotDriveAndDumpTeamMarker(goldMineralLocation);
        robot.drivetrain.strafeRight(1, 7, 4);
        robot.drivetrain.gyroDrive(.75, 61, 90, 10); // drive to opponent crater
        }
}