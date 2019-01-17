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
        robot.drivetrain.alignWithTarget();
        robot.sampleMineral(goldMineralLocation);
        robot.driveAndDumpTeamMarker();
        //robot.backUpToCrater();

        while (opModeIsActive()) {
            robot.vuforiaNav.navTelemetry();
            telemetry.update();
        }
    }
}