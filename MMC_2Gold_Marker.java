package org.firstinspires.ftc.teamcode.opmodes12833;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Crater - 2 mineral", group = "MM")
//@Disabled
public class MMC_2Gold_Marker extends LinearOpMode {
    private MM_Tote_Bot robot = new MM_Tote_Bot(this);

    private ElapsedTime runtime = new ElapsedTime();
    private String goldMineralLocation = "";

    @Override
    public void runOpMode() {
        telemetry.addLine("Waiting for Init");
        telemetry.update();
        robot.init();
        runtime.reset();

        robot.adjustPhone();

        while (!isStarted()){
            telemetry.addLine("Press 'Play' to start or 'A' to adjust phone");
            telemetry.update();

            if (gamepad1.a){
                robot.adjustPhone();
            }
        }

        waitForStart();

        goldMineralLocation = robot.deployAndDetect(); // Detect gold location while lowering from lander
        robot.craterLeaveLander();
//        goldMineralLocation = "Left";
        robot.sampleMineralCrater(goldMineralLocation);   // use goldMineralLocation to knock off gold mineral
        robot.findAndMoveToPic();   // use vuforia to line up to picture parallel to wall
        robot.driveAndDumpTeamMarker(goldMineralLocation);
        robot.sampleDoubleMineral(goldMineralLocation);
    }
}