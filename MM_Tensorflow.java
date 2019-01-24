package org.firstinspires.ftc.teamcode.opmodes12833;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class MM_Tensorflow {

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private TFObjectDetector tfod;

    private LinearOpMode opMode;

    private double goldMineralX = -1;
    private double silverMineralX = -1;

    public MM_Tensorflow(LinearOpMode opMode, VuforiaLocalizer vuforia) {
        this.opMode = opMode;
        initTfod(vuforia);
    }

    public void initTfod(VuforiaLocalizer vuforia) {
        int tfodMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters();
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public void activateTfod() {
        if (tfod != null) {
            tfod.activate();
        }
    }

    public void shutdownTfod() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    public String mineForGold() {
        String goldMineralLocation = "";
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        opMode.telemetry.addData("Mining for gold",goldMineralLocation);

        if (updatedRecognitions != null) {
            opMode.telemetry.addData("# Object Detected", updatedRecognitions.size());
            if (updatedRecognitions.size() == 2) { // if we detected 2 and only 2 minerals
                goldMineralLocation = determineGoldPosition(updatedRecognitions);
            }
        }
        return goldMineralLocation;
    }

    private String determineGoldPosition(List<Recognition> updatedRecognitions) {
        String goldMineralLocation = "";
        for (Recognition recognition : updatedRecognitions) { //runs through logic for each mineral detected
            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                goldMineralX = recognition.getLeft();
            } else {
                silverMineralX = recognition.getLeft();
            }
        }

        // We are looking at the left 2 minerals
        if (getGoldMineralX() == -1) { // Only see two silver minerals
            goldMineralLocation = "Right";
        } else { // We see a gold mineral
            if (getGoldMineralX() > getSilverMineralX()) { // Determine gold mineral location
                goldMineralLocation = "Center";
            } else {
                goldMineralLocation = "Left";
            }
        }
        return goldMineralLocation;
    }

    public double getGoldMineralX() {
        return goldMineralX;
    }

    public double getSilverMineralX() {
        return silverMineralX;
    }
}