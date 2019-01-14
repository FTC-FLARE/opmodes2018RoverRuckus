package org.firstinspires.ftc.teamcode.opmodes12833;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class MM_VuforiaNav {
    private static final int MAX_TARGETS = 4;
    private static final double Y_CLOSE_ENOUGH = 40;      // Within 4.0 cm of target Y
    private static final double X_CLOSE_ENOUGH = 40;      // Within 1.0 cm of target X
    static final double TARGET_DISTANCE = 400.0;    // Hold robot's center 400 mm (16 inches) from target

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = VuforiaLocalizer.CameraDirection.BACK;
    private static final String VUFORIA_KEY = "AZ5woGn/////AAABmSDumo9pA0BDovmvaV5gG7wLT6ES1QrKcI14JsHiEtQ7Gb6e+KM8ILBQGt8hjfHFNwKixlUDQ6vuz0AdKiYelGz5KcfJ9UV4xCMuDxDGvzOqYIS46QLHeFtsx4c4EP5o5a+H4ZM4crit1cva6avYORJXAH4EYCNluvawI+qm7qOru223kxOmNw83qfl17h9ASLtxxZuZ6OiAnQEq0OsSJf5n43QzVRFI55ZYdVAq+7bSeBEMptf1ZbrzvAZWnq8diTq+ojaADlkeZloub6tSLn4OqqbVtnjk65dNVejK2nTY1y7j7v0BQAkqc0w6oMkg30ynxOoyGid1xjSDDEaS1DvbVjQO0ODZZ4O9v6C30dtQ";
    private VuforiaLocalizer vuforia;

    private LinearOpMode opMode;
    private ElapsedTime runtime = new ElapsedTime();

    private static final float mmPerInch = 25.4f;
    private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    static final double DISTANCE_TOLERANCE = 3 * mmPerInch;  // how close is good enough?
    static final double ANGLE_TOLERANCE = 2;

    public static final double YAW_GAIN = 0.018;   // Rate at which we respond to heading error
    public static final double LATERAL_GAIN = 0.0027;  // Rate at which we respond to off-axis error
    public static final double AXIAL_GAIN = 0.0017;  // Rate at which we respond to target distance errors

    final int CAMERA_FORWARD_DISPLACEMENT = 0;   // Camera is 0 mm in front of robot center
    final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // Camera is 200 mm above ground
    final int CAMERA_LEFT_DISPLACEMENT = -203;     // Camera is on robot's right side

    private MM_DriveTrain driveTrain;        // Access to the Robot hardware
    private VuforiaTrackables targetsRoverRuckus;        // List of active targetsRoverRuckus

    private int targetFound;    // set to index of target found; will be -1 if no target found
    private String targetName;     // Name of the currently tracked target
    private double robotX;         // X displacement from target center
    private double robotY;         // Y displacement from target center
    private double robotBearing;   // Robot's rotation around the Z axis (CCW is positive)
    private double targetRange;    // Range from robot's center to target in mm
    private double targetBearing;  // Heading of the target , relative to the robot's unrotated center
    private double relativeBearing;// Heading to the target from the robot's current bearing.
    private double goalRange;

    private double goalX;
    private double goalY;
    private double goalBearing;
    private double errorX; // X displacement of robot from goal
    private double errorY; // Y displacement of robot from goal
    private double errorBearing;

    public MM_VuforiaNav(LinearOpMode opMode, MM_DriveTrain driveTrain) {
        this.opMode = opMode;
        this.driveTrain = driveTrain;
        initVuforia();
        activateTracking();

        targetFound = -1;
        targetName = null;

        robotX = 0;
        robotY = 0;
        targetRange = 0;
        targetBearing = 0;
        robotBearing = 0;
        relativeBearing = 0;
    }

    public int targetsAreVisible() {
        targetFound = -1;
        targetName = "None";

        int targetNumber = 0;
        for (VuforiaTrackable target : targetsRoverRuckus) {
            VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) target.getListener();
            OpenGLMatrix location = null;

            if ((target != null) && (listener != null) && listener.isVisible()) {
                targetName = target.getName();

                location = listener.getUpdatedRobotLocation();
                if (location != null) {

                    VectorF trans = location.getTranslation();
                    Orientation rot = Orientation.getOrientation(location, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    robotX = trans.get(0);
                    robotY = trans.get(1);

                    robotBearing = rot.thirdAngle;
//                    robotBearing = rot.thirdAngle;
                    targetRange = Math.hypot(robotX, robotY);
                    targetBearing = Math.toDegrees(Math.asin(robotX / targetRange));
                    relativeBearing = targetBearing - robotBearing;
                }
                targetFound = targetNumber;
                break;
            }
            targetNumber++;
        }
        return targetFound;
    }

    public boolean cruiseControl(double goalX, double goalY, double goalBearing, double tolerance) {
        this.goalX = goalX * mmPerInch;
        this.goalY = goalY * mmPerInch;
        this.goalBearing = goalBearing;
        double angleTolerance = tolerance;
        tolerance = tolerance * mmPerInch;
        errorX = this.goalX - robotX;
        errorY = this.goalY - robotY;

        errorBearing  = this.goalBearing - robotBearing;
        if (errorBearing > 180){
            errorBearing -= 360;
        }
        else if (errorBearing < -180){
            errorBearing += 360;
        }

        if (Math.abs(errorBearing) < angleTolerance) {
            errorBearing = 0;
        }

        goalRange = Math.hypot(errorX, errorY);

        double yawWeight = (-errorBearing * YAW_GAIN);
        double axialWeight = 0;
        double lateralWeight = 0;

        switch (targetFound) {
            case 0: {  // blue rover
                errorX *= -1;
                lateralWeight = (Math.abs(errorY) < tolerance) ? 0 : (errorY * LATERAL_GAIN);
                axialWeight = (Math.abs(errorX) < tolerance) ? 0 : (errorX * AXIAL_GAIN);
                break;
            }
            case 1: {  // red footprints
                errorY *= -1;
                lateralWeight = (Math.abs(errorY) < tolerance) ? 0 : (errorY * LATERAL_GAIN);
                axialWeight = (Math.abs(errorX) < tolerance) ? 0 : (errorX * AXIAL_GAIN);
                break;
            }
            case 2: {  // front crater
                errorX *= -1;
                errorY *= -1;
                axialWeight = (Math.abs(errorY) < tolerance) ? 0 : (errorY * AXIAL_GAIN);
                lateralWeight = (Math.abs(errorX) < tolerance) ? 0 : (errorX * LATERAL_GAIN);
                break;
            }
            case 3: {  // back space
                axialWeight = (Math.abs(errorY) < tolerance) ? 0 : (errorY * AXIAL_GAIN);
                lateralWeight = (Math.abs(errorX) < tolerance) ? 0 : (errorX * LATERAL_GAIN);
                break;
            }
            default: {
                break;
            }
        }

        driveTrain.setYaw(yawWeight);
        driveTrain.setAxial(axialWeight);
        driveTrain.setLateral(lateralWeight);

        return (axialWeight == 0 && lateralWeight == 0 && yawWeight == 0);
    }

    public boolean cruiseControl(double goalX, double goalY, double goalBearing, double gainAxial, double gainLateral, double gainYaw, double tolerance) {
        this.goalX = goalX * mmPerInch;
        this.goalY = goalY * mmPerInch;
        this.goalBearing = goalBearing;
        double angleTolerance = tolerance * 2;
        tolerance = tolerance * mmPerInch;
        errorX = this.goalX - robotX;
        errorY = this.goalY - robotY;

        errorBearing  = this.goalBearing - robotBearing;
        if (errorBearing > 180){
            errorBearing -= 360;
        }
        else if (errorBearing < -180){
            errorBearing += 360;
        }

        if (Math.abs(errorBearing) < angleTolerance) {
            errorBearing = 0;
        }

        goalRange = Math.hypot(errorX, errorY);

        double yawWeight = (-errorBearing * YAW_GAIN);
        double axialWeight = 0;
        double lateralWeight = 0;

        switch (targetFound) {
            case 0: {  // blue rover
                errorX *= -1;
                lateralWeight = (Math.abs(errorY) < tolerance) ? 0 : (errorY * gainLateral);
                axialWeight = (Math.abs(errorX) < tolerance) ? 0 : (errorX * gainAxial);
                break;
            }
            case 1: {  // red footprints
                errorY *= -1;
                lateralWeight = (Math.abs(errorY) < tolerance) ? 0 : (errorY * gainLateral);
                axialWeight = (Math.abs(errorX) < tolerance) ? 0 : (errorX * gainAxial);
                break;
            }
            case 2: {  // front crater
                errorX *= -1;
                errorY *= -1;
                axialWeight = (Math.abs(errorY) < tolerance) ? 0 : (errorY * gainAxial);
                lateralWeight = (Math.abs(errorX) < tolerance) ? 0 : (errorX * gainLateral);
                break;
            }
            case 3: {  // back space
                axialWeight = (Math.abs(errorY) < tolerance) ? 0 : (errorY * gainAxial);
                lateralWeight = (Math.abs(errorX) < tolerance) ? 0 : (errorX * gainLateral);
                break;
            }
            default: {
                break;
            }
        }

        driveTrain.setYaw(yawWeight);
        driveTrain.setAxial(axialWeight);
        driveTrain.setLateral(lateralWeight);

        return (axialWeight == 0 && lateralWeight == 0 && yawWeight == 0);
    }
    public boolean cruiseControl(double goalX, double goalY, double goalBearing, int rotateFactor) {
//    public boolean cruiseControl(double goalX, double goalY, double goalBearing, double driveSpeed, double rotateFactor) {
        this.goalX = goalX * mmPerInch;
        this.goalY = goalY * mmPerInch;
        this.goalBearing = goalBearing;
        errorX = this.goalX - robotX;
        errorY = this.goalY - robotY;

        goalRange = Math.hypot(errorX, errorY);
        if (Math.abs(goalRange) < DISTANCE_TOLERANCE) {
            errorX = 0;
            errorY = 0;
        }

        errorBearing  = this.goalBearing - robotBearing;
        if (errorBearing > 180){
            errorBearing -= 360;
        }
        else if (errorBearing < -180){
            errorBearing += 360;
        }

        double rotateAdder = 0;
        if (Math.abs(errorBearing) < ANGLE_TOLERANCE) {
            errorBearing = 0;
        }
        else {
            rotateAdder = Math.abs(errorBearing) * rotateFactor;
        }

//        if (goalRange < (6 * mmPerInch)){   // time to ramp down
//            driveSpeed = (goalRange - (2 * mmPerInch)) / (6 * mmPerInch);
//        }

//        double flPower = driveSpeed * Math.sin(Math.toRadians(errorBearing)+ Math.toRadians(90))+ rotateSpeed;
//        double frPower = driveSpeed * Math.cos(Math.toRadians(errorBearing)+ Math.toRadians(90))- rotateSpeed;
//        double blPower = driveSpeed * Math.cos(Math.toRadians(errorBearing)+ Math.toRadians(90))+ rotateSpeed;
//        double brPower = driveSpeed * Math.sin(Math.toRadians(errorBearing)+ Math.toRadians(90))- rotateSpeed;

        double cosA = Math.cos(Math.toRadians(errorBearing - 90));
        double sinA = Math.sin(Math.toRadians(errorBearing - 90));
        double x1 = errorX*cosA - errorY*sinA;
        double y1 = errorX*sinA + errorY*cosA;

        double flPower = x1 + y1 + rotateAdder;
        double frPower = -x1 + y1 - rotateAdder;
        double blPower = -x1 + y1 + rotateAdder;
        double brPower = x1 + y1 - rotateAdder;

        opMode.telemetry.addData("x1: y1 : rotate", "[%+5.2f] : [%+5.2f] : [%+5.2f]", x1, y1, rotateAdder);
//        opMode.telemetry.addData("Before normal", "FL[%+5.2f], FR[%+5.2f], BL[%+5.2f], BR[%+5.2f]", flPower, frPower, blPower, brPower);

        double max = Math.max(Math.abs(flPower), Math.abs(frPower));
        max = Math.max(max, Math.abs(blPower));
        max = Math.max(max, Math.abs(brPower));
        if (max > 1.0)
        {
            flPower /= max;
            frPower /= max;
            blPower /= max;
            brPower /= max;
        }
//        opMode.telemetry.addData("After normal", "FL[%+5.2f], FR[%+5.2f], BL[%+5.2f], BR[%+5.2f]", flPower, frPower, blPower, brPower);

        driveTrain.setFrontLeftPowerForVuforia(flPower);
        driveTrain.setFrontRightPowerForVuforia(frPower);
        driveTrain.setBackLeftPowerForVuforia(blPower);
        driveTrain.setBackRightPowerForVuforia(brPower);

        return (Math.abs(errorBearing) < ANGLE_TOLERANCE && Math.abs(goalRange) < DISTANCE_TOLERANCE);
    }

    public void setGoalToCurrent() {
        goalX = robotX;
        goalY = robotY;
        goalBearing = robotBearing;
    }

    public void navTelemetry() {
        if (targetFound >= 0) {
            opMode.telemetry.addData("Visible", targetName);
            opMode.telemetry.addData("Robot", "[X]:[Y] (B) [%5.1fin]:[%5.1fin] (%4.0f°)", robotX / mmPerInch, robotY / mmPerInch, robotBearing);
//            opMode.telemetry.addData("Target", "[R] (B):(RB) [%5.1fin] (%4.0f°):(%4.0f°)", targetRange / mmPerInch, targetBearing, relativeBearing);
            opMode.telemetry.addData("Goal", "[X]:[Y]  (GB)  [%5.1fin]:[%5.1fin]  (%4.0f°)", goalX / mmPerInch, goalY / mmPerInch, goalBearing);
            opMode.telemetry.addData("Error", "[X]:[Y] (B)  [%5.1fin]:[%5.1fin]  (%4.0f°)", errorX / mmPerInch, errorY / mmPerInch, errorBearing);
            opMode.telemetry.addData("- Turn    ", "%s %4.0f°", errorBearing < 0 ? ">>> CW " : "<<< CCW", Math.abs(errorBearing));
            if (targetFound == 2 || targetFound == 3) {
                opMode.telemetry.addData("- Strafe  ", "%s %5.1fin", Math.abs(errorX) < X_CLOSE_ENOUGH ? "STOP!" : (errorX > 0 ? "RIGHT" : "LEFT"), Math.abs(errorX / mmPerInch));
                opMode.telemetry.addData("- Drive", "%s %5.1fin", Math.abs(errorY) < Y_CLOSE_ENOUGH ? "STOP!" : errorY > 0 ? "FORWARD" : "BACK", Math.abs(errorY / mmPerInch));
            }else{
                opMode.telemetry.addData("- Strafe  ", "%s %5.1fin", Math.abs(errorY) < Y_CLOSE_ENOUGH ? "STOP!" : (errorY > 0 ? "RIGHT" : "LEFT"), Math.abs(errorY / mmPerInch));
                opMode.telemetry.addData("- Drive", "%s %5.1fin", Math.abs(errorX) < X_CLOSE_ENOUGH ? "STOP!" : errorX > 0 ? "FORWARD" : "BACK", Math.abs(errorX / mmPerInch));
            }
        } else {
            opMode.telemetry.addData("Visible", "- - - -");
        }
    }

    public void navTelemetryTrig() {
        if (targetFound >= 0) {
            opMode.telemetry.addData("Visible", targetName);
            opMode.telemetry.addData("Robot", "[X]:[Y] (B) [%5.1fin]:[%5.1fin] (%4.0f°)", robotX / mmPerInch, robotY / mmPerInch, robotBearing);
//            opMode.telemetry.addData("Target", "[R] (B):(RB) [%5.1fin] (%4.0f°):(%4.0f°)", targetRange / mmPerInch, targetBearing, relativeBearing);
            opMode.telemetry.addData("Goal", "[X]:[Y]  (GB)  [%5.1fin]:[%5.1fin]  (%4.0f°)", goalX / mmPerInch, goalY / mmPerInch, goalBearing);
            opMode.telemetry.addData("Error", "[X]:[Y] (B)  [%5.1fin]:[%5.1fin]  (%4.0f°)", errorX / mmPerInch, errorY / mmPerInch, errorBearing);
            opMode.telemetry.addData("Distance", goalRange / mmPerInch);
        } else {
            opMode.telemetry.addData("Visible", "- - - -");
        }
    }

    public void initVuforia() {
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;
//        parameters.useExtendedTracking = false;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZX,
                        AngleUnit.DEGREES, -90, -90, 0));

        targetsRoverRuckus = vuforia.loadTrackablesFromAsset("RoverRuckus");

        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);  // 90deg y+
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);  // -90deg y-
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);  // 180deg x-
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);  // 0deg x+
        backSpace.setName("Back-Space");

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);

        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);
        ((VuforiaTrackableDefaultListener) blueRover.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        redFootprint.setLocation(redFootprintLocationOnField);
        ((VuforiaTrackableDefaultListener) redFootprint.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

/*
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
        frontCraters.setLocation(frontCratersLocationOnField);
        ((VuforiaTrackableDefaultListener) frontCraters.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

*/
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
        frontCraters.setLocation(frontCratersLocationOnField);
        ((VuforiaTrackableDefaultListener) frontCraters.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);
        ((VuforiaTrackableDefaultListener) backSpace.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
    }

    public double getRotateFactor(double goalX, double goalY, double goalBearing) {
        errorX = goalX * mmPerInch - robotX;
        errorY = goalY * mmPerInch - robotY;
        errorBearing = goalBearing - robotBearing;

        double hypot = Math.hypot(errorX, errorY);
        return errorBearing / hypot;
    }

    public void activateTracking() {
        if (targetsRoverRuckus != null)
            targetsRoverRuckus.activate();
    }

    public VuforiaLocalizer getVuforia() {
        return vuforia;
    }

    public boolean findTarget(){
        return false;
    }
}