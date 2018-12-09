/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.opmodes12833;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class MM_Tote_Bot
{
    public MM_DriveTrain drivetrain = null;
    public MM_Arm arm = null;
    public MM_Collector collector = null;
    public MM_Lift lift = null;

    private LinearOpMode opMode;

    public MM_Tote_Bot(LinearOpMode opMode){
        this.opMode = opMode;
    }

    public void init() {
        drivetrain = new MM_DriveTrain(opMode);
        arm = new MM_Arm(opMode);
        collector = new MM_Collector(opMode);
        lift = new MM_Lift(opMode);
    }
    public void moveAwayFromLander() {
        drivetrain.backward(1, 7, 5);
        drivetrain.gyroTurn(.4, 0);
        drivetrain.strafeLeft(1, .5, 2);
    }
    public void alignWithMinerals() {
        drivetrain.strafeRight(1, 15, 5);
        drivetrain.forward(1, 10, 5);
        drivetrain.gyroTurn(0.7 , -90);
    }

    public void pushMineralCrater(String goldMineralLocation){

        if (goldMineralLocation.equals("Left")) {
            drivetrain.strafeLeft(1, 18, 10);
        } else if (goldMineralLocation.equals("Right")) {
            drivetrain.strafeRight(1, 18, 10);
        }
            drivetrain.forward(1, 18, 5);
    }
    public void strafeMineralCrater(String goldMineralLocation){

        drivetrain.encoderDrive(.9, 13, -13, -13, 13, 20);
        drivetrain.encoderDrive(.9, 6, 6, 6, 6, 20);

        if (goldMineralLocation.equals("Left")) {
            drivetrain.encoderDrive(.9, 18, 18, 18, 18, 20); // drive forward
            drivetrain.strafeRightUntilCrater();
        } else if (goldMineralLocation.equals("Right")) {
            drivetrain.encoderDrive(.9, -18, -18, -18, -18, 20); //
            drivetrain.strafeRightUntilCrater();
        } else {
            opMode.telemetry.addData("Location", goldMineralLocation);
            drivetrain.strafeRightUntilCrater();
        }

    }

    public void pushMineralDepot(String goldMineralLocation){

        drivetrain.encoderDrive(.7, 16, 16, 16, 16, 20);

        if (goldMineralLocation.equals("Left")) {
            drivetrain.encoderDrive(.7, -18, 18, 18, -18, 20); // strafe
            drivetrain.driveUntilDepot();
        } else if (goldMineralLocation.equals("Right")) {
            drivetrain.encoderDrive(.7, 18, -18, -18, 18, 20); // strafe
            drivetrain.driveUntilDepot();
        } else {
            opMode.telemetry.addData("Location", goldMineralLocation);
            drivetrain.driveUntilDepot();
        }

    }


}
