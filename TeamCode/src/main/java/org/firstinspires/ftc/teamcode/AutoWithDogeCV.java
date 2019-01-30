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

package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="AutoWithDogeCV", group="DogeCV")

public class AutoWithDogeCV extends LinearOpMode {
    // Detector object

    private GoldAlignDetector detector;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor landerLift = null;
    private DcMotor linearExtender = null;
    private DcMotor collectorAngle = null;
    private DcMotor collectorSpinner = null;



    Servo landerStopper;




    public int currentPosition;
    public int newPosition;

    private static final double STOP_MOTOR = 0;
    private static final double DRIVE_POWER = 0.4;
    private static final int SHORT_BACK = -1000;
    private static final double LANDER_LIFT_POWER = 1;
    private static final int LAND_ROBOT = -11076;
    private static final int MOVE_JEWEL = 1600;
    private static final double LANDER_CATCH_UP = 0.75;
    private static final double LANDER_CATCH_DOWN = 0;
    private static final int SHORTER_BACK = -200;
    private static final int SHORT_FORWARD = 100;
public double Time;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        landerLift = hardwareMap.get(DcMotor.class, "Lander_Lift");
        linearExtender = hardwareMap.get(DcMotor.class, "Linear_Extension");
        collectorAngle = hardwareMap.get(DcMotor.class, "Collector_Angle");
        collectorSpinner = hardwareMap.get(DcMotor.class, "Collector_Spinner");


        landerStopper = hardwareMap.servo.get("Lander_Stop");



        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);


        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment
        detector.enable();
        // Start the detector!


            waitForStart();

        lander_Run(LAND_ROBOT,"Land The Robot");
        moveStraight(SHORTER_BACK,"Unhook");
             moveStraight(SHORT_FORWARD," Short back ");

            RightDrive(-350,"getting off");
            moveStraight(SHORTER_BACK,"Not get stuck");
        currentPosition = rightDrive.getCurrentPosition() ;
        runtime.reset();
        while(!detector.getAligned())
        {

            rightDrive.setPower(DRIVE_POWER);
            newPosition = rightDrive.getCurrentPosition() ;
            telemetry.addData("IsAligned", detector.getAligned()); // Is the bot aligned with the gold mineral?
            telemetry.addData("X Pos", detector.getXPosition()); // Gold X position.
            telemetry.addData("Pos rightDrive", rightDrive.getCurrentPosition()); // Gold X position.


        }
        Time = runtime.time();
        detector.disable();
        rightDrive.setPower(STOP_MOTOR);
        telemetry.addData("Pos at TIme", Time); // Gold X position.
        if  (runtime.time() > 2.5)
        {
            RightDrive(250,"getting off");
            telemetry.addData("Time", runtime.time() ); // Gold X position.

        }
        telemetry.addData("Pos rightDrive", rightDrive.getCurrentPosition()); // Gold X position.
            moveStraight(2000,"KnockJewelOff");
        if(Time<2.8)
        {
            RightDrive(450, "face base");
             moveStraight(1500,"Get to base");
            while(runtime.time()<0.25)
            {
                collectorSpinner.setPower(.25);
            }
            collectorSpinner.setPower(STOP_MOTOR);
        }
        else if(Time<3.8)
        {
            moveStraight(1000,"get to base");
            runtime.reset();
            while(runtime.time()<0.25)
            {
                collectorSpinner.setPower(.25);
            }
            collectorSpinner.setPower(STOP_MOTOR);
        }
        else if (Time>3.8)
        {
            telemetry.addData("Need to increase",2000); // Gold X position.

        }
            //RotateInPlace(500,"get in depot");



    }



    public void lander_Run(int target_interval, String Status) {

        int initialPos = landerLift.getCurrentPosition();
        int target_pos = initialPos + target_interval;
        if (initialPos < target_pos) {
            while (landerLift.getCurrentPosition() < target_pos) {
                landerLift.setPower(LANDER_LIFT_POWER);
            }
        } else if (initialPos > target_pos) {
            while (landerLift.getCurrentPosition() > target_pos) {
                landerLift.setPower(-LANDER_LIFT_POWER);
            }
        }

        landerLift.setPower(STOP_MOTOR);
    }

    public void RotateInPlace(int target_interval, String Status) {
        int initialLeft = leftDrive.getCurrentPosition();

        int initialRight = rightDrive.getCurrentPosition();

        int target_Right = initialRight + target_interval;

        int target_Left = initialLeft - target_interval;

        if (initialRight < target_Right) {
            while (rightDrive.getCurrentPosition() < target_Right && leftDrive.getCurrentPosition() < target_Left) {
                rightDrive.setPower(-DRIVE_POWER);
                leftDrive.setPower(DRIVE_POWER);

                telemetry.addData("Run", "Time:   " + runtime.toString());
                telemetry.addLine(Status);
                telemetry.addData("initialLeft", "Left Motor Position:  " + initialLeft);
                telemetry.addData("CurrentLeftPosition", "Current Pos:  " + leftDrive.getCurrentPosition());
                telemetry.addData("TargetLeft", "Target Left Position:  " + target_Left);

                telemetry.addData("initialRight", "Right Motor Position:  " + initialRight);
                telemetry.addData("CurrentRightPosition", "Current Pos:  " + rightDrive.getCurrentPosition());
                telemetry.addData("TargetRight", "Target Right Position:  " + target_Right);

                telemetry.update();
            }
        }
    }

    public void moveStraight(int target_interval, String Status) {

        int initialLeft = leftDrive.getCurrentPosition();

        int initialRight = rightDrive.getCurrentPosition();

        int target_Right = initialRight + target_interval;

        int target_Left = initialLeft + target_interval;

        if (initialRight < target_Right) {
            while (rightDrive.getCurrentPosition() < target_Right && leftDrive.getCurrentPosition() < target_Left) {
                rightDrive.setPower(DRIVE_POWER);
                leftDrive.setPower(DRIVE_POWER);

                telemetry.addData("Run", "Time:   " + runtime.toString());
                telemetry.addLine(Status);
                telemetry.addData("initialLeft", "Left Motor Position:  " + initialLeft);
                telemetry.addData("CurrentLeftPosition", "Current Pos:  " + leftDrive.getCurrentPosition());
                telemetry.addData("TargetLeft", "Target Left Position:  " + target_Left);

                telemetry.addData("initialRight", "Right Motor Position:  " + initialRight);
                telemetry.addData("CurrentRightPosition", "Current Pos:  " + rightDrive.getCurrentPosition());
                telemetry.addData("TargetRight", "Target Right Position:  " + target_Right);

                telemetry.update();
            }
        } else if (initialRight > target_Right) {
            while (rightDrive.getCurrentPosition() > target_Right && leftDrive.getCurrentPosition() > target_Left) {
                rightDrive.setPower(-DRIVE_POWER);
                leftDrive.setPower(-DRIVE_POWER);

                telemetry.addData("Run", "Time:   " + runtime.toString());
                telemetry.addLine(Status);
                telemetry.addData("initialLeft", "Left Motor Position:  " + initialLeft);
                telemetry.addData("CurrentLeftPosition", "Current Pos:  " + leftDrive.getCurrentPosition());
                telemetry.addData("TargetLeft", "Target Left Position:  " + target_Left);

                telemetry.addData("initialRight", "Right Motor Position:  " + initialRight);
                telemetry.addData("CurrentRightPosition", "Current Pos:  " + rightDrive.getCurrentPosition());
                telemetry.addData("TargetRight", "Target Right Position:  " + target_Right);

                telemetry.update();
            }
        }

        rightDrive.setPower(STOP_MOTOR);
        leftDrive.setPower(STOP_MOTOR);


    }

    public void RightDrive (int target_interval, String Status) {

        int initialPos = rightDrive.getCurrentPosition();
        int target_pos = initialPos + target_interval;
        if (initialPos < target_pos) {
            while (rightDrive.getCurrentPosition() < target_pos) {
                rightDrive.setPower(LANDER_LIFT_POWER);
            }
        } else if (initialPos > target_pos) {
            while (rightDrive.getCurrentPosition() > target_pos) {
                rightDrive.setPower(-LANDER_LIFT_POWER);
            }
        }

        rightDrive.setPower(STOP_MOTOR);
    }






}
