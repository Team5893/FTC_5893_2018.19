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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;


@Autonomous(name="Auto3", group="Linear Opmode")
public class RealAuto extends LinearOpMode {

    // Declare OpMode members.
    private GoldAlignDetector detector;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor landerLift = null;
    private DcMotor linearExtender = null;
    private DcMotor collectorAngle = null;
    Servo pincherR;
    Servo pincherL;
    Servo landerStopper;

    private static final double SERVOL_OPEN = 0.5;
    private static final double SERVOL_CLOSED = 1;
    private static final double SERVOR_OPEN = 0.5;
    private static final double SERVOR_CLOSED = 0;

    private static final double STOP_MOTOR = 0;
    private static final double DRIVE_POWER = 0.75;

    private static final double LANDER_LIFT_POWER = 0.5;
    private static final int SHORT_BACK = -500;
    private static final int BOX = 2300;
    private static final int TURN_TO_UNHOOK = -800;
    private static final int JIGGLE_TURN = -500;
    private static final int LAND_ROBOT = -9076;
    private static final double LANDER_CATCH_UP = 0.75;
    private static final double LANDER_CATCH_DOWN = 0;
    private static final double TURN_SHARPNESS = 0.25;
    private static final int MOVE_STRAIGHT_TO_CRATER = 2500;

    private static final int DETACHED_ROBOT = 1000;
    private static final int RIGHT_QUARTER_CIRCLE = 2700;
    private static final int JEWEL_POSITION = 1000;

    private static final int SMALL_DISTANCE = 40;
    private static final int JIGGLE_RIGHT = 250;
    private static final int JIGGLE_LEFT = -250;
    private static final int FORTY_FIVE_DEGREE_ROTATE = 500;
    //gets us tp base
    private static final int FORWARD_BASE = 3000;
boolean Tester= true;
    //public ElapsedTime mRuntime=new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        landerLift = hardwareMap.get(DcMotor.class, "Lander_Lift");
        linearExtender = hardwareMap.get(DcMotor.class, "Linear_Extension");
        collectorAngle = hardwareMap.get(DcMotor.class, "Collector_Angle");

        pincherR = hardwareMap.servo.get("right_pincher");
        pincherL = hardwareMap.servo.get("left_pincher");
        landerStopper = hardwareMap.servo.get("Lander_Stop");

        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        landerStopper.setPosition(LANDER_CATCH_DOWN);
        pincherL.setPosition(SERVOL_CLOSED);
        pincherR.setPosition(SERVOR_CLOSED);

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


        waitForStart();
        runtime.reset();



        lander_Run(LAND_ROBOT, "Lowering Lift");

        landerStopper.setPosition(LANDER_CATCH_UP);

        telemetry.addLine("Landed");
        telemetry.update();

        //Back up a short distance
        moveStraight(SHORT_BACK, "BACKING UP");


        //Turn 90 left forwards
        turnRight(RIGHT_QUARTER_CIRCLE, "First Turn");
        telemetry.addLine("TURNED 90 RIGHT");
        telemetry.update();

        detector.enable();
        while (Tester) {
            telemetry.addData("IsAligned", detector.getAligned()); // Is the bot aligned with the gold mineral?
            telemetry.addData("X Pos", detector.getXPosition()); // Gold X position.


        if(detector.getAligned()){}

        }



    }









    /******************************************************************************/
    public void turnRight(int target_interval, String Status) {

        int initialLeft = leftDrive.getCurrentPosition();

        int initialRight = rightDrive.getCurrentPosition();

        int target_Right = initialRight + target_interval;

        double target_Left = initialLeft + (TURN_SHARPNESS * target_interval);

        while (rightDrive.getCurrentPosition() < target_Right && leftDrive.getCurrentPosition() < target_Left) {
            rightDrive.setPower(DRIVE_POWER);
            leftDrive.setPower(TURN_SHARPNESS * DRIVE_POWER);

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

        rightDrive.setPower(STOP_MOTOR);
        leftDrive.setPower(STOP_MOTOR);
    }

    public void turnLeft(int target_interval, String Status) {

        int initialLeft = leftDrive.getCurrentPosition();

        int initialRight = rightDrive.getCurrentPosition();

        double target_Right = initialRight + (TURN_SHARPNESS * target_interval);

        int target_Left = initialLeft + (target_interval);

        while (rightDrive.getCurrentPosition() < target_Right && leftDrive.getCurrentPosition() < target_Left) {
            rightDrive.setPower(TURN_SHARPNESS * DRIVE_POWER);
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

        rightDrive.setPower(STOP_MOTOR);
        leftDrive.setPower(STOP_MOTOR);
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

    public void Unhook(String Status) {
        int initialLeft = leftDrive.getCurrentPosition();

        int initialRight = rightDrive.getCurrentPosition();

        int targetForwardLeft = initialLeft + SMALL_DISTANCE;

        int targetBackLeft = initialLeft - SMALL_DISTANCE;

        int targetForwardRight = initialRight + SMALL_DISTANCE;

        int targetBackRight = initialRight - SMALL_DISTANCE;


        for (int i=0; i<3; i++){

            while (rightDrive.getCurrentPosition() < targetForwardRight && leftDrive.getCurrentPosition() < targetForwardLeft) {
                rightDrive.setPower(DRIVE_POWER);
                leftDrive.setPower(DRIVE_POWER);

                telemetry.addLine(Status);
                telemetry.update();
            }
            while (rightDrive.getCurrentPosition() < targetBackRight && leftDrive.getCurrentPosition() < targetBackLeft) {
                rightDrive.setPower(-DRIVE_POWER);
                leftDrive.setPower(-DRIVE_POWER);

                telemetry.addLine(Status);
                telemetry.update();
            }
        }
    }
}





