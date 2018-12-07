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
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="IMU_test", group="Linear Opmode")
public class IMUtest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor landerLift = null;
    private DcMotor linearExtender = null;
    private DcMotor collectorAngle = null;
    Servo pincherR;
    Servo pincherL;
    Servo landerStopper;

    //
    //DigitalChannel          touch;
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double globalAngle, power = .30, correction;

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
    private static final int LEFT_QUARTER_CIRCLE = 2700;
    private static final int JEWEL_POSITION = 1000;

    private static final int DISTANCE = 2500;
    private static final int JIGGLE_RIGHT = 250;
    private static final int JIGGLE_LEFT = -250;
    private static final int FORTYFIVEDEGREEROTATE = 500;
    //gets us tp base
    private static final int FORWARD_BASE = 3000;

    //public ElapsedTime mRuntime=new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
       //
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;






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

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        landerStopper.setPosition(LANDER_CATCH_DOWN);
        pincherL.setPosition(SERVOL_CLOSED);
        pincherR.setPosition(SERVOR_CLOSED);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
        telemetry.addData("1 imu heading", lastAngles.firstAngle);
        telemetry.addData("2 global heading", globalAngle);
        telemetry.addData("3 correction", correction);
        telemetry.update();
        telemetry.addLine("done");
        telemetry.update();

        waitForStart();
        runtime.reset();

        telemetry.addData("1 imu heading", lastAngles.firstAngle);
        telemetry.addData("2 global heading", globalAngle);
        telemetry.addData("3 correction", correction);
        telemetry.update();
        telemetry.addLine("done");
        telemetry.update();

        //Back up a short distance
      //  turnLeft(DISTANCE, "2500 ec");

        rotate(90, DRIVE_POWER);

        runtime.reset();
        while (runtime.time() < 2) {
        }

        rotate(-90, DRIVE_POWER);

        runtime.reset();
        while (runtime.time() < 2) {
        }

        rotate(178, DRIVE_POWER);
        rotate(178, DRIVE_POWER);
        rotate(4, DRIVE_POWER);


        telemetry.addData("1 imu heading", lastAngles.firstAngle);
        telemetry.addData("2 global heading", globalAngle);
        telemetry.addData("3 correction", correction);
        telemetry.update();
        telemetry.addLine("done");
        telemetry.update();

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

        int targetForwardLeft = initialLeft + DISTANCE;

        int targetBackLeft = initialLeft - DISTANCE;

        int targetForwardRight = initialRight + DISTANCE;

        int targetBackRight = initialRight - DISTANCE;


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

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = -power;
            rightPower = power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = power;
            rightPower = -power;
        }
        else return;

        // set power to rotate.
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.update();
            }

            while (opModeIsActive() && getAngle() > degrees) {
                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.update();
            }
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {
                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.update();
            }

        // turn the motors off.
        rightDrive.setPower(0);
        leftDrive.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
        telemetry.addData("1 imu heading", lastAngles.firstAngle);
        telemetry.addData("2 global heading", globalAngle);
        telemetry.addData("3 correction", correction);
        telemetry.update();
    }
}














