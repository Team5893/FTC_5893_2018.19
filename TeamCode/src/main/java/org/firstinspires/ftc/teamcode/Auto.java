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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auto", group="Linear Opmode")
public class Auto extends LinearOpMode {
//Test
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor landerLift = null;

    private static final double LANDER_LIFT_POWER = 0.5;
    private static final double STOP_MOTOR = 0;
    private static final double DRIVE_POWER = 0.5;
    //Insert Quality Description
    private static final int LIFTED_ROBOT = 1000;
    //Insert Quality Description
    private static final int LANDED_ROBOT = -1000;
// This is to move the Hook off the handle
private static final int DETACHED_ROBOT = 1000;
    private static final int JEWEL_POSITION = 1000;
//gets us tp base
private static final int FORWARD_BASE = 3000;
public ElapsedTime mRuntime=new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        landerLift = hardwareMap.get(DcMotor.class, "lander_lift");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
       // while (opModeIsActive()) {
           // lander_Run(LANDED_ROBOT);
telemetry.addLine("This is when you lower off the Lander");
        turnRobot(DETACHED_ROBOT,"First Turn");
           //mRuntime.reset();
        runtime.reset();

        while(runtime.time()<9){
            telemetry.update();
        }


        turnRobot(JEWEL_POSITION, "Second Turn");
        runtime.reset();
        while(runtime.time()<9){
            telemetry.update();
        }

        moveForward(FORWARD_BASE, "Move Forward");
        runtime.reset();
        while(runtime.time()<9){
            telemetry.update();
        }
        //}
    }
    /**********************************************************************/
    //The function definition that Bob added as an example

    /*NOTE: THIS MAY NOT WORK THE WAY IT IS CURRENTLY WRITTEN*/
        public void lander_Run (int target_interval)
        {

            int initialPos = landerLift.getCurrentPosition();
            int target_pos = initialPos + target_interval;
            if (initialPos < target_pos)
            {
                while (landerLift.getCurrentPosition() < target_pos)
                {
                    landerLift.setPower(LANDER_LIFT_POWER);
                }
            }

            else if (initialPos > target_pos)
            {
                while (landerLift.getCurrentPosition() > target_pos)
                {
                    landerLift.setPower(-LANDER_LIFT_POWER);
                }
            }

            else
            {
                landerLift.setPower(STOP_MOTOR);
            }

        }
        /******************************************************************************/
        public void turnRobot (int target_interval, String Status)
        {

            int initialLeft = leftDrive.getCurrentPosition();

            int initialRight = rightDrive.getCurrentPosition();

            int target_Right = initialRight + target_interval;

            double target_Left = initialLeft + (0.25*target_interval);

                if (initialRight > target_Right)
            {
                while (rightDrive.getCurrentPosition() <target_Right && leftDrive.getCurrentPosition()<target_Left)
                {
                    rightDrive.setPower(DRIVE_POWER);
                    leftDrive.setPower(0.25*DRIVE_POWER);

                    telemetry.addData("Run","Time:   "+runtime.toString());
                    telemetry.addLine(Status);
                    telemetry.addData("initialLeft", "Left Motor Position:  "+initialLeft);
                    telemetry.addData("CurrentLeftPosition", "Current Pos:  "+leftDrive.getCurrentPosition());
                    telemetry.addData("TargetLeft", "Target Left Position:  "+target_Left);

                    telemetry.addData("initialRight", "Right Motor Position:  "+initialRight);
                    telemetry.addData("CurrentRightPosition", "Current Pos:  "+rightDrive.getCurrentPosition());
                    telemetry.addData("TargetRight", "Target Right Position:  "+target_Right);

                    telemetry.update();
                }
            } else
            {
                rightDrive.setPower(STOP_MOTOR);
                leftDrive.setPower(STOP_MOTOR);
            }

        }
    public void moveForward (int target_interval, String Status)
    {

        int initialLeft = leftDrive.getCurrentPosition();

        int initialRight = rightDrive.getCurrentPosition();

        int target_Right = initialRight + target_interval;

        int target_Left = initialLeft + target_interval;

        if (initialRight > target_Right)
        {
            while (rightDrive.getCurrentPosition() <target_Right && leftDrive.getCurrentPosition()<target_Left)
            {
                rightDrive.setPower(DRIVE_POWER);
                leftDrive.setPower(DRIVE_POWER);

                telemetry.addData("Run","Time:   "+runtime.toString());
                telemetry.addLine(Status);
                telemetry.addData("initialLeft", "Left Motor Position:  "+initialLeft);
                telemetry.addData("CurrentLeftPosition", "Current Pos:  "+leftDrive.getCurrentPosition());
                telemetry.addData("TargetLeft", "Target Left Position:  "+target_Left);

                telemetry.addData("initialRight", "Right Motor Position:  "+initialRight);
                telemetry.addData("CurrentRightPosition", "Current Pos:  "+rightDrive.getCurrentPosition());
                telemetry.addData("TargetRight", "Target Right Position:  "+target_Right);

                telemetry.update();
            }
        } else
        {
            rightDrive.setPower(STOP_MOTOR);
            leftDrive.setPower(STOP_MOTOR);
        }

    }


}




