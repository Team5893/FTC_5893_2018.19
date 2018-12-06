

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name="TeleOp2", group="Linear Opmode")
public class TeleOp2 extends LinearOpMode {

    //Example change for github
//@Disabled



//Declare hardware




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







    //declare constants
    private static final double SERVOL_OPEN = 0.5;
    private static final double SERVOL_CLOSED = 1;
    private static final double SERVOR_OPEN = 0.5;
    private static final double SERVOR_CLOSED = 0;

    private static final double TRIGGER_THRESHOLD = 0.7;

    private static final double LANDER_POWER = 1;

    private static final double LANDER_CATCH_UP = 1;
    private static final double LANDER_CATCH_DOWN = 0;

    private static final double SLOW_MODE = 0.25;
    private static final double FAST_MODE =1;
    private static final double MEDIUM_MODE = .5;








    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).




        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
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
        landerLift.setDirection(DcMotor.Direction.REVERSE);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        //this has to be outside so it doesn't keep setting back to one
        double speedMultiplier=FAST_MODE;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;



            if (gamepad2.a)
            {
             speedMultiplier=FAST_MODE;
             telemetry.addLine("Fast Mode");
            }
            else if(gamepad2.b)
            {
            speedMultiplier=MEDIUM_MODE;
            telemetry.addLine("Medium Speed");
            }
            else if (gamepad2.y)
            {
                speedMultiplier=SLOW_MODE;
                telemetry.addLine("Slow Mode");
            }




            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad2.left_stick_y;
            double turn  =  gamepad2.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Send calculated power to wheels
            leftDrive.setPower(leftPower*speedMultiplier);
            rightDrive.setPower(rightPower*speedMultiplier);



            //Servo Code
            if(gamepad1.left_bumper || gamepad2.left_bumper)
            {
                pincherL.setPosition(SERVOL_OPEN);
            }



            else if(  gamepad1.left_trigger > TRIGGER_THRESHOLD || gamepad2.left_trigger > TRIGGER_THRESHOLD)//make a constant for trigger threshold
            {
                pincherL.setPosition(SERVOL_CLOSED);
            }

            if(gamepad1.right_bumper || gamepad2.right_bumper) {
                pincherR.setPosition(SERVOR_OPEN);
            } else if ( gamepad1.right_trigger > TRIGGER_THRESHOLD || gamepad2.right_trigger > TRIGGER_THRESHOLD) //make a constant for trigger threshold
            {
                pincherR.setPosition(SERVOR_CLOSED);
            }

            telemetry.addData("Right Pincher: Servo Position", pincherR.getPosition());
            telemetry.addData("Left Pincher: Servo Position", pincherL.getPosition());
            telemetry.addData("Status", "Running");



            if (gamepad1.y)
            {
                landerLift.setPower(LANDER_POWER); //create constant for power
            }
            else if (gamepad1.a)
            {
                landerLift.setPower(-LANDER_POWER);
            }
            else
            {
                landerLift.setPower(0);
            }

            if(gamepad1.dpad_down)
            {
                landerStopper.setPosition(LANDER_CATCH_DOWN);
            }
            else if (gamepad1.dpad_up)
            {
                landerStopper.setPosition(LANDER_CATCH_UP);
            }




            linearExtender.setPower(-gamepad1.left_stick_y);

            collectorAngle.setPower(-gamepad1.right_stick_y);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Lander Lift Position",landerLift.getCurrentPosition() );
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Speed Multiplier",speedMultiplier);
            telemetry.update();
        }
    }

}



