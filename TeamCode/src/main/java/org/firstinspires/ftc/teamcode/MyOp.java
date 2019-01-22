package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
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


@TeleOp(name="MyOpMode", group="Linear Opmode")
public class MyOp extends LinearOpMode {

        // Declare OpMode members.
        private ElapsedTime runtime = new ElapsedTime();
        private DcMotor driveFrontLeft = null;
        private DcMotor driveFrontRight = null;
        private DcMotor driveRearLeft = null;
        private DcMotor driveRearRight = null;
        private DcMotor mechRotation = null;
        private DcMotor mechLiftLeft = null;
        private DcMotor mechLiftRight = null;
        private CRServo mechExt = null;

        @Override
        public void runOpMode() {
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            // Initialize the hardware variables. Note that the strings used here as parameters
            // to 'get' must correspond to the names assigned during the robot configuration
            // step (using the FTC Robot Controller app on the phone).
            driveFrontLeft  = hardwareMap.get(DcMotor.class, "driveFrontLeft");
            driveFrontRight = hardwareMap.get(DcMotor.class, "driveFrontRight");
            driveRearLeft  = hardwareMap.get(DcMotor.class, "driveRearLeft");
            driveRearRight = hardwareMap.get(DcMotor.class, "driveRearRight");
            mechRotation = hardwareMap.get(DcMotor.class, "mechRotation");
            mechLiftLeft = hardwareMap.get(DcMotor.class, "mechLiftLeft");
            mechLiftRight = hardwareMap.get(DcMotor.class, "mechLiftRight");
            mechExt = hardwareMap.get(CRServo.class, "mechExt");

            // Most robots need the motor on one side to be reversed to drive forward
            // Reverse the motor that runs backwards when connected directly to the battery
            driveFrontLeft.setDirection(DcMotor.Direction.REVERSE);
            driveFrontRight.setDirection(DcMotor.Direction.FORWARD);
            driveRearLeft.setDirection(DcMotor.Direction.REVERSE);
            driveRearRight.setDirection(DcMotor.Direction.FORWARD);
            mechRotation.setDirection(DcMotor.Direction.FORWARD);
            mechLiftLeft.setDirection(DcMotor.Direction.FORWARD);
            mechLiftRight.setDirection(DcMotor.Direction.FORWARD);
            mechExt.setDirection(CRServo.Direction.FORWARD);

            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();

            double ext=0;

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {

                // Setup a variable for each drive wheel to save power level for telemetry
                double FrontLeftPower;
                double FrontRightPower;
                double RearLeftPower;
                double RearRightPower;
                double RotationPower;
                double LiftPower;

                // Choose to drive using either Tank Mode, or POV Mode
                // Comment out the method that's not used.  The default below is POV.

                // POV Mode uses left stick to go forward, and right stick to turn.
                // - This uses basic math to combine motions and is easier to drive straight.
                double driveX = -gamepad1.left_stick_x;
                double driveY =  gamepad1.left_stick_y;
                double turn  =  gamepad1.right_stick_x;
                double azimuth = gamepad1.right_stick_y;
                double lift = gamepad1.right_trigger;
                double Shift = gamepad1.left_trigger;
                double SHift2 = gamepad1.left_trigger;
                boolean updn = gamepad2.right_bumper;
                double dir = 1;
                double ExtCRot=gamepad2.left_stick_y;


                if (!gamepad1.dpad_up && !gamepad1.dpad_down){
                    ext=0;
                }
                else if(gamepad1.dpad_up){
                    ext=0.8;
                }

                else if (gamepad1.dpad_down){
                    ext=-0.8;
                }


                //ExtCRot = ext*Shift;
                Shift=1-Shift+0.001;
                mechExt.setPower(ext*Shift);


                if(updn){
                    dir=-dir;
                }

                FrontLeftPower    = Range.clip((( driveX + driveY) - turn)*Shift, -1.0, 1.0) ;
                FrontRightPower   = Range.clip(((-driveX + driveY) + turn)*Shift, -1.0, 1.0) ;
                RearLeftPower     = Range.clip(((-driveX + driveY) - turn)*Shift, -1.0, 1.0) ;
                RearRightPower    = Range.clip((( driveX + driveY) + turn)*Shift, -1.0, 1.0) ;
                RotationPower     = Range.clip( azimuth*Shift, -1.0, 1.0) ;
                LiftPower         = Range.clip(dir *  lift * 10 *Shift, -1.0, 1.0) ;



                // Tank Mode uses one stick to control each wheel.
                // - This requires no math, but it is hard to drive forward slowly and keep straight.
                // leftPower  = -gamepad1.left_stick_y ;
                // rightPower = -gamepad1.right_stick_y ;

                // Send calculated power to wheel
                driveFrontLeft.setPower(FrontLeftPower);
                driveFrontRight.setPower(FrontRightPower);
                driveRearLeft.setPower(RearLeftPower);
                driveRearRight.setPower(RearRightPower);
                mechRotation.setPower(RotationPower);
                mechLiftRight.setPower(LiftPower);
                mechLiftLeft.setPower(LiftPower);

                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Motors", "left (%.2f), right (%.2f)", FrontLeftPower, FrontRightPower, RearLeftPower, RearRightPower);
                telemetry.addData("ExtensionCtrl",ext);
                telemetry.addData("Extension",ExtCRot);
                telemetry.addData("Extension",ExtCRot+ext);
                telemetry.update();
            }
        }
    }


