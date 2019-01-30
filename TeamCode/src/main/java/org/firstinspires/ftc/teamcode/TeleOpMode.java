package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="OpMode", group="Linear OpMode")
public class TeleOpMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    //new object as robot with all properties and methods
    Robot robot = new Robot();

    //all the strange local variables such as brakeFactor
    double brakeFactor = 1, brakeFactor_2 = 1;
    double mecanumX, mecanumY;
    double turn;
    double mechExtSpeed = 0;
    int armPosition = 0;
    int grabDirection = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, true, telemetry);

        waitForStart();
        runtime.reset();

        while(opModeIsActive()){

            //Assigning values to variables dependant on the controller
            mecanumX = -gamepad1.left_stick_x * robot.DRIVING_COEF;
            mecanumY = gamepad1.left_stick_y * robot. DRIVING_COEF;
            turn = gamepad1.right_stick_x;

            brakeFactor = 1 - gamepad1.left_trigger;
            brakeFactor_2 = 1 - gamepad2.left_trigger;



            //mechExt Servo
            if(!gamepad1.dpad_left && !gamepad1.dpad_right)
                mechExtSpeed = 0;
            else if(gamepad1.dpad_left)
                mechExtSpeed = 0.85;
            else if(gamepad1.dpad_right)
                mechExtSpeed = -0.85;


            robot.mechExt.setPower(robot.useBrake(mechExtSpeed, brakeFactor, true));

            //Lift motors
            if(gamepad1.b){
                robot.liftMovement(robot.useBrake(robot.LIFT_SPEED, brakeFactor, false));
            }
            else if(gamepad1.a){
                robot.liftMovement(robot.useBrake(-robot.LIFT_SPEED, brakeFactor, false));
            }
            else if(!gamepad2.dpad_up && !gamepad2.dpad_down){
                robot.liftMovement(0);
            }


            telemetry.addData("rotating arm pos", robot.driveRearLeft.getCurrentPosition());
            telemetry.addData("rotating arm speed", robot.driveRearLeft.getPower());

            //Rotation of the main arm
            if (gamepad1.dpad_up) {
                robot.rotationMovement(true, brakeFactor);
            } else if (gamepad1.dpad_down) {
                robot.rotationMovement(false, brakeFactor);
            } else {
                if(robot.mechRotation.getCurrentPosition() <= robot.MIN_ROTATION + 10){
                    if(robot.mechRotation.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                        armPosition = robot.mechRotation.getCurrentPosition();
                        robot.mechRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.mechRotation.setTargetPosition(armPosition);
                        robot.mechRotation.setPower(1);
                    }
                }
                else {
                    robot.mechRotation.setPower(0);
                }
            }


            //Grabber servo
            if (gamepad1.right_trigger != 0){
                if(gamepad1.right_bumper){
                    grabDirection = -1;
                }
                else {
                    grabDirection = 1;
                }

                robot.mechGrab.setPower(grabDirection * robot.MAX_CRSERVO_INPUT);

            }
            else {
                robot.mechGrab.setPower(0);
                grabDirection = 1;
            }



            //Mecanum driving
            robot.mecanumMovement(robot.useBrake(mecanumX, brakeFactor, false), robot.useBrake(mecanumY, brakeFactor, false), robot.useBrake(turn, brakeFactor, false));

            telemetry.addData("Lift position", robot.mechRotation.getCurrentPosition());
            telemetry.addData("FR", robot.driveFrontRight.getCurrentPosition());
            telemetry.addData("Angle", robot.tickToRad(robot.mechRotation.getCurrentPosition()));
            telemetry.update();
        }
    }
}
