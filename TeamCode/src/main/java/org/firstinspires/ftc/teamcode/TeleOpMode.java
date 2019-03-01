package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="OpMode", group="Linear OpMode")
public class TeleOpMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    ElapsedTime drivingModeSwitchTimer = new ElapsedTime();
    ElapsedTime stopperTimer = new ElapsedTime();

    //new object as robot with all properties and methods
    Robot robot = new Robot();

    //all the strange local variables such as brakeFactor
    double brakeFactor = 1, brakeFactor_2 = 1;
    double mecanumX, mecanumY;
    double turn;
    double mechExtSpeed = 0;
    double rotationspeed = 0;
    int armPosition = 0;
    int grabDirection = 1;
    int extensionGrabber = 0; // 1 -> extending, 0 -> idle, -1 -> retracting
    String drivingMode = "Local";
    boolean rotationAdjust = false;
    boolean grabberMoving = false;
    boolean stopperOpen = false;
    boolean liftOverwitting = false;

    int newMaxRotation = -420;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, true, telemetry, this);

        waitForStart();
        runtime.reset();
        drivingModeSwitchTimer.reset();
        stopperTimer.reset();

        while(opModeIsActive()){

            //driving mode choice
            if(gamepad1.x && (1==0)){
                if(drivingModeSwitchTimer.seconds() > 0.5) {
                    if (drivingMode == "Global") {
                        drivingMode = "Local";

                        telemetry.clear();
                        telemetry.addData("Driving mode", drivingMode);
                        telemetry.update();

                        drivingModeSwitchTimer.reset();
                    } else if (drivingMode == "Local") {
                        drivingMode = "Global";

                        drivingModeSwitchTimer.reset();
                    }
                }
            }


            /**Controller 1 input -> to be driving controller*/
            mecanumX = -gamepad1.left_stick_x * robot.DRIVING_COEF;
            mecanumY = gamepad1.left_stick_y * robot. DRIVING_COEF;
            turn = gamepad1.right_stick_x;

            /** Controller 2 input -> to be aux controller */
            rotationspeed = gamepad2.left_stick_y;

            brakeFactor = 1 - gamepad1.right_trigger;

            if(drivingMode == "Global"){
                robot.mecanumGlobalCoordinatesDriving(robot.useBrake(mecanumX, brakeFactor, false),
                        robot.useBrake(mecanumY, brakeFactor, false),
                        robot.useBrake(turn, brakeFactor, false),
                        robot.globalGyroValue(robot.Side));
            }
            else if(drivingMode == "Local"){
                robot.mecanumMovement(robot.useBrake(mecanumX, brakeFactor, false),
                        robot.useBrake(mecanumY, brakeFactor, false),
                        robot.useBrake(turn, brakeFactor, false));
            }




            if(!gamepad2.y && !gamepad1.a)
                extensionGrabber = 0;
            else if(gamepad2.y) {
                extensionGrabber = 1;
                if(robot.mechExt.getMode() != DcMotor.RunMode.RUN_USING_ENCODER)
                robot.mechExt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            else if(gamepad2.a) {
                extensionGrabber = -1;
                if(robot.mechExt.getMode() != DcMotor.RunMode.RUN_USING_ENCODER)
                robot.mechExt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            /**Controller 2 input -> to be mineral collection controller*/
            brakeFactor_2 = 1 - gamepad2.right_trigger;


            //mechExt Servo
            if(extensionGrabber == 0) {
                mechExtSpeed = 0;
            }
            else if(extensionGrabber == 1) {
                mechExtSpeed = 0.30;
                if(robot.mechExt.getCurrentPosition() > robot.MAX_EXT - 10){
                    mechExtSpeed = 0;
                }
            }
            else if(extensionGrabber == -1) {
                mechExtSpeed = -0.30;
                if(robot.mechExt.getCurrentPosition() < robot.MIN_EXT + 10){
                    mechExtSpeed = 0;
                }
            }

            if(mechExtSpeed == 0){
                if(robot.mechExt.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
                    robot.mechExt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.mechExt.setTargetPosition(robot.mechExt.getCurrentPosition());
                    robot.mechExt.setPower(0.5);
                }
            }
            else {
                robot.mechExt.setPower(robot.useBrake(mechExtSpeed, brakeFactor_2, false));
            }

            //Lift motors

                //Overwritting lift upper limit
                if(gamepad1.left_bumper){
                    liftOverwitting = true;
                }
                else if(!gamepad1.left_bumper){
                    liftOverwitting = false;
                }

            if(gamepad1.b){
                if(liftOverwitting){
                    robot.liftMovement(robot.LIFT_SPEED, true);
                }
                else {
                    robot.liftMovement(robot.LIFT_SPEED, false);
                }
            }
            else if(gamepad1.a){
                if(liftOverwitting){
                    robot.liftMovement(-robot.LIFT_SPEED, true);
                }
                else {
                    robot.liftMovement(-robot.LIFT_SPEED, false);
                }
            }
            else if(!gamepad1.a && !gamepad1.b){
                robot.liftMovement(0, false);
            }


                //adjusting lift motors if needed -> 2nd controller
//
            else if(gamepad2.b){
                robot.mechLiftRight.setPower(1);
            }
            else if(gamepad2.a){
                robot.mechLiftRight.setPower(-1);
            }

            else if(gamepad2.y){
                robot.mechLiftLeft.setPower(1);
            }
            else if(gamepad2.x){
                robot.mechLiftLeft.setPower(-1);
            }



            //patching for arm movement


            if(gamepad1.x && rotationAdjust){
                newMaxRotation = robot.mechRotation.getCurrentPosition();
            }

            if(gamepad1.x){
                rotationAdjust = false;
            }
            else{
                rotationAdjust = true;
            }



            //Rotation of the main arm
            /*
            if (gamepad1.dpad_up) {
                robot.rotationMovementWIP(true, brakeFactor);
            } else if (gamepad1.dpad_down) {
                robot.rotationMovementWIP(false, brakeFactor);
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
            */

            //rotation movement

//            if(gamepad2.left_trigger > 0.5){
//                rotationAdjust = !rotationAdjust;
//                if(rotationAdjust){
//                    robot.mechRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    robot.mechRotation.setPower(0);
//                    robot.mechRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                }
//            }


                robot.rotationMovement(rotationspeed * robot.ROTATION_SPEED_MODIFIER, newMaxRotation);

            if(gamepad1.dpad_down && robot.mechRotation.getMode() == DcMotor.RunMode.RUN_TO_POSITION){
                robot.mechRotation.setTargetPosition(robot.mechExt.getCurrentPosition() + 10);
            }



            //Grabber servo

            if(gamepad1.left_bumper){
                grabberMoving = true;
            }
            else {
                grabberMoving = false;
            }

            if (grabberMoving) {
                robot.mechGrab.setPower(-robot.GRABBING_SPEED);
            }
            if(!grabberMoving){
                robot.mechGrab.setPower(0);
            }
            telemetry.addData("mechGRab", robot.mechGrab.getPower());


            //stopper servo
            if(gamepad2.left_bumper && !grabberMoving) {
                    stopperOpen = !stopperOpen;
            }

            if(stopperOpen){
                robot.mechStopper.setPosition(robot.STOPPER_OPEN);
            }
            else{
                robot.mechStopper.setPosition(robot.STOPPER_CLOSED);
            }




            telemetry.addData("Arm adjust", rotationAdjust);
            telemetry.addData("controller", rotationspeed);
            telemetry.addData("Driving", robot.driveFrontLeft.getCurrentPosition());
            telemetry.addData("arm pos", robot.mechRotation.getCurrentPosition());
            telemetry.addData("arm mode", robot.mechRotation.getMode());
            telemetry.update();

        }
    }
}
