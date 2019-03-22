package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="OpMode", group="Linear OpMode")
public class TeleOpMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    ElapsedTime drivingModeSwitchTimer = new ElapsedTime();
    ElapsedTime stopperTimer = new ElapsedTime();

    //new object as robot with all properties and methods
    Robot robot = new Robot();

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

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
    boolean grabberMoving = false;
    boolean liftOverwitting = false;
    int testAngle = 45;

    boolean armUp = false;
    boolean armGoingUp = false;
    boolean armDown = false;
    boolean armGoingDown = false;

    ElapsedTime grabberTimer = new ElapsedTime();
    ElapsedTime inputWaitingTimeUp = new ElapsedTime();
    ElapsedTime inputWaitingTimeDown = new ElapsedTime();

    int newMaxRotation = -420;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, true, telemetry, this);

//        robot.mechExt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.mechRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        waitForStart();
        runtime.reset();
        drivingModeSwitchTimer.reset();
        stopperTimer.reset();

        telemetry.clear();



        while(opModeIsActive()){

            //test distance


            //end test


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
            rotationspeed = gamepad2.right_stick_y;

            brakeFactor = 1 - gamepad1.right_trigger;

            if(drivingMode == "Global"){
                robot.mecanumGlobalCoordinatesDriving(robot.useBrake(mecanumX, brakeFactor, false),
                        robot.useBrake(mecanumY, brakeFactor, false),
                        robot.useBrake(turn, brakeFactor, false),
                        robot.globalGyroValue("Crater"));
            }
            else if(drivingMode == "Local"){
                robot.mecanumMovement(robot.useBrake(mecanumX, brakeFactor, false),
                        robot.useBrake(mecanumY, brakeFactor, false),
                        robot.useBrake(turn, brakeFactor, false));
            }


            brakeFactor_2 = 1;


            //Lift motors

                //Overwritting lift upper limit
                liftOverwitting = true;

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

                //adjusting lift motors if needed -> 2nd controller

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
            else if(!gamepad1.a && !gamepad1.b){
                robot.liftMovement(0, false);
            }




            //Manual extension of the arm
            //gamepad2 -> left_trigger , right_trigger




            //Rotation of the main arm



            //Grabber servo

            if(gamepad2.right_bumper){
                grabberMoving = true;
            }
            else {
                grabberMoving = false;
            }

            if (grabberMoving) {
                robot.mechGrab.setPower(robot.GRABBING_SPEED);
            }
            if(!grabberMoving && !gamepad2.dpad_right){
                robot.mechGrab.setPower(0);
            }


            if(gamepad2.dpad_down){
                robot.mechGrab.setPower(-robot.GRABBING_SPEED);
            }

            if(gamepad2.dpad_up){
//                robot.setDriveTrainPostionDIY(1000, "translation", 1);
                testAngle = 0;
                robot.mechGrab.setPower(-1);
            }
            if(gamepad2.dpad_down){
//                robot.setDriveTrainPostionDIY(1000, "rotation", 1);
                testAngle = 180;
                robot.mechGrab.setPower(1);
            }
            if(gamepad2.dpad_right){
//                robot.setDriveTrainPostionDIY(1000, "strafing", 1);
                testAngle = 210;
                robot.mechGrab.setPower(0);
            }

            if(gamepad1.right_bumper){
                robot.mechExt.setPower(0.5);
            }
            else if(gamepad1.left_bumper){
                robot.mechExt.setPower(-0.5);
            }
            else
                robot.mechExt.setPower(0);




//            if(gamepad2.dpad_up) {
//
//                armPidDown.stop = true;
//
//                if(!armThreadUp.isAlive()){
//                    armPidUp.stop = false;
//                    armThreadUp.start();
//                }
//
//
//                if(robot.mechRotation.getCurrentPosition() < -400){
//                    if(!extThreadIn.isAlive()) {
//                        extPidIn.stop = false;
//                        extThreadIn.start();
//                    }
//                }
//                else{
//                    extPidIn.stop = true;
//
//                    if(!extThreadOut.isAlive()){
//                        extPidOut.stop = false;
//                        extThreadOut.start();
//                    }
//                }
//
//            }
//            else if(gamepad2.dpad_down){
//
//                armPidUp.stop = true;
//
//                if(!armThreadDown.isAlive()){
//                    armThreadDown.start();
//                }
//
//                if(robot.mechRotation.getCurrentPosition() > -400) {
//                    if (!extThreadIn.isAlive()) {
//                        extPidIn.stop = false;
//                        extThreadIn.start();
//                    }
//                }
//                else{
//                        extPidIn.stop = true;
//
//                        if(!extThreadOut.isAlive()){
//                            extPidOut.stop = false;
//                            extThreadOut.start();
//                        }
//                    }
//
//            }
//            else{
//                armPidUp.stop = true;
//                armPidDown.stop = true;
//            }


            if(gamepad2.dpad_up){
                armUp = true;
                armDown = false;
            }
            else if(gamepad2.dpad_down){
                armUp = false;
                armDown = true;
            }
            else{
                armUp = false;
                armDown = false;

                if(robot.mechExt.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){
                    robot.mechExt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }

                if(gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0){
                    mechExtSpeed = 0;
                }
                else if(gamepad2.left_trigger > 0){
                    mechExtSpeed = gamepad2.left_trigger;
                }
                else if(gamepad2.right_trigger > 0){
                    mechExtSpeed = -gamepad2.right_trigger;
                }

                robot.mechExt.setPower(mechExtSpeed);

            }

            if(!armUp){
                inputWaitingTimeUp.reset();
            }
            else if(inputWaitingTimeUp.seconds() < 0.5){
                //nothing to do here
            }
            else if(!armGoingUp){
                armGoingUp = true;
            }

            if(!armDown){
                inputWaitingTimeDown.reset();
            }
            else if(inputWaitingTimeDown.seconds() < 0.5){
                //nothing to do here
            }
            else{
                armGoingDown = true;
            }



            if(armGoingUp){
                inputWaitingTimeUp.reset();
                armGoingUp = robot.rotatingUp();
            }
            else if(armGoingDown){
                inputWaitingTimeDown.reset();
                armGoingDown = robot.rotatingDown();
            }


            
//           telemetry.addData("rot position", robot.mechRotation.getCurrentPosition());
//           telemetry.addData("rot Power", robot.mechRotation.getPower());
//           telemetry.addData("going up", armGoingUp);
            telemetry.addData("ext Power", robot.mechExt.getPower());
           telemetry.addData("ext Pos", robot.mechExt.getCurrentPosition());
////            telemetry.addData("distance sensor", robot.rightDistanceSensor.getDistance(DistanceUnit.CM));
         telemetry.update();

        }



    }


}
