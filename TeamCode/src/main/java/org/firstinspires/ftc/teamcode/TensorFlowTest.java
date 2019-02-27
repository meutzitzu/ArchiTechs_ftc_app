/* Copyright (c) 2018 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@TeleOp(name = "TF Test", group = "Concept")
public class TensorFlowTest extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";


    private static final String VUFORIA_KEY = "AWcgpa7/////AAABmcy/3X8/j0O5rVl/TFsI7jtI2X65iRJuPT0JA+JxFlcGoXjuri+AHgItnHFgUGE5xkMhjhPpZ57eT9HxlpFmryfrXSxOYlX58SyvvCbZo+ftIlY4+x3iNw03eNywXKmPBdM7jmGEk6G1HViitwJy8CrOooxYAl37Vh7w0BZipSRVSDKg0AA+jj7ExvVYPedxSBlkTpR9VyUe7hNfWlK/ijmNcpmiYVYomUbPmef2TqIkxSYvBJKZF7vblCmtlmiSrmY1zyO7Y9xKk46vQ8x7cL8tTZG0zDzfDEC12KbCAJLqSN0qju6Z1gsTAIEJmwvAG0YAfKvZf7oSwtno0t7ZfhfY/2LUws3ydkJUVyZGOB7k";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;



    private ElapsedTime runtime = new ElapsedTime();
    ElapsedTime drivingModeSwitchTimer = new ElapsedTime();

    //new object as robot with all properties and methods
    Robot robot = new Robot();

    //all the strange local variables such as brakeFactor
    double brakeFactor = 1, brakeFactor_2 = 1;
    double mecanumX, mecanumY;
    double turn;
    double mechExtSpeed = 0;
    int armPosition = 0;
    int grabDirection = 1;
    int extensionGrabber = 0; // 1 -> extending, 0 -> idle, -1 -> retracting
    String drivingMode = "Global";

    @Override
    public void runOpMode() throws InterruptedException{
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        robot.init(hardwareMap, true, telemetry, this);

        drivingModeSwitchTimer.reset();
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive()) {
                handleRobotMovement();

                // TensorFlow stuff goes here
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }

                        telemetry.addData("X", goldMineralX);
                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    private void handleRobotMovement() {



        //driving mode choice
        if(gamepad1.x){
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

        brakeFactor = 1 - gamepad1.left_trigger;

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




        if(!gamepad1.dpad_left && !gamepad1.dpad_right)
            extensionGrabber = 0;
        else if(gamepad1.dpad_left)
            extensionGrabber = 1;
        else if(gamepad1.dpad_right)
            extensionGrabber = -1;

        /**Controller 2 input -> to be mineral collection controller*/
        brakeFactor_2 = 1 - gamepad2.left_trigger;


        //mechExt Servo
        if(extensionGrabber == 0)
            mechExtSpeed = 0;
        else if(extensionGrabber == 1)
            mechExtSpeed = 0.85;
        else if(extensionGrabber == -1)
            mechExtSpeed = -0.85;


        //robot.mechExt.setPower(robot.useBrake(mechExtSpeed, brakeFactor, true));

        //Lift motors
        if(gamepad1.b){
            robot.liftMovement(robot.useBrake(robot.LIFT_SPEED, brakeFactor, false));
        }
        else if(gamepad1.a){
            robot.liftMovement(robot.useBrake(-robot.LIFT_SPEED, brakeFactor, false));
        }
        else if(!gamepad1.a && !gamepad1.b && !gamepad2.a && !gamepad2.b && !gamepad2.x && !gamepad2.y){
            robot.liftMovement(0);
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


        if(gamepad2.right_bumper){
            robot.mechLiftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.mechLiftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.mechLiftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.mechLiftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }




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

    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
