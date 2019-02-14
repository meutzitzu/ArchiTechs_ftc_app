package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.List;

@TeleOp (name="TensorFlow", group="Linear OpMode")
public class TensorFlowTeleOpTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Robot robot = new Robot();

    private static final String VUFORIA_KEY = "AWcgpa7/////AAABmcy/3X8/j0O5rVl/TFsI7jtI2X65iRJuPT0JA+JxFlcGoXjuri+AHgItnHFgUGE5xkMhjhPpZ57eT9HxlpFmryfrXSxOYlX58SyvvCbZo+ftIlY4+x3iNw03eNywXKmPBdM7jmGEk6G1HViitwJy8CrOooxYAl37Vh7w0BZipSRVSDKg0AA+jj7ExvVYPedxSBlkTpR9VyUe7hNfWlK/ijmNcpmiYVYomUbPmef2TqIkxSYvBJKZF7vblCmtlmiSrmY1zyO7Y9xKk46vQ8x7cL8tTZG0zDzfDEC12KbCAJLqSN0qju6Z1gsTAIEJmwvAG0YAfKvZf7oSwtno0t7ZfhfY/2LUws3ydkJUVyZGOB7k";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";


    double brakeFactor = 1, brakeFactor_2 = 1;
    double mecanumX, mecanumY;
    double turn;
    double mechExtSpeed = 0;
    int armPosition = 0;
    int grabDirection = 1;
    int extensionGrabber = 0; // 1 -> extending, 0 -> idle, -1 -> retracting



    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, true, telemetry, this);
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            telemetry.update();
//            return;
        }


        waitForStart();
        runtime.reset();



        while(opModeIsActive()){


        //    handleRobotMovement();
          //  handleTensorFlow();

        }

    }

    private void handleTensorFlow() {
        if(tfod != null) {
            tfod.activate();

            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null)
                updateData(updatedRecognitions);

            tfod.shutdown();

        }

    }

    private void updateData(List<Recognition> updatedRecognitions) {

        telemetry.addData("# Object Detected", updatedRecognitions.size());
        if (updatedRecognitions.size() == 3) {
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
            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                    telemetry.addData("Gold Mineral Position", "Left");
                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                    telemetry.addData("Gold Mineral Position", "Right");
                } else {
                    telemetry.addData("Gold Mineral Position", "Center");
                }
            }
        }
        telemetry.update();

    }

    private void handleRobotMovement() {

        mecanumX = -gamepad1.left_stick_x * robot.DRIVING_COEF;
        mecanumY = gamepad1.left_stick_y * robot. DRIVING_COEF;
        turn = gamepad1.right_stick_x;

        brakeFactor = 1 - gamepad1.left_trigger;

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

//        telemetry.addData("Color Sensor output: ", robot.getColorSensorData());
//        telemetry.addData("Lift position", robot.mechRotation.getCurrentPosition());
//        telemetry.addData("FR", robot.driveFrontRight.getCurrentPosition());
//        telemetry.addData("Angle", robot.tickToRad(robot.mechRotation.getCurrentPosition()));
//        telemetry.update();


    }

    private void initVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);


    }


    private void initTfod() {

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
