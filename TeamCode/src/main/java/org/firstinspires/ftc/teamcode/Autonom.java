package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name="Autonom Crater", group="Linear OpMode")
public class Autonom extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AWcgpa7/////AAABmcy/3X8/j0O5rVl/TFsI7jtI2X65iRJuPT0JA+JxFlcGoXjuri+AHgItnHFgUGE5xkMhjhPpZ57eT9HxlpFmryfrXSxOYlX58SyvvCbZo+ftIlY4+x3iNw03eNywXKmPBdM7jmGEk6G1HViitwJy8CrOooxYAl37Vh7w0BZipSRVSDKg0AA+jj7ExvVYPedxSBlkTpR9VyUe7hNfWlK/ijmNcpmiYVYomUbPmef2TqIkxSYvBJKZF7vblCmtlmiSrmY1zyO7Y9xKk46vQ8x7cL8tTZG0zDzfDEC12KbCAJLqSN0qju6Z1gsTAIEJmwvAG0YAfKvZf7oSwtno0t7ZfhfY/2LUws3ydkJUVyZGOB7k";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    Robot robot = new Robot();
    ElapsedTime runtime = new ElapsedTime();
    private int initialLiftPosition = 0;
    private boolean testingEnabled = true;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, false, telemetry, this);
        initStuff();

        robot.mechLiftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mechLiftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.mechLiftRight.setPower(0);
        robot.mechLiftRight.setPower(0);

        robot.mechLiftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.mechLiftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.update();

        waitForStart();

        runtime.reset();

        deployRobot();
        telemetry.clear();
        telemetry.addData("Deploy", "Over");
        telemetry.update();

        initiateRecognition();
        telemetry.clear();
        telemetry.addData("Hitting mineral", "Over");

        navigateToDeploy();

    }

    private void navigateToDeploy() {

        stop();

    }

    private  void  initiateRecognition(){

        boolean foundGoldMineral = false;

        if (tfod != null) {
            tfod.activate();
        }

        for(int index = 0; index < 3 && !isStopRequested(); index++){

            ElapsedTime timeForCheck = new ElapsedTime();

            while(tfod != null && !isStopRequested() && !foundGoldMineral && timeForCheck.milliseconds() < 1000) {
                telemetry.addLine("Searching..");
                telemetry.update();

                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                if (updatedRecognitions != null) {

                    for(Recognition recognition : updatedRecognitions){
                        if(recognition.getLabel().equals(LABEL_GOLD_MINERAL)){
                            telemetry.addLine("Found gold mineral at position: " + index +
                                    " after: " + timeForCheck.milliseconds() + " milliseconds");
                            foundGoldMineral = true;
                            telemetry.update();
                        }
                    }

                }
            }
            if(foundGoldMineral)
                break;
            if(index != 2)
                robot.setDrivetrainPosition(1600, "translation", 1);

        }
        if(foundGoldMineral) {
            robot.setDrivetrainPosition(200, "translation", .7);
            robot.absgyroRotation(-90, "absolute");

            robot.setDrivetrainPosition(-1200, "translation", 1);

            robot.setDrivetrainPosition(1200, "translation", 1);

            robot.absgyroRotation(0, "absolute");
        }

        if(tfod != null){
            tfod.shutdown();
        }

        while(opModeIsActive()){

        }

    }


    private void deployRobot() {

        while(robot.mechLiftLeft.getCurrentPosition() < robot.MAX_LIFT_POSITION &&
                robot.mechLiftRight.getCurrentPosition() < robot.MAX_LIFT_POSITION && !isStopRequested()
                && !testingEnabled){
            robot.liftMovement(robot.LIFT_SPEED, false);
        }

        robot.liftMovement(0, false);
        telemetry.clear();

        robot.setDrivetrainPosition(-300, "translation", 1);

        robot.setDrivetrainPosition(1500, "strafing", .5);

        robot.absgyroRotation(-45, "absolute");

        robot.setDrivetrainPosition(-1500, "strafing", .5);

        robot.setDrivetrainPosition(-1600, "translation", 1);

        robot.absgyroRotation(0, "absolute");

        robot.setDrivetrainPosition(400, "translation", 1);


    }

    private void initStuff() throws InterruptedException{
        robot.init(hardwareMap, false, telemetry, this);
        robot.setDrivetrainMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setDrivetrainMode(DcMotor.RunMode.RUN_USING_ENCODER);
        initialLiftPosition = robot.mechLiftLeft.getCurrentPosition();

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
            telemetry.addLine("TFOD INIT FINISHED");
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

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
        tfodParameters.minimumConfidence = .8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
