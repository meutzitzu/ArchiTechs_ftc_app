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
import java.util.concurrent.Executors;
import java.util.concurrent.*;

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
    private boolean testingEnabled = false;

    private ExecutorService executorService = Executors.newSingleThreadExecutor();

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

        navigateToDeploy();

    }

    private void navigateToDeploy() {

        robot.gyroRotationWIP(45, "absolute", "Crater");
        double velocity = 1;
        boolean isClose = false;
        robot.mecanumMovement(0, 1, 0);
        while(robot.leftDistanceSensor.getDistance(DistanceUnit.CM) > 15 && !isStopRequested()){
            if(robot.leftDistanceSensor.getDistance(DistanceUnit.CM) < 50 && !isClose) {
                isClose = true;
            }
            if(isClose){
                robot.mecanumMovement(0, velocity, 0);
                velocity -= .03;
            } else {
                robot.mecanumMovement(0, velocity, 0);
            }
            telemetry.addLine("Distance from sensor: " + robot.leftDistanceSensor.getDistance(DistanceUnit.CM));
            if(velocity < 0)
                break;
        }
        telemetry.update();
        robot.mecanumMovement(0, 0, 0);

        while(opModeIsActive()){

        }

        robot.gyroRotationWIP(90, "absolute", "Crater");

        while(robot.leftDistanceSensor.getDistance(DistanceUnit.CM) > 40 && !isStopRequested()){
            robot.mecanumMovement(0, 1, 0);
        }

        robot.mecanumMovement(0, 0, 0);
        telemetry.clear();

        double time = runtime.seconds();
        while(opModeIsActive()){
            telemetry.addLine("Seconds: " + time);
            telemetry.addLine("Placing totem");
            telemetry.update();
        }

        stop();
    }

    private Future<Integer> getMineralAsync() throws InterruptedException {
        return executorService.submit(new Callable<Integer>() {
            @Override
            public Integer call() throws Exception {

                boolean foundMineral = false;
                int mineralPosition = -1;
                ElapsedTime time = new ElapsedTime();
                if(tfod != null)
                    tfod.activate();
                while(time.seconds() < 8 && !foundMineral && tfod != null){
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    telemetry.addLine("seconds passed: " + time.seconds());
                    telemetry.update();
                    if (updatedRecognitions != null && updatedRecognitions.size() == 2) {
                        foundMineral = true;
                        float goldMineralX = -1;
                        float silverMineral1X = -1;
                        float silverMineral2X = -1;

                        for(int i=0; i<2; i++){
                            if(updatedRecognitions.get(i).getLabel().equals(LABEL_GOLD_MINERAL)){
                                goldMineralX = updatedRecognitions.get(i).getLeft();
                            } else if(silverMineral1X == -1){
                                silverMineral1X = updatedRecognitions.get(i).getLeft();
                            } else {
                                silverMineral2X = updatedRecognitions.get(i).getLeft();
                            }
                        }

                        if(goldMineralX == -1)
                            mineralPosition = 3;
                        else if(goldMineralX < silverMineral1X){
                            mineralPosition = 1;
                        } else {
                            mineralPosition = 2;
                        }
                    }
                }
                tfod.shutdown();
                return mineralPosition;
            }

        });

    }

    private void deployRobot() throws InterruptedException {

        Future<Integer> mineralPosition = getMineralAsync();
        while(robot.mechLiftLeft.getCurrentPosition() < robot.MAX_LIFT_POSITION &&
                robot.mechLiftRight.getCurrentPosition() < robot.MAX_LIFT_POSITION && !isStopRequested()
                && !testingEnabled){
            robot.liftMovement(robot.LIFT_SPEED, false);
        }

        robot.liftMovement(0, false);

        // wait till func return something
        while(!mineralPosition.isDone()){
            telemetry.addLine("Still looking...");
            telemetry.update();
        }
        telemetry.update();


        robot.setDrivetrainPosition(-500, "translation", 1);
        robot.setDrivetrainPosition(1500, "strafing", .6);
        robot.setDrivetrainPosition(500, "translation", 1);

        try {
            sampleMineral(mineralPosition.get());
            executorService.shutdown();
        } catch (ExecutionException e){
            telemetry.addLine("Something went wrong: " + e.toString());
        } finally {
            telemetry.update();
        }

    }

    private void sampleMineral(int mineralPosition) {
        telemetry.addLine("mineral position: " + mineralPosition);
        telemetry.update();
        int distanceToTravel;
        switch (mineralPosition){
            case 1: robot.gyroRotationWIP(0, "absolute", "Crater");
                    distanceToTravel = -2700;
                    break;
            case 2: robot.gyroRotationWIP(315, "absolute", "Crater");
                    distanceToTravel = -2200;
                    break;
            case 3: robot.gyroRotationWIP(270, "absolute", "Crater");
                    distanceToTravel = -2700;
                    break;
            default: distanceToTravel = 0;
        }

        robot.setDrivetrainPosition(distanceToTravel, "translation", 1);

        robot.setDrivetrainPosition((-distanceToTravel) / 10 * 7, "translation", 1);
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
        tfodParameters.minimumConfidence = .75;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
