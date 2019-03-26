package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
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

        robot.mechLiftRight.setPower(0);
        robot.mechLiftLeft.setPower(0);

        telemetry.update();

        waitForStart();

        runtime.reset();

        deployRobot();

        //navigateToDeploy();

        stop();


    }

    private double distWallLeft(){
        double distL = robot.leftDistanceSensor.getDistance(DistanceUnit.CM);
        distL= Math.sqrt(2)*distL/2;
        return distL;
    }

    private double distWallRight(){
        double distR = robot.rightDistanceSensor.getDistance(DistanceUnit.CM);
        distR= Math.sqrt(2)*distR/2;
        return distR;
    }



    private void navigateToDeploy() {

        robot.gyroRotationWIP(45, "absolute", "Crater");
        double velocity = 1;
        boolean isClose = false;


        ///mitza start

        while(distWallLeft()>5&&distWallRight()>10 && !isStopRequested()) {
            robot.mecanumMovement(0, 1, 0);
            if (distWallLeft()<25||distWallRight()<30)
                robot.mecanumMovement(0, 0.3, 0);
            if(distWallLeft()<8||distWallRight()<12){
                robot.mecanumMovement(0, 0, 0);
            }
        }

        robot.gyroRotationWIP(90, "absolute", "Crater");

        while (robot.leftDistanceSensor.getDistance(DistanceUnit.CM)>55) {
            robot.mecanumMovement(0, 0.5, 0);
        }

        if(robot.leftDistanceSensor.getDistance(DistanceUnit.CM)<58) {
            robot.mecanumMovement(0,0,0);
            robot.mechGrab.setPower(-0.4);
            ElapsedTime time = new ElapsedTime();
            while(time.seconds()<1.5 && !isStopRequested()) {

            }
            robot.mechGrab.setPower(0);
        }

        robot.setDrivetrainPosition(7000,"translation",1);

        robot.mechRotation.setPower(1);
        ElapsedTime time = new ElapsedTime();
        while(time.seconds()<1.5 && !isStopRequested()) {

        }
        robot.mechRotation.setPower(0);

        stop();

    }

    private Future<int[]> getMineralAsync() {
        return executorService.submit(new Callable<int[]>() {
            @Override
            public int[] call() {

                int[] minerals = new int[]{0, 0, 0, 0};

                boolean foundMineral = false;
                int mineralPosition = -1;
                ElapsedTime time = new ElapsedTime();
                if(tfod != null)
                    tfod.activate();
                while(time.seconds() < 10 && !foundMineral && tfod != null && !isStopRequested()){

                    List<Recognition> updatedRecognitionsUnfiltered = tfod.getUpdatedRecognitions();
                    List<Recognition> updatedRecognitions = new ArrayList<>();

                    if(updatedRecognitionsUnfiltered != null)
                        for(Recognition rec :updatedRecognitionsUnfiltered){
                            if(rec.getHeight() / rec.getWidth() > .7 && rec.getWidth() > 55 && rec.getHeight() > 55) {
                                updatedRecognitions.add(rec);
                            }
                        }


                    if (updatedRecognitions.size() == 2) {
                        telemetry.addLine("seconds passed: " + time.seconds());
                        telemetry.addLine("objects detected: 2");

                        foundMineral = true;
                        float goldMineralX = -1;
                        float silverMineral1X = -1;
                        float silverMineral2X = -1;

                        for(int i=0; i<2; i++){
                            if(updatedRecognitions.get(i).getLabel().equals(LABEL_GOLD_MINERAL)){
                                goldMineralX = updatedRecognitions.get(i).getLeft();
                                telemetry.addLine("gold " + i + ": width: " + Float.toString(updatedRecognitions.get(i).getWidth()) + " height: "
                                        + Float.toString(updatedRecognitions.get(i).getHeight()));
                            } else if(silverMineral1X == -1){

                                telemetry.addLine("silver " + i + ": width: " + Float.toString(updatedRecognitions.get(i).getWidth()) + " height: "
                                        + Float.toString(updatedRecognitions.get(i).getHeight()));
                                silverMineral1X = updatedRecognitions.get(i).getLeft();
                            } else {

                                telemetry.addLine("silver " + i + ": width: " + Float.toString(updatedRecognitions.get(i).getWidth()) + " height: "
                                        + Float.toString(updatedRecognitions.get(i).getHeight()));
                                silverMineral2X = updatedRecognitions.get(i).getLeft();
                            }
                        }

                        if(goldMineralX == -1)
                            minerals[3] = 2;
                        else if(goldMineralX < silverMineral1X){
                            minerals[1] = 2;
                        } else {
                            minerals[2] = 2;
                        }

                    } else if(updatedRecognitions.size() == 1 && time.seconds() > 7.5){

                        telemetry.addLine("seconds passed: " + time.seconds());
                        telemetry.addLine("objects detected: 1");
                        telemetry.addLine("obj " + 0 + ": width: " + Float.toString(updatedRecognitions.get(0).getWidth()) + "height: "
                                + Float.toString(updatedRecognitions.get(0).getHeight()));

                        float goldMineralX = -1;
                        float silverMineralX = -1;

                        if(updatedRecognitions.get(0).getLabel().equals(LABEL_GOLD_MINERAL)){
                            goldMineralX = updatedRecognitions.get(0).getLeft();
                        } else {
                            silverMineralX = updatedRecognitions.get(0).getLeft();
                        }

                        if(goldMineralX != -1 && goldMineralX < 300){
                            minerals[1] = 2;
                        } else if(goldMineralX != -1 && goldMineralX > 300) {
                            minerals[2] = 2;
                        }

                    }
                }

                for(int i=1; i<= 3; i++)
                    if(minerals[i] == 2)
                        mineralPosition = i;
                telemetry.addLine("mineral position: " + mineralPosition);
                telemetry.update();

                return minerals;
            }

        });

    }

    private void deployRobot() throws InterruptedException {

        Future<int[]> mineralPosition = getMineralAsync();
        while(robot.mechLiftLeft.getCurrentPosition() < robot.MAX_LIFT_POSITION &&
                robot.mechLiftRight.getCurrentPosition() < robot.MAX_LIFT_POSITION && !isStopRequested()
                && !testingEnabled){
            robot.liftMovement(robot.LIFT_SPEED, false);
        }

        robot.liftMovement(0, false);

        robot.setDrivetrainPosition(-500, "translation", 1);
        robot.setDrivetrainPosition(800, "strafing", .6);
        robot.setDrivetrainPosition(500, "translation", 1);

        try {
            if(!mineralPosition.isDone()){
                mineralPosition.cancel(true);
                sampleMineral(new int[]{0, 0, 0, 0});
            } else {
                sampleMineral(mineralPosition.get());
            }
            executorService.shutdown();
        } catch (ExecutionException e){
            telemetry.addLine("Something went wrong: " + e.toString());
        } finally {
            telemetry.update();
        }

    }

    private void sampleMineral(int[] mineralPosition) {
        int goldMineralPosition = -1;

//        mineralPosition[1] = 0;
//        mineralPosition[2] = 0;
//        mineralPosition[3] = 0;

        for(int index = 1; index <= 3; index++){
            if(mineralPosition[index] == 2){
                goldMineralPosition = index;
            }
        }

        if(goldMineralPosition == -1){
            mineralPosition = new int[] {0, 0, 0, 0};
            telemetry.clear();
            attemptSampleFromGround(mineralPosition); // aia daca nu merge bine principala ;)
        } else {
            int distanceToTravel;
            switch (goldMineralPosition){
                case 1: robot.gyroRotationWIP(350, "absolute", "Crater");
                    distanceToTravel = -3300;
                    break;
                case 2: robot.gyroRotationWIP(315, "absolute", "Crater");
                    distanceToTravel = -2900;
                    break;
                case 3: robot.gyroRotationWIP(100, "absolute", "Crater");
                    distanceToTravel = +3300;
                    break;
                default: distanceToTravel = 0;
            }
            telemetry.clear();
            telemetry.addLine("rotation: " + robot.globalGyroValue("Crater"));
            telemetry.update();

            int distanceToGoBack = -distanceToTravel / 10 * 5;

            robot.setDrivetrainPosition(distanceToTravel, "translation", 1);

            robot.setDrivetrainPosition(distanceToGoBack, "translation", 1);

            new AutonomCrater_V2(robot, null, "Crater").gettingInLanderPosition(goldMineralPosition);
        }

    }

    private void attemptSampleFromGround(int[] mineralPosition) {

        new AutonomCrater_V2(robot, tfod, "Crater").samplingStuff(mineralPosition);


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

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = .85;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
