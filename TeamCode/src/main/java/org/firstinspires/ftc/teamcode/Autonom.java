package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name="A1", group="Linear OpMode")
public class Autonom extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final double DETECTIONSPEED = 0.3;
    private static final double MAXDETECTIONTIME = 100;
    public int detectionDirection = 1;
    private int initialLiftPosition;

    private static final String VUFORIA_KEY = "AWcgpa7/////AAABmcy/3X8/j0O5rVl/TFsI7jtI2X65iRJuPT0JA+JxFlcGoXjuri+AHgItnHFgUGE5xkMhjhPpZ57eT9HxlpFmryfrXSxOYlX58SyvvCbZo+ftIlY4+x3iNw03eNywXKmPBdM7jmGEk6G1HViitwJy8CrOooxYAl37Vh7w0BZipSRVSDKg0AA+jj7ExvVYPedxSBlkTpR9VyUe7hNfWlK/ijmNcpmiYVYomUbPmef2TqIkxSYvBJKZF7vblCmtlmiSrmY1zyO7Y9xKk46vQ8x7cL8tTZG0zDzfDEC12KbCAJLqSN0qju6Z1gsTAIEJmwvAG0YAfKvZf7oSwtno0t7ZfhfY/2LUws3ydkJUVyZGOB7k";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    boolean goldMineral = false;

    Robot robot = new Robot();
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime mineralRetrievalTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, false, telemetry, this);
        initStuff();

        telemetry.update();

        waitForStart();

        runtime.reset();


        deployRobot();

        telemetry.clear();
        telemetry.addData("Deploy", "Over");
        telemetry.update();

        sleep(1000);

        mineralRetrievalTimer.reset();

        initiateRecognition();

        while(opModeIsActive()){

        }



//        robot.absgyroRotation(30, "absolute");
//
//        sleep(2000);
//
//        robot.absgyroRotation(30, "relative");
//
//        sleep(2000);
//
//        robot.absgyroRotation(360, "absolute");
//
//        stop();
//
//        robot.mechRotation.setTargetPosition(-400);
//        robot.mechRotation.setPower(0.1);
//
//        while(robot.mechLiftLeft.getCurrentPosition() < robot.MAX_LIFT_POSITION && !isStopRequested()){
//            robot.liftMovement(robot.LIFT_SPEED);
//            telemetry.addData("Status:  ", "Detaching from lander I");
//        }
//
//        robot.liftMovement(0);
//
//
//        robot.setDrivetrainPosition(-600, "rotation", 0.3);
//
//        while(robot.driveRearLeft.isBusy()) {
//            telemetry.addData("Status:  ", "Detaching from lander II");
//            telemetry.update();
//        }
//
//        robot.setDrivetrainPosition(300,"translation", 0.3);
//
//
//        while(robot.driveFrontLeft.isBusy()){
//            telemetry.addData("Status:  ", "Detaching from lander III");
//            telemetry.update();
//        }


        ///Here comes tensor flow stuff --- gyro would probably come in handy


        ///Placing the team marker


        ///Parking in the crater


    }



    private  void  initiateRecognition(){

        int goldMineralXL = -1;
        int goldMineralXR = -1;


        if (tfod != null) {
            tfod.activate();
        }

        while(!goldMineral && tfod != null && !isStopRequested() && mineralRetrievalTimer.seconds() <= MAXDETECTIONTIME) {

            if(robot.modernRoboticsI2cGyro.getIntegratedZValue() < -125 && detectionDirection == 1){
                detectionDirection = -1;
            }
            else if(robot.modernRoboticsI2cGyro.getIntegratedZValue() > -45 && detectionDirection == -1){
                detectionDirection = 1;
            }
            robot.mecanumMovement(0,0,detectionDirection * DETECTIONSPEED);

            // TensorFlow stuff goes here

            ///////////////////////////////////

            //robot.setDrivetrainPosition(robot.driveFrontLeft.getCurrentPosition()-40, "rotating", .3);

            /* while((robot.driveFrontLeft.isBusy() || robot.driveFrontRight.isBusy()
                    || robot.driveRearLeft.isBusy() || robot.driveRearRight.isBusy()) && !isStopRequested()){
                telemetry.addData("Drivetrain front left", robot.driveFrontLeft.getCurrentPosition());
                telemetry.addData("Drivetrain front right", robot.driveFrontRight.getCurrentPosition());
                telemetry.addData("Drivetrain rear left", robot.driveRearLeft.getCurrentPosition());
                telemetry.addData("Drivetrain rear right", robot.driveRearRight.getCurrentPosition());
                telemetry.update();
            } */


            ///////////////////////////////////

            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();


            if(updatedRecognitions != null) {
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        goldMineral = true;
                        goldMineralXL = (int) recognition.getLeft();
                        goldMineralXR = (int) recognition.getRight();
                        telemetry.clear();
                        telemetry.addData("Gold mineral", goldMineralXL);
                    }
                }
            }
            else{
                telemetry.clear();
                telemetry.addData("Gyro Value", robot.modernRoboticsI2cGyro.getIntegratedZValue());
                telemetry.addData("No", "Object");
            }
            telemetry.update();

        }

        robot.mecanumMovement(0,0,0);

        telemetry.addData("mineralposition", goldMineralXL);
        telemetry.update();

        sleep(1000);

        while(goldMineral == true && !(goldMineralXL < 450 && goldMineralXL > 175) && !isStopRequested()) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if (updatedRecognitions != null) {
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        goldMineralXL = (int) recognition.getLeft();
                    }
                }
            }

            if(goldMineralXL > 700){
                detectionDirection = 1;
            }
            else if(goldMineralXL < 560){
                detectionDirection = -1;
            }

            robot.mecanumMovement(0,0,DETECTIONSPEED * detectionDirection);

            telemetry.addData("mineral", goldMineralXL);
            telemetry.update();

        }

        telemetry.clear();
        telemetry.addLine("out of the while :)");
        telemetry.addData("mineral position", goldMineralXL);
        telemetry.update();

        robot.setDrivetrainPosition(-2300, "translation", .3);

//        sleep(3000);



        //robot.setDrivetrainPosition(-1000, "translation", 0.5);

        while(opModeIsActive()){

        }

        int minxl=0;
        int minx2;

        if(goldMineral==true){

            while (goldMineralXL<minxl) {
                robot.setDrivetrainPosition(robot.driveFrontLeft.getCurrentPosition()-40, "rotating", .3);

                while((robot.driveFrontLeft.isBusy() || robot.driveFrontRight.isBusy()
                        || robot.driveRearLeft.isBusy() || robot.driveRearRight.isBusy()) && !isStopRequested()){
                    telemetry.addData("Drivetrain front left", robot.driveFrontLeft.getCurrentPosition());
                    telemetry.addData("Drivetrain front right", robot.driveFrontRight.getCurrentPosition());
                    telemetry.addData("Drivetrain rear left", robot.driveRearLeft.getCurrentPosition());
                    telemetry.addData("Drivetrain rear right", robot.driveRearRight.getCurrentPosition());
                    telemetry.update();
                }

            }
            }

        }


    private void deployRobot() {
        while(robot.mechLiftLeft.getCurrentPosition() < robot.MAX_LIFT_POSITION && !isStopRequested()){
            robot.liftMovement(robot.LIFT_SPEED);
            if(robot.mechLiftRight.getCurrentPosition() > robot.MAX_LIFT_POSITION)
                break;
            telemetry.addData("Lift position", robot.mechLiftLeft.getCurrentPosition());
            telemetry.update();
        }

        robot.liftMovement(0);
        telemetry.clear();

        robot.setDrivetrainMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.setDrivetrainPosition(-300, "translation", .3);
        while(robot.driveFrontLeft.isBusy() && robot.driveFrontRight.isBusy()
                && robot.driveRearLeft.isBusy() && robot.driveRearRight.isBusy() && !isStopRequested()){
            telemetry.addLine("Going back");
            telemetry.update();
        }


        robot.setDrivetrainPosition(1000, "strafing", .3);

        while(robot.driveFrontLeft.isBusy() && robot.driveFrontRight.isBusy()
                && robot.driveRearLeft.isBusy() && robot.driveRearRight.isBusy() && !isStopRequested()){
            telemetry.addLine("Mode: strafing");
            telemetry.addData("Busy motors", robot.driveFrontLeft.isBusy() + " " + robot.driveFrontRight.isBusy() + " " + robot.driveRearLeft.isBusy() + " " + robot.driveRearRight.isBusy() + " ");
            telemetry.update();
        }

        robot.setDrivetrainPosition(300, "translation", .3);
        while(robot.driveFrontLeft.isBusy() && robot.driveFrontRight.isBusy()
                && robot.driveRearLeft.isBusy() && robot.driveRearRight.isBusy() && !isStopRequested()){
            telemetry.addLine("Going forward");
            telemetry.addData("Busy motors", robot.driveFrontLeft.isBusy() + " " + robot.driveFrontRight.isBusy() + " " + robot.driveRearLeft.isBusy() + " " + robot.driveRearRight.isBusy() + " ");
            telemetry.update();
        }


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
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

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
