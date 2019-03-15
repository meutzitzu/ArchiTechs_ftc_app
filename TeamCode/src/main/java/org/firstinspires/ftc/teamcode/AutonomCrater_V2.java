package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(name="Autonom Crater v2", group="Linear OpMode")
public class AutonomCrater_V2 extends LinearOpMode {


        /** TensorFlow Variables */
        private final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
        private final String LABEL_GOLD_MINERAL = "Gold Mineral";
        private final String LABEL_SILVER_MINERAL = "Silver Mineral";

        private final double DETECTIONSPEED = 0.25;
        private final double MAXDETECTIONTIME = 100;
        private int initialLiftPosition;

        private final String VUFORIA_KEY = "AWcgpa7/////AAABmcy/3X8/j0O5rVl/TFsI7jtI2X65iRJuPT0JA+JxFlcGoXjuri+AHgItnHFgUGE5xkMhjhPpZ57eT9HxlpFmryfrXSxOYlX58SyvvCbZo+ftIlY4+x3iNw03eNywXKmPBdM7jmGEk6G1HViitwJy8CrOooxYAl37Vh7w0BZipSRVSDKg0AA+jj7ExvVYPedxSBlkTpR9VyUe7hNfWlK/ijmNcpmiYVYomUbPmef2TqIkxSYvBJKZF7vblCmtlmiSrmY1zyO7Y9xKk46vQ8x7cL8tTZG0zDzfDEC12KbCAJLqSN0qju6Z1gsTAIEJmwvAG0YAfKvZf7oSwtno0t7ZfhfY/2LUws3ydkJUVyZGOB7k";
        final String side = "Crater";

        private VuforiaLocalizer vuforia;
        private TFObjectDetector tfod;

        boolean goldMineral = false;
        boolean samplingFailure = false;
        boolean autonomonTesting = true;
        boolean tensorFlowSafetyNotInitialized = false;


        Robot robot;
        ElapsedTime runtime = new ElapsedTime();
        ElapsedTime errorTimer = new ElapsedTime();

        public AutonomCrater_V2(Robot robot, TFObjectDetector tfod){
            this.robot = robot;
            this.tfod = tfod;
        }

        @Override
        public void runOpMode () throws InterruptedException {

            samplingStuff(new int[]{0, 0, 0, 0});


        }


            /** TensorFlow inits*/
//        void initStuff() throws InterruptedException {
//            robot.init(hardwareMap, false, telemetry, this);
//
//            robot.mechLiftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.mechLiftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//            robot.mechLiftRight.setPower(0);
//            robot.mechLiftRight.setPower(0);
//
//            robot.mechLiftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.mechLiftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            initVuforia();
//            initTfod();
//            initSafeties();
//        }
//
//        void initVuforia() {
//        /*
//         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
//         */
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//
//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
//
//        //  Instantiate the Vuforia engine
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//
//        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
//
//            if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
//                initTfod();
//                tensorFlowSafetyNotInitialized = false;
//            } else {
//                tensorFlowSafetyNotInitialized = true;
//            }
//        }
//
//        void initTfod() {
//            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//            tfodParameters.minimumConfidence = .8;
//            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
//        }
//
//            /**After init seafeties
//            *  Alerts if something is not ok
//            */
//        void initSafeties(){
//            gyroSafety();
//            tensorFlowSafety();
//
//            telemetry.addLine("Init finalized successfully");
//            telemetry.update();
//        }
//
//        void gyroSafety(){
//            //not sure how to implement it
//        }
//
//        void tensorFlowSafety(){
//            if(!tensorFlowSafetyNotInitialized){
//                telemetry.addLine("TensorFlow is ok");
//            }
//            else{
//
//                errorTimer.reset();
//                while(errorTimer.seconds() <= 5){
//                    telemetry.addLine("Something went wrong with Tensor flow");
//                    telemetry.update();
//                }
//
//                stop();
//            }
//        }

            /** Robot going down
            *  Separating frm the lander to begin recognition */
        void deployRobot(){

            //Robot going down
            //Can be omitted if for test purposes
//            if(!autonomonTesting) {
//                while (robot.mechLiftLeft.getCurrentPosition() < robot.MAX_LIFT_POSITION &&
//                        robot.mechLiftRight.getCurrentPosition() < robot.MAX_LIFT_POSITION && !isStopRequested()) {
//                    robot.liftMovement(robot.LIFT_SPEED, false);
//                }
//                telemetry.addData("Lift Position", robot.mechLiftLeft.getCurrentPosition());
//            }
//            else{
//                telemetry.addLine("Tesint mode, without lift");
//            }
//            robot.liftMovement(0, false);
//            telemetry.update();


            //tensor activation
//            if (tfod != null) {
//                tfod.activate();
//            }

            //moving to get in position for TensorFlow scan

            //out of the hook
            robot.setDrivetrainPosition(-300, "translation", 1);

            //away from the lander
            robot.setDrivetrainPosition(800, "strafing", 1);

            //back to initial
            robot.setDrivetrainPosition(300, "translation", 1);

        }


            /** TensorFlow here*/
        void samplingStuff(int[] mineralSequence){
            tensorDetection(mineralSequence);
        }

        void tensorDetection(int[] mineralSequence) {

            int goldMineralX; //x coordinate of the
            int silverMineralX; //x coordinate of silver mineral
            int[][] extremecoordinatesMinerals = new int[][]{{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {-1, -1, -1, -1}};
            int mineralSum = 0; // sum of the first 3 elements of the array
            int recognitionPosition = 2;
            boolean goldMineral = false;
            boolean newSphere;
            int sphereTruePosition, robotSphereAngle, sphereVuAngle;
            int firstRecognitionRaw = 343, secondRecognitionRaw = 305;
            int firstRecognition, secondRecognition;
            int supportAngle = 20;

            ElapsedTime mineralRetrievalTimer = new ElapsedTime();
            boolean firstRecognitionCheck = true;

            firstRecognition = robot.mathModulo(firstRecognitionRaw - (270 + supportAngle), 360);
            secondRecognition = robot.mathModulo(secondRecognitionRaw - (270 + supportAngle), 360);

            robot.telemetry.clear();

            for(int i = 1; i <= 3; i++){
                if(mineralSequence[i] != 0){
                    extremecoordinatesMinerals[0][1] = mineralSequence[i];
                    if(i == 1){
                        recognitionPosition = 1;
                        extremecoordinatesMinerals[3][1] = 360;
                    }
                    else if(i == 2) {
                        recognitionPosition = 2;
                        extremecoordinatesMinerals[3][1] = 315;
                    }
                    else{
                        recognitionPosition = 2;
                    }
                }
                robot.telemetry.addData(Integer.toString(i ), mineralSequence[i]);
            }





            robot.telemetry.clear();
            while (!isStopRequested() && mineralSum < 2 && tfod != null) {

                mineralSum = 0;

                if ((mineralRetrievalTimer.milliseconds() >= 1000  || firstRecognitionCheck) && recognitionPosition == 2 ) {
                    robot.gyroRotationWIP(firstRecognition, "absolute", "Crater");
                    recognitionPosition = 1;
                    firstRecognitionCheck = false;
                    mineralRetrievalTimer.reset();
                }
                else if((mineralRetrievalTimer.milliseconds() >= 1000  || firstRecognitionCheck) && recognitionPosition == 1 ) {
                    robot.gyroRotationWIP(secondRecognition, "absolute", "Crater");
                    recognitionPosition = 2;
                    firstRecognitionCheck = false;
                    mineralRetrievalTimer.reset();
                }



                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                if (updatedRecognitions != null) {

                    for (Recognition recognition : updatedRecognitions) {

                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft() + (int)recognition.getWidth() / 2;

                            goldMineral = true;
                            for (int i = 1; i <= 3; i++) {
                                if (extremecoordinatesMinerals[0][i] == 0) {
                                    extremecoordinatesMinerals[2][i] = goldMineralX;
                                    extremecoordinatesMinerals[1][i] = robot.globalGyroValue("Crater") + 270 + supportAngle;
                                    extremecoordinatesMinerals[0][i] = 2;
                                    extremecoordinatesMinerals[3][i] = robot.VutoDegrees(extremecoordinatesMinerals[2][i]) + robot.mathModulo(extremecoordinatesMinerals[1][i], 360);
                                    extremecoordinatesMinerals[3][i] = robot.mathModulo(extremecoordinatesMinerals[3][i], 360);
                                    break;
                                }
                            }

                        }

                        if(recognition.getLabel().equals(LABEL_SILVER_MINERAL)){
                            newSphere = true;
                            silverMineralX = (int) recognition.getLeft() + (int)recognition.getWidth() / 2;
                            robotSphereAngle = robot.globalGyroValue("Crater") + 270 + supportAngle;
                            sphereVuAngle = robot.VutoDegrees(silverMineralX);

                            sphereTruePosition = robot.mathModulo(sphereVuAngle + Math.abs(robotSphereAngle), 360);

                            for(int i = 1;i <= 3;i ++){
                                if(extremecoordinatesMinerals[0][i] != 0){
                                    if(extremecoordinatesMinerals[3][i] < 100){
                                        extremecoordinatesMinerals[3][i] = 350;
                                    }
                                    if(Math.abs(sphereTruePosition - extremecoordinatesMinerals[3][i]) < 20){
                                        newSphere = false;
                                    }
                                }
                            }

                            if(newSphere) {
                                for (int i = 1; i <= 3; i++) {
                                    if (extremecoordinatesMinerals[0][i] == 0) {
                                        extremecoordinatesMinerals[2][i] = silverMineralX;
                                        extremecoordinatesMinerals[1][i] = robotSphereAngle;
                                        extremecoordinatesMinerals[0][i] = 1;
                                        extremecoordinatesMinerals[3][i] = sphereTruePosition;
                                        break;
                                    }
                                }
                            }


                        }

                        if(goldMineral){
                            break;
                        }
                    }


                }


                for(int i = 1; i <= 3; i++){
                    mineralSum += extremecoordinatesMinerals[0][i];
                }

                robot.telemetry.addData("P", extremecoordinatesMinerals[0][1] + " " + extremecoordinatesMinerals[0][2] +" " + extremecoordinatesMinerals[0][3]);
                robot.telemetry.addData("X", extremecoordinatesMinerals[1][1] + " " + extremecoordinatesMinerals[1][2] +" " + extremecoordinatesMinerals[1][3]);
                robot.telemetry.addData("Vu", extremecoordinatesMinerals[2][1] + " " + extremecoordinatesMinerals[2][2] +" " + extremecoordinatesMinerals[2][3]);
                robot.telemetry.addData("Pos", extremecoordinatesMinerals[3][1] + " " + extremecoordinatesMinerals[3][2] +" " + extremecoordinatesMinerals[3][3]);
                robot.telemetry.update();


            }


            for(int i = 1;i <= 3; i++){
                if(extremecoordinatesMinerals[3][i] == -1){
                    //pur si simplu trece peste
                }
                else if(extremecoordinatesMinerals[3][i] > 337 || extremecoordinatesMinerals[3][i] < 100){
                    if(mineralSequence[1] == 0) {
                        mineralSequence[1] = extremecoordinatesMinerals[0][i];
                    }
                    else{
                        mineralSequence[2] = extremecoordinatesMinerals[0][i];
                    }
                }
                else if(extremecoordinatesMinerals[3][i] <= 337 && extremecoordinatesMinerals[3][i] >= 293){
                    if(mineralSequence[2] == 0) {
                        mineralSequence[2] = extremecoordinatesMinerals[0][i];
                    }
                    else{
                        mineralSequence[3] = extremecoordinatesMinerals[0][i];
                    }
                }
                else if(extremecoordinatesMinerals[3][i] < 293){
                    mineralSequence[3] = extremecoordinatesMinerals[0][i];
                }
            }

            robot.telemetry.addData("order", mineralSequence[1] + " " + mineralSequence[2] + " " + mineralSequence[3]);
            robot.telemetry.update();



            for(int i = 1;i <=3; i++){
                if(mineralSequence[i] == 0 && goldMineral) {
                    mineralSequence[i] = 1;
                }
                else if(mineralSequence[i] == 0 && !goldMineral){
                    mineralSequence[i] = 2;
                }
            }



            mineralDisplacement(mineralSequence);


//            for(int i = 1; i <= 3; i++){
//                if(extremecoordinatesMinerals[0][i] != 0){
//
//                }
//            }

        }
        void mineralDisplacement(int[] mineralSequence){

            int goldMineralPosition = -1; //can be 1, 2, 3

            int lateralDistance = -3300; //-3200
            int midDistance = -2900; //-2500
            int hittingMineralDistance;
            int leftPosition = 350, midPosition = 315, rightPosition = 280;

            for(int i = 1; i <= 3; i++){
                if(mineralSequence[i] == 2){
                    goldMineralPosition = i;
                }
            }



            switch (goldMineralPosition){
                case 1:
                    robot.gyroRotationWIP(leftPosition, "absolute", "Crater");
                    hittingMineralDistance = lateralDistance;
                    break;

                case 2:
                    robot.gyroRotationWIP(midPosition, "absolute", "Crater");
                    hittingMineralDistance = midDistance;
                    break;

                case 3:
                    robot.gyroRotationWIP(rightPosition, "absolute", "Crater");
                    hittingMineralDistance = lateralDistance;
                    break;

                default:
                    hittingMineralDistance = 0;
                    robot.telemetry.addLine("No gold mineral");
                    robot.telemetry.update();
                    stop();
            }

            robot.setDrivetrainPosition(hittingMineralDistance, "translation", 1);

            robot.setDrivetrainPosition((-hittingMineralDistance) / 10 * 7, "translation", 1);

            this.gettingInLanderPosition();


        }

            /** Getting near the wall
            *  Going for the placement of the toy
            *  Actually placing the toy*/
        void toyPlacingfStuff(){
            //nu stiu daca asta ramane
            gettingBackToInitial();
            nearingTheWallBefore();
            parallelToTheWall();
            actualToyPlacing();
        }

        void gettingBackToInitial(){
            //getting back to what position we want before going in to place the team marker
//            robot.setDrivetrainPosition(-hittingMineralDistance, "translation", 1.0);
        }
        void nearingTheWallBefore(){
            //parralel to the lander -> not hitting other minerals and knowing an approximate robot position
            robot.absgyroRotation(0, "absolute");

            //alligning with the wall before going for the toy
            while(robot.leftDistanceSensor.getDistance(DistanceUnit.CM) > 15 && !isStopRequested()){
                if(robot.leftDistanceSensor.getDistance(DistanceUnit.CM) < 30)
                    robot.mecanumMovement(0, .3, 0);
                else
                    robot.mecanumMovement(0, .7, 0);

                robot.telemetry.addLine("Distance from sensor: " + robot.leftDistanceSensor.getDistance(DistanceUnit.CM));
                robot.telemetry.update();
            }
            robot.mecanumMovement(0, 0, 0);

            robot.absgyroRotation(45, "absolute");
        }
        void parallelToTheWall(){
                while(opModeIsActive()){

                }
        }
        void actualToyPlacing(){

        }

            /** Parking methods
            *  getting near the crater
            *  parking the arm
            */

         void parkingMethod(){
             nearingTheCrater();
             parkingTheArm();
         }

         void nearingTheCrater(){

         }

         void parkingTheArm(){

         }


         //auxiliary functions
         int gyroValue(){ return robot.globalGyroValue(side); }



         public void gettingInLanderPosition(){
             double proportionalConstant = 1.0 / 60;
             double currentError = robot.leftDistanceSensor.getDistance(DistanceUnit.CM) - 5;
             double outSpeed; // 1->strafing speed (negative for left), 2->forwardSpeed (positive for reverse)
             int ticksToDepot;
             double minDistanceToWall = 12;

             robot.gyroRotationWIP(45, "absolute", "Crater");

             robot.setDrivetrainPosition(-3000, "translation", 1);

             robot.gyroRotationWIP(90, "absolute", "Crater");

             while(currentError > minDistanceToWall && !robot.opMode.isStopRequested()){
                 currentError = robot.leftDistanceSensor.getDistance(DistanceUnit.CM);

                 outSpeed = proportionalConstant * currentError;

                 outSpeed = Range.clip(outSpeed, -1, 1);

                 robot.mecanumMovement(outSpeed, 0, 0);

                 robot.telemetry.addData("distance", currentError);
                 robot.telemetry.update();
             }

             robot.setDrivetrainPosition(-300, "strafing", 1);

//             robot.setDrivetrainMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

             ticksToDepot = robot.driveFrontLeft.getCurrentPosition();

             robot.mecanumMovement(0, 1, 0);


             while(robot.rightDistanceSensor.getDistance(DistanceUnit.CM) > 55){
                 robot.telemetry.addData("Distance", robot.rightDistanceSensor.getDistance(DistanceUnit.CM));
                 robot.telemetry.update();
             }

             sleep(1000);

             ticksToDepot = -ticksToDepot + robot.driveFrontLeft.getCurrentPosition();

             robot.setDrivetrainPosition(ticksToDepot + 200 , "translation", 1);

             robot.mecanumMovement(0,0,0);

             while(robot.opMode.opModeIsActive()){
                 robot.telemetry.addData("dwqdwq", "over");
                 robot.telemetry.update();
             }

         }

}
