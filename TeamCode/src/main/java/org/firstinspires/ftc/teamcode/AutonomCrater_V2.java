package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

@Autonomous(name="Autonom Crater", group="Linear OpMode")
public class AutonomCrater_V2 extends LinearOpMode {


        /** TensorFlow Variables */
        private final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
        private final String LABEL_GOLD_MINERAL = "Gold Mineral";
        private final String LABEL_SILVER_MINERAL = "Silver Mineral";

        private final double DETECTIONSPEED = 0.25;
        private final double MAXDETECTIONTIME = 100;
        private int detectionDirection = 1;
        private int initialLiftPosition;

        private final String VUFORIA_KEY = "AWcgpa7/////AAABmcy/3X8/j0O5rVl/TFsI7jtI2X65iRJuPT0JA+JxFlcGoXjuri+AHgItnHFgUGE5xkMhjhPpZ57eT9HxlpFmryfrXSxOYlX58SyvvCbZo+ftIlY4+x3iNw03eNywXKmPBdM7jmGEk6G1HViitwJy8CrOooxYAl37Vh7w0BZipSRVSDKg0AA+jj7ExvVYPedxSBlkTpR9VyUe7hNfWlK/ijmNcpmiYVYomUbPmef2TqIkxSYvBJKZF7vblCmtlmiSrmY1zyO7Y9xKk46vQ8x7cL8tTZG0zDzfDEC12KbCAJLqSN0qju6Z1gsTAIEJmwvAG0YAfKvZf7oSwtno0t7ZfhfY/2LUws3ydkJUVyZGOB7k";

        private VuforiaLocalizer vuforia;
        private TFObjectDetector tfod;

        boolean goldMineral = false;
        boolean samplingFailure = false;
        boolean autonomonTesting = true;
        boolean tensorFlowSafetyNotInitialized = false;

        int goldMineralPosition = -1; //can be 1, 2, 3
        int leftPosition = -50, midPosition = -87, rightPosition = -120;
        int lateralDistance = -3200;
        int midDistance = -2500;
        int hittingMineralDistance = 0;

        Robot robot = new Robot();
        ElapsedTime runtime = new ElapsedTime();
        ElapsedTime mineralRetrievalTimer = new ElapsedTime();
        ElapsedTime errorTimer = new ElapsedTime();

        @Override
        public void runOpMode () throws InterruptedException {
            initStuff();

            deployRobot();

            samplingStuff();

            toyPlacingfStuff();

            parkingMethod();

        }


            /** TensorFlow inits*/
        void initStuff() throws InterruptedException {
            robot.init(hardwareMap, false, telemetry, this);
            initVuforia();
            initTfod();
            initSafeties();
        }

        void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.

            if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                initTfod();
                tensorFlowSafetyNotInitialized = false;
            } else {
                tensorFlowSafetyNotInitialized = true;
            }
        }

        void initTfod() {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParameters.minimumConfidence = .8;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
        }

            /**After init seafeties
            *  Alerts if something is not ok
            */
        void initSafeties(){
            gyroSafety();
            tensorFlowSafety();

            telemetry.addLine("Init finalized successfully");
            telemetry.update();
        }

        void gyroSafety(){
            //not sure how to implement it
        }

        void tensorFlowSafety(){
            if(!tensorFlowSafetyNotInitialized){
                telemetry.addLine("TensorFlow is ok");
            }
            else{

                errorTimer.reset();
                while(errorTimer.seconds() <= 5){
                    telemetry.addLine("Something went wrong with Tensor flow");
                    telemetry.update();
                }

                stop();
            }
        }

            /** Robot going down
            *  Separating frm the lander to begin recognition */
        void deployRobot(){

            //Robot going down
            //Can be omitted if for test purposes
            if(!autonomonTesting) {
                while (robot.mechLiftLeft.getCurrentPosition() < robot.MAX_LIFT_POSITION &&
                        robot.mechLiftRight.getCurrentPosition() < robot.MAX_LIFT_POSITION && !isStopRequested()) {
                    robot.liftMovement(robot.LIFT_SPEED, false);
                }
                telemetry.addData("Lift Position", robot.mechLiftLeft.getCurrentPosition());
            }
            else{
                telemetry.addLine("Tesint mode, without lift");
            }
            robot.liftMovement(0, false);
            telemetry.update();

            //moving to get in position for TensorFlow scan

            //out of the hook
            robot.setDrivetrainPosition(-300, "translation", .6);

            //away from the lander
            robot.setDrivetrainPosition(1000, "strafing", .3);

            //back to initial
            robot.setDrivetrainPosition(300, "translation", .6);

        }


            /** TensorFlow here*/
        void samplingStuff(){
            tensorDetection();
            mineralDisplacement();
        }

        void tensorDetection(){

        }
        void mineralDisplacement(){

            switch (goldMineralPosition){
                case 1:
                    robot.absgyroRotation(leftPosition, "absolute");
                    hittingMineralDistance = lateralDistance;
                    break;

                case 2:
                    robot.absgyroRotation(midPosition, "absolute");
                    hittingMineralDistance = midDistance;
                    break;

                case 3:
                    robot.absgyroRotation(rightPosition, "absolute");
                    hittingMineralDistance = lateralDistance;
                    break;

                default:
                    hittingMineralDistance = 0;
                    telemetry.addLine("No gold mineral");
                    telemetry.update();
                    stop();
            }

            robot.setDrivetrainPosition(hittingMineralDistance, "translation", .8);

            telemetry.addData("Mineral distance travelled to hit", hittingMineralDistance);

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
            robot.setDrivetrainPosition(-hittingMineralDistance, "translation", 1.0);
            telemetry.addLine("Back to before sampling");
        }
        void nearingTheWallBefore(){
            //parralel to the lander -> not hitting other minerals and knowing an approximate robot position
            robot.absgyroRotation(0, "absolute");

            //alligning with the wall before going for the toy
            while(robot.distanceSensor.getDistance(DistanceUnit.CM) > 15 && !isStopRequested()){
                if(robot.distanceSensor.getDistance(DistanceUnit.CM) < 30)
                    robot.mecanumMovement(0, .3, 0);
                else
                    robot.mecanumMovement(0, .7, 0);

                telemetry.addLine("Distance from sensor: " + robot.distanceSensor.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
            robot.mecanumMovement(0, 0, 0);

            robot.absgyroRotation(45, "absolute");
        }
        void parallelToTheWall(){

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

}
