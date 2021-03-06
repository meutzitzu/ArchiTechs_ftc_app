package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.sin;
import static java.lang.Thread.sleep;

public class Robot {

    /**Declare hardware variables */

        public DcMotor driveFrontLeft = null;
        public DcMotor driveFrontRight = null;
        public DcMotor driveRearLeft = null;
        public DcMotor driveRearRight = null;
        public DcMotor mechRotation = null;  //arm movement
        public DcMotor mechExt = null;
        public DcMotor mechLiftLeft = null;
        public DcMotor mechLiftRight = null;
        public Servo mechStopper = null;
        public CRServo mechGrab = null;  //servo used on the grabber
        IntegratingGyroscope gyro;
        ModernRoboticsI2cGyro modernRoboticsI2cGyro;
        public DistanceSensor leftDistanceSensor;
        public DistanceSensor rightDistanceSensor;
        public Telemetry telemetry;
        public LinearOpMode opMode;


    /** Global constants */
        public final double MAX_CRSERVO_INPUT = 0.82;  //max power that can be
        public final double LIFT_SPEED = 1;
        public final double ROTATION_SPEED_MODIFIER = 0.2;  //max angular velocity of the arm
        public final double MIN_LIFT_POSITION = 0;
        public final int MAX_LIFT_POSITION = 23000;
        public final int MAX_ROTATION = 2300;  //will probably be ignored; used only as reference for minimum position
        public final int MIN_ROTATION = 0;  //useful
        public final int ROTATION_LENGTH = MAX_ROTATION - MIN_ROTATION; //
        public double PI = 3.14159;
        public final double DRIVING_COEF = 0.7; //max speed is a little to much for our competent drivers
        public final double MAX_EXT = 1700;
        public final double MIN_EXT = 0;
        public final double STOPPER_OPEN = 0;
        public final double STOPPER_CLOSED = 90;
        public final double GRABBING_SPEED = 0.82;
        public final double RAMPING_DOWN_TIME = 300; //milliseconds



    /** Auxiliary variables */


    public void init(HardwareMap hashMap, boolean teleOp, Telemetry tele, LinearOpMode mode) throws InterruptedException {

        /** Initializing hardware variables */

        driveFrontLeft  = hashMap.get(DcMotor.class, "driveFrontLeft");
        driveFrontRight = hashMap.get(DcMotor.class, "driveFrontRight");
        driveRearLeft  = hashMap.get(DcMotor.class, "driveRearLeft");
        driveRearRight = hashMap.get(DcMotor.class, "driveRearRight");
        mechRotation = hashMap.get(DcMotor.class, "mechRotation");
        mechLiftLeft = hashMap.get(DcMotor.class, "mechLiftLeft");
        mechLiftRight = hashMap.get(DcMotor.class, "mechLiftRight");
        mechExt = hashMap.get(DcMotor.class, "mechExt");
        mechGrab = hashMap.get(CRServo.class, "mechGrab");
        mechStopper = hashMap.get(Servo.class, "mechStopper");
        leftDistanceSensor = hashMap.get(DistanceSensor.class, "distanceSensorLeft");
        rightDistanceSensor = hashMap.get(DistanceSensor.class, "distanceSensorRight");
        modernRoboticsI2cGyro = hashMap.get(ModernRoboticsI2cGyro .class, "gyro");
        gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;
        telemetry = tele;
        opMode = mode;

        /** Reseting motors' encoders + setting mode of operation
         *  --TeleOp                                             */

        driveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveRearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveRearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //mechLiftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mechLiftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mechLiftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //mechLiftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mechLiftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mechLiftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mechExt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mechExt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mechExt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        if(!teleOp) {

            this.setDrivetrainMode(DcMotor.RunMode.RUN_USING_ENCODER);

            mechRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mechRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            modernRoboticsI2cGyro.calibrate();

            while (modernRoboticsI2cGyro.isCalibrating())  {
                telemetry.addData("gyro", "calib");
                telemetry.update();
                sleep(50);
            }

            telemetry.clear();
            telemetry.addData("calib", "over");
            telemetry.update();

        }
        else{
            mechRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            this.setDrivetrainMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


        mechGrab.setPower(0);


        /** Setting direction of hardware components
         *  --poate unele trebuie doar la TeleOp si nu la Autonom
         *  --de retinut
         */

        //setting all motors to take negative (<0) input to move forward
        driveFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        driveFrontRight.setDirection(DcMotor.Direction.FORWARD);
        driveRearLeft.setDirection(DcMotor.Direction.REVERSE);
        driveRearRight.setDirection(DcMotor.Direction.FORWARD);

        mechRotation.setDirection(DcMotor.Direction.REVERSE);
        mechLiftLeft.setDirection(DcMotor.Direction.FORWARD);
        mechLiftRight.setDirection(DcMotor.Direction.FORWARD);
        mechExt.setDirection(DcMotor.Direction.FORWARD);

        mechStopper.setDirection(Servo.Direction.FORWARD);
        mechGrab.setDirection(DcMotorSimple.Direction.FORWARD);


    }

    /** Global methods */

        /** Braking (using a trigger) as a rescale
            Almost all movement actions
         */

        public double useBrake(double initialSpeed, double brakeFactor, boolean isCRServo){

            int speedSign = (int)(initialSpeed / abs(initialSpeed));
            double finalSpeed;
            finalSpeed = initialSpeed;

            //Clip speed for continuous servo
            if (isCRServo) {
                if(abs(initialSpeed) > MAX_CRSERVO_INPUT){
                    initialSpeed = speedSign * MAX_CRSERVO_INPUT;
                }
            }
            //Clip speed for dcmotors
            else if(!isCRServo) {
                if(abs(initialSpeed) > 1){
                    initialSpeed = speedSign * 1;
                }
            }

            /** Check that the brake factor is actually braking */
            if(brakeFactor <= 1 && brakeFactor >=0){
                finalSpeed = initialSpeed*brakeFactor;
            }
            else if(brakeFactor < 0){
                finalSpeed = 0;
            }
            else if (brakeFactor > 1){
                finalSpeed = initialSpeed;
            }

            return finalSpeed;
        }

        /** Mecanun Driving for TeleOp
        *   Any kind of movement possible
        *   --driveX & driveY ->translation (even diagonally)
        *   --turn -> rotational
        *   note: don't strafe for your own good
        *   Can be used with robot.useBrake()
         *   CW -> turn > 0
         *   */

        public void mecanumMovement(double driveX, double driveY, double turn){
            double FrontLeftPower;
            double FrontRightPower;
            double RearLeftPower;
            double RearRightPower;

            if(this.driveRearLeft.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){
                this.setDrivetrainMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            /** Clip value between -1 & 1 */
            FrontLeftPower    = Range.clip(( driveX + driveY) - turn, -1.0, 1.0) ;
            FrontRightPower   = Range.clip((-driveX + driveY) + turn, -1.0, 1.0) ;
            RearLeftPower     = Range.clip((-driveX + driveY) - turn, -1.0, 1.0) ;
            RearRightPower    = Range.clip(( driveX + driveY) + turn, -1.0, 1.0) ;

            /** Set motor speed */
            driveFrontLeft.setPower(FrontLeftPower);
            driveFrontRight.setPower(FrontRightPower);
            driveRearLeft.setPower(RearLeftPower);
            driveRearRight.setPower(RearRightPower);
        }


        public void mecanumGlobalCoordinatesDriving(double driveX, double driveY, double turn, int gyroGlobalValue){

            double initialDriveX = driveX;
            double initialDriveY = driveY;
            double theta;

            theta = (gyroGlobalValue * Math.PI) / 180;

            driveX = cos(theta) * initialDriveX + sin(theta) * initialDriveY;
            driveY = -sin(theta) * initialDriveX + cos(theta) * initialDriveY;
            this.mecanumMovement(driveX, driveY, turn);

            telemetry.clear();
            telemetry.addData("Driving mode", "Global");
            telemetry.addData("new driveX", driveX);
            telemetry.addData("new driveY", driveY);
            telemetry.update();

        }


        /** Motors used for lifting the robot */

        public void liftMovement(double liftPower, boolean nuAutonom){
            liftPower = Range.clip(liftPower, -1, 1) ;


            if(!nuAutonom) {
                //Lowering safety
                if (((mechLiftLeft.getCurrentPosition() > MAX_LIFT_POSITION - 5) && (mechLiftRight.getCurrentPosition() > MAX_LIFT_POSITION - 5)) && liftPower > 0) {
                    liftPower = 0;
                }

                //Raising safety
                if ((mechLiftLeft.getCurrentPosition() < MIN_LIFT_POSITION + 5 && mechLiftRight.getCurrentPosition() < MIN_LIFT_POSITION + 5) && liftPower < 0) {
                    liftPower = 0;
                }
            }

            mechLiftLeft.setPower(liftPower);
            mechLiftRight.setPower(liftPower);
        }

        /** "PID" for motor used for arm rotation
        *   waiting a full restructure           */

        public void rotationMovementWIP(boolean goingUp, double brakeFactor){

            double rotationSpeed = 0.3;
            int theta = mechRotation.getCurrentPosition();
            int error;
            boolean up = true;

            if(mechRotation.getMode() == DcMotor.RunMode.RUN_TO_POSITION){
                mechRotation.setPower(0);
                mechRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if(goingUp) {
                error = theta - MIN_ROTATION;
                if(mechRotation.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){
                    mechRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            }
            else{
                error = theta - MIN_ROTATION;
                rotationSpeed = -rotationSpeed;
                up = false;
            }




            if(up) {

                if(error >= 0) {
                    rotationSpeed = (rotationSpeed) * Math.sin(0.23 * (tickToRad(error)) + 0.3);
                }
                else{
                    rotationSpeed = 0.3;
                }

                if(ROTATION_LENGTH - error < 10){
                    rotationSpeed = 0;
                }
            }
            else {
                if(error > (ROTATION_LENGTH *((double)2/3))){
                    if(mechRotation.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){
                        mechRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                    rotationSpeed = rotationSpeed * ((double)1/error) * ((ROTATION_LENGTH * ((double)1/2)));
                }
                else if(error <= (ROTATION_LENGTH *((double)2/3)) && error > 10){

                    /** Daca vreti sa coboare bratul cu encoder, comentati linia chiar de sus
                     *  cu encoder - coboara mai controlat, dar sacadat
                     *  fara encoder - nu sacadeaza, dar poate se izbeste
                     *
                     *  Va puteti juca si cu puterea de aici un pic -> aia cu rotationSpeed = -0.05 */

                    mechRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rotationSpeed = -0.05;
                }
                else {
                    rotationSpeed = -0.05;
                }
            }


            rotationSpeed = this.useBrake(rotationSpeed, brakeFactor, false);




                mechRotation.setPower(rotationSpeed);
                telemetry.addData("Error", error);

        }

        double alpha = 0;
        boolean rampingDown = false;
        double beaconMotorSpeed;
        ElapsedTime rampingDownTime = new ElapsedTime();
        double timeVariable;

        public void rotationMovement(double rotationSpeed, int newMaxRotation){

            int maxRotation = MAX_ROTATION;

            if(newMaxRotation != -420){
                maxRotation = newMaxRotation;
            }

            if(rotationSpeed == 0){

                timeVariable = rampingDownTime.milliseconds();

                if(rampingDownTime.milliseconds() > RAMPING_DOWN_TIME){
                    timeVariable = RAMPING_DOWN_TIME;
                }

                if(rampingDown){
                    rotationSpeed = alpha * timeVariable + beaconMotorSpeed;
                }

                if(!rampingDown){
                    beaconMotorSpeed = mechRotation.getPower();
                    alpha = -(beaconMotorSpeed / RAMPING_DOWN_TIME);
                    rampingDownTime.reset();
                    rampingDown = true;
                }


            }


            if(mechRotation.getCurrentPosition() > maxRotation - 5 && rotationSpeed > 0){
                rotationSpeed = 0;
            }

            if(rotationSpeed == 0 && mechRotation.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
                mechRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mechRotation.setTargetPosition(mechRotation.getCurrentPosition());
                mechRotation.setPower(0.4);
            }
            else if (mechRotation.getMode() == DcMotor.RunMode.RUN_TO_POSITION && rotationSpeed == 0){
                //nimic aici
            }
            else{
                mechRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                mechRotation.setPower(rotationSpeed);
            }


        }


        //turns ticks into rads, almost
        public double tickToRad(int ticks){
            return (ticks*2*PI)/1120;
        }

        public void setDrivetrainMode(DcMotor.RunMode runMode){

            driveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveRearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveRearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            driveFrontLeft.setMode(runMode);
            driveFrontRight.setMode(runMode);
            driveRearLeft.setMode(runMode);
            driveRearRight.setMode(runMode);
        }


        //Autonom all-around method for movement of the robot
        //     --movement type : "translation", "rotation", "strafing"
        //          --"translation" : forward -> ticks > 0
        //          --"rotation" : CCW
        //          --"strafing" : right -> ticks > 0
        public void setDrivetrainPosition(int ticks, String movementType, double maxSpeed){


            if(driveRearLeft.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
                this.setDrivetrainMode(DcMotor.RunMode.RUN_TO_POSITION);
            }


            switch(movementType){
                case "translation":
                    driveFrontLeft.setTargetPosition(this.driveFrontLeft.getCurrentPosition()-ticks);
                    driveFrontRight.setTargetPosition(this.driveFrontRight.getCurrentPosition()-ticks);
                    driveRearLeft.setTargetPosition(this.driveRearLeft.getCurrentPosition()-ticks);
                    driveRearRight.setTargetPosition(this.driveRearRight.getCurrentPosition()-ticks);
                    break;
                case "rotation":
                    driveFrontLeft.setTargetPosition(this.driveFrontLeft.getCurrentPosition()+ticks);
                    driveFrontRight.setTargetPosition(this.driveFrontRight.getCurrentPosition()-ticks);
                    driveRearLeft.setTargetPosition(this.driveRearLeft.getCurrentPosition()+ticks);
                    driveRearRight.setTargetPosition(this.driveRearRight.getCurrentPosition()-ticks);
                    break;
                case "strafing":
                    driveFrontLeft.setTargetPosition(this.driveFrontLeft.getCurrentPosition() + ticks);
                    driveFrontRight.setTargetPosition(this.driveFrontRight.getCurrentPosition()- ticks);
                    driveRearLeft.setTargetPosition(this.driveRearLeft.getCurrentPosition() - ticks);
                    driveRearRight.setTargetPosition(this.driveRearRight.getCurrentPosition() + ticks);
                    break;

            }


            driveFrontLeft.setPower(maxSpeed);
            driveFrontRight.setPower(maxSpeed);
            driveRearLeft.setPower(maxSpeed);
            driveRearRight.setPower(maxSpeed);

            while(driveRearLeft.isBusy() && driveRearRight.isBusy() && driveFrontRight.isBusy() && driveFrontLeft.isBusy() && !opMode.isStopRequested()){
                //this while is waiting for the wheels to get in position
            }


        }

        //rotationType  -->relative or absolute
        public void absgyroRotation(double desiredTheta, String rotationType){
            double initialTheta, currentTheta;
            double error;
            double outSpeed;
            boolean CCW = false;
            

            this.setDrivetrainMode(DcMotor.RunMode.RUN_USING_ENCODER);

            initialTheta = modernRoboticsI2cGyro.getIntegratedZValue();

            if(rotationType == "relative"){
                desiredTheta = initialTheta + desiredTheta;
            }


            if(desiredTheta > initialTheta){
                CCW = true;
            }

            //converting to rads
            desiredTheta = (desiredTheta * Math.PI) / 180;
            initialTheta = (initialTheta * Math.PI) / 180;

            error = desiredTheta - initialTheta;

            while(abs(error) > 0.05 && !opMode.isStopRequested()){
                outSpeed = 0.8 * sin((Math.PI / (desiredTheta - initialTheta)) * error) + 0.1;

                if(CCW){
                    outSpeed = -outSpeed;
                }

                if(abs(outSpeed) > 0.8){
                    outSpeed = 0.8 * (outSpeed / abs(outSpeed));
                }

                this.mecanumMovement(0,0, outSpeed);

                currentTheta = modernRoboticsI2cGyro.getIntegratedZValue();

                //current theta to rads
                currentTheta = (currentTheta * Math.PI) / 180;

                error = desiredTheta - currentTheta;

            }

            this.mecanumMovement(0,0,0);

        }

        public int globalGyroValue(String side){
            int gyroRawZValue = modernRoboticsI2cGyro.getIntegratedZValue();
            int gyroOrientatedZValue;
            int gyroReference = 0;

            if(side == "Crater"){
                gyroReference = 45;
            }
            else if(side == "Deploy"){
                gyroReference = 135;
            }

            gyroOrientatedZValue = gyroRawZValue + gyroReference;

            gyroOrientatedZValue = gyroOrientatedZValue % 360;

            if(gyroOrientatedZValue < 0){
                gyroOrientatedZValue = gyroOrientatedZValue + 360;
            }

            return gyroOrientatedZValue;

        }

        public int VutoDegrees(int VuX){

            double degrees = -(25 / 270.0) * VuX + 25;

            return (int) degrees;
        }


        public void gyroRotationWIP(int desiredTheta, String rotationType, String side){
            int initialTheta, currentTheta;
            int error;
            double outSpeed;
            double proportionalConstant = 0.027;

            this.setDrivetrainMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if(side.equals("Crater")){
                desiredTheta = desiredTheta - 45;
            }
            else if(side.equals("Deploy")){
                desiredTheta = desiredTheta - 135;
            }

            initialTheta = mathModulo(modernRoboticsI2cGyro.getIntegratedZValue(),360);

            desiredTheta = mathModulo(desiredTheta, 360);

            //all here is done under the gyro's initial calib in a non-incremental way


            //making the initial position equal the 0 indication o gyro
            currentTheta = 0;

            if(mathModulo(desiredTheta - initialTheta, 360) <= 180){
                ///CCW
                error = mathModulo(desiredTheta - initialTheta, 360);
            }
            else{
                ///CW
                error = mathModulo(desiredTheta - initialTheta, 360) - 360;
            }

            //getting the error

            while (Math.abs(error) > 2 && !opMode.isStopRequested()) {


                currentTheta = mathModulo(modernRoboticsI2cGyro.getIntegratedZValue(), 360);

                if(mathModulo(desiredTheta - currentTheta, 360) <= 180){
                        error = mathModulo(desiredTheta - currentTheta, 360);
                        outSpeed = proportionalConstant * error;
                }
                else{
                        error = mathModulo(desiredTheta-currentTheta, 360) - 360;
                        outSpeed = proportionalConstant * error;

                }


                outSpeed = Range.clip(outSpeed, -1, 1);


                mecanumMovement(0, 0 ,-outSpeed);

            }



            this.mecanumMovement(0,0,0);

            telemetry.addLine("Rotation over");
            telemetry.update();

        }

        public int mathModulo(int numberToBeModuloed, int n){
            int result = numberToBeModuloed % n;

            if(numberToBeModuloed < 0){
                numberToBeModuloed += 360;
            }

            return  result;
            }

}
