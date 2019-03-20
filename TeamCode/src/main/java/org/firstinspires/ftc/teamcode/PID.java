package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PID implements Runnable {

    boolean stop = false;
    DcMotor endActuator = null;
    double feedBackReading;
    double desiredPosition;
    double initialPosition;
    Telemetry tele;

    LinearOpMode primitiveOpMode;

    double proportionalConstant, derivativeConstant, integralConstant;

    public PID(DcMotor endActuator, double desiredPosition, double KP, double KD, double KI, LinearOpMode opMode, Telemetry tele){
        this.endActuator = endActuator;
        this.tele = tele;

        this.proportionalConstant = KP;
        this.derivativeConstant = KD;
        this.integralConstant = KI;

        this.desiredPosition = desiredPosition;
        this.primitiveOpMode = opMode;

        this.initialPosition = endActuator.getCurrentPosition();
    }

    @Override
    public void run() {
        double proportionalTerm, derivativeTerm, integralTerm;
        double integralSum = 0;

        double output;

        double currentError, previousError;

        ElapsedTime cycleTime = new ElapsedTime();

        endActuator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        currentError = desiredPosition - initialPosition;
        cycleTime.reset();

        while(!stop && !primitiveOpMode.isStopRequested()){
            previousError = currentError;
            currentError = desiredPosition - endActuator.getCurrentPosition();

            proportionalTerm = proportionalConstant * currentError;
            derivativeTerm = derivativeConstant * ((currentError - previousError));
            integralTerm = integralConstant * integralSum;

            output = proportionalTerm + derivativeTerm + integralTerm;

            output = Range.clip(output, -0.8, 0.8);

            endActuator.setPower(output);

            integralSum += currentError * cycleTime.seconds();
            cycleTime.reset();


        }

        endActuator.setPower(0);
    }
}
