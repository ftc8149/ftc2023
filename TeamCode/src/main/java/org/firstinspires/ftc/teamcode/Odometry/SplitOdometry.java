package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.checkerframework.checker.initialization.qual.Initialized;

@Disabled
@Autonomous(name="Split Odometry")
public class SplitOdometry extends LinearOpMode {

    DcMotor FR, RR, FL, RL;
    final double TICKS_PER_ROTATION = 2000; //CHANGE THIS IF NEEDED
    //GoBilda Odometry Pod used (2000 ticks per rev, 48mm wheel diameter
    final double TICKS_PER_INCH = TICKS_PER_ROTATION / ((48 * Math.PI) / 25.4); //CHANGE THIS IF NEEDED
    //TICKS_PER_ROTATION / (omni-wheel diameter * Math.PI) / 25.4
    //This finds your wheel circumference, and turns it into inches
    final double LR_RADIUS = 5.6693;
    //5.6693 = radius of left and right dead wheel
    //MAKE SURE THESE DEAD WHEELS ARE IN LINE WITH EACH OTHER!!
    final double CENTER_TO_AUX = 4.896646;
    //4.896646 = distance from center of robot and the horizontal dead wheel
    //MAKE SURE THE HORIZONTAL ONE IS CENTERED!!

    //Create coordinate system, we want our starting position to be 0,0,0
    double RR_offset = 0;
    double FL_offset = 0;
    double FR_offset = 0;

    @Override
    public void runOpMode() throws InterruptedException{

        initialize();
        telemetry.addData("Status", "Initialized");
        telemetry.addLine();
        telemetry.addLine("Waiting for Start");
        telemetry.update();

        waitForStart();

        moveToPosition(30, 0, 0.5, 0.25);
        //CHANGE THE ALL GOTOPOSITIONS!!!!
        sleep(500);
        turnToPosition(90,0.2, 1);


    }

    private void moveToPosition(double changeX, double changeY, double speed, double positionTolerance){
        changeX *= TICKS_PER_INCH;
        changeY *= TICKS_PER_INCH;
        positionTolerance *= TICKS_PER_INCH;

        double yPos = ((getMotorPosition("FR") + getMotorPosition("FL")) / 2);
        double angle = (getMotorPosition("FL") - getMotorPosition("FR")) / (2 * LR_RADIUS);
        double xPos = ((getMotorPosition("RR") - (CENTER_TO_AUX * angle)));

        double xDistance = Math.abs(changeX - xPos);
        double yDistance = Math.abs(changeY - yPos);
        double distance = Math.hypot(xDistance, yDistance);

        while (opModeIsActive() && (distance > positionTolerance)){
            telemetry.addData("xDistance (TICKS)", xDistance / TICKS_PER_INCH);
            telemetry.addData("yDistance (TICKS)", yDistance / TICKS_PER_INCH);
            telemetry.addData("distance (TICKS)", distance / TICKS_PER_INCH);
            telemetry.addLine();
            telemetry.addData("xDistance (INCHES)", xDistance);
            telemetry.addData("yDistance (INCHES)", yDistance);
            telemetry.addData("distance (INCHES)", distance);

            double distanceRatio = Math.abs(xDistance / yDistance);

            yPos = (((getMotorPosition("FR") + getMotorPosition("FL")) / 2));
            angle = (getMotorPosition("FL") - getMotorPosition("FR")) / (2 * LR_RADIUS);
            xPos = ((getMotorPosition("RR") - (CENTER_TO_AUX * angle)));
            xDistance = Math.abs(changeX) - Math.abs(xPos);
            yDistance = Math.abs(changeY) - Math.abs(yPos);
            distance = Math.hypot(xDistance, yDistance);
            distanceRatio = Math.abs(xDistance / yDistance);

            double xSpeed = speed * distanceRatio;
            double ySpeed = speed * distanceRatio;

            double FLPower = ySpeed + xSpeed;
            double FRPower = ySpeed - xSpeed;
            double RRPower = ySpeed + xSpeed;
            double RLPower = ySpeed - xSpeed;

            double frontMax = Math.max(Math.abs(FLPower), Math.abs(FRPower));
            double backMax = Math.max(Math.abs(RLPower), Math.abs(RRPower));
            double max = Math.max(frontMax, backMax);

            //ensures that no motor can go past a value of 1
            if (max > 1.0) {
                FLPower /= max;
                FRPower /= max;
                RLPower /= max;
                RRPower /= max;
            }

            //Actually set power to the motors
            FL.setPower(FLPower);
            FR.setPower(FRPower);
            RR.setPower(RRPower);
            RL.setPower(RLPower);

            telemetry.addData("FL", FLPower);
            telemetry.addData("FR", FRPower);
            telemetry.addData("RR", RRPower);
            telemetry.addData("RL", RLPower);

            yPos = -1 * ((getMotorPosition("FR") + getMotorPosition("FL")) / 2);
            angle = (getMotorPosition("FL") - getMotorPosition("FR")) / (2 * LR_RADIUS);
            xPos = -1 * (getMotorPosition("RR") - (CENTER_TO_AUX * angle));

            telemetry.addData("xPos", xPos);
            telemetry.addData("angle", angle);
            telemetry.addData("yPos", yPos);

            //Recalculate our distance left
            xDistance = Math.abs(changeX) - Math.abs(xPos);
            yDistance = Math.abs(changeY) - Math.abs(yPos);
            distance = Math.hypot(xDistance, yDistance);

            telemetry.update();
        }
        FL.setPower(0);
        FR.setPower(0);
        RR.setPower(0);
        RL.setPower(0);
    }

    private void turnToPosition(double targetAngle, double angularSpeed, double angleTolerance){
        targetAngle = Math.toRadians(targetAngle);
        angleTolerance = Math.toRadians(angleTolerance);

        double angle = (getMotorPosition("FL") - getMotorPosition("FR")) / (2 * LR_RADIUS);

        double angleDifference = targetAngle - angle;

        while (opModeIsActive() && (angle < targetAngle - angleTolerance || angle > targetAngle + angleTolerance)) {
            telemetry.addData("Current Heading", Math.toDegrees(angle));
            telemetry.addData("Target Heading", Math.toDegrees(targetAngle));
            telemetry.addData("Distance Left", Math.toDegrees(angleDifference));

            angle = (getMotorPosition("FL") - getMotorPosition("FR")) / (2 * LR_RADIUS);
            angleDifference = targetAngle - angle;

            double turningSpeed = Math.signum(angleDifference) * Math.min(angularSpeed, Math.abs(angleDifference));
            //Math.signum takes the sign of angleDistance, if it is positive, the output is +1, if it is negative, the output is -1
            turningSpeed *= Math.min(1.0, Math.abs(angleDifference / (3 * angleTolerance)));

            double FLPower = turningSpeed;
            double FRPower = -turningSpeed;
            double RRPower = -turningSpeed;
            double RLPower = turningSpeed;

            double frontMax = Math.max(Math.abs(FLPower), Math.abs(FRPower));
            double backMax = Math.max(Math.abs(RLPower), Math.abs(RRPower));
            double max = Math.max(frontMax, backMax);

            if (max > 1.0) {
                FLPower /= max;
                FRPower /= max;
                RLPower /= max;
                RRPower /= max;
            }

            FL.setPower(FLPower);
            FR.setPower(FRPower);
            RR.setPower(RRPower);
            RL.setPower(RLPower);

            telemetry.addData("FL", FLPower);
            telemetry.addData("FR", FRPower);
            telemetry.addData("RR", RRPower);
            telemetry.addData("RL", RLPower);

            angle = (getMotorPosition("FL") - getMotorPosition("FR")) / (2 * LR_RADIUS);
            angleDifference = targetAngle - angle;

            telemetry.addData("Current Heading", Math.toDegrees(angle));
            telemetry.addData("Target Heading", Math.toDegrees(targetAngle));
            telemetry.addData("Distance Left", Math.toDegrees(angleDifference));

            telemetry.update();


        }

        FL.setPower(0);
        FR.setPower(0);
        RR.setPower(0);
        RL.setPower(0);

    }

    private void initialize() {
        FR = hardwareMap.dcMotor.get("FR");
        RR = hardwareMap.dcMotor.get("RR");
        FL = hardwareMap.dcMotor.get("FL");
        RL = hardwareMap.dcMotor.get("RL");

        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // No dead wheel

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FL.setDirection(DcMotor.Direction.FORWARD);
        RL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);
        RR.setDirection(DcMotor.Direction.REVERSE);

        FR_offset = FR.getCurrentPosition();
        RR_offset = RR.getCurrentPosition();
        FL_offset = FL.getCurrentPosition();
    }

    private double getMotorPosition(String motorName) {
        switch (motorName) {
            case "FR":
                return FR.getCurrentPosition() - FR_offset;
            case "RR":
                return RR.getCurrentPosition() - RR_offset;
            case "FL":
                return FL.getCurrentPosition() - FL_offset;
            case "RL":
                return RL.getCurrentPosition();
            default:
                return 0;
        }
    }
}

