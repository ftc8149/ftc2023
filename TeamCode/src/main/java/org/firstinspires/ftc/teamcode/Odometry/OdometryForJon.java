package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@Autonomous(name="Odometry Test")
public class OdometryForJon extends LinearOpMode {

    DcMotor FR, FL, RR, RL;
    final double TICKS_PER_ROTATION = 2000;
    final double TICKS_PER_INCH = TICKS_PER_ROTATION * ((48 * Math.PI)/25.4);
    final double LR_RADIUS = 5.6693;
    final double CENTER_TO_AUX = 4.896646;

    double RR_offset = 0;
    double FL_offset = 0;
    double FR_offset = 0;
    double xPos = 0;
    double yPos = 0;
    double heading = 0;

    @Override
    public void runOpMode() throws InterruptedException{

        initialize();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        goToPosition(20,20,0,0.25,0.1,.5);
        sleep(500);
        goToPosition(0,0,0, 0.25, 0.1, 0.5);

    }

    private void goToPosition(double targetX, double targetY, double targetHeading, double power, double angularPower, double tolerance){

       targetX = targetX * TICKS_PER_INCH;
       targetY = targetY * TICKS_PER_INCH;
       tolerance = tolerance * TICKS_PER_INCH;

       targetHeading = Math.toRadians(targetHeading);

       double xDistance = Math.abs(targetX) - Math.abs(xPos);
       double yDistance = Math.abs(targetY) - Math.abs(yPos);
       double distanceToTarget = Math.hypot(xDistance, yDistance);

       double headingToTarget = Math.atan2(yDistance, xDistance);
       double headingDifference = (headingToTarget - heading + Math.PI) % (2 * Math.PI) - Math.PI;

        // forward/drive = ((left) + (right)) / 2
        // strafe = (aux/horizontal) - k(distance from center to horizontal wheel)*theta
        // theta = ((right) - (left)) / 2r (r is the radius or the distance from the left or right dead wheel to the middle

       heading = (getMotorPosition("FL") - getMotorPosition("FR")) / (2 * LR_RADIUS);
       xPos = (getMotorPosition("RR") - (CENTER_TO_AUX * heading));
       yPos = ((getMotorPosition("FR") + getMotorPosition("FL")) / 2);

       xDistance = targetX - xPos;
       yDistance = targetY - yPos;
       distanceToTarget = Math.hypot(xDistance, yDistance);

       while(opModeIsActive() && distanceToTarget > tolerance) {

           double distanceRatio = Math.abs(xDistance / yDistance);

           heading = (getMotorPosition("FL") - getMotorPosition("FR")) / (2 * LR_RADIUS);
           yPos = -1 * (((getMotorPosition("FR") + getMotorPosition("FL")) / 2));
           xPos = -1 * ((getMotorPosition("RR") - (CENTER_TO_AUX * heading)));


           xDistance = Math.abs(targetX) - Math.abs(xPos);
           yDistance = Math.abs(targetY) - Math.abs(yPos);
           distanceToTarget = Math.hypot(xDistance, yDistance);

           headingToTarget = Math.atan2(yDistance, xDistance);
           headingDifference = (headingToTarget - heading + Math.PI) % (2 * Math.PI) - Math.PI;

           distanceRatio = Math.abs(xDistance / yDistance);

           double xSpeed = power * Math.cos(headingDifference) * distanceRatio;
           double ySpeed = power * Math.sin(headingDifference) * distanceRatio;
           double turningSpeed = angularPower * headingDifference;
           //double turningSpeed = Math.signum(headingDifference) * Math.min(angularPower, Math.abs(headingDifference));

           turningSpeed *= Math.min(1.0, Math.abs(headingDifference / (3 * tolerance)));

           double FRPower = ySpeed + turningSpeed + xSpeed;
           double FLPower = ySpeed - turningSpeed - xSpeed;
           double RRPower = ySpeed - turningSpeed + xSpeed;
           double RLPower = ySpeed + turningSpeed - xSpeed;

           double frontMax = Math.max(Math.abs(FLPower), Math.abs(FRPower));
           double backMax = Math.max(Math.abs(RLPower), Math.abs(RRPower));
           double max = Math.max(frontMax, backMax);


           if (max > 1.0) {
               FLPower /= max;
               FRPower /= max;
               RLPower /= max;
               RRPower /= max;
           }

           FR.setPower(FRPower);
           FL.setPower(FLPower);
           RR.setPower(RRPower);
           RL.setPower(RLPower);

           //updateOdometry();

           yPos = -1 * ((getMotorPosition("FR") + getMotorPosition("FL")) / 2);
           heading = (getMotorPosition("FL") - getMotorPosition("FR")) / (2 * 5.6693);
           xPos = -1 * (getMotorPosition("RR") - (4.896646 * heading));

           xDistance = Math.abs(targetX) - Math.abs(xPos);
           yDistance = Math.abs(targetY) - Math.abs(yPos);
           distanceToTarget = Math.hypot(xDistance, yDistance);

           telemetry.addData("xPos", xPos);
           telemetry.addData("heading", heading);
           telemetry.addData("yPos", yPos);

           telemetry.addData("FL", FLPower);
           telemetry.addData("FR", FRPower);
           telemetry.addData("RR", RRPower);
           telemetry.addData("RL", RLPower);

           telemetry.addData("xDistance (Ticks)", xDistance / TICKS_PER_INCH);
           telemetry.addData("yDistance (Ticks)", yDistance / TICKS_PER_INCH);
           telemetry.addData("distance (Ticks)", distanceToTarget / TICKS_PER_INCH);

           telemetry.addData("xDistance (IN)", xDistance);
           telemetry.addData("yDistance (IN)", yDistance);
           telemetry.addData("distanceToTarget (IN)", distanceToTarget);

           telemetry.update();
       }

       FR.setPower(0);
       FL.setPower(0);
       RR.setPower(0);
       RL.setPower(0);

    }




    private void initialize() {
        FR = hardwareMap.dcMotor.get("FR");
        RR = hardwareMap.dcMotor.get("RR");
        FL = hardwareMap.dcMotor.get("FL");
        RL = hardwareMap.dcMotor.get("RL");

        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

    private double lastEncoderFR = 0.0;
    private double lastEncoderFL = 0.0;
    private double lastEncoderRR = 0.0;
    private void updateOdometry() {
        double currentLeftEncoder = FR.getCurrentPosition();
        double currentRightEncoder = FR.getCurrentPosition();
        double currentAuxEncoder = FR.getCurrentPosition();
        //Calculates the current encoder value

        double deltaLeftEncoder = currentLeftEncoder - lastEncoderFR;
        double deltaRightEncoder = currentRightEncoder - lastEncoderFL;
        double deltaAuxEncoder = currentAuxEncoder - lastEncoderRR;
        //calculates the current change in encoder values

        lastEncoderFR = currentLeftEncoder;
        lastEncoderFL = currentRightEncoder;
        lastEncoderRR = currentAuxEncoder;
        //makes the previous encoder value the new lastEncoder value

        double deltaInchesX = ((deltaLeftEncoder + deltaRightEncoder) / 2.0) / TICKS_PER_INCH;
        double deltaInchesY = (deltaAuxEncoder / TICKS_PER_INCH);
        //calculates change in inches

        xPos += deltaInchesX * Math.cos(heading) - deltaInchesY * Math.sin(heading);
        yPos += deltaInchesX * Math.sin(heading) + deltaInchesY * Math.cos(heading);

        double deltaHeading = (deltaLeftEncoder - deltaRightEncoder) / (TICKS_PER_INCH * LR_RADIUS);
        heading += deltaHeading;

        heading = (heading + 2 * Math.PI) % (2 * Math.PI);
    }
}
