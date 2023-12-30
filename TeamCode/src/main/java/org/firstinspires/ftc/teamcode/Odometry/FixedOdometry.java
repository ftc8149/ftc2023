package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Odometry Calculations")
public class FixedOdometry extends LinearOpMode {

    DcMotor FR, RR, FL, RL;
    final double TICKS_PER_ROTATION = 2000;
    final double TICKS_PER_INCH = TICKS_PER_ROTATION / ((48 * Math.PI) / 25.4); 
    final double LR_RADIUS = 5.6693;
    final double CENTER_TO_AUX = 4.896646;
    double RR_offset = 0;
    double FL_offset = 0;
    double FR_offset = 0;
    double xPos = 0;
    double yPos = 0;
    double angle = 0;
    double lastXPos = 0;
    double lastYPos = 0;
    double lastAngle = 0;
    double lastFL = 0;
    double lastFR = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        //resetEncoder();

        telemetry.addData("Status", "initialized");
        telemetry.update();

        waitForStart();

        //CHANGES AND TOLERANCES CANNOT EQUAL 0!!!!

        goToPosition(20,20,0,0.25,0,0.75,0.1);
        sleep(500);
        goToPosition(-20,-20,90,0.15,0.2,1,7);
    }

    /**
     * Calculates the distance to the new points
     *
     * @param changeX X change in distance relative to the robot
     * @param changeY Y change in distance relative to the robot
     * @return diagonal distance to the relative points
     */
    private double getDistance(double changeX, double changeY) {
        return Math.hypot(changeX, changeY);
    }

    /**
     * Calculates the angle between the new points
     *
     * @param changeX X change in distance relative to the robot
     * @param changeY Y change in distance relative to the robot
     * @return angle between the change in X and Y
     */
    private double getAngle(double changeX, double changeY) {

        return Math.atan2(changeX, changeY);
    }

    /**
     * Changes the position by a specified value and angle
     *
     * @param changeX change in X position, in inches (sideways)
     * @param changeY change in Y position, in inches (forward)
     * @param targetAngle direction to face when completed, in degrees (0 is no turn)
     * @param speed maximum lateral power
     * @param angularSpeed maximum turning power
     * @param positionTolerance range of error in ending position, in inches
     * @param angleTolerance range of error in the ending direction, in degrees
     */
    private void goToPosition(double changeX, double changeY, double targetAngle, double speed, double angularSpeed, double positionTolerance, double angleTolerance) {
        changeX *= TICKS_PER_INCH;
        changeY *= TICKS_PER_INCH;
        positionTolerance *= TICKS_PER_INCH;
        double horizontalOffset = 0.944882 * TICKS_PER_INCH;

        targetAngle = Math.toRadians(targetAngle);
        angleTolerance = Math.toRadians(angleTolerance);

        double angleToTarget = getAngle(changeX, changeY);
        yPos = -1 * ((getMotorPosition("FR") + getMotorPosition("FL")) / 2) - lastYPos;
        angle = ((((getMotorPosition("FL") - lastFL) / TICKS_PER_INCH) - ((getMotorPosition("FR") - lastFR) / TICKS_PER_INCH)) / (2 * LR_RADIUS)) - lastAngle;
        xPos = -1 * ((getMotorPosition("RR") - (CENTER_TO_AUX * angle))) - lastXPos; // - horizontalOffset after 4.896646*angle
        
        double xDistance = Math.abs(changeX) - Math.abs(xPos);
        double yDistance = Math.abs(changeY) - Math.abs(yPos);
        double distance = getDistance(xDistance, yDistance);


        // forward/drive = (FR(left) + FL(right))/2
        // strafe = RR(aux/horizontal) - k(distance from center to horizontal wheel)*theta
        // theta = (FL(right) - FR(left)) / 2r (r is the radius/ the distance from the left or right dead wheel to the middle


        // while (opModeIsActive() && (distance > positionTolerance || (angle < targetAngle - angleTolerance || angle > targetAngle + angleTolerance))) {
        while (opModeIsActive() && (distance > positionTolerance)) {
            //calculate the distance ratio of remaining distance

            angleToTarget = getAngle(changeX, changeY);
            yPos = -1 * ((getMotorPosition("FR") + getMotorPosition("FL")) / 2) - lastYPos;
            angle = ((((getMotorPosition("FL") - lastFL) / TICKS_PER_INCH) - ((getMotorPosition("FR") - lastFR) / TICKS_PER_INCH)) / (2 * LR_RADIUS)) - lastAngle;
            xPos = -1 * ((getMotorPosition("RR") - (CENTER_TO_AUX * angle))) - lastXPos; // - horizontalOffset after 4.896646*angle
            xDistance = Math.abs(changeX) - Math.abs(xPos);
            yDistance = Math.abs(changeY) - Math.abs(yPos);
            distance = getDistance(xDistance, yDistance);
            
            double distanceRatio = Math.abs(xDistance / yDistance);

            double angleDistance = targetAngle - angle;
            double xSpeed = speed * Math.cos(angleToTarget) * distanceRatio;
            double ySpeed = speed * Math.sin(angleToTarget) / distanceRatio;
            double turningSpeed = Math.min(angularSpeed, Math.abs(angleDistance));
            turningSpeed *= Math.min(1.0, Math.abs(angleDistance / (3 * angleTolerance)));

            if(Math.abs(angleDistance) <= angleTolerance){
                turningSpeed = 0;
            }

            double FLPower = ySpeed + turningSpeed + xSpeed;
            double FRPower = ySpeed - turningSpeed - xSpeed;
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

            FL.setPower(FLPower);
            FR.setPower(FRPower);
            RR.setPower(RRPower);
            RL.setPower(RLPower);



            yPos = -1 * ((getMotorPosition("FR") + getMotorPosition("FL")) / 2) - lastYPos;
            angle = ((((getMotorPosition("FL") - lastFL) / TICKS_PER_INCH) - ((getMotorPosition("FR") - lastFR) / TICKS_PER_INCH)) / (2 * LR_RADIUS)) - lastAngle;
            xPos = -1 * ((getMotorPosition("RR") - (CENTER_TO_AUX * angle))) - lastXPos; // - horizontalOffset after 4.896646*angle

            xDistance = Math.abs(changeX) - Math.abs(xPos);
            yDistance = Math.abs(changeY) - Math.abs(yPos);
            distance = getDistance(xDistance, yDistance);

            telemetry.addLine("*POSITIONS*");
            telemetry.addData("xPos", xPos / TICKS_PER_INCH);
            telemetry.addData("angle", angle);
            telemetry.addData("yPos", yPos / TICKS_PER_INCH);

            telemetry.addLine();
            telemetry.addLine("*SPEEDS*");
            telemetry.addData("Distance Ratio", distanceRatio);
            telemetry.addData("angleDistance", angleDistance);
            telemetry.addData("xSpeed", xSpeed);
            telemetry.addData("ySpeed", ySpeed);
            telemetry.addData("turningSpeed", turningSpeed);

            telemetry.addLine();
            telemetry.addData("FL", FLPower);
            telemetry.addData("FR", FRPower);
            telemetry.addData("RR", RRPower);
            telemetry.addData("RL", RLPower);

            telemetry.addLine();
            telemetry.addLine("*DISTANCES*");
            telemetry.addData("xDistance INCHES", xDistance / TICKS_PER_INCH);
            telemetry.addData("yDistance INCHES", yDistance / TICKS_PER_INCH);
            telemetry.addData("distance INCHES", distance / TICKS_PER_INCH);

            telemetry.addData("Loop", "Running");

            telemetry.update();
        }

        FL.setPower(0);
        FR.setPower(0);
        RR.setPower(0);
        RL.setPower(0);

        lastFL = getMotorPosition("FL");
        lastFR = getMotorPosition("FR");
        lastXPos = xPos;
        lastYPos = yPos;
        lastAngle = angle;

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


