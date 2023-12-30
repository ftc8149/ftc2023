package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Odometry Telemetry")
public class OdometryPositionTester extends LinearOpMode {

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


        while(opModeIsActive()){
            updatePosition();
        }
        //CHANGES AND TOLERANCES CANNOT EQUAL 0!!!!

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

        return Math.atan2(changeY, changeX);
    }


    private void updatePosition() {

        //changeX *= TICKS_PER_INCH;
        //changeY *= TICKS_PER_INCH;
        //positionTolerance *= TICKS_PER_INCH;
        double horizontalOffset = 0.944882 * TICKS_PER_INCH;

        //targetAngle = Math.toRadians(targetAngle);
        //angleTolerance = Math.toRadians(angleTolerance);

       // double angleToTarget = getAngle(changeX, changeY);
       // ANGLE IS IN RADIANS
        yPos = -1 * ((getMotorPosition("FR") + getMotorPosition("FL")) / 2) - lastYPos;
        angle = ((((getMotorPosition("FL") - lastFL) / TICKS_PER_INCH) - ((getMotorPosition("FR") - lastFR) / TICKS_PER_INCH)) / (2 * LR_RADIUS)) - lastAngle;
        xPos = -1 * (((getMotorPosition("RR") - (CENTER_TO_AUX * angle))) - lastXPos); // - horizontalOffset after 4.896646*angle

        //double xDistance = Math.abs(Math.abs(changeX) - xPos);
        //double yDistance = Math.abs(Math.abs(changeY) - yPos);
        //double distance = getDistance(xDistance, yDistance);


        // forward/drive = (FR(left) + FL(right))/2
        // strafe = RR(aux/horizontal) - k(distance from center to horizontal wheel)*theta
        // theta = (FL(right) - FR(left)) / 2r (r is the radius/ the distance from the left or right dead wheel to the middle


        // while (opModeIsActive() && (distance > positionTolerance || (angle < targetAngle - angleTolerance || angle > targetAngle + angleTolerance))) {


        telemetry.addLine("*X POSITION*");
        telemetry.addData("xPos TICKS", xPos);
        telemetry.addData("xPos INCHES", xPos / TICKS_PER_INCH);

        telemetry.addLine("*ANGLES*");
        telemetry.addData("angle", angle);
        telemetry.addData("angle TODEGREES", Math.toDegrees(angle));
        telemetry.addData("angle TORADIANS", Math.toRadians(angle));

        telemetry.addLine("*Y POSITION");
        telemetry.addData("yPos TICKS", yPos);
        telemetry.addData("yPos INCHES", yPos / TICKS_PER_INCH);

        telemetry.addLine("*ENCODERS*");
        telemetry.addData("Left Wheel", getMotorPosition("FR"));
        telemetry.addData("Right Wheel", getMotorPosition("FL"));
        telemetry.addData("Aux Wheel", getMotorPosition("RR"));

        telemetry.addLine();
        telemetry.addData("Loop", "Running");

        telemetry.update();

            if(gamepad1.a){
                lastFL = getMotorPosition("FL");
                lastFR = getMotorPosition("FR");
                lastXPos = xPos;
                lastYPos = yPos;
                lastAngle = angle;
            }
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


