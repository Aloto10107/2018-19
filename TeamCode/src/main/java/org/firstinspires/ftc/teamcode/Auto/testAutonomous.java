/*
package org.firstinspires.ftc.teamcode.Auto;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.BasicFunctions;
import org.firstinspires.ftc.teamcode.SoftwareRobotMap;
import org.firstinspires.ftc.teamcode.StateFunctions;

import static java.lang.Thread.sleep;

@Autonomous(name="testAutonomous", group="Auto")
public class testAutonomous extends OpMode {

    SoftwareRobotMap robit = new SoftwareRobotMap();

    enum state {INITIAL, DETECT, ALIGN, DRIVE, FINAL}

    enum gold {RIGHT, LEFT, CENTER}

    private GoldDetector detector;

    private BasicFunctions robot;

    state robotState = state.INITIAL;

    gold goldPos;

    public DcMotor leftBack = null;
    public DcMotor rightFront = null;
    public DcMotor leftFront = null;
    public DcMotor rightBack = null;
    //public NavxMicroNavigationSensor navx = null;
    public BNO055IMU imu = null;
    double cubePush;

    @Override
    public void init() {

        leftFront  = hardwareMap.get(DcMotor.class, "left_Front");
        rightBack = hardwareMap.get(DcMotor.class, "right_Back");
        leftBack    = hardwareMap.get(DcMotor.class, "left_Back");
        rightFront = hardwareMap.get(DcMotor.class, "right_Front");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//navx = hwMap.get(NavxMicroNavigationSensor.class, "navx");
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        //parameters.loggingEnabled      = true;
        //parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        detector = new GoldDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        // Optional Tuning
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();

        telemetry.addLine("Ready");

    }

    @Override
    public void loop() {

        //Orientation pos = robot.imu.getAngularOrientation();

        telemetry.addData("position", detector.getScreenPosition().y);
        telemetry.addData("goldPos", goldPos);
        telemetry.addData("heading", getHeading());
        telemetry.update();
        switch (robotState) {

            case INITIAL: {
                robotState = state.DETECT;
            }

            case DETECT: {
                sleepy(1000);
                if (detector.getScreenPosition().y < 100) {
                    goldPos = gold.LEFT;
                    robotState = state.ALIGN;
                }
                if (detector.getScreenPosition().y > 300) {
                    goldPos = gold.RIGHT;
                    robotState = state.ALIGN;
                }
                if ((detector.getScreenPosition().y > 100) && (detector.getScreenPosition().y < 300)) {
                    goldPos = gold.CENTER;
                    robotState = state.ALIGN;
                }
            }
            case ALIGN: {
                if (goldPos == gold.CENTER) {
                    cubePush= 22;
                    robotState = state.DRIVE;
                }
                if (goldPos == gold.LEFT) {
                    cubePush= 24;
                    gyroTurn(20);
                    robotState = state.DRIVE;
                }
                if (goldPos == gold.RIGHT) {
                    cubePush= 24;
                    gyroTurn(-30);
                    robotState = state.DRIVE;
                }
            }
            case DRIVE: {
                encoderDrive(50, 50, cubePush, 10, 10);
                if (goldPos == gold.CENTER){
                    gyroTurn(-50);
                    sleepy(250);
                    gyroTurn(0);
                }
                robotState = state.FINAL;
            }
            case FINAL:{
                requestOpModeStop();
            }

        }
    }
    public void encoderDrive(double Lspeed, double Rspeed, double Inches, double timeoutS, double rampup) {

        double COUNTS_PER_MOTOR_REV = 560;    //Set for NevRest 20 drive. For 40's change to 1120. For 60's 1680
        double DRIVE_GEAR_REDUCTION = 1.0;     // This is the ratio between the motor axle and the wheel
        double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
        double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);

        ElapsedTime runtime = new ElapsedTime();

        //initialise some variables for the subroutine
        int newLeftTarget;
        int newRightTarget;
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Ensure that the opmode is still active
        // Determine new target position, and pass to motor controller we only do this in case the encoders are not totally zero'd
        newLeftTarget = (leftFront.getCurrentPosition() + leftBack.getCurrentPosition()) / 2 + (int) (Inches * COUNTS_PER_INCH);
        newRightTarget = (rightFront.getCurrentPosition() + rightBack.getCurrentPosition()) / 2 + (int) (Inches * COUNTS_PER_INCH);
        // reset the timeout time and start motion.
        runtime.reset();
        // keep looping while we are still active, and there is time left, and neither set of motors have reached the target
        while ((runtime.seconds() < timeoutS) &&
                (Math.abs(leftFront.getCurrentPosition() + leftBack.getCurrentPosition()) / 2 < newLeftTarget &&
                        Math.abs(rightFront.getCurrentPosition() + rightBack.getCurrentPosition()) / 2 < newRightTarget)) {

            double rem = (Math.abs(leftFront.getCurrentPosition()) + Math.abs(leftBack.getCurrentPosition()) + Math.abs(rightFront.getCurrentPosition()) + Math.abs(rightBack.getCurrentPosition())) / 4;
            double NLspeed;
            double NRspeed;
            //To Avoid spinning the wheels, this will "Slowly" ramp the motors up over
            //the amount of time you set for this SubRun
            double R = runtime.seconds();
            if (R < rampup) {
                double ramp = R / rampup;
                NLspeed = Lspeed * ramp;
                NRspeed = Rspeed * ramp;
            }
//Keep running until you are about two rotations out
            else if (rem > (1000)) {
                NLspeed = Lspeed;
                NRspeed = Rspeed;
            }
            //start slowing down as you get close to the target
            else if (rem > (200) && (Lspeed * .2) > .1 && (Rspeed * .2) > .1) {
                NLspeed = Lspeed * (rem / 1000);
                NRspeed = Rspeed * (rem / 1000);
            }
            //minimum speed
            else {
                NLspeed = Lspeed * .2;
                NRspeed = Rspeed * .2;

            }
            //Pass the seed values to the motors
            leftFront.setPower(NLspeed);
            leftBack.setPower(NLspeed);
            rightFront.setPower(NRspeed);
            rightBack.setPower(NRspeed);
        }
        // Stop all motion;
        //Note: This is outside our while statement, this will only activate once the time, or distance has been met
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        // show the driver how close they got to the last target
        */
/*telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
        telemetry.addData("Path2", "Running at %7d :%7d", robot.leftFront.getCurrentPosition(), robot.rightFront.getCurrentPosition());
        telemetry.update();*//*

        //setting resetC as a way to check the current encoder values easily
        double resetC = ((Math.abs(leftFront.getCurrentPosition()) + Math.abs(leftBack.getCurrentPosition()) + Math.abs(rightFront.getCurrentPosition()) + Math.abs(rightBack.getCurrentPosition())));
        //Get the motor encoder resets in motion
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //keep waiting while the reset is running
        while (Math.abs(resetC) > 0) {
            telemetry.addData("value", resetC);
            telemetry.update();
            resetC = ((Math.abs(leftFront.getCurrentPosition()) + Math.abs(leftBack.getCurrentPosition()) + Math.abs(rightFront.getCurrentPosition()) + Math.abs(rightBack.getCurrentPosition())));
            //idle();
        }
        */
/**
         //MIGHT BE CAUSING ISSUES
         *//*

        // switch the motors back to RUN_USING_ENCODER mode
        sleepy(250);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleepy(250);
    }
    public float getHeading(){
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
    public void gyroTurn (float degrees){

        //read orientation values from navx
        double speed;

        float kp = (float)0.008;

        float error = degrees - getHeading();
        //run loop to turn
        while (Math.abs(error) > 5){
            error = degrees - getHeading();
            speed = kp * error;
            leftBack.setPower(-speed);
            rightBack.setPower(speed);
            leftFront.setPower(-speed);
            rightFront.setPower(speed);
        }
        leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
        sleepy(250);
    }
    public void goldCube (float degrees){
        double speed;
        float kp = (float)0.008;
        float error = (float)200 - degrees;
        while ((Math.abs(error) > 5) && (detector.getScreenPosition().y != 0)){
            error = (float)200 - degrees;
            speed = kp * error;
            leftBack.setPower(-speed);
            rightBack.setPower(speed);
            leftFront.setPower(-speed);
            rightFront.setPower(speed);
        }
        leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
    }
    public void sleepy(int milis){
        try {
            Thread.sleep(milis);
        } catch (Exception e){}
    }

}
*/
