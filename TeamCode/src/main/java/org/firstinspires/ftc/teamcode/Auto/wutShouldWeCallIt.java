package org.firstinspires.ftc.teamcode.Auto;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.SoftwareRobotMap;


@Autonomous(name="plzwork", group="Auto")
public class wutShouldWeCallIt extends OpMode {

    private GoldAlignDetector detector;
    enum state {INITIAL, DETECT, ALIGN, ALIGN2, ALIGN3,  HIT, FINAL}
    state robotState = state.INITIAL;

    public DcMotor leftBack = null;
    public DcMotor rightFront = null;
    public DcMotor leftFront = null;
    public DcMotor rightBack = null;
    //public NavxMicroNavigationSensor navx = null;
    public BNO055IMU imu = null;

    @Override
    public void init() {
        leftFront  = hardwareMap.get(DcMotor.class, "left_Front");
        rightBack = hardwareMap.get(DcMotor.class, "right_Back");
        leftBack    = hardwareMap.get(DcMotor.class, "left_Back");
        rightFront = hardwareMap.get(DcMotor.class, "right_Front");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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


        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        detector.downscale = 0.4; // How much to downscale the input frames

        // Optional Tuning
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.001;

        detector.ratioScorer.weight = 15;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();
        telemetry.addData("Status", "Ready");


    }

    @Override
    public void loop() {
        telemetry.addData("heading", getHeading());
        telemetry.update();
        switch (robotState){
            case INITIAL: {
                drive(.5, 600);
                gyroTurn(-90);
                drive(-.5, 700);
                robotState = state.ALIGN;
            }
            case ALIGN: {
                ElapsedTime time = new ElapsedTime();
                time.reset();
                while (!detector.isFound() && time.milliseconds() < 4000) {
                    leftBack.setPower(0.2);
                    rightBack.setPower(0.2);
                    leftFront.setPower(0.2);
                    rightFront.setPower(0.2);
                }
                robotState = state.ALIGN2;
            }
            case ALIGN2: {
                ElapsedTime time = new ElapsedTime();
                time.reset();
                while (!detector.isFound() && time.milliseconds() < 4000) {
                    leftBack.setPower(0.2);
                    rightBack.setPower(0.2);
                    leftFront.setPower(0.2);
                    rightFront.setPower(0.2);
                }
                robotState = state.ALIGN3;
            }
            case ALIGN3: {
                ElapsedTime time = new ElapsedTime();
                time.reset();
                while (!detector.isFound() && time.milliseconds() < 4000) {
                    leftBack.setPower(0.2);
                    rightBack.setPower(0.2);
                    leftFront.setPower(0.2);
                    rightFront.setPower(0.2);
                }
                leftBack.setPower(0);
                rightBack.setPower(0);
                leftFront.setPower(0);
                rightFront.setPower(0);
                robotState = state.HIT;
            }
            case HIT:{
                gyroTurn(-180);
                drive(-.5, 200);
                robotState = state.FINAL;
            }
            case FINAL:{
                requestOpModeStop();
            }
        }


    }

    public void drive (double MotorPower, long time){
    leftBack.setPower(MotorPower);
    rightBack.setPower(MotorPower);
    leftFront.setPower(MotorPower);
    rightFront.setPower(MotorPower);
    sleepy(time);
    leftBack.setPower(0);
    rightBack.setPower(0);
    leftFront.setPower(0);
    rightFront.setPower(0);
    sleepy(500);
    }
    public float getHeading(){
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
    public void gyroTurn (float degrees){

        //read orientation values from navx
        double speed;

        float kp = (float)0.009;

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
        sleepy(500);
    }
    public void sleepy(long milis){
        try {
            Thread.sleep(milis);
        } catch (Exception e){}
    }


}
