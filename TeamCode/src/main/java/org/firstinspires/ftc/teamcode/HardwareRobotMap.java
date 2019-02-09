package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class  HardwareRobotMap {
    //public DcMotor leftBack = null;
    //public DcMotor rightFront = null;
    public DcMotor leftFront = null;
    public DcMotor rightBack = null;
    public DcMotor left = null;
    public DcMotor right = null;
    public Servo yee = null;
    public DcMotor lift = null;
    //public NavxMicroNavigationSensor navx = null;
    public BNO055IMU imu = null;
    public DistanceSensor sensorRange;
    public Servo ratchet = null;
    public DcMotor intakeSweeper = null;


    //HardwareMap hwMap =  null;


    /* Constructor */
    public HardwareRobotMap(){

    }

    public void init(HardwareMap ahwmap){

        //hwMap =  ahwmap;
        leftFront  = ahwmap.get(DcMotor.class, "left_Front");
        rightBack = ahwmap.get(DcMotor.class, "right_Back");
        left  = ahwmap.get(DcMotor.class, "left");
        right = ahwmap.get(DcMotor.class, "right");
        //leftBack    = ahwmap.get(DcMotor.class, "left_Back");
        //rightFront = ahwmap.get(DcMotor.class, "right_Front");
        lift = ahwmap.get(DcMotor.class, "lift_Arm");
        ratchet = ahwmap.get(Servo.class, "ratchet");
        intakeSweeper = ahwmap.get(DcMotor.class, "intake");

        //yee = ahwmap.get(Servo.class, "yee");
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.REVERSE);
        //leftBack.setDirection(DcMotor.Direction.FORWARD);
        //rightFront.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeSweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //navx = hwMap.get(NavxMicroNavigationSensor.class, "navx");
        //rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        //parameters.loggingEnabled      = true;
        //parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = ahwmap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        sensorRange = ahwmap.get(DistanceSensor.class, "sensor_range");

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;







    }
}