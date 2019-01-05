package org.firstinspires.ftc.teamcode.Auto;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareRobotMap;

@Autonomous(name="StraightToCrater", group="Auto")
public class StraightToCrater extends LinearOpMode {
    HardwareRobotMap robot = new HardwareRobotMap();

    private GoldAlignDetector detector;

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = (50/62) ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

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

        //robot.ratchet.setPosition(0);

        telemetry.addLine("ready spaghetty");

        telemetry.update();

        waitForStart();

        robot.lift.setPower(-.3);
        sleep(9500);
        robot.lift.setPower(0);
        robot.rightBack.setPower(-.5);
        robot.leftFront.setPower(-.5);
        sleep(5000);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
    }

//    public void drive(double motorpower, double time){
//        double speed;
//        //was 0.0072
//        double kp = 0.0075;
//
//        ElapsedTime runTime = new ElapsedTime();
//
//        float error = (degrees - getHeading());
//        //run loop to turn
//        while (time > runTime.milliseconds()){
//            error = (degrees - getHeading());
//            speed = kp * error;
//            //robot.leftBack.setPower(motorpower - speed);
//            robot.rightBack.setPower(motorpower + speed);
//            robot.leftFront.setPower(motorpower - speed);
//            //robot.rightFront.setPower(motorpower + speed);
//        }
//        //robot.leftBack.setPower(0);
//        robot.rightBack.setPower(0);
//        robot.leftFront.setPower(0);
//        //robot.rightFront.setPower(0);
//        sleep(500);
//    }
}
