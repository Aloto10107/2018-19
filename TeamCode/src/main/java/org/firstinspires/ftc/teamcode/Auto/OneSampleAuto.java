package org.firstinspires.ftc.teamcode.Auto;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.HardwareRobotMap;
import org.firstinspires.ftc.teamcode.SoftwareRobotMap;

@Autonomous(name="OneSampleAUTO", group="Auto")
public class OneSampleAuto extends LinearOpMode {

    HardwareRobotMap robot = new HardwareRobotMap();

    private GoldAlignDetector detector;

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

        telemetry.addLine("ready spaghetty");

        telemetry.update();

        waitForStart();

        drive(.5, 650);
        turn(.5, 350);
        drive(-.5, 500);

        while (!detector.isFound()) {
            robot.leftBack.setPower(0.2);
            robot.rightBack.setPower(0.2);
            robot.leftFront.setPower(0.2);
            robot.rightFront.setPower(0.2);
        }
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);  //hi
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);

        turn(.5, 350);

        drive(-.5, 300);

        drive(.5, 300);

        turn(-.5, 350);

        drive(.5, 800);

        while (robot.sensorRange.getDistance(DistanceUnit.CM) >= 60){
            robot.leftBack.setPower(.5);
            robot.rightBack.setPower(.5);
            robot.leftFront.setPower(.5);
            robot.rightFront.setPower(.5);
        }

        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);

        turn(.5, 200);

        drive(.5, 1000);

        robot.yee.setPosition(1);

        drive(-.5, 3000);
    }
    public void drive (double MotorPower, long time){
        robot.leftBack.setPower(MotorPower);
        robot.rightBack.setPower(MotorPower);
        robot.leftFront.setPower(MotorPower);
        robot.rightFront.setPower(MotorPower);
        sleep(time);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        sleep(500);

    }
    public void turn(double MotorPower, long time){
        robot.leftBack.setPower(-MotorPower);
        robot.rightBack.setPower(MotorPower);
        robot.leftFront.setPower(-MotorPower);
        robot.rightFront.setPower(MotorPower);
        sleep(time);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        sleep(500);

    }
    public void encoderDrive(double power, int distance){
        int lefttarget = robot.leftBack.getCurrentPosition();
        int righttarget = robot.rightFront.getCurrentPosition();
    }
    public float getHeading(){
        Orientation angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
    public void gyroTurn (float degrees){

        //read orientation values from navx
        double speed;

        double kp = 0.00005;

        float error = (degrees - getHeading());
        //run loop to turn
        while (Math.abs(error) > 5){
            error = (degrees - getHeading());
            speed = kp * error;
            robot.leftBack.setPower(-speed);
            robot.rightBack.setPower(speed);
            robot.leftFront.setPower(-speed);
            robot.rightFront.setPower(speed);
        }
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        sleep(500);
    }
}
