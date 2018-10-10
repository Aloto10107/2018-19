package org.firstinspires.ftc.teamcode.Auto;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.SoftwareRobotMap;
import org.firstinspires.ftc.teamcode.StateFunctions;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.disnodeteam.dogecv.filters.HSVColorFilter;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Thread.sleep;

@Autonomous(name="testAutonomous", group="Auto")
public class testAutonomous extends StateFunctions {

    SoftwareRobotMap robot = new SoftwareRobotMap();

    enum state{INITIAL, DETECT, ALIGN, DRIVE}

    private SamplingOrderDetector detector;

    state robotState = state.INITIAL;
    @Override
    public void init() {

        //robot.init(hardwareMap);

        detector = new SamplingOrderDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

/*        detector.downscale = 0.4; // How much to downscale the input frames

        // Optional Tuning
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.001;

        detector.ratioScorer.weight = 15;
        detector.ratioScorer.perfectRatio = 1.0;*/

        detector.enable();

    }

    @Override
    public void loop() {

        //Orientation pos = robot.imu.getAngularOrientation();

        telemetry.addData("sampleorder", detector.getLastOrder());

        switch (robotState){

            case INITIAL:
            {
                robotState = state.DETECT;
            }

            case DETECT:
            {

            }
        }

/*        telemetry.addData("heading1", pos.firstAngle);
        telemetry.addData("heading2", pos.secondAngle);
        telemetry.addData("heading3", pos.thirdAngle);*/


/*        telemetry.addData("leftback", robot.leftBack.getCurrentPosition());
        telemetry.addData("rightfront", robot.rightFront.getCurrentPosition());
        telemetry.addData("leftfront", robot.leftFront.getCurrentPosition());
        telemetry.addData("rightback", robot.rightBack.getCurrentPosition());*/
    }
}
