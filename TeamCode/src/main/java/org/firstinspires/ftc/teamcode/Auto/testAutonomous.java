package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.SoftwareRobotMap;
import org.firstinspires.ftc.teamcode.StateFunctions;

import static java.lang.Thread.sleep;

@Autonomous(name="testAutonomous", group="Auto")
public class testAutonomous extends StateFunctions {

    SoftwareRobotMap robot = new SoftwareRobotMap();

    @Override
    public void init() {

        robot.init(hardwareMap);

    }

    @Override
    public void loop() {

        Orientation pos = robot.imu.getAngularOrientation();

        telemetry.addData("heading1", pos.firstAngle);
        telemetry.addData("heading2", pos.secondAngle);
        telemetry.addData("heading3", pos.thirdAngle);


        telemetry.addData("leftback", robot.leftBack.getCurrentPosition());
        telemetry.addData("rightfront", robot.rightFront.getCurrentPosition());
        telemetry.addData("leftfront", robot.leftFront.getCurrentPosition());
        telemetry.addData("rightback", robot.rightBack.getCurrentPosition());
    }
}
