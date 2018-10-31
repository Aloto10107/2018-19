package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="teleop", group="teleop")
public class Teleop extends OpMode {

    SoftwareRobotMap robit = new SoftwareRobotMap();

    @Override
    public void init() {

        robit.init(hardwareMap);

    }

    @Override
    public void loop() {
        telemetry.addData("heading", robit.imu.getAngularOrientation().firstAngle);
        telemetry.addData("rightFront", robit.rightFront.getCurrentPosition());
        telemetry.addData("leftFront", robit.leftFront.getCurrentPosition());
        telemetry.addData("rightBack", robit.rightBack.getCurrentPosition());
        telemetry.addData("leftBack", robit.leftBack.getCurrentPosition());
        telemetry.update();
    }
}
