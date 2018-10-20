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

    }
}
