package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Base.BaseOpMode;
import org.firstinspires.ftc.teamcode.Base.Robot;
import org.firstinspires.ftc.teamcode.Bots.PostBot;

@Autonomous
public class TestAuto extends BaseOpMode {
    private PostBot robot;

    @Override
    protected boolean setTeleOp() {
        return false;
    }


    protected Robot setRobot() {
        robot = new PostBot();
        return robot;
    }


    public void onStart() throws InterruptedException{
       robot.encoderMecanum.driveForward(10);
    }

}
