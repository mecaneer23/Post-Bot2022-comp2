package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Base.BaseOpMode;
import org.firstinspires.ftc.teamcode.Base.Robot;
import org.firstinspires.ftc.teamcode.Bots.PostBot;
import org.firstinspires.ftc.teamcode.Pipelines.SleeveDetection;

@Autonomous
public class Park extends BaseOpMode {
    public PostBot robot;

    SleeveDetection.ParkingPosition parkingPosition;

    @Override
    protected Robot setRobot() {
        this.robot = new PostBot();
        return this.robot;
    }

    @Override
    protected boolean setTeleOp() {
        return false;
    }

    @Override
    public void onInit() throws InterruptedException {
        robot.camera.requestStart();
        robot.grabber.close();
    }

    @Override
    public void onStart() throws InterruptedException {
        parkingPosition = robot.camera.getParkingPosition();
        robot.encoderMecanum.driveForward(5);
//        maybe strafe slightly depending on camera position

        robot.encoderMecanum.driveForward(24);
        if (parkingPosition == SleeveDetection.ParkingPosition.LEFT) {
            robot.encoderMecanum.strafeLeft(24);
        } else if (parkingPosition == SleeveDetection.ParkingPosition.RIGHT) {
            robot.encoderMecanum.strafeRight(24);
        }
    }
}