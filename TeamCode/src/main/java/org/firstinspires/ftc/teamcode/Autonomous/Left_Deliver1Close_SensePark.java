package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Base.BaseOpMode;
import org.firstinspires.ftc.teamcode.Base.Robot;
import org.firstinspires.ftc.teamcode.Bots.PostBot;
import org.firstinspires.ftc.teamcode.Components.Camera;

@Autonomous
public class Left_Deliver1Close_SensePark extends BaseOpMode {
    public PostBot robot;

    Camera.ParkingPosition parkingPosition;

    @Override
    public Robot setRobot() {
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
        robot.arm.toGround();

        // move to junction pole
        robot.encoderMecanum.driveForward(5);
        robot.encoderMecanum.strafeRight(36);

        // deliver preload cone
        robot.arm.toHigh();
        robot.encoderMecanum.driveForward(30);
        robot.grabber.open();
        sleep(500);
        robot.arm.toHigh();
        robot.encoderMecanum.driveBackward(10);

        // line up for parking
        robot.encoderMecanum.turnLeft();
        robot.arm.toZero();

        // park
        if (parkingPosition == Camera.ParkingPosition.LEFT) {
            robot.encoderMecanum.driveForward(60);
        } else if (parkingPosition == Camera.ParkingPosition.CENTER) {
            robot.encoderMecanum.driveForward(36);
        } else if (parkingPosition == Camera.ParkingPosition.RIGHT) {
            robot.encoderMecanum.driveForward(12);
        }
        robot.encoderMecanum.turnLeft();
    }
}