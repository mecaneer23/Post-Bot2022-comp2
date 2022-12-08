package org.firstinspires.ftc.teamcode.Autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.Base.BaseOpMode
import org.firstinspires.ftc.teamcode.Base.Robot
import org.firstinspires.ftc.teamcode.Bots.PostBot
import org.firstinspires.ftc.teamcode.Pipelines.SleeveDetection.ParkingPosition

@Autonomous
class Park : BaseOpMode() {
    val robot: PostBot = PostBot()
    var parkingPosition: ParkingPosition = ParkingPosition.DEFAULT
    override fun setRobot(): Robot {
        return robot
    }

    override fun setTeleOp(): Boolean {
        return false
    }

    override fun onInit() {
        robot.camera.requestStart()
        robot.grabber.close()
    }

    override fun onStart() {
        parkingPosition = robot.camera.parkingPosition
        robot.encoderMecanum.driveForward(4.0)
        robot.encoderMecanum.strafeLeft(4.0)
        robot.encoderMecanum.driveForward(24.0)
        if (parkingPosition == ParkingPosition.LEFT) {
            robot.encoderMecanum.strafeLeft(24.0)
        } else if (parkingPosition == ParkingPosition.RIGHT) {
            robot.encoderMecanum.strafeRight(24.0)
        }
    }
}