package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Pipelines.PoleAlignment;
import org.firstinspires.ftc.teamcode.Pipelines.SleeveDetection;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Base.Component;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class OLDCAMERA implements Component {
    private final OpenCvCamera camera;
    private final Telemetry telemetry;
    private boolean isRunning = false;
    public LinearOpMode opMode;
    private SleeveDetection sleeveDetection = new SleeveDetection();
    private PoleAlignment poleAlignment = new PoleAlignment();

    public OLDCAMERA(LinearOpMode opMode, String deviceName, HardwareMap hardwareMap, Telemetry telemetry) {
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, deviceName), hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
        this.telemetry = telemetry;
        this.opMode = opMode;
    }

    @Override
    public void init() {
    }

    @Override
    public void start() {
    }

    @Override
    public void update() {
    }

    @Override
    public String getTelemetry() {
        return telemetry.toString();
    }

    public boolean getIsRunning() {
        return isRunning;
    }

    public void requestStart() {
        camera.setPipeline(new SleeveDetection());

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
                isRunning = true;
            }

            @Override
            public void onError(int errorCode) {
            }
        });
//        while (opMode.opModeInInit()) {
//            telemetry.addData("Rotation", getParkingPosition());
//            telemetry.update();
//        }
    }

    public SleeveDetection.ParkingPosition getParkingPosition() {
        if (!isRunning) {
            requestStart();
        }
        while (!getIsRunning()) {
            opMode.idle();
        }
        opMode.sleep(250);
        SleeveDetection.ParkingPosition localPosition = sleeveDetection.getPosition();
        requestStop();
        return localPosition;
    }

    public PoleAlignment.goDirection getGoDirection() {
        return getGoDirection(false);
    }

    public PoleAlignment.goDirection getGoDirection(boolean shouldStop) {
        if (!isRunning) {
            requestStart();
        }
        while (!getIsRunning()) {
            opMode.idle();
        }
        opMode.sleep(250);
        PoleAlignment.goDirection localDirection = poleAlignment.getDirection();
        if (shouldStop) {
            requestStop();
        }
        return localDirection;
    }

    public void requestStop() {
        camera.stopStreaming();
        isRunning = false;
    }

    public void goThisDirection(Mecanum mecanum, PoleAlignment.goDirection direction) {
//        if (direction == PoleAlignment.goDirection.LEFT) {
//            mecanum.strafeLeft();
//        } else if (direction == PoleAlignment.goDirection.RIGHT) {
//            mecanum.strafeRight();
//        } else {
//            mecanum.stopDriving();
//        }
        throw new UnsupportedOperationException();
    }
}
