package org.firstinspires.ftc.teamcode.Base;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Utils.GamepadListener;

public abstract class BaseOpMode extends LinearOpMode {
    private Robot robot;
    private boolean isTeleOp;

    protected GamepadListener gamepadListener1 = new GamepadListener();
    protected GamepadListener gamepadListener2 = new GamepadListener();


    @Override
    public void runOpMode() throws InterruptedException {
        robot = setRobot();
        isTeleOp = setTeleOp();

        try {
            robot.mapHardware(hardwareMap, telemetry, this, isTeleOp);
            robot.components.forEach(Component::init);
        }catch (NullPointerException e){
            throw new NullPointerException("robot is null or component is null");
        }

        onInit();

        waitForStart();

        onStart();
        robot.components.forEach(Component::start);

        while(opModeIsActive()){
            gamepadListener1.update(gamepad1);
            gamepadListener2.update(gamepad2);
            onUpdate();
            robot.components.forEach(Component::update);
        }
    }

    protected double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    protected abstract Robot setRobot();
    protected abstract boolean setTeleOp();

    public void onInit() throws InterruptedException {}
    public void onStart() throws InterruptedException {}
    public void onUpdate() throws InterruptedException {}

}
