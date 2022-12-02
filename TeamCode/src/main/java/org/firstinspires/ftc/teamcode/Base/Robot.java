package org.firstinspires.ftc.teamcode.Base;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public abstract class Robot {
    protected List<Component> components = new ArrayList<>();

    public void addComponents(Component... components){
        this.components.addAll(Arrays.asList(components));
    }

    protected abstract void mapHardware(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode, boolean isTeleOp);
}
