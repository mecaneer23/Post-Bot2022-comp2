package org.firstinspires.ftc.teamcode.Components;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.Component;

import java.util.ArrayList;

public class EncoderMecanum implements Component {
    private static final double PULSES_PER_REVOLUTION = 384.5; // 435 rpm goBilda 5202
    private static final double WHEEL_DIAMETER_IN = 3.77953; // 96 mm
    private static final double PULSES_PER_IN = PULSES_PER_REVOLUTION / (WHEEL_DIAMETER_IN * Math.PI);
    private static double DRIVE_SPEED, TURN_SPEED, STRAFE_MULTIPLIER, DELAY_BETWEEN_METHODS, TURN_CONSTANT;
    private static boolean USE_PID;
    private final LinearOpMode opMode;
    private final double kP, kI, kD;
    private final boolean isTeleOp;
    public Mecanum mecanum;
    private double updateMotorPower, updateTotalTicks;
    private Telemetry telemetry;
    private boolean isRunning = false;
    private final ArrayList<goFunction> instructionList = new ArrayList<>();
    private final ArrayList<double[]> params = new ArrayList<>();
    private int instructionPointer = 0;
    private boolean hasUpdated = false;

    public EncoderMecanum(
            LinearOpMode opMode,
            String leftFrontName,
            String rightFrontName,
            String leftBackName,
            String rightBackName,
            @NonNull HardwareMap hardwareMap,
            Telemetry telemetry,
            boolean isTeleOp,
            double driveSpeed, // 1.0 power
            double turnSpeed, // 0.5 power
            double lengthInches, // front-back axle to axle
            double widthInches, // left-right wheel center to wheel center
            double strafeMultiplier, // 1.13 units
            double delay, // 100 ms
            boolean usePID, // https://www.ctrlaltftc.com/the-pid-controller/tuning-methods-of-a-pid-controller
            double kP,
            double kI,
            double kD
    ) {
        DRIVE_SPEED = driveSpeed;
        TURN_SPEED = turnSpeed;
        STRAFE_MULTIPLIER = strafeMultiplier;
        DELAY_BETWEEN_METHODS = delay;
        USE_PID = usePID;
        TURN_CONSTANT = (Math.PI * Math.sqrt((Math.pow(lengthInches / 2.0, 2.0) + Math.pow(widthInches / 2.0, 2.0)) / 2.0)) / 90.0;

        this.mecanum = new Mecanum(
                hardwareMap,
                leftFrontName,
                rightFrontName,
                leftBackName,
                rightBackName,
                telemetry
        );

        this.telemetry = telemetry;
        this.isTeleOp = isTeleOp;
        this.opMode = opMode;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    private void addInstruction(@NonNull goFunction direction, double distanceIN, double motorPower) throws InterruptedException {
        instructionList.add(direction);
        params.add(new double[]{distanceIN, motorPower});
    }

    private void drive(@NonNull goFunction direction, double distanceIN, double motorPower) throws InterruptedException {
//        ElapsedTime timer = new ElapsedTime();
//        double proportional, integral = 0, derivative, pid, prevError = 0;
        double totalTicks = PULSES_PER_IN * distanceIN;
        resetEncoders();
        direction.run((int) totalTicks);
        if (USE_PID) {
            setRunWithoutEncoders();
        } else {
            setRunToPosition();
        }
        updateMotorPower = motorPower;
        updateTotalTicks = totalTicks;
//        while (
//                mecanum.fl.getCurrentPosition() != mecanum.fl.getTargetPosition()
//        ) {
//            if (USE_PID) {
//                proportional = totalTicks - mecanum.fl.getCurrentPosition();
//                integral += proportional * timer.seconds();
//                derivative = (proportional - prevError) / timer.seconds();
//                pid = (kP * proportional) + (kI * integral) + (kD * derivative);
//                setMotors(Math.min(pid, motorPower));
//                prevError = proportional;
//                timer.reset();
//            } else {
//                setMotors(calculateMotorPower(mecanum.fl.getCurrentPosition(), motorPower, totalTicks));
//                this.telemetry.addData("motorPower", mecanum.fl.getPower());
//                this.telemetry.update();
//            }
//        }
//        stopDriving();
//        setRunWithoutEncoders();
//        opMode.sleep((long) DELAY_BETWEEN_METHODS);
    }

    @Override
    public void init() {
        if (isTeleOp) {
            mecanum.init();
        }
    }

    @Override
    public void start() {
        mecanum.start();
    }

    @Override
    public void update() throws InterruptedException {
        if (isTeleOp) {
            mecanum.update();
            return;
        }
        if (!getIsRunning() && !hasUpdated) {
            instructionPointer++;
            hasUpdated = true;
        } else if (getIsRunning() && hasUpdated) {
            hasUpdated = false;
        }
        drive(instructionList.get(instructionPointer), params.get(instructionPointer)[0], params.get(instructionPointer)[1]);

        if (mecanum.fl.getCurrentPosition() != mecanum.fl.getTargetPosition()) {
            setMotors(calculateMotorPower(mecanum.fl.getCurrentPosition(), updateMotorPower, updateTotalTicks));
            this.telemetry.addData("motorPower", mecanum.fl.getPower());
            this.telemetry.update();
        } else {
            stopDriving();
            setRunWithoutEncoders();
            opMode.sleep((long) DELAY_BETWEEN_METHODS);
        }
    }

    public boolean getIsRunning() {
        return this.isRunning;
    }

    private double calculateMotorPower(int currentPositionTicks, double maxPower, double totalDistanceTicks) {
        double motorPower = ((-4.0 * maxPower) / Math.pow(totalDistanceTicks, 2.0)) * Math.pow(totalDistanceTicks / 4.0 - currentPositionTicks, 2.0) + maxPower;
        if (motorPower < 0.1) this.isRunning = false;
        return motorPower;
    }

    @Override
    public String getTelemetry() {
        return this.telemetry.toString();
    }

    private void setRunToPosition() {
        mecanum.fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mecanum.fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mecanum.bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mecanum.br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void resetEncoders() {
        mecanum.fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mecanum.fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mecanum.bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mecanum.br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setRunWithoutEncoders() {
        mecanum.fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mecanum.fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mecanum.bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mecanum.br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void goForward(Object distance) {
        mecanum.fl.setTargetPosition((int) distance);
        mecanum.fr.setTargetPosition((int) distance);
        mecanum.bl.setTargetPosition((int) distance);
        mecanum.br.setTargetPosition((int) distance);
    }

    private void goBackward(Object distance) {
        mecanum.fl.setTargetPosition(-(int) distance);
        mecanum.fr.setTargetPosition(-(int) distance);
        mecanum.bl.setTargetPosition(-(int) distance);
        mecanum.br.setTargetPosition(-(int) distance);
    }

    private void goLeft(Object distance) {
        mecanum.fl.setTargetPosition(-(int) distance);
        mecanum.fr.setTargetPosition((int) distance);
        mecanum.bl.setTargetPosition((int) distance);
        mecanum.br.setTargetPosition(-(int) distance);
    }

    private void goRight(Object distance) {
        mecanum.fl.setTargetPosition((int) distance);
        mecanum.fr.setTargetPosition(-(int) distance);
        mecanum.bl.setTargetPosition(-(int) distance);
        mecanum.br.setTargetPosition((int) distance);
    }

    private void goNW(Object distance) {
        mecanum.fl.setTargetPosition(0);
        mecanum.fr.setTargetPosition((int) distance);
        mecanum.bl.setTargetPosition((int) distance);
        mecanum.br.setTargetPosition(0);
    }

    private void goNE(Object distance) {
        mecanum.fl.setTargetPosition((int) distance);
        mecanum.fr.setTargetPosition(0);
        mecanum.bl.setTargetPosition(0);
        mecanum.br.setTargetPosition((int) distance);
    }

    private void goSW(Object distance) {
        mecanum.fl.setTargetPosition(-(int) distance);
        mecanum.fr.setTargetPosition(0);
        mecanum.bl.setTargetPosition(0);
        mecanum.br.setTargetPosition(-(int) distance);
    }

    private void goSE(Object distance) {
        mecanum.fl.setTargetPosition(0);
        mecanum.fr.setTargetPosition(-(int) distance);
        mecanum.bl.setTargetPosition(-(int) distance);
        mecanum.br.setTargetPosition(0);
    }

    private void goTurnLeft(Object distance) {
        mecanum.fl.setTargetPosition(-(int) distance);
        mecanum.fr.setTargetPosition((int) distance);
        mecanum.bl.setTargetPosition(-(int) distance);
        mecanum.br.setTargetPosition((int) distance);
    }

    private void goTurnRight(Object distance) {
        mecanum.fl.setTargetPosition((int) distance);
        mecanum.fr.setTargetPosition(-(int) distance);
        mecanum.bl.setTargetPosition((int) distance);
        mecanum.br.setTargetPosition(-(int) distance);
    }

    private void stopDriving() {
        mecanum.fl.setPower(0);
        mecanum.fr.setPower(0);
        mecanum.bl.setPower(0);
        mecanum.br.setPower(0);
    }

    private void setMotors(double power) {
        if (power < 0.05) {
            power = 0.05;
        }
        mecanum.fl.setPower(power);
        mecanum.fr.setPower(power);
        mecanum.bl.setPower(power);
        mecanum.br.setPower(power);
    }

    public void driveForward(double distanceIN) throws InterruptedException {
        driveForward(distanceIN, DRIVE_SPEED);
    }

    public void driveForward(double distanceIN, double motorPower) throws InterruptedException {
        addInstruction(this::goForward, distanceIN, motorPower);
    }

    public void driveBackward(double distanceIN) throws InterruptedException {
        driveBackward(distanceIN, DRIVE_SPEED);
    }

    public void driveBackward(double distanceIN, double motorPower) throws InterruptedException {
        addInstruction(this::goBackward, distanceIN, motorPower);
    }

    public void strafeLeft(double distanceIN) throws InterruptedException {
        strafeLeft(distanceIN, DRIVE_SPEED);
    }

    public void strafeLeft(double distanceIN, double motorPower) throws InterruptedException {
        addInstruction(this::goLeft, distanceIN * STRAFE_MULTIPLIER, motorPower);
    }

    public void strafeRight(double distanceIN) throws InterruptedException {
        strafeRight(distanceIN, DRIVE_SPEED);
    }

    public void strafeRight(double distanceIN, double motorPower) throws InterruptedException {
        addInstruction(this::goRight, distanceIN * STRAFE_MULTIPLIER, motorPower);
    }

    public void strafeNW(double distanceIN) throws InterruptedException {
        strafeNW(distanceIN, DRIVE_SPEED);
    }

    public void strafeNW(double distanceIN, double motorPower) throws InterruptedException {
        addInstruction(this::goNW, distanceIN, motorPower);
    }

    public void strafeNE(double distanceIN) throws InterruptedException {
        strafeNE(distanceIN, DRIVE_SPEED);
    }

    public void strafeNE(double distanceIN, double motorPower) throws InterruptedException {
        addInstruction(this::goNE, distanceIN, motorPower);
    }

    public void strafeSW(double distanceIN) throws InterruptedException {
        strafeSW(distanceIN, DRIVE_SPEED);
    }

    public void strafeSW(double distanceIN, double motorPower) throws InterruptedException {
        addInstruction(this::goSW, distanceIN, motorPower);
    }

    public void strafeSE(double distanceIN) throws InterruptedException {
        strafeSE(distanceIN, DRIVE_SPEED);
    }

    public void strafeSE(double distanceIN, double motorPower) throws InterruptedException {
        addInstruction(this::goSE, distanceIN, motorPower);
    }

    public void turnLeft() throws InterruptedException {
        turnLeft(90);
    }

    public void turnLeft(int degrees) throws InterruptedException {
        turnLeft(degrees, TURN_SPEED);
    }

    public void turnLeft(int degrees, double motorPower) throws InterruptedException {
        addInstruction(this::goTurnLeft, (int) (TURN_CONSTANT * degrees), motorPower);
    }

    public void turnRight() throws InterruptedException {
        turnRight(90);
    }

    public void turnRight(int degrees) throws InterruptedException {
        turnRight(degrees, TURN_SPEED);
    }

    public void turnRight(int degrees, double motorPower) throws InterruptedException {
        addInstruction(this::goTurnRight, (int) (TURN_CONSTANT * degrees), motorPower);
    }

    private interface goFunction {
        void run(int distanceTicks);
    }
}
