package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadListener {

    public final Button a = new Button();
    public final Button b = new Button();
    public final Button x = new Button();
    public final Button y = new Button();

    public final Button du = new Button();
    public final Button dd = new Button();
    public final Button dl = new Button();
    public final Button dr = new Button();

    public final Button rb = new Button();
    public final Button lb = new Button();

    public final Button lsb = new Button();
    public final Button rsb = new Button();

    public final Button start = new Button();
    public final Button guide = new Button();
    public final Button back = new Button();

    private double plx = 0;
    private double ply = 0;
    private double prx = 0;
    private double pry = 0;

    private double plt = 0;
    private double prt = 0;

    public Runnable onJoystickMove = () -> {};
    public Runnable onRTMove = () -> {};
    public Runnable onLTMove = () -> {};

    private Button[] buttons = new Button[]{
            a,
            b,
            x,
            y,
            du,
            dd,
            dl,
            dr,
            rb,
            lb,
            lsb,
            rsb,
            start,
            guide,
            back
    };

    public void update(Gamepad gamepad) {
        a.update(gamepad.a);
        b.update(gamepad.b);
        x.update(gamepad.x);
        y.update(gamepad.y);

        du.update(gamepad.dpad_up);
        dd.update(gamepad.dpad_down);
        dl.update(gamepad.dpad_left);
        dr.update(gamepad.dpad_right);

        rb.update(gamepad.right_bumper);
        lb.update(gamepad.left_bumper);

        lsb.update(gamepad.left_stick_button);
        rsb.update(gamepad.right_stick_button);

        start.update(gamepad.start);
        back.update(gamepad.back);
        guide.update(gamepad.guide);

        if (gamepad.left_stick_x != plx ||
                gamepad.left_stick_y != ply ||
                gamepad.right_stick_x != prx ||
                gamepad.right_stick_y != pry) {

            plx = gamepad.left_stick_x;
            ply = gamepad.left_stick_y;
            prx = gamepad.right_stick_x;
            pry = gamepad.right_stick_y;

            onJoystickMove.run();
        }

        if(gamepad.left_trigger != plt){
            plt = gamepad.left_trigger;
            onLTMove.run();
        }

        if(gamepad.right_trigger != prt){
            prt = gamepad.right_trigger;
            onRTMove.run();
        }
    }

    public class Button {
        public Runnable onPress = () -> {};
        public Runnable onRelease = () -> {};
        public Runnable onHold = () -> {};
        public boolean isPressed = false;
        public boolean isReleased = false;
        public boolean isHeld = false;

        private boolean pressBuffer = false;
        private boolean releaseBuffer = false;

        private void update(boolean buttonState) {
            if (buttonState && !pressBuffer) {
                isPressed = true;
                pressBuffer = true;
                onPress.run();
            } else {
                if (!buttonState) {
                    pressBuffer = false;
                }
                isPressed = false;
            }

            if (!buttonState && !releaseBuffer) {
                isReleased = true;
                releaseBuffer = true;
                onRelease.run();
            } else {
                if (buttonState) {
                    releaseBuffer = false;
                }
                isReleased = false;
            }

            isHeld = buttonState;

            if (buttonState) {
                onHold.run();
            }
        }
    }
}