package org.firstinspires.ftc.teamcode.RavenRobotics;

import com.qualcomm.robotcore.hardware.Gamepad;

public class WaverlyGamepad {

    private final Gamepad gp;

    // Latest button press state:
    //  - x = button is currently down.
    //  - xPressed = button was just pressed down for the first time.
    //  - xWasPressed = button was pressed down the last time the loop ran -
    //                  if it is still down, don't count it as a new press
    //  - xToggled = bool that flips when xPressed

    public boolean a = false;
    public boolean aPressed = false;
    private boolean aWasPressed = false;
    public boolean aToggled = false;

    public boolean b = false;
    public boolean bPressed = false;
    private boolean bWasPressed = false;
    public boolean bToggled = false;

    public boolean x = false;
    public boolean xPressed = false;
    private boolean xWasPressed = false;
    public boolean xToggled = false;

    public boolean y = false;
    public boolean yPressed = false;
    private boolean yWasPressed = false;
    public boolean yToggled = false;

    public boolean dpadUp = false;
    public boolean dpadUpPressed = false;
    private boolean dpadUpWasPressed = false;
    public boolean dpadUpToggled = false;

    public boolean dpadDown = false;
    public boolean dpadDownPressed = false;
    private boolean dpadDownWasPressed = false;
    public boolean dpadDownToggled = false;

    public boolean dpadLeft = false;
    public boolean dpadLeftPressed = false;
    private boolean dpadLeftWasPressed = false;
    public boolean dpadLeftToggled = false;

    public boolean dpadRight = false;
    public boolean dpadRightPressed = false;
    private boolean dpadRightWasPressed = false;
    public boolean dpadRightToggled = false;

    public boolean rightBumper = false;
    public boolean rightBumperPressed = false;
    private boolean rightBumperWasPressed = false;
    public boolean rightBumperToggled = false;

    public boolean leftBumper = false;
    public boolean leftBumperPressed = false;
    private boolean leftBumperWasPressed = false;
    public boolean leftBumperToggled = false;

    public boolean rightTrigger = false;
    public boolean rightTriggerPressed = false;
    private boolean  rightTriggerWasPressed = false;
    public boolean rightTriggerToggled = false;

    public boolean leftTrigger = false;
    public boolean leftTriggerPressed = false;
    private boolean  leftTriggerWasPressed = false;
    public boolean leftTriggerToggled = false;

    public WaverlyGamepad(Gamepad gamepad) {
        gp = gamepad;
    }

    public void readButtons() {
        a = gp.a;
        aPressed = a && !aWasPressed;
        aWasPressed = a;
        if (aPressed) aToggled = !aToggled;

        b = gp.b;
        bPressed = b && !bWasPressed;
        bWasPressed = b;
        if (bPressed) bToggled = !bToggled;

        x = gp.x;
        xPressed = x && !xWasPressed;
        xWasPressed = x;
        if (xPressed) xToggled = !xToggled;

        y = gp.y;
        yPressed = y && !yWasPressed;
        yWasPressed = y;
        if (yPressed) yToggled = !yToggled;

        dpadUp = gp.dpad_up;
        dpadUpPressed = dpadUp && !dpadUpWasPressed;
        dpadUpWasPressed = dpadUp;
        if (dpadUpPressed) dpadUpToggled = !dpadUpToggled;

        dpadDown = gp.dpad_down;
        dpadDownPressed = dpadDown && !dpadDownWasPressed;
        dpadDownWasPressed = dpadDown;
        if (dpadDownPressed) dpadDownToggled = !dpadDownToggled;

        dpadLeft = gp.dpad_left;
        dpadLeftPressed = dpadLeft && !dpadLeftWasPressed;
        dpadLeftWasPressed = dpadLeft;
        if (dpadLeftPressed) dpadLeftToggled = !dpadLeftToggled;

        dpadRight = gp.dpad_right;
        dpadRightPressed = dpadRight && !dpadRightWasPressed;
        dpadRightWasPressed = dpadRight;
        if (dpadRightPressed) dpadRightToggled = !dpadRightToggled;

        leftBumper = gp.left_bumper;
        leftBumperPressed = leftBumper && !leftBumperWasPressed;
        leftBumperWasPressed = leftBumper;
        if (leftBumperPressed) leftBumperToggled = !leftBumperToggled;

        rightBumper = gp.right_bumper;
        rightBumperPressed = rightBumper && !rightBumperWasPressed;
        rightBumperWasPressed = rightBumper;
        if (rightBumperPressed) rightBumperToggled = !rightBumperToggled;

        leftTrigger = gp.left_trigger != 0;
        leftTriggerPressed = leftTrigger && !leftTriggerWasPressed;
        leftTriggerWasPressed = leftTrigger;
        if (leftTriggerPressed) leftTriggerToggled = !leftTriggerToggled;

        rightTrigger = gp.right_trigger != 0;
        rightTriggerPressed = rightTrigger && !rightTriggerWasPressed;
        rightTriggerWasPressed = rightTrigger;
        if (rightTriggerPressed) rightTriggerToggled = !rightTriggerToggled;
    }

}