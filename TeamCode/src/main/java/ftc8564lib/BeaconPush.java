/*
 * Lockdown Framework Library
 * Copyright (c) 2015 Lockdown Team 8564 (lockdown8564.weebly.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package ftc8564lib;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;
import ftc8564opMode.LockdownAutonomous;
import hallib.HalDashboard;
import hallib.HalUtil;

public class BeaconPush {

    ColorSensor redColorSensor, blueColorSensor;
    CRServo redRack, blueRack;
    LinearOpMode opMode;
    STATE_RED state_red;
    STATE_BLUE state_blue;
    HalDashboard dashboard;
    I2cAddr newAddress = I2cAddr.create8bit(0x3e);

    static final double BUTTON_PUSHER_RETRACT_POSITION = 1;
    static final double BUTTON_PUSHER_EXTEND_POSITION = -1;
    static final double BUTTON_PUSHER_REST_POSITION = 0;
    private double endTimeRed, endTimeBlue;

    private ElapsedTime mClock = new ElapsedTime();

    private enum Color {
        RED,
        BLUE,
        OTHER
    }

    private enum STATE_RED
    {
        EXTENDING,
        RETRACTING,
        EXTENDED,
        RETRACTED
    }

    private enum STATE_BLUE
    {
        EXTENDING,
        RETRACTING,
        EXTENDED,
        RETRACTED
    }

    public BeaconPush(LinearOpMode opMode)
    {
        this.opMode = opMode;
        blueColorSensor = opMode.hardwareMap.colorSensor.get("colorSensor");
        blueColorSensor.enableLed(false);
        redColorSensor = opMode.hardwareMap.colorSensor.get("colorSensor1");
        redColorSensor.enableLed(false);
        redColorSensor.setI2cAddress(newAddress);
        blueRack = opMode.hardwareMap.crservo.get("rack");
        blueRack.setPower(0.1);
        redRack = opMode.hardwareMap.crservo.get("rack1");
        state_red = STATE_RED.RETRACTED;
        state_blue = STATE_BLUE.RETRACTED;
        dashboard = Robot.getDashboard();
        mClock.reset();
    }

    public boolean beaconColorIsAlliance(LockdownAutonomous.Alliance alliance)
    {
        return (alliance == LockdownAutonomous.Alliance.RED_ALLIANCE && getColor(alliance) == Color.RED) || (alliance == LockdownAutonomous.Alliance.BLUE_ALLIANCE && getColor(alliance) == Color.BLUE);
    }

    public void pushBeacon(boolean redAllianceRack)
    {
        setButtonPusherExtendPosition(redAllianceRack);
        setButtonPusherRetractPosition(redAllianceRack);
    }

    private void setButtonPusherExtendPosition(boolean redAllianceRack)
    {
        // Assuming pusher state is RETRACTED
        if (state_red == STATE_RED.RETRACTED && redAllianceRack)
        {
            redRack.setPower(BUTTON_PUSHER_RETRACT_POSITION);
            endTimeRed = HalUtil.getCurrentTime() + 0.73;
            changeState(STATE_RED.EXTENDING);
        } else if(state_blue == STATE_BLUE.RETRACTED && !redAllianceRack)
        {
            blueRack.setPower(BUTTON_PUSHER_EXTEND_POSITION);
            endTimeBlue = HalUtil.getCurrentTime() + 0.73;
            changeStateBlue(STATE_BLUE.EXTENDING);
        }
    }

    private void setButtonPusherRetractPosition(boolean redAllianceRack)
    {
        // Assuming pusher state is EXTENDED
        if (state_red == STATE_RED.EXTENDED && redAllianceRack)
        {
            redRack.setPower(BUTTON_PUSHER_EXTEND_POSITION);
            endTimeRed = HalUtil.getCurrentTime() + 0.73;
            changeState(STATE_RED.RETRACTING);
        } else if(state_blue == STATE_BLUE.EXTENDED && !redAllianceRack)
        {
            blueRack.setPower(BUTTON_PUSHER_RETRACT_POSITION);
            endTimeBlue = HalUtil.getCurrentTime() + 0.73;
            changeStateBlue(STATE_BLUE.RETRACTING);
        }
    }

    public void waitUntilPressed()
    {
        while((state_red == STATE_RED.EXTENDING || state_red == STATE_RED.RETRACTING) || (state_blue == STATE_BLUE.EXTENDING || state_blue == STATE_BLUE.RETRACTING))
        {
            buttonPusherTask();
        }
    }

    public void buttonPusherTask()
    {
        // If we are extending or retracting, check if the endTime to see if we are done.
        if ((state_red == STATE_RED.EXTENDING || state_red == STATE_RED.RETRACTING) && HalUtil.getCurrentTime() >= endTimeRed)
        {
            redRack.setPower(BUTTON_PUSHER_REST_POSITION);
            changeState(state_red == STATE_RED.EXTENDING ? STATE_RED.EXTENDED: STATE_RED.RETRACTED);
        }
        if ((state_blue == STATE_BLUE.EXTENDING || state_blue == STATE_BLUE.RETRACTING) && HalUtil.getCurrentTime() >= endTimeBlue)
        {
            blueRack.setPower(0.1);
            changeStateBlue(state_blue == STATE_BLUE.EXTENDING ? STATE_BLUE.EXTENDED: STATE_BLUE.RETRACTED);
        }
    }

    private Color getColor(LockdownAutonomous.Alliance alliance)
    {
        if(LockdownAutonomous.Alliance.RED_ALLIANCE == alliance)
        {
            if(redColorSensor.red() > redColorSensor.blue() && redColorSensor.red() > redColorSensor.green())
            {
                return Color.RED;
            } else if(redColorSensor.blue() > redColorSensor.red() && redColorSensor.blue() > redColorSensor.green())
            {
                return Color.BLUE;
            }
        } else if(LockdownAutonomous.Alliance.BLUE_ALLIANCE == alliance)
        {
            if(redColorSensor.red() > redColorSensor.blue() && redColorSensor.red() > redColorSensor.green())
            {
                return Color.RED;
            } else if(redColorSensor.blue() > redColorSensor.red() && redColorSensor.blue() > redColorSensor.green())
            {
                return Color.BLUE;
            }
        }
        return Color.OTHER;
    }

    private void changeState(STATE_RED newState)
    {
        state_red = newState;
    }

    private void changeStateBlue(STATE_BLUE newState)
    {
        state_blue = newState;
    }

    public void resetRacks()
    {
        redRack.setPower(BUTTON_PUSHER_REST_POSITION);
        blueRack.setPower(0.1);
    }

}
