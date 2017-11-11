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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import hallib.HalDashboard;
import hallib.HalUtil;

public class PulleySystem {

    LinearOpMode opMode;
    DcMotor leftPulley, rightPulley;
    Servo ropeHolder;
    HalDashboard dashboard;

    private boolean slow;

    public PulleySystem(LinearOpMode opMode) {
        this.opMode = opMode;
        dashboard = Robot.getDashboard();
        leftPulley = opMode.hardwareMap.dcMotor.get("leftPulley");
        rightPulley = opMode.hardwareMap.dcMotor.get("rightPulley");
        ropeHolder = opMode.hardwareMap.servo.get("ropeHolder");
        ropeHolder.setPosition(1);
        leftPulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightPulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftPulley.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightPulley.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setSlow(boolean slowSpeed)
    {
        slow = slowSpeed;
    }

    public void manualControl(double leftPower, double rightPower)
    {
        if(slow)
        {
            leftPower = scalePowerSlow(leftPower);
            rightPower = scalePowerSlow(rightPower);
        } else {
            leftPower = scalePower(leftPower);
            rightPower = scalePower(rightPower);
        }
        leftPulley.setPower(leftPower);
        rightPulley.setPower(rightPower);
    }

    private double scalePower(double dVal)
    {
        return -(Math.signum(dVal) * ((Math.pow(dVal, 2) * (.9)) + .1));
    }

    private double scalePowerSlow(double dVal)
    {
        return -(Math.signum(dVal) * ((Math.pow(dVal, 2) * (.25)) + .1));
    }

    public void openForkLift()
    {
        ropeHolder.setPosition(0);
    }

    public void resetMotors()
    {
        leftPulley.setPower(0);
        rightPulley.setPower(0);
    }

    /*public void setSyncMotorPower(double power)
    {
        if (power != 0.0)
        {
            int targetPosition = power < 0.0? MIN_DISTANCE: MAX_DISTANCE;
            boolean leftOnTarget = Math.abs(targetPosition - leftPulley.getCurrentPosition()) <= LIFT_POSITION_TOLERANCE;
            boolean rightOnTarget = Math.abs(targetPosition - rightPulley.getCurrentPosition()) <= LIFT_POSITION_TOLERANCE;
            if (!leftOnTarget || !rightOnTarget)
            {
                double differentialPower = Range.clip((rightPulley.getCurrentPosition() - leftPulley.getCurrentPosition())*LIFT_SYNC_KP, -1.0, 1.0);
                double leftPower = power + differentialPower;
                double rightPower = power - differentialPower;
                double minPower = Math.min(leftPower, rightPower);
                double maxPower = Math.max(leftPower, rightPower);
                double scale = maxPower > 1.0? 1.0/maxPower: minPower < -1.0? -1.0/minPower: 1.0;
                leftPulley.setPower(leftPower*scale);
                rightPulley.setPower(rightPower*scale);
            }
            else
            {
                leftPulley.setPower(0.0);
                rightPulley.setPower(0.0);
            }
        }
        else
        {
            leftPulley.setPower(0.0);
            rightPulley.setPower(0.0);
        }
    }*/
}
