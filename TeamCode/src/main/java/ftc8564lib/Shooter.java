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
import com.qualcomm.robotcore.util.ElapsedTime;

import hallib.HalDashboard;
import hallib.HalUtil;

public class Shooter {

    LinearOpMode opMode;
    DcMotor tennisArm;
    DcMotor highSpeed;
    ElapsedTime mClock = new ElapsedTime();
    HalDashboard dashboard;
    State shooter;
    State_Autonomous autoShooter;

    private double shootTime;

    private enum State_Autonomous {
        HOME,
        READY,
        FIRED,
        MOVING_HOME,
        READYING,
        FIRING
    }

    private enum State {
        HOME,
        LOADED,
        READY,
        FIRED,
        MOVING_HOME,
        LOADING,
        READYING,
        FIRING
    }

    public Shooter(LinearOpMode opMode) {
        this.opMode = opMode;
        tennisArm = opMode.hardwareMap.dcMotor.get("tennisArm");
        tennisArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tennisArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        highSpeed = opMode.hardwareMap.dcMotor.get("highSpeed");
        highSpeed.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        highSpeed.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        highSpeed.setTargetPosition(0);
        highSpeed.setPower(0.5);
        shooter = State.HOME;
        autoShooter = State_Autonomous.HOME;
        dashboard = Robot.getDashboard();
        mClock.reset();
    }

    public void waitForShootAuto()
    {
        while(autoShooter == State_Autonomous.READYING || autoShooter == State_Autonomous.FIRING || autoShooter == State_Autonomous.MOVING_HOME)
        {
            autoShooterTask();
        }
    }

    public void shootSequenceAuto(boolean redSide)
    {
        moveCenterAuto(redSide);
        setBallAuto(redSide);
        shootBallAuto(redSide);
    }

    private void moveCenterAuto(boolean redSide)
    {
        if(autoShooter == State_Autonomous.FIRED)
        {
            if(redSide)
            {
                highSpeed.setTargetPosition(-1680);
            } else {
                highSpeed.setTargetPosition(1680);
            }
            highSpeed.setPower(1);
            shootTime = HalUtil.getCurrentTime() + 0.75;
            changeStateAuto(State_Autonomous.MOVING_HOME);
        }
    }

    private void setBallAuto(boolean redSide)
    {
        if(autoShooter == State_Autonomous.HOME)
        {
            highSpeed.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            highSpeed.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(redSide)
            {
                highSpeed.setTargetPosition(-350);
            } else {
                highSpeed.setTargetPosition(350);
            }
            shootTime = HalUtil.getCurrentTime() + 0.5;
            highSpeed.setPower(0.3);
            changeStateAuto(State_Autonomous.READYING);
        }
    }

    private void shootBallAuto(boolean redSide)
    {
        if(autoShooter == State_Autonomous.READY)
        {
            if(redSide)
            {
                highSpeed.setTargetPosition(-700);
            } else {
                highSpeed.setTargetPosition(700);
            }
            shootTime = HalUtil.getCurrentTime() + 0.5;
            highSpeed.setPower(1);
            changeStateAuto(State_Autonomous.FIRING);
        }
    }

    private void autoShooterTask()
    {
        if(autoShooter == State_Autonomous.FIRING && HalUtil.getCurrentTime() >= shootTime)
        {
            changeStateAuto(State_Autonomous.FIRED);
        } else if(autoShooter == State_Autonomous.READYING && HalUtil.getCurrentTime() >= shootTime)
        {
            changeStateAuto(State_Autonomous.READY);
        } else if(autoShooter == State_Autonomous.MOVING_HOME && HalUtil.getCurrentTime() >= shootTime)
        {
            changeStateAuto(State_Autonomous.HOME);
        }
    }

    public void shootSequence(boolean redSide)
    {
        moveCenter(redSide);
        loadBall(redSide);
        setBall(redSide);
        shootBall(redSide);
    }

    private void moveCenter(boolean redSide)
    {
        if(shooter == State.FIRED)
        {
            if(redSide)
            {
                highSpeed.setTargetPosition(-1680);
            } else {
                highSpeed.setTargetPosition(1680);
            }
            highSpeed.setPower(1);
            shootTime = HalUtil.getCurrentTime() + 0.75;
            changeState(State.MOVING_HOME);
        }
    }

    private void loadBall(boolean redSide)
    {
        if(shooter == State.HOME)
        {
            highSpeed.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            highSpeed.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(redSide)
            {
                highSpeed.setTargetPosition(300);
            } else {
                highSpeed.setTargetPosition(-300);
            }
            shootTime = HalUtil.getCurrentTime() + 0.5;
            highSpeed.setPower(0.5);
            changeState(State.LOADING);
        }
    }

    private void setBall(boolean redSide)
    {
        if(shooter == State.LOADED)
        {
            if(redSide)
            {
                highSpeed.setTargetPosition(-300);
            } else {
                highSpeed.setTargetPosition(300);
            }
            shootTime = HalUtil.getCurrentTime() + 0.25;
            highSpeed.setPower(0.3);
            changeState(State.READYING);
        }
    }

    private void shootBall(boolean redSide)
    {
        if(shooter == State.READY)
        {
            if(redSide)
            {
                highSpeed.setTargetPosition(-700);
            } else {
                highSpeed.setTargetPosition(700);
            }
            shootTime = HalUtil.getCurrentTime() + 0.5;
            highSpeed.setPower(1);
            changeState(State.FIRING);
        }
    }

    public void shooterTask()
    {
        if(shooter == State.FIRING && HalUtil.getCurrentTime() >= shootTime)
        {
            changeState(State.FIRED);
        } else if(shooter == State.READYING && HalUtil.getCurrentTime() >= shootTime)
        {
            changeState(State.READY);
        } else if(shooter == State.LOADING && HalUtil.getCurrentTime() >= shootTime)
        {
            changeState(State.LOADED);
        } else if(shooter == State.MOVING_HOME && HalUtil.getCurrentTime() >= shootTime)
        {
            changeState(State.HOME);
        }
    }

    public void setTennisArmPower(boolean up)
    {
        if(up)
        {
            tennisArm.setPower(-1);
        } else {
            tennisArm.setPower(1);
        }
    }

    public void setTennisArmPower() { tennisArm.setPower(0);}

    public void resetMotors()
    {
        highSpeed.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tennisArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void changeState(State newState)
    {
        shooter = newState;
    }

    private void changeStateAuto(State_Autonomous newState) { autoShooter = newState; }

}
