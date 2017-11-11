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

package ftc8564opMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import ftc8564lib.Robot;
import hallib.HalDashboard;

@TeleOp(name="LockdownTeleOp", group="TeleOp")
public class LockdownTeleOp extends LinearOpMode {

    private Robot robot;
    private boolean invertDrive, useForklift;

    @Override
    public void runOpMode() throws InterruptedException {
        invertDrive = false;
        robot = new Robot(this,false);
        robot.driveBase.noEncoders();
        waitForStart();
        while (opModeIsActive()) {
            // Drive Train command
            if(gamepad1.dpad_up)
            {
                invertDrive = true;
            } else if(gamepad1.dpad_down)
            {
                invertDrive = false;
            } else if(gamepad1.dpad_left)
            {
                robot.driveBase.slowSpeed(true);
            } else if(gamepad1.dpad_right)
            {
                robot.driveBase.slowSpeed(false);
            }
            if(invertDrive)
            {
                robot.driveBase.tankDrive(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
            } else {
                robot.driveBase.tankDrive(gamepad1.right_stick_y, gamepad1.left_stick_y);
            }
            // Pulley System
            if(gamepad2.dpad_up)
            {
                robot.pulleySystem.setSlow(true);
            } else if(gamepad2.dpad_down)
            {
                robot.pulleySystem.setSlow(false);
            }
            if(gamepad2.left_bumper && useForklift)
            {
                robot.pulleySystem.manualControl(-gamepad2.left_stick_y, -gamepad2.left_stick_y);
            } else if(useForklift) {
                robot.pulleySystem.manualControl(-gamepad2.left_stick_y, -gamepad2.right_stick_y);
            }
            //Beacon Push
            if(gamepad2.left_trigger != 0)
            {
                robot.beaconPush.pushBeacon(true);
            } else if(gamepad2.right_trigger != 0)
            {
                robot.beaconPush.pushBeacon(false);
            }
            robot.beaconPush.buttonPusherTask();
            // Tennis Arm
            if(gamepad1.left_bumper)
            {
                robot.shooter.setTennisArmPower(true);
            } else if(gamepad1.left_trigger != 0)
            {
                robot.shooter.setTennisArmPower(false);
            } else {
                robot.shooter.setTennisArmPower();
            }
            //Shooting Mechanism
            if(gamepad2.dpad_left)
            {
                robot.shooter.shootSequence(false);
            } else if(gamepad2.dpad_right)
            {
                robot.shooter.shootSequence(true);
            }
            robot.shooter.shooterTask();
            //Forklift
            if(gamepad2.a)
            {
                robot.pulleySystem.openForkLift();
                useForklift = true;
            }
        }

        robot.pulleySystem.resetMotors();
        robot.shooter.resetMotors();
        robot.beaconPush.resetRacks();
        robot.driveBase.resetMotors();
        robot.driveBase.resetPIDDrive();

    }
}