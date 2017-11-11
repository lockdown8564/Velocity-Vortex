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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import ftc8564lib.*;
import hallib.HalUtil;

@Autonomous(name="TestAutonomous", group="Autonomous")
public class TestAutonomous extends LinearOpMode implements DriveBase.AbortTrigger {

    Robot robot;

    private int numParticles = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this,true);
        waitForStart();
        robot.driveBase.resetHeading();

        robot.driveBase.drivePID(6, false, null);
        robot.driveBase.spinPID(38);
        robot.driveBase.drivePID(32, false, null);
        // Loads and fires numParticles
        for(int i = 0; i < numParticles; i++)
        {
            robot.shooter.shootSequenceAuto(true);
            robot.shooter.waitForShootAuto();
            robot.shooter.shootSequenceAuto(true);
            robot.shooter.waitForShootAuto();
            robot.shooter.shootSequenceAuto(true);
            if(i != numParticles-1) robot.shooter.waitForShootAuto();
        }
        robot.driveBase.spinPID(27);
        robot.driveBase.drivePID(49, false, null);
        robot.driveBase.curveDrive(0.4,0.7,false,true);
        HalUtil.sleep(875);
        robot.driveBase.spinPID(0);
        robot.driveBase.drivePID(5, true, this);

        //At First Beacon
        robot.driveBase.sleep(0.25);
        if(robot.beaconPush.beaconColorIsAlliance(LockdownAutonomous.Alliance.RED_ALLIANCE))
        {
            robot.beaconPush.pushBeacon(true);
            robot.beaconPush.waitUntilPressed();
            robot.beaconPush.pushBeacon(true);
            robot.beaconPush.waitUntilPressed();
            robot.driveBase.spinPID(-1);
            robot.driveBase.drivePID(-40, false, null);
            robot.driveBase.drivePID(-10, true, this);
        } else {
            robot.driveBase.drivePID(5, false, null);
            robot.driveBase.sleep(0.25);
            if(robot.beaconPush.beaconColorIsAlliance(LockdownAutonomous.Alliance.RED_ALLIANCE))
            {
                robot.beaconPush.pushBeacon(true);
                robot.beaconPush.waitUntilPressed();
                robot.beaconPush.pushBeacon(true);
                robot.beaconPush.waitUntilPressed();
            }
            robot.driveBase.spinPID(-1);
            robot.driveBase.drivePID(-45, false, null);
            robot.driveBase.drivePID(-10, true, this);
        }

        //At Second Beacon
        robot.driveBase.sleep(0.25);
        if(robot.beaconPush.beaconColorIsAlliance(LockdownAutonomous.Alliance.RED_ALLIANCE))
        {
            robot.beaconPush.pushBeacon(true);
            robot.beaconPush.waitUntilPressed();
            robot.beaconPush.pushBeacon(true);
            robot.beaconPush.waitUntilPressed();
        } else {
            robot.driveBase.drivePID(5, false, null);
            robot.driveBase.sleep(0.25);
            if(robot.beaconPush.beaconColorIsAlliance(LockdownAutonomous.Alliance.RED_ALLIANCE))
            {
                robot.beaconPush.pushBeacon(true);
                robot.beaconPush.waitUntilPressed();
                robot.beaconPush.pushBeacon(true);
                robot.beaconPush.waitUntilPressed();
            }
        }

        /*
        //Red Side Center
        robot.driveBase.curve(0.6, -0.5698,false, true);
        HalUtil.sleep(800);
        robot.driveBase.spinPID(-45);
        robot.driveBase.drivePID(40, false, null);
        robot.driveBase.spinPID(-135);
        robot.driveBase.drivePID(25, false, null);


        //Red Side Corner
        robot.driveBase.curve(0.6, -0.5698,false, true);
        HalUtil.sleep(2800);
        robot.driveBase.spinPID(-180);
        robot.driveBase.drivePID(30, false, null);*/

        robot.driveBase.spinPID(-25);

        robot.shooter.resetMotors();
        robot.beaconPush.resetRacks();
        robot.pulleySystem.resetMotors();
        robot.driveBase.resetMotors();
        robot.driveBase.resetPIDDrive();

    }

    @Override
    public boolean shouldAbort() { return robot.odsLeft.getRawLightDetected() >= 0.6 || robot.odsRight.getRawLightDetected() >= 0.6; }

}
