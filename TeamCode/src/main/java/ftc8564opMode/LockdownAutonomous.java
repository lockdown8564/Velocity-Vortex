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
import com.qualcomm.robotcore.util.ElapsedTime;
import ftclib.*;
import ftc8564lib.*;
import hallib.HalUtil;

@Autonomous(name="LockdownAutonomous", group="Autonomous")
public class LockdownAutonomous extends LinearOpMode implements FtcMenu.MenuButtons, DriveBase.AbortTrigger {

    Robot robot;
    private ElapsedTime mClock = new ElapsedTime();

    public enum Alliance {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }

    private enum Strategy {
        SHOOT_BALL,
        HIGH_RISK,
        DEFENSE
    }

    private enum ParkOption {
        NOTHING,
        CENTER_VORTEX,
        CORNER_VORTEX
    }

    private Alliance alliance = Alliance.RED_ALLIANCE;
    private Strategy strategy = Strategy.DEFENSE;
    private ParkOption parkOption = ParkOption.NOTHING;
    private int numParticles = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this,true);
        doMenus();
        waitForStart();

        robot.driveBase.resetHeading();

        switch (strategy) {
            case HIGH_RISK:
                runHighRisk();
                break;
            case SHOOT_BALL:
                runShootBall();
                break;
            case DEFENSE:
                runDefense();
                break;
        }
        runCleanUp();
    }

    private void runHighRisk() throws InterruptedException {
        robot.driveBase.drivePID(6, false, null);
        robot.driveBase.spinPID(alliance == Alliance.RED_ALLIANCE ? 38 : -38);
        robot.driveBase.drivePID(31, false, null);
        // Loads and fires numParticles
        for(int i = 0; i < numParticles; i++)
        {
            robot.shooter.shootSequenceAuto(alliance == Alliance.RED_ALLIANCE);
            robot.shooter.waitForShootAuto();
            robot.shooter.shootSequenceAuto(alliance == Alliance.RED_ALLIANCE);
            robot.shooter.waitForShootAuto();
            robot.shooter.shootSequenceAuto(alliance == Alliance.RED_ALLIANCE);
            if(i != numParticles-1) robot.shooter.waitForShootAuto();
        }
        robot.driveBase.spinPID(alliance == Alliance.RED_ALLIANCE ? 27 : -27);
        robot.driveBase.drivePID(45, false, null);
        robot.driveBase.curveDrive(alliance == Alliance.RED_ALLIANCE ? 0.4 : 0.7, alliance == Alliance.RED_ALLIANCE ? 0.7 : 0.4,false,true);
        HalUtil.sleep(875);
        robot.driveBase.spinPID(0);
        robot.driveBase.drivePID(10, true, this);
        //At First Beacon
        robot.driveBase.sleep(0.25);
        if(robot.beaconPush.beaconColorIsAlliance(alliance))
        {
            robot.beaconPush.pushBeacon(alliance == Alliance.RED_ALLIANCE);
            robot.beaconPush.waitUntilPressed();
            robot.beaconPush.pushBeacon(alliance == Alliance.RED_ALLIANCE);
            robot.beaconPush.waitUntilPressed();
            robot.driveBase.spinPID(alliance == Alliance.RED_ALLIANCE ? -1 : 1);
            robot.driveBase.drivePID(-40, false, null);
            robot.driveBase.drivePID(-10, true, this);
        } else {
            robot.driveBase.drivePID(5, false, null);
            robot.driveBase.sleep(0.25);
            if(robot.beaconPush.beaconColorIsAlliance(alliance))
            {
                robot.beaconPush.pushBeacon(alliance == Alliance.RED_ALLIANCE);
                robot.beaconPush.waitUntilPressed();
                robot.beaconPush.pushBeacon(alliance == Alliance.RED_ALLIANCE);
                robot.beaconPush.waitUntilPressed();
            }
            robot.driveBase.spinPID(alliance == Alliance.RED_ALLIANCE ? -1 : 1);
            robot.driveBase.drivePID(-45, false, null);
            robot.driveBase.drivePID(-10, true, this);
        }
        //At Second Beacon
        robot.driveBase.sleep(0.2);
        if(robot.beaconPush.beaconColorIsAlliance(alliance))
        {
            robot.beaconPush.pushBeacon(alliance == Alliance.RED_ALLIANCE);
            robot.beaconPush.waitUntilPressed();
            robot.beaconPush.pushBeacon(alliance == Alliance.RED_ALLIANCE);
            robot.beaconPush.waitUntilPressed();
        } else {
            robot.driveBase.drivePID(7, false, null);
            robot.driveBase.sleep(0.2);
            if(robot.beaconPush.beaconColorIsAlliance(alliance))
            {
                robot.beaconPush.pushBeacon(alliance == Alliance.RED_ALLIANCE);
                robot.beaconPush.waitUntilPressed();
                robot.beaconPush.pushBeacon(alliance == Alliance.RED_ALLIANCE);
                robot.beaconPush.waitUntilPressed();
            }
        }
        if(parkOption == ParkOption.CENTER_VORTEX)
        {
            robot.driveBase.curve(0.6, alliance == Alliance.RED_ALLIANCE ? -0.5698 : 0.5698,false, true);
            HalUtil.sleep(800);
            robot.driveBase.spinPID(alliance == Alliance.RED_ALLIANCE ? -45 : 45);
            robot.driveBase.drivePID(40, false, null);
            robot.driveBase.spinPID(alliance == Alliance.RED_ALLIANCE ? -135 : 135);
            robot.driveBase.drivePID(25, false, null);
        } else if(parkOption == ParkOption.CORNER_VORTEX) {
            robot.driveBase.curve(0.6, alliance == Alliance.RED_ALLIANCE ? -0.5698 : 0.5698,false, true);
            HalUtil.sleep(2800);
            robot.driveBase.spinPID(alliance == Alliance.RED_ALLIANCE ? -180 : 180);
            robot.driveBase.drivePID(30, false, null);
        }
    }

    private void runShootBall() throws InterruptedException {
        HalUtil.sleep(9750);
        robot.driveBase.drivePID(8, false ,null);
        robot.driveBase.spinPID(alliance == Alliance.RED_ALLIANCE ? 45 : -45);
        robot.driveBase.drivePID(15, false, null);
        robot.driveBase.spinPID(alliance == Alliance.RED_ALLIANCE ? 90 : -90);
        robot.driveBase.drivePID(30, false, null);
        for(int i = 0; i < numParticles; i++)
        {
            robot.shooter.shootSequenceAuto(alliance == Alliance.RED_ALLIANCE);
            robot.shooter.waitForShootAuto();
            robot.shooter.shootSequenceAuto(alliance == Alliance.RED_ALLIANCE);
            robot.shooter.waitForShootAuto();
            robot.shooter.shootSequenceAuto(alliance == Alliance.RED_ALLIANCE);
            if(i != numParticles-1) robot.shooter.waitForShootAuto();
        }
        if(parkOption == ParkOption.CENTER_VORTEX)
        {
            robot.driveBase.drivePID(-20, false, null);
            robot.driveBase.spinPID(0);
            robot.driveBase.drivePID(25, false, null);
            robot.driveBase.spinPID(alliance == Alliance.RED_ALLIANCE ? 45 : -45);
            robot.driveBase.drivePID(30, false, null);
        } else if(parkOption == ParkOption.CORNER_VORTEX) {
            robot.driveBase.drivePID(43, false, null);
        }
    }

    private void runDefense() throws InterruptedException {
        mClock.reset();
        mClock.startTime();
        robot.driveBase.drivePID(9, false, null);
        while (mClock.time() <= 10.0) {
        }
        robot.driveBase.drivePID(75, false, null);
        robot.driveBase.spinPID(alliance == Alliance.RED_ALLIANCE ? 60 : -60);
        robot.driveBase.drivePID(50, false, null);
    }

    private void runCleanUp() throws InterruptedException {
        robot.shooter.resetMotors();
        robot.beaconPush.resetRacks();
        robot.pulleySystem.resetMotors();
        robot.driveBase.resetMotors();
        robot.driveBase.resetPIDDrive();
    }

    @Override
    public boolean shouldAbort() { return robot.odsLeft.getRawLightDetected() >= 0.6 || robot.odsRight.getRawLightDetected() >= 0.6; }

    @Override
    public boolean isMenuUpButton() {
        return gamepad1.dpad_up;
    }

    @Override
    public boolean isMenuDownButton() {
        return gamepad1.dpad_down;
    }

    @Override
    public boolean isMenuEnterButton() {
        return gamepad1.dpad_right;
    }

    @Override
    public boolean isMenuBackButton() {
        return gamepad1.dpad_left;
    }

    private void doMenus() throws InterruptedException {
        FtcChoiceMenu allianceMenu = new FtcChoiceMenu("Alliance:", null, this);
        FtcValueMenu numParticlesMenu = new FtcValueMenu("Shoot Particles:", allianceMenu, this, 0.0, 2.0, 1.0, 2.0, " %.0f");
        FtcChoiceMenu parkOptionMenu = new FtcChoiceMenu("Park Option:", numParticlesMenu, this);
        FtcChoiceMenu strategyMenu = new FtcChoiceMenu("Strategy:", parkOptionMenu, this);

        numParticlesMenu.setChildMenu(parkOptionMenu);

        allianceMenu.addChoice("Red", Alliance.RED_ALLIANCE, numParticlesMenu);
        allianceMenu.addChoice("Blue", Alliance.BLUE_ALLIANCE, numParticlesMenu);

        parkOptionMenu.addChoice("Do Nothing", ParkOption.NOTHING, strategyMenu);
        parkOptionMenu.addChoice("Park Center", ParkOption.CENTER_VORTEX, strategyMenu);
        parkOptionMenu.addChoice("Park Corner", ParkOption.CORNER_VORTEX, strategyMenu);

        strategyMenu.addChoice("Close", Strategy.HIGH_RISK);
        strategyMenu.addChoice("Far", Strategy.SHOOT_BALL);
        strategyMenu.addChoice("Defense: Far", Strategy.DEFENSE);

        FtcMenu.walkMenuTree(allianceMenu);
        alliance = (Alliance) allianceMenu.getCurrentChoiceObject();
        numParticles = (int) numParticlesMenu.getCurrentValue();
        parkOption = (ParkOption) parkOptionMenu.getCurrentChoiceObject();
        strategy = (Strategy) strategyMenu.getCurrentChoiceObject();
    }

}