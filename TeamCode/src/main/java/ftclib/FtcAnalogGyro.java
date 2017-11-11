/*
 * Copyright (c) 2016 Titan Robotics Club (http://www.titanrobotics.com)
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

package ftclib;

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import hallib.HalUtil;


public class FtcAnalogGyro implements Runnable
{
    private static final int numCalSamples = 100;
    private static final int calInterval = 10; //in msec
    private LinearOpMode opmode;
    private double voltPerDegPerSec;
    private AnalogInput gyro;
    private double zeroOffset;
    private double deadband;
    private Thread integrationTask;
    private double heading;
    private double prevTime;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param opmode specifies the LinearOpMode object.
     * @param instanceName specifies the instance name.
     * @param voltPerDegPerSec specifies the rotation rate scale.
     */
    public FtcAnalogGyro(LinearOpMode opmode, String instanceName, double voltPerDegPerSec)
    {
        this.opmode = opmode;
        this.voltPerDegPerSec = voltPerDegPerSec;
        gyro = opmode.hardwareMap.analogInput.get(instanceName);
        zeroOffset = gyro.getMaxVoltage()/2;
        deadband = 0.0;

        integrationTask = new Thread(this, "GyroIntegrationTask");
        integrationTask.start();
    }

    public synchronized void calibrate()
    {
        double minValue = gyro.getVoltage();
        double maxValue = minValue;
        double sum = 0.0;

        for (int n = 0; n < numCalSamples; n++)
        {
            double value = gyro.getVoltage();
            sum += value;
            if (value < minValue) minValue = value;
            if (value > maxValue) maxValue = value;
            HalUtil.sleep(calInterval);
        }

        zeroOffset = sum/numCalSamples;
        deadband = maxValue - minValue;
        resetIntegrator();
    }

    public synchronized void resetIntegrator()
    {
        heading = 0.0;
        prevTime = HalUtil.getCurrentTime();
    }

    public synchronized double getRotationRate()
    {
        return applyDeadband(gyro.getVoltage() - zeroOffset, deadband)/voltPerDegPerSec;
    }

    public synchronized double getHeading()
    {
        return heading;
    }

    private synchronized void integrate()
    {
        double currTime = HalUtil.getCurrentTime();
        double rate = getRotationRate();
        Log.i("Debug", String.format("deltaTime=%.3f ms, volt=%.3f, rate=%.3f, heading=%.3fn", (currTime - prevTime)*1000.0, gyro.getVoltage(), rate, heading));
        heading += rate*(currTime - prevTime);
        prevTime = currTime;
    }

    @Override
    public void run()
    {
        resetIntegrator();
        while (!opmode.isStopRequested())
        {
            integrate();
            opmode.idle();
        }
    }   //run

    private double applyDeadband(double value, double deadband)
    {
        return Math.abs(value) >= deadband ? value: 0.0;
    }

}