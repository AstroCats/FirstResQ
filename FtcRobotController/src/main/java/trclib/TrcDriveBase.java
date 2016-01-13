/*
 * Titan Robotics Framework Library
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.net)
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

package trclib;

import hallib.HalRobotDrive;

public class TrcDriveBase extends HalRobotDrive implements TrcTaskMgr.Task
{
    private static final String moduleName = "TrcDriveBase";
    private static final boolean debugEnabled = false;
    private TrcDbgTrace dbgTrace = null;

    private TrcMotorController leftFrontMotor;
    private TrcMotorController leftRearMotor;
    private TrcMotorController rightFrontMotor;
    private TrcMotorController rightRearMotor;
    private TrcGyro gyro;

    private boolean fourMotors;
    private double xPos;
    private double yPos;
    private double rotPos;
    private double heading;
    private double xScale;
    private double yScale;
    private double rotScale;
    private double xSpeed;
    private double ySpeed;
    private double turnSpeed;

    public TrcDriveBase(
            TrcMotorController leftFrontMotor,
            TrcMotorController leftRearMotor,
            TrcMotorController rightFrontMotor,
            TrcMotorController rightRearMotor,
            TrcGyro gyro)
    {
        super(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);

        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(
                    moduleName,
                    false,
                    TrcDbgTrace.TraceLevel.API,
                    TrcDbgTrace.MsgLevel.INFO);
        }

        this.leftFrontMotor = leftFrontMotor;
        this.leftRearMotor = leftRearMotor;
        this.rightFrontMotor = rightFrontMotor;
        this.rightRearMotor = rightRearMotor;
        this.gyro = gyro;
        fourMotors = leftFrontMotor != null && rightFrontMotor != null;
        xScale = 1.0;
        yScale = 1.0;
        rotScale = 1.0;
        resetPosition();

        TrcTaskMgr taskMgr = TrcTaskMgr.getInstance();
        taskMgr.registerTask(
                moduleName,
                this,
                TrcTaskMgr.TaskType.START_TASK);
        taskMgr.registerTask(
                moduleName,
                this,
                TrcTaskMgr.TaskType.STOP_TASK);
        taskMgr.registerTask(
                moduleName,
                this,
                TrcTaskMgr.TaskType.PRECONTINUOUS_TASK);
    }   //TrcDriveBase

    public TrcDriveBase(
            TrcMotorController leftFrontMotor,
            TrcMotorController leftRearMotor,
            TrcMotorController rightFrontMotor,
            TrcMotorController rightRearMotor)
    {
        this(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor, null);
    }   //TrcDriveBase

    public TrcDriveBase(
            TrcMotorController leftMotor,
            TrcMotorController rightMotor,
            TrcGyro gyro)
    {
        this(null, leftMotor, null, rightMotor, gyro);
    }   //TrcDriveBase

    public TrcDriveBase(TrcMotorController leftMotor, TrcMotorController rightMotor)
    {
        this(null, leftMotor, null, rightMotor, null);
    }   //TrcDriveBase

    public void resetPosition()
    {
        final String funcName = "resetPosition";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (leftFrontMotor != null)
        {
            leftFrontMotor.resetPosition();
        }

        if (leftRearMotor != null)
        {
            leftRearMotor.resetPosition();
        }

        if (rightFrontMotor != null)
        {
            rightFrontMotor.resetPosition();
        }

        if (rightRearMotor != null)
        {
            rightRearMotor.resetPosition();
        }

        if (gyro != null)
        {
            gyro.resetZIntegrator();
        }

        xPos = 0.0;
        yPos = 0.0;
        rotPos = 0.0;
        heading = 0.0;
        xSpeed = 0.0;
        ySpeed = 0.0;
        turnSpeed = 0.0;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //resetPosition

    public void setXPositionScale(double scale)
    {
        final String funcName = "setXPositionScale";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "scale=%f", scale);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.xScale = scale;
    }   //setXPositionScale

    public void setYPositionScale(double scale)
    {
        final String funcName = "setYPositionScale";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "scale=%f", scale);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.yScale = scale;
    }   //setYPositionScale

    public void setRotationScale(double scale)
    {
        final String funcName = "setRotationScale";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "scale=%f", scale);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.rotScale = scale;
    }   //setRotationScale

    public double getXPosition()
    {
        final String funcName = "getXPosition";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "=%f", xPos);
        }

        return xPos;
    }   //getXPosition

    public double getYPosition()
    {
        final String funcName = "getYPosition";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "=%f", yPos);
        }

        return yPos;
    }   //getYPosition

    public double getRotatePosition()
    {
        final String funcName = "getRotatePosition";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "=%f", rotPos);
        }

        return rotPos;
    }   //getRotatePosition

    public double getHeading()
    {
        final String funcName = "getHeading";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "=%f", heading);
        }

        return heading;
    }   //getHeading

    public double getXSpeed()
    {
        final String funcName = "getXSpeed";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "=%f", xSpeed);
        }

        return xSpeed;
    }   //getXSpeed

    public double getYSpeed()
    {
        final String funcName = "getYSpeed";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "=%f", ySpeed);
        }

        return ySpeed;
    }   //getYSpeed

    public double getTurnSpeed()
    {
        final String funcName = "getTurnSpeed";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "=%f", turnSpeed);
        }

        return turnSpeed;
    }   //getTurnSpeed

    /*
    public void setBrakeMode(boolean enabled)
    {
        final String funcName = "setBrakeMode";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "enabled=%s", Boolean.toString(enabled));
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        if (leftFrontMotor != null)
        {
            if (leftFrontMotor instanceof CANTalon)
            {
                ((CANTalon)leftFrontMotor).enableBrakeMode(enabled);
            }
            else if (leftFrontMotor instanceof CANJaguar)
            {
                ((CANJaguar)leftFrontMotor).configNeutralMode(
                        enabled? NeutralMode.Brake: NeutralMode.Coast);
            }
        }

        if (rightFrontMotor != null)
        {
            if (rightFrontMotor instanceof CANTalon)
            {
                ((CANTalon)rightFrontMotor).enableBrakeMode(enabled);
            }
            else if (rightFrontMotor instanceof CANJaguar)
            {
                ((CANJaguar)rightFrontMotor).configNeutralMode(
                        enabled? NeutralMode.Brake: NeutralMode.Coast);
            }
        }

        if (leftRearMotor != null)
        {
            if (leftRearMotor instanceof CANTalon)
            {
                ((CANTalon)leftRearMotor).enableBrakeMode(enabled);
            }
            else if (leftRearMotor instanceof CANJaguar)
            {
                ((CANJaguar)leftRearMotor).configNeutralMode(
                        enabled? NeutralMode.Brake: NeutralMode.Coast);
            }
        }

        if (rightRearMotor != null)
        {
            if (rightRearMotor instanceof CANTalon)
            {
                ((CANTalon)rightRearMotor).enableBrakeMode(enabled);
            }
            else if (rightRearMotor instanceof CANJaguar)
            {
                ((CANJaguar)rightRearMotor).configNeutralMode(
                        enabled? NeutralMode.Brake: NeutralMode.Coast);
            }
        }
    }   //setBrakeMode
    */

    public void stop()
    {
        final String funcName = "stop";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
        }

        stopMotor();

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //stop

    //
    // Implements TrcTaskMgr.Task
    //

    @Override
    public void startTask(TrcRobot.RunMode runMode)
    {
        final String funcName = "startTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.TASK,
                    "mode=%s", runMode.toString());
        }

        if (runMode != TrcRobot.RunMode.DISABLED_MODE)
        {
            resetPosition();
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //startTask

    @Override
    public void stopTask(TrcRobot.RunMode runMode)
    {
        final String funcName = "stopTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.TASK,
                    "mode=%s", runMode.toString());
        }

        if (runMode != TrcRobot.RunMode.DISABLED_MODE)
        {
            stop();
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //stopTask

    @Override
    public void prePeriodicTask(TrcRobot.RunMode runMode)
    {
    }   //prePeriodicTask

    @Override
    public void postPeriodicTask(TrcRobot.RunMode runMode)
    {
    }   //postPeriodicTask

    @Override
    public void preContinuousTask(TrcRobot.RunMode runMode)
    {
        final String funcName = "preContinuousTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.TASK,
                    "mode=%s", runMode.toString());
        }

        //
        // According to RobotDrive.mecanumDrive_Cartesian in WPILib:
        //
        // LF =  x + y + rot    RF = -x + y - rot
        // LR = -x + y + rot    RR =  x + y - rot
        //
        // (LF + RR) - (RF + LR) = (2x + 2y) - (-2x + 2y)
        // => (LF + RR) - (RF + LR) = 4x
        // => x = ((LF + RR) - (RF + LR))/4
        //
        // LF + RF + LR + RR = 4y
        // => y = (LF + RF + LR + RR)/4
        //
        // (LF + LR) - (RF + RR) = (2y + 2rot) - (2y - 2rot)
        // => (LF + LR) - (RF + RR) = 4rot
        // => rot = ((LF + LR) - (RF + RR))/4
        //
        double lfEnc = 0.0, lrEnc = 0.0, rfEnc = 0.0, rrEnc = 0.0;
        double lfSpeed = 0.0, lrSpeed = 0.0, rfSpeed = 0.0, rrSpeed = 0.0;
        if (leftFrontMotor != null)
        {
            try
            {
                lfEnc = leftFrontMotor.getPosition();
            }
            catch (UnsupportedOperationException e)
            {
            }

            try
            {
                lfSpeed = leftFrontMotor.getSpeed();
            }
            catch (UnsupportedOperationException e)
            {
            }
        }
        if (leftRearMotor != null)
        {
            try
            {
                lrEnc = leftRearMotor.getPosition();
            }
            catch (UnsupportedOperationException e)
            {
            }

            try
            {
                lrSpeed = leftRearMotor.getSpeed();
            }
            catch (UnsupportedOperationException e)
            {
            }
        }
        if (rightFrontMotor != null)
        {
            try
            {
                rfEnc = rightFrontMotor.getPosition();
            }
            catch (UnsupportedOperationException e)
            {
            }

            try
            {
                rfSpeed = rightFrontMotor.getSpeed();
            }
            catch (UnsupportedOperationException e)
            {
            }
        }
        if (rightRearMotor != null)
        {
            try
            {
                rrEnc = rightRearMotor.getPosition();
            }
            catch (UnsupportedOperationException e)
            {
            }

            try
            {
                rrSpeed = rightRearMotor.getSpeed();
            }
            catch (UnsupportedOperationException e)
            {
            }
        }

        if (fourMotors)
        {
            xPos = ((lfEnc + rrEnc) - (rfEnc + lrEnc))*xScale/4.0;
            yPos = (lfEnc + lrEnc + rfEnc + rrEnc)*yScale/4.0;
            rotPos = ((lfEnc + lrEnc) - (rfEnc + rrEnc))*rotScale/4.0;
            xSpeed = ((lfSpeed + rrSpeed) - (rfSpeed + lrSpeed))*xScale/4.0;
            ySpeed = (lfSpeed + lrSpeed + rfSpeed + rrSpeed)*yScale/4.0;
        }
        else
        {
            yPos = (lrEnc + rrEnc)*yScale/2.0;
            rotPos = (lrEnc - rrEnc)*rotScale/2.0;
            ySpeed = (lrSpeed + rrSpeed)*yScale/2.0;
        }

        if (gyro != null)
        {
            heading = (Double)gyro.getZHeading().value;
            turnSpeed = (Double)gyro.getZRotationRate().value;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //preContinuousTask

    @Override
    public void postContinuousTask(TrcRobot.RunMode runMode)
    {
    }   //postContinuousTask

}   //class TrcDriveBase