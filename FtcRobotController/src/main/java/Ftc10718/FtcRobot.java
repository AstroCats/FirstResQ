package Ftc10718;

import com.qualcomm.robotcore.hardware.HardwareMap;

import ftclib.FtcDcMotor;
import ftclib.FtcHiTechnicGyro;
import ftclib.FtcOpMode;
import hallib.HalUtil;
import trclib.TrcDriveBase;
import trclib.TrcPidController;
import trclib.TrcPidDrive;
import trclib.TrcRobot;

public class FtcRobot implements TrcPidController.PidInput
{
    //
    // PID drive constants.
    //
    private static final double DRIVE_KP                = 0.04;
    private static final double DRIVE_KI                = 0.0;
    private static final double DRIVE_KD                = 0.0;
    private static final double DRIVE_KF                = 0.0;
    private static final double DRIVE_TOLERANCE         = 1.0;
    private static final double DRIVE_SETTLING          = 0.2;
    private static final double DRIVE_INCHES_PER_TICK   = (67.0/4941.0);

    //
    // PID turn constants.
    //
    private static final double TURN_KP                 = 0.1;
    private static final double TURN_KI                 = 0.0;
    private static final double TURN_KD                 = 0.0;
    private static final double TURN_KF                 = 0.0;
    private static final double TURN_TOLERANCE          = 1.0;
    private static final double TURN_SETTLING           = 0.2;

    //
    // Sensors.
    //
    public FtcHiTechnicGyro gyro;

    //
    // DriveBase subsystem.
    //
    public FtcDcMotor motorFrontLeft;
    public FtcDcMotor motorFrontRight;
    public FtcDcMotor motorBackLeft;
    public FtcDcMotor motorBackRight;
    public TrcDriveBase driveBase;

    //
    // PID drive.
    //
    public TrcPidController drivePidCtrl;
    public TrcPidController turnPidCtrl;
    public TrcPidDrive pidDrive;

    public FtcRobot(TrcRobot.RunMode runMode)
    {
        HardwareMap hardwareMap = FtcOpMode.getInstance().hardwareMap;
        hardwareMap.logDevices();
        //
        // Sensors.
        //
        gyro = new FtcHiTechnicGyro("hitechnicGyro");
        gyro.calibrate();
        //
        // DriveBase subsystem.
        //
        motorFrontLeft = new FtcDcMotor("frontleft");
        motorFrontRight = new FtcDcMotor("frontright");
        motorBackLeft = new FtcDcMotor("backleft");
        motorBackRight = new FtcDcMotor("backright");
        motorFrontLeft.setInverted(true);
        motorFrontRight.setInverted(false);
        motorBackLeft.setInverted(false);
        motorBackRight.setInverted(true);
        driveBase = new TrcDriveBase(motorFrontLeft, motorBackLeft,
                                     motorFrontRight, motorBackRight,
                                     gyro);
        driveBase.setYPositionScale(DRIVE_INCHES_PER_TICK);
        //
        // PID drive.
        //
        drivePidCtrl = new TrcPidController(
                "drivePid",
                DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KF,
                DRIVE_TOLERANCE, DRIVE_SETTLING, this);
        turnPidCtrl = new TrcPidController(
                "turnPid",
                TURN_KP, TURN_KI, TURN_KD, TURN_KF,
                TURN_TOLERANCE, TURN_SETTLING, this);
        pidDrive = new TrcPidDrive("pidDrive", driveBase, null, drivePidCtrl, turnPidCtrl);
    }   //FtcRobot

    public void startMode(TrcRobot.RunMode runMode)
    {
        FtcOpMode.getOpModeTracer().traceInfo(
                FtcOpMode.getOpModeName(), "Starting: %.3f", HalUtil.getCurrentTime());
        gyro.resetZIntegrator();
        gyro.setEnabled(true);
        driveBase.resetPosition();
    }   //startMode

    public void stopMode(TrcRobot.RunMode runMode)
    {
        FtcOpMode.getOpModeTracer().traceInfo(
                FtcOpMode.getOpModeName(), "Stopping: %.3f", HalUtil.getCurrentTime());
        gyro.setEnabled(false);
    }   //stopMode

    //
    // Implements TrcPidController.PidInput
    //

    @Override
    public double getInput(TrcPidController pidCtrl)
    {
        double input = 0.0;

        if (pidCtrl == drivePidCtrl)
        {
            input = driveBase.getYPosition();
        }
        else if (pidCtrl == turnPidCtrl)
        {
            input = driveBase.getHeading();
        }

        return input;
    }   //getInput

}   //class FtcRobot
