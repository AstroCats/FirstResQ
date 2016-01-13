package Ftc10718;

import ftclib.FtcGamepad;
import ftclib.FtcOpMode;
import hallib.HalDashboard;
import trclib.TrcRobot;

public class FtcTeleOp extends FtcOpMode implements FtcGamepad.ButtonHandler
{
    public FtcRobot robot;

    private HalDashboard dashboard;
    private FtcGamepad driverGamepad;
    private FtcGamepad operatorGamepad;

    //
    // Implements FtcOpMode abstract methods.
    //

    @Override
    public void robotInit()
    {
        robot = new FtcRobot(TrcRobot.RunMode.TELEOP_MODE);
        dashboard = HalDashboard.getInstance();
        //
        // Initializing Gamepads.
        //
        driverGamepad = new FtcGamepad("Driver", gamepad1, this);
        operatorGamepad = new FtcGamepad("Operator", gamepad2, this);
        driverGamepad.setYInverted(true);
        operatorGamepad.setYInverted(true);
    }   //robotInit

    @Override
    public void startMode()
    {
        robot.startMode(TrcRobot.RunMode.TELEOP_MODE);
        dashboard.clearDisplay();
        //
        // There is an issue with the gamepad objects that may not be valid
        // before waitForStart() is called. So we call the setGamepad method
        // here to update their references in case they have changed.
        //
        driverGamepad.setGamepad(gamepad1);
        operatorGamepad.setGamepad(gamepad2);
    }   //startMode

    @Override
    public void stopMode()
    {
        robot.stopMode(TrcRobot.RunMode.TELEOP_MODE);
    }   //stopMode

    @Override
    public void runPeriodic()
    {
        //
        // DriveBase subsystem.
        //
        double leftPower = driverGamepad.getLeftStickY(true);
        double rightPower = driverGamepad.getRightStickY(true);
        robot.driveBase.tankDrive(leftPower, rightPower);
    }   //runPeriodic

    @Override
    public void runContinuous()
    {
    }   //runContinuous

    //
    // Implements FtcGamepad.ButtonHandler interface.
    //

    @Override
    public void gamepadButtonEvent(FtcGamepad gamepad, int btnMask, boolean pressed)
    {
        if (gamepad == driverGamepad)
        {
            switch (btnMask)
            {
                case FtcGamepad.GAMEPAD_A:
                    break;

                case FtcGamepad.GAMEPAD_Y:
                    break;

                case FtcGamepad.GAMEPAD_X:
                    break;

                case FtcGamepad.GAMEPAD_B:
                    break;
            }
        }
        else if (gamepad == operatorGamepad)
        {
            switch (btnMask)
            {
                case FtcGamepad.GAMEPAD_A:
                    break;

                case FtcGamepad.GAMEPAD_Y:
                    break;

                case FtcGamepad.GAMEPAD_X:
                    break;

                case FtcGamepad.GAMEPAD_B:
                    break;
            }
        }
    }   //gamepadButtonEvent

}   //class FtcTeleOp
