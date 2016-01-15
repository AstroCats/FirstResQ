package Ftc10718;

import ftclib.FtcMenu;
import hallib.HalDashboard;
import trclib.TrcEvent;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

public class FtcTest extends FtcTeleOp implements FtcMenu.MenuButtons
{
    private enum Test
    {
        SENSORS_TEST,
        MOTORS_TEST,
        TIMED_DRIVE,
        DISTANCE_DRIVE,
        DEGREES_TURN
    }   //enum Test

    private enum State
    {
        START,
        DONE
    }   //enum State

    private HalDashboard dashboard;
    private int motorIndex = 0;

    //
    // State machine.
    //
    private TrcStateMachine sm;
    private TrcEvent event;
    private TrcTimer timer;
    //
    // Menu choices.
    //
    private Test test = Test.SENSORS_TEST;
    private double driveTime = 0.0;
    private double driveDistance = 0.0;
    private double turnDegrees = 0.0;

    //
    // Implements FtcOpMode abstract methods.
    //

    @Override
    public void robotInit()
    {
        super.robotInit();
        dashboard = HalDashboard.getInstance();

        //
        // Miscellaneous.
        //
        sm = new TrcStateMachine("TestSM");
        event = new TrcEvent("TestEvent");
        timer = new TrcTimer("TestTimer");

        //
        // Choice menus.
        //
        doMenus();
        sm.start(State.START);
    }   //robotInit

    @Override
    public void runPeriodic()
    {
    }   //runPeriodic

    @Override
    public void runContinuous()
    {
        State state = (State)sm.getState();
        dashboard.displayPrintf(
                8, "%s: %s", test.toString(), state != null? state.toString(): "STOPPED!");
        switch (test)
        {
            case SENSORS_TEST:
                doSensorsTest();
                break;

            case MOTORS_TEST:
                doMotorsTest();
                break;

            case TIMED_DRIVE:
                doTimedDrive(driveTime);
                break;

            case DISTANCE_DRIVE:
                doDistanceDrive(driveDistance);
                break;

            case DEGREES_TURN:
                doDegreesTurn(turnDegrees);
                break;
        }
    }   //runPeriodic

    //
    // Implements MenuButtons
    //

    @Override
    public boolean isMenuUpButton()
    {
        return gamepad1.dpad_up;
    }   //isMenuUpButton

    @Override
    public boolean isMenuDownButton()
    {
        return gamepad1.dpad_down;
    }   //isMenuDownButton

    @Override
    public boolean isMenuEnterButton()
    {
        return gamepad1.a;
    }   //isMenuEnterButton

    @Override
    public boolean isMenuBackButton()
    {
        return gamepad1.dpad_left;
    }   //isMenuBackButton

    private void doMenus()
    {
        FtcMenu testMenu = new FtcMenu("Tests:", null, this);
        FtcMenu driveTimeMenu = new FtcMenu("Drive time:", testMenu, this);
        FtcMenu driveDistanceMenu = new FtcMenu("Drive distance:", testMenu, this);
        FtcMenu turnDegreesMenu = new FtcMenu("Turn degrees:", testMenu, this);

        testMenu.addChoice("Sensors test", Test.SENSORS_TEST);
        testMenu.addChoice("Motors test", Test.MOTORS_TEST);
        testMenu.addChoice("Timed drive", Test.TIMED_DRIVE, driveTimeMenu);
        testMenu.addChoice("Distance drive", Test.DISTANCE_DRIVE, driveDistanceMenu);
        testMenu.addChoice("Degrees turn", Test.DEGREES_TURN, turnDegreesMenu);

        driveTimeMenu.addChoice("1 sec", 1.0);
        driveTimeMenu.addChoice("2 sec", 2.0);
        driveTimeMenu.addChoice("4 sec", 4.0);
        driveTimeMenu.addChoice("8 sec", 8.0);

        driveDistanceMenu.addChoice("2 ft", 24.0);
        driveDistanceMenu.addChoice("4 ft", 48.0);
        driveDistanceMenu.addChoice("8 ft", 96.0);
        driveDistanceMenu.addChoice("10 ft", 120.0);

        turnDegreesMenu.addChoice("-90 degrees", -90.0);
        turnDegreesMenu.addChoice("-180 degrees", -180.0);
        turnDegreesMenu.addChoice("-360 degrees", -360.0);
        turnDegreesMenu.addChoice("90 degrees", 90.0);
        turnDegreesMenu.addChoice("180 degrees", 180.0);
        turnDegreesMenu.addChoice("360 degrees", 360.0);

        FtcMenu.walkMenuTree(testMenu);

        test = (Test)testMenu.getCurrentChoiceObject();
        driveTime = (Double)driveTimeMenu.getCurrentChoiceObject();
        driveDistance = (Double)driveDistanceMenu.getCurrentChoiceObject();
        turnDegrees = (Double)turnDegreesMenu.getCurrentChoiceObject();

        dashboard.displayPrintf(0, "Test: %s", testMenu.getCurrentChoiceText());
    }   //doMenus

    private void doSensorsTest()
    {
        //
        // Allow TeleOp to run so we can control the robot in test sensor mode.
        //
        super.runPeriodic();
        //
        // Read all sensors and display on the dashboard.
        // Drive the robot around to sample different locations of the field.
        //
        dashboard.displayPrintf(9, "Sensors Test:");
        dashboard.displayPrintf(10, "Enc:lf=%.0f,rf=%.0f",
                                robot.motorFrontLeft.getPosition(),
                                robot.motorFrontRight.getPosition());
        dashboard.displayPrintf(11, "Enc:lr=%.0f,rr=%.0f",
                                robot.motorBackLeft.getPosition(),
                                robot.motorBackRight.getPosition());
        dashboard.displayPrintf(12, "Gyro:Rate=%.1f,Heading=%.1f",
                                robot.gyro.getZRotationRate().value,
                                robot.gyro.getZHeading().value);
    }   //doSensorsTest

    private void doMotorsTest()
    {
        dashboard.displayPrintf(9, "Motors Test: motorIndex=%d", motorIndex);
        dashboard.displayPrintf(10, "Enc: LF=%d, RF=%d",
                                robot.motorFrontLeft.getPosition(),
                                robot.motorFrontRight.getPosition());
        dashboard.displayPrintf(11, "Enc: LB=%d, RB=%d",
                                robot.motorBackLeft.getPosition(),
                                robot.motorBackRight.getPosition());
        if (sm.isReady())
        {
            State state = (State)sm.getState();
            switch (state)
            {
                case START:
                    switch (motorIndex)
                    {
                        case 0:
                            robot.motorFrontLeft.setPower(0.5);
                            robot.motorFrontRight.setPower(0.0);
                            robot.motorBackLeft.setPower(0.0);
                            robot.motorBackRight.setPower(0.0);
                            break;

                        case 1:
                            robot.motorFrontLeft.setPower(0.0);
                            robot.motorFrontRight.setPower(0.5);
                            robot.motorBackLeft.setPower(0.0);
                            robot.motorBackRight.setPower(0.0);
                            break;

                        case 2:
                            robot.motorFrontLeft.setPower(0.0);
                            robot.motorFrontRight.setPower(0.0);
                            robot.motorBackLeft.setPower(0.5);
                            robot.motorBackRight.setPower(0.0);
                            break;

                        case 3:
                            robot.motorFrontLeft.setPower(0.0);
                            robot.motorFrontRight.setPower(0.0);
                            robot.motorBackLeft.setPower(0.0);
                            robot.motorBackRight.setPower(0.5);
                            break;
                    }
                    motorIndex = (motorIndex + 1)%4;
                    timer.set(5.0, event);
                    sm.addEvent(event);
                    sm.waitForEvents(State.START);
                    break;

                case DONE:
                default:
                    sm.stop();
                    break;
            }
        }
    }   //doMotorsTest

    private void doTimedDrive(double time)
    {
        double lfEnc = robot.motorFrontLeft.getPosition();
        double rfEnc = robot.motorFrontRight.getPosition();
        double lrEnc = robot.motorBackLeft.getPosition();
        double rrEnc = robot.motorBackRight.getPosition();
        dashboard.displayPrintf(9, "Timed Drive: %.0f sec", time);
        dashboard.displayPrintf(10, "Enc:lf=%.0f,rf=%.0f", lfEnc, rfEnc);
        dashboard.displayPrintf(11, "Enc:lr=%.0f,rr=%.0f", lrEnc, rrEnc);
        dashboard.displayPrintf(12, "average=%f", (lfEnc + rfEnc + lrEnc + rrEnc)/4.0);
        dashboard.displayPrintf(13, "xPos=%.1f,yPos=%.1f,heading=%.1f",
                                robot.driveBase.getXPosition(),
                                robot.driveBase.getYPosition(),
                                robot.driveBase.getHeading());

        if (sm.isReady())
        {
            State state = (State)sm.getState();
            switch (state)
            {
                case START:
                    //
                    // Drive the robot forward and set a timer for the given time.
                    //
                    robot.driveBase.tankDrive(0.2, 0.2);
                    timer.set(time, event);
                    sm.addEvent(event);
                    sm.waitForEvents(State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done, stop the robot.
                    //
                    robot.driveBase.stop();
                    sm.stop();
                    break;
            }
        }
    }   //doDriveTime

    private void doDistanceDrive(double distance)
    {
        dashboard.displayPrintf(9, "Distance Drive: %.1f ft", distance/12.0);
        dashboard.displayPrintf(10, "xPos=%.1f,yPos=%.1f,heading=%.1f",
                                robot.driveBase.getXPosition(),
                                robot.driveBase.getYPosition(),
                                robot.driveBase.getHeading());
        robot.drivePidCtrl.displayPidInfo(11);
        robot.turnPidCtrl.displayPidInfo(13);

        if (sm.isReady())
        {
            State state = (State)sm.getState();
            switch (state)
            {
                case START:
                    //
                    // Drive the given distance.
                    //
                    robot.pidDrive.setTarget(distance, 0.0, false, event);
                    sm.addEvent(event);
                    sm.waitForEvents(State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    sm.stop();
                    break;
            }
        }
    }   //doDriveDistance

    private void doDegreesTurn(double degrees)
    {
        dashboard.displayPrintf(9, "Degrees Turn: %.1f", degrees);
        dashboard.displayPrintf(10, "xPos=%.1f,yPos=%.1f,heading=%.1f",
                                robot.driveBase.getXPosition(),
                                robot.driveBase.getYPosition(),
                                robot.driveBase.getHeading());
        robot.drivePidCtrl.displayPidInfo(11);
        robot.turnPidCtrl.displayPidInfo(13);

        if (sm.isReady())
        {
            State state = (State)sm.getState();
            switch (state)
            {
                case START:
                    //
                    // Turn the given degrees.
                    //
                    robot.pidDrive.setTarget(0.0, degrees, false, event);
                    sm.addEvent(event);
                    sm.waitForEvents(State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    sm.stop();
                    break;
            }
        }
    }   //doTurnDegrees

}   //class FtcTest
