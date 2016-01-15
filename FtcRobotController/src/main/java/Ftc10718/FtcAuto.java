package Ftc10718;

import ftclib.FtcMenu;
import ftclib.FtcOpMode;
import hallib.HalDashboard;
import trclib.TrcEvent;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

public class FtcAuto extends FtcOpMode implements FtcMenu.MenuButtons
{
    private enum AutoStrategy
    {
        DO_NOTHING,
        PARK_MOUNTAIN
    }   //enum AutoStrategy

    private enum Alliance
    {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }   //enum Alliance

    private enum ParkMountainState
    {
        DO_DELAY,
        GO_FORWARD,
        TURN_TO_MOUNTAIN,
        GO_UP_MOUNTAIN,
        DONE
    }   //enum ParkMountainState

    public FtcRobot robot;

    private HalDashboard dashboard;

    //
    // State machine.
    //
    private TrcStateMachine sm;
    private TrcEvent event;
    private TrcTimer timer;

    //
    // Menu choices.
    //
    private Alliance alliance = Alliance.RED_ALLIANCE;
    private double delay = 0.0;
    private AutoStrategy autoStrategy = AutoStrategy.DO_NOTHING;

    private void doMenus()
    {
        //
        // Create the menus.
        //
        FtcMenu allianceMenu = new FtcMenu("Alliance:", null, this);
        FtcMenu delayMenu = new FtcMenu("Delay time:", allianceMenu, this);
        FtcMenu strategyMenu = new FtcMenu("Auto Strategies:", delayMenu, this);

        allianceMenu.addChoice("Red", Alliance.RED_ALLIANCE, delayMenu);
        allianceMenu.addChoice("Blue", Alliance.BLUE_ALLIANCE, delayMenu);

        delayMenu.addChoice("No delay", 0.0, strategyMenu);
        delayMenu.addChoice("1 sec", 1.0, strategyMenu);
        delayMenu.addChoice("2 sec", 2.0, strategyMenu);
        delayMenu.addChoice("4 sec", 4.0, strategyMenu);
        delayMenu.addChoice("8 sec", 8.0, strategyMenu);
        delayMenu.addChoice("10 sec", 10.0, strategyMenu);
        delayMenu.addChoice("12 sec", 12.0, strategyMenu);
        delayMenu.addChoice("15 sec", 15.0, strategyMenu);

        strategyMenu.addChoice("Do nothing", AutoStrategy.DO_NOTHING);
        strategyMenu.addChoice("Park Mountain", AutoStrategy.PARK_MOUNTAIN);

        //
        // Walk the menu tree starting with the alliance menu as the root
        // menu and get user choices.
        //
        FtcMenu.walkMenuTree(allianceMenu);
        //
        // Set choices variables.
        //
        alliance = (Alliance)allianceMenu.getCurrentChoiceObject();
        delay = (Double)delayMenu.getCurrentChoiceObject();
        autoStrategy = (AutoStrategy)strategyMenu.getCurrentChoiceObject();

        dashboard.displayPrintf(0, "Auto Strategy: %s", strategyMenu.getCurrentChoiceText());
    }   //doMenus

    //
    // Autonomous strategies.
    //

    private void doParkMountain(Alliance alliance, double delay)
    {
        if (sm.isReady())
        {
            ParkMountainState state = (ParkMountainState)sm.getState();
            dashboard.displayPrintf(1, "State: %s", state.toString());
            switch (state)
            {
                case DO_DELAY:
                    //
                    // Do delay if necessary.
                    //
                    if (delay == 0.0)
                    {
                        sm.setState(ParkMountainState.GO_FORWARD);
                    }
                    else
                    {
                        timer.set(delay, event);
                        sm.addEvent(event);
                        sm.waitForEvents(ParkMountainState.GO_FORWARD);
                    }
                    break;

                case GO_FORWARD:
                    robot.pidDrive.setTarget(50.0, 0.0, false, event);
                    sm.addEvent(event);
                    sm.waitForEvents(ParkMountainState.TURN_TO_MOUNTAIN);
                    break;

                case TURN_TO_MOUNTAIN:
                    robot.pidDrive.setTarget(
                            0.0,
                            alliance == Alliance.RED_ALLIANCE? -90.0: 90.0,
                            false, event);
                    sm.addEvent(event);
                    sm.waitForEvents(ParkMountainState.GO_UP_MOUNTAIN);
                    break;

                case GO_UP_MOUNTAIN:
                    robot.pidDrive.setTarget(50.0, 0.0, false, event);
                    sm.addEvent(event);
                    sm.waitForEvents(ParkMountainState.DONE);
                    break;

                case DONE:
                default:
                    sm.stop();
                    break;
            }
        }
    }   //doParkMountain

    //
    // Implements FtcOpMode abstract methods.
    //

    @Override
    public void robotInit()
    {
        robot = new FtcRobot(TrcRobot.RunMode.AUTO_MODE);
        dashboard = HalDashboard.getInstance();
        //
        // State machine.
        //
        sm = new TrcStateMachine("autoSM");
        event = new TrcEvent("autoEvent");
        timer = new TrcTimer("autoTimer");
        //
        // Choice menus.
        //
        doMenus();
    }   //robotInit

    @Override
    public void startMode()
    {
        robot.startMode(TrcRobot.RunMode.AUTO_MODE);
        dashboard.clearDisplay();
        //
        // Start the state machine according to the auto strategy.
        //
        switch (autoStrategy)
        {
            case PARK_MOUNTAIN:
                sm.start(ParkMountainState.DO_DELAY);
                break;
        }
    }   //startMode

    @Override
    public void stopMode()
    {
        robot.stopMode(TrcRobot.RunMode.AUTO_MODE);
    }   //stopMode

    @Override
    public void runPeriodic()
    {
    }   //runPeriodic

    @Override
    public void runContinuous()
    {
        switch (autoStrategy)
        {
            case PARK_MOUNTAIN:
                doParkMountain(alliance, delay);
                break;
        }
    }   //runContinuous

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

}   //class FtcAuto
