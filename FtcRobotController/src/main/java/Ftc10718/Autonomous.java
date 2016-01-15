package Ftc10718;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by mjjus on 12/12/2015.
 */
public class Autonomous extends OpMode {

    // TODO: Replace these two guessed distances in measured distances
    final static int FIRST_DISTANCE_IN_ROTATION = 20;
    final static int SECOND_DISTANCE_IN_ROTATION = 30;

    final static int PULSES_PER_ROTATION = 1440;
    final static int FIRST_DISTANCE_IN_PULSES = PULSES_PER_ROTATION * FIRST_DISTANCE_IN_ROTATION;
    final static int SECOND_DISTANCE_IN_PULSES = PULSES_PER_ROTATION * SECOND_DISTANCE_IN_ROTATION;

    DcMotor motorRightF;
    DcMotor motorRightB;
    DcMotor motorLeftF;
    DcMotor motorLeftB;

    @Override
    public void init() {

        try {
            telemetry.addData("GLaDOS: Autonomous.Init() (attempting global takeover...)", "");
            telemetry.addData("GLaDOS: Code Version: ", DriverMode.CODE_VERSION);
            motorRightF = hardwareMap.dcMotor.get("frontright");
            motorRightB = hardwareMap.dcMotor.get("backright");
            motorLeftF = hardwareMap.dcMotor.get("frontleft");
            motorLeftB = hardwareMap.dcMotor.get("backleft");

            // TODO: Make sure the motors directions are correct
            motorRightF.setDirection(DcMotor.Direction.FORWARD);
            motorRightB.setDirection(DcMotor.Direction.FORWARD);
            motorLeftF.setDirection(DcMotor.Direction.FORWARD);
            motorLeftB.setDirection(DcMotor.Direction.FORWARD);

            motorRightF.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
            motorRightB.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
            motorLeftF.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
            motorLeftB.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);



            telemetry.addData("GLaDOS: Autonomous.Init() (takeover complete without errors).", "");
        }catch(Exception e){
            telemetry.addData("GLaDOS: Autonomous.Init() Error: ",e.getMessage());
        }
    }

    @Override
    public void start() {
        motorLeftB.setTargetPosition((int) FIRST_DISTANCE_IN_PULSES);
        motorRightB.setTargetPosition((int) SECOND_DISTANCE_IN_PULSES);
        motorLeftB.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRightB.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);

        motorLeftB.setPower(0.5);
        motorRightB.setPower(0.5);
        // need to add front motors, after swapping in front motors with encoders
    }

    @Override
    public void loop() {
        telemetry.addData("Left Position", motorLeftB.getCurrentPosition());
        telemetry.addData("Right Position", motorRightB.getCurrentPosition());

        // 1. Place robot in starting position

        // 2. go forward x rotations
        // 3. if red, rotate 90 counterclockwise
        // else, rotate 90 clockwise
        // 4. go forward y rotations
    }
}