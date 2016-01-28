/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package Ftc10718;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class DriverMode2 extends OpMode {

	/*
	 * Note: the configuration of the servos is such that
	 * as the arm servo approaches 0, the arm position moves up (away from the floor).
	 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
	 */
	// TETRIX VALUES.
	final static double ARM_MIN_RANGE  = 0.0;
	final static double ARM_MAX_RANGE  = 1.0;
	final static double CLAW_MIN_RANGE  = 0.0;
	final static double CLAW_MAX_RANGE  = 1.0;
	final static double SCISSOR_LIFT_MAX_RANGE  = 1.0;
	final static double SCISSOR_LIFT_MIN_RANGE  = 0.0;

	final static double CODE_VERSION  = 3.1415962;

	// position of the arm servo.
	double armPosition = 0.0;

	// amount to change the arm servo position.
	double armDelta = 0.004;

	// position of the claw servo
	double clawPosition = 0.0;

	// amount to change the claw servo position by
	double clawDelta = 0.004;

	double scissorLiftDelta = 0.0042;
	double scissorLiftPosition = 0;


	//DcMotor motorRightF;
	DcMotor motorRightB;
	//DcMotor motorLeftF;
	DcMotor motorLeftB;

	/**
	 * Constructor
	 */
	public DriverMode2() {
	}

	/*
	 * Code to run when the op mode is first enabled goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
	 */
	@Override
	public void init() {
		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */
		try {
            telemetry.addData("GLaDOS: DriverMode.Init() (attempting global takeover...)", "");
            telemetry.addData("GLaDOS: Code Version: ", CODE_VERSION);
			//motorRightF = hardwareMap.dcMotor.get("frontright");
			motorRightB = hardwareMap.dcMotor.get("backright");
			//motorLeftF = hardwareMap.dcMotor.get("frontleft");
			motorLeftB = hardwareMap.dcMotor.get("backleft");

			// TODO: Make sure the motors directions are correct
			///motorRightF.setDirection(DcMotor.Direction.FORWARD);
			motorRightB.setDirection(DcMotor.Direction.FORWARD);
	//		motorLeftF.setDirection(DcMotor.Direction.FORWARD);
			motorLeftB.setDirection(DcMotor.Direction.FORWARD);

            telemetry.addData("GLaDOS: DriverMode.Init() (takeover complete without errors).", "");
        }catch(Exception e){
            telemetry.addData("GLaDOS: DriverMode.Init() Error: ",e.getMessage());
        }
	}

	/*
	 * This method will be called repeatedly in a loop
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
	 */
	@Override
	public void loop() {

		/*
		 * Gamepad 1 controls the left motors via the left stick, and it controls the
		 * right motors using the right stick
		 *
		 * Gamepad 2 controls the arm with the y and a buttons and the claw with the x and b buttons
		 * Gamepad 2 also controls the scissor lift with left stick and the string motor with right stick
		 *
		 * Scale the joystick value to make it easier to control the robot more precisely at slower
		 * speeds.
		 */
		float rightTread = gamepad1.right_stick_y;
		float leftTread = gamepad1.left_stick_y;

		if (rightTread < 0.0){
			rightTread = (float)-1.0*(rightTread*rightTread);
		}
		else{
			rightTread = rightTread*rightTread;
		}

		if (leftTread < 0.0){
			leftTread = (float)-1.0*(leftTread*leftTread);
		}
		else{
			leftTread = leftTread*leftTread;
		}

		float stringMotorPower = gamepad2.right_stick_y;

		try {
			telemetry.addData("Gamepad1 leftTread x", gamepad1.left_stick_x);
			telemetry.addData("Gamepad1 leftTread y", gamepad1.left_stick_y);
			telemetry.addData("Gamepad1 rightTread x", gamepad1.right_stick_x);
			telemetry.addData("Gamepad1 rightTread y", gamepad1.right_stick_y);
			telemetry.addData("servo1", armPosition);
			telemetry.addData("Gamepad2 scissorLift y", gamepad2.left_stick_y);
			telemetry.addData("Gamepad2 stringMotor y", gamepad2.right_stick_y);

	//		motorRightF.setPower(-rightTread);
			motorRightB.setPower(-rightTread);
			//motorLeftF.setPower(leftTread);
			motorLeftB.setPower(leftTread);
			if (gamepad2.a) {
				// if the A button is pushed on gamepad1, increment the position of
				// the arm servo.
				armPosition += armDelta;
			}

			if (gamepad2.y) {
				// if the Y button is pushed on gamepad1, decrease the position of
				// the arm servo.
				armPosition -= armDelta;
			}

			// update the position of the claw
			if (gamepad2.x) {
				clawPosition += clawDelta;
			}

			if (gamepad2.b) {
				clawPosition -= clawDelta;
			}

			if (gamepad2.left_stick_y > .2) {
				scissorLiftPosition += scissorLiftDelta;
			}

			if (gamepad2.left_stick_y < -.2) {
				scissorLiftPosition -= scissorLiftDelta;
			}

		}catch(Exception e) {
			telemetry.addData("Error", "Cannot detect game controller");
		}

		// clip the position values so that they never exceed their allowed range.
		armPosition = Range.clip(armPosition, ARM_MIN_RANGE, ARM_MAX_RANGE);
		clawPosition = Range.clip(clawPosition, CLAW_MIN_RANGE, CLAW_MAX_RANGE);
		scissorLiftPosition = Range.clip(scissorLiftPosition, SCISSOR_LIFT_MIN_RANGE, SCISSOR_LIFT_MAX_RANGE);

		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
		telemetry.addData("Text", "Hello Driver");
		telemetry.addData("Text", "*** Robot Data***");
		/*telemetry.addData("arm", "arm:  " + String.format("%.2f", armPosition));
		telemetry.addData("claw", "claw:  " + String.format("%.2f", clawPosition));*/
		telemetry.addData("leftTread tgt pwr",  "leftTread  pwr: " + String.format("%.2f", leftTread));
		telemetry.addData("rightTread tgt pwr", "rightTread pwr: " + String.format("%.2f", rightTread));
		/*telemetry.addData("Touch Sensor: ", Boolean.toString(isTouch));
		telemetry.addData("Color",String.format("r%d,g%d,b%d",red, green, blue));
		telemetry.addData("Distance: ",distance);*/

	}

	/*
	 * Code to run when the op mode is first disabled goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
	@Override
	public void stop() {

	}


	/*
	 * This method scales the joystick input so for low joystick values, the
	 * scaled value is less than linear.  This is to make it easier to drive
	 * the robot more precisely at slower speeds.
	 */
	double scaleInput(double dVal)  {
		double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
				0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

		// get the corresponding index for the scaleInput array.
		int index = (int) (dVal * 16.0);

		// index should be positive.
		if (index < 0) {
			index = -index;
		}

		// index cannot exceed size of array minus 1.
		if (index > 16) {
			index = 16;
		}

		// get value from the array.
		double dScale = 0.0;
		if (dVal < 0) {
			dScale = -scaleArray[index];
		} else {
			dScale = scaleArray[index];
		}

		// return scaled value.
		return dScale;
	}

}