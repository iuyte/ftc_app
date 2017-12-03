package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor

/**
 * Created by ethan on 7/29/17.
 */

@TeleOp(name = "robot: Teleop Mecanum", group = "robot")
class TeleopMecanum : OpMode() {

	/* Declare OpMode members. */
	private var robot = Hardware(hardwareMap, this, Hardware.DriveMode.Mecanum, false) // use the class created to define a robot's hardware
	private var speedMultiplier = 0.0
	private val liftPower = 0.8
	private val liftMove = 100
	private var liftStop = false

	/*
	 * Code to run ONCE when the driver hits INIT
	 */
	override fun init() {
		/* Initialize the hardware variables.
		 * The init() method of the hardware class does all the work here
		 */
		robot.init(hardwareMap, DcMotor.RunMode.RUN_USING_ENCODER)
		for (i in 0..1) {
			robot.motors["arm"]!![i].mode = DcMotor.RunMode.RUN_TO_POSITION
			robot.motors["arm"]!![i].power = liftPower
		}

		// Send telemetry message to signify robot waiting;
		telemetry.addData("Initialization", "Complete")
	}

	/*
	 * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
	 */
	override fun init_loop() {}

	/*
	 * Code to run ONCE when the driver hits PLAY
	 */
	override fun start() {}

	/*
	 * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
	 */
	override fun loop() {
		speedMultiplier = 1.0 - (gamepad1.left_trigger.toDouble() * 0.45) -
				(if (gamepad2.b) 0.0 else 0.3)

		// Run the robot drive in mecanum mode
		robot.drive(
			-gamepad1.left_stick_x * speedMultiplier,
			-gamepad1.left_stick_y * speedMultiplier,
			-gamepad1.right_stick_x * speedMultiplier
		)

		when {
			gamepad1.right_bumper || gamepad2.right_bumper -> {
				robot.motors["arm"]!![0].targetPosition = robot.motors["arm"]!![0].currentPosition + liftMove
			}
			gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1 -> {
				robot.motors["arm"]!![0].targetPosition = robot.motors["arm"]!![0].currentPosition - liftMove
			}
			else -> robot.motors["arm"]!![0].targetPosition = robot.motors["arm"]!![0].currentPosition
		}
		robot.motors["arm"]!![1].targetPosition = robot.motors["arm"]!![0].targetPosition

		when {
			(gamepad1.dpad_up || gamepad2.left_bumper) && !robot.button!!.state && !liftStop ->
				robot.motors["arm"]!![2].power = 1.0 * speedMultiplier

			gamepad1.dpad_down || gamepad2.left_trigger > 0.1 -> {
				robot.motors["arm"]!![2].power = -1.0 * speedMultiplier
				liftStop = false
			}

			robot.button!!.state -> {
				liftStop = true
				robot.motors["arm"]!![2].power = 0.0
			}

			else -> robot.motors["arm"]!![2].power = 0.0
		}

		when {
			gamepad1.x || gamepad2.x -> robot.roller[0].power = 1.0
			gamepad1.a || gamepad2.a -> robot.roller[0].power = -1.0
			else -> robot.roller[0].power = 0.0
		}
		robot.roller[1].power = robot.roller[0].power

		robot.update(0)
	}

	/*
	 * Code to run ONCE after the driver hits STOP
	 */
	override fun stop() {}
}