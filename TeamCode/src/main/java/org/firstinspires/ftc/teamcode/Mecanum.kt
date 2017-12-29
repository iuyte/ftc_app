package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor

@TeleOp(name = "Mecanum", group = "Drive")
class Mecanum: LinearOpMode() {
	/* Declare OpMode members. */
	private var robot = Hardware(hardwareMap, this, Hardware.DriveMode.Mecanum, false) // use the class created to define a robot's hardware
	private var speedMultiplier = 0.0
	private val liftPower = 0.8
	private val liftMove = 100
	private var liftStop = false

	override fun runOpMode() {
		/* Initialize the hardware variables.
		 * The init() method of the hardware class does all the work here
		 */
		robot.init(hardwareMap, DcMotor.RunMode.RUN_USING_ENCODER)
		for (i in 0..1) {
			robot.motors["arm"]!![i].mode = DcMotor.RunMode.RUN_TO_POSITION
			robot.motors["arm"]!![i].power = liftPower
		}

		val drive = robot.motors["drive"]!!
		val arm = robot.motors["arm"]!!
		val button = robot.button!!

		// Send telemetry message to signify robot waiting;
		telemetry.addData("Initialization", "Complete")

		waitForStart()

		while (opModeIsActive()) {
			speedMultiplier = 1.0 - (gamepad1.left_trigger.toDouble() * 0.45) -
					(if (gamepad2.b) 0.0 else 0.3)
			val armPosition = arrayOf(
					arm[0].currentPosition,
					arm[1].currentPosition,
					arm[2].currentPosition)

			// Run the robot drive in mecanum mode
			val x = -gamepad1.left_stick_x.toDouble()
			val y = -gamepad1.left_stick_y.toDouble()
			val z = -gamepad1.right_stick_x.toDouble()

			for (i in 0..3) {
				drive[i].power = robot.mecanum(x, y, z, i)
			}

			when {
				gamepad1.right_bumper || gamepad2.right_bumper -> {
					arm[0].targetPosition = armPosition[0] + liftMove
				}
				gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1 -> {
					arm[0].targetPosition = armPosition[0] - liftMove
				}
				else -> arm[0].targetPosition = armPosition[0]
			}
			arm[1].targetPosition = arm[0].targetPosition

			when {
				(gamepad1.dpad_up || gamepad2.left_bumper) && !button.state && !liftStop ->
					arm[2].power = 1.0 * speedMultiplier

				gamepad1.dpad_down || gamepad2.left_trigger > 0.1 -> {
					arm[2].power = -1.0 * speedMultiplier
					liftStop = false
				}

				button.state -> {
					liftStop = true
					arm[2].power = 0.0
				}

				else -> arm[2].power = 0.0
			}

			when {
				gamepad1.x || gamepad2.x -> robot.roller[0].power = 1.0
				gamepad1.a || gamepad2.a -> robot.roller[0].power = -1.0
				else -> robot.roller[0].power = 0.0
			}
			robot.roller[1].power = robot.roller[0].power

			robot.update(25)
		}

		drive.forEach { motor ->
			motor.power = 0.0
		}
	}
}
