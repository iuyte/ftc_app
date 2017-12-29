package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor

@TeleOp(name = "Mecanum Base", group = "Drive")
class MecanumBase : LinearOpMode() {
	/* Declare OpMode members. */
	private var robot = Hardware(hardwareMap, this, Hardware.DriveMode.Mecanum, false) // use the class created to define a robot's hardware

	override fun runOpMode() {
		/* Initialize the hardware variables.
		 * The init() method of the hardware class does all the work here
		 */
		robot.init(hardwareMap, DcMotor.RunMode.RUN_WITHOUT_ENCODER)

		// Send telemetry message to signify robot waiting;
		telemetry.addData("Initialization", "Complete")

		val lastPosition = arrayOf(0, 0, 0, 0)
		val drive = robot.motors["drive"]!!
		var roller : Double

		waitForStart()

		while (opModeIsActive()) {
			roller = expandBool(
					gamepad1.x,
					gamepad1.a
			).toDouble()

			// Run the robot drive in mecanum mode
			val x = -gamepad1.left_stick_x.toDouble()
			val y = -gamepad1.left_stick_y.toDouble()
			val z = -gamepad1.right_stick_x.toDouble()

			for (i in 0..3) {
				drive[i].power = robot.mecanum(x, y, z, i)
				val current = drive[i].currentPosition
				telemetry.addData("drive[$i]",
						(current - lastPosition[i]) / 50)
				lastPosition[i] = current
			}

			robot.roller[0].power = roller
			robot.roller[1].power = roller
			robot.update(50)
		}

		drive.forEach { motor ->
			motor.power = 0.0
		}
	}

	private fun expandBool(b1: Boolean, b2: Boolean, high: Number = 1, low: Number = -1): Number {
		when {
			b1 && !b2 -> return high
			b2 -> return low
		}
		return high.toDouble() + low.toDouble()
	}
}
