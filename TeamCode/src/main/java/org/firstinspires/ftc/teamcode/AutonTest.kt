package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark

/**
 * Created by ethan on 7/29/17.
 */

@Autonomous(name = "Test Auton", group = "Robot")
class AutonTest: LinearOpMode() {

	/* Declare OpMode members. */
	private var robot = Hardware(
			hardwareMap,
			this,
			Hardware.DriveMode.Mecanum,
			true,
			false,
			false
	)

	override fun runOpMode() {
		/*
		 * Initialize the hardware variables.
		 * The init() method of the hardware class does all the work here
		 */
		robot.init(hardwareMap, DcMotor.RunMode.RUN_TO_POSITION, ::opModeIsActive)

		waitForStart()

		if (robot.useVuforia) {
			robot.relicTrackables!!.activate()
			var vuMark: RelicRecoveryVuMark
			do {
				vuMark = robot.vuMark
				telemetry.addData("VuMark", "%s visible", vuMark)
				robot.update()
			} while (vuMark == RelicRecoveryVuMark.UNKNOWN)
			robot.relicTrackables!!.deactivate()
		}

		val arm = robot.motors["arm"]!![0]
		arm.power = 1.0
		arm.targetPosition = 200

		robot.roller[0].power = 1.0
		robot.roller[1].power = 1.0

		robot.setDriveTargets(arrayOf(
				2300.0,
				2300.0,
				2300.0,
				2300.0
		), 1.0)

		do {
			var isBusy = false
			robot.motors["drive"]!!.forEach { motor ->
				if (motor.isBusy) {
					isBusy = true
				} else {
					robot.motors["drive"]!![1].power = 0.0
					robot.motors["drive"]!![3].power = 0.0
				}
			}
			robot.update()
		} while (opModeIsActive() && isBusy)

		Thread.sleep(3500)

		robot.setDriveTargets(arrayOf(
				2000.0,
				2000.0,
				2000.0,
				2000.0
		))

		do {
			var isBusy = false
			robot.motors["drive"]!!.forEach { motor ->
				if (motor.isBusy) {
					isBusy = true
				} else {
					robot.motors["drive"]!![1].power = 0.0; robot.motors["drive"]!![3].power = 0.0
				}
			}
			robot.update()
		} while (opModeIsActive() && isBusy)

		while (opModeIsActive()) {
			robot.update()
		}
	}
}