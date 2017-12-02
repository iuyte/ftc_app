package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark

/**
 * Created by ethan on 7/29/17.
 */

@Autonomous(name = "Test Auton", group = "Robot")
class AutonTest : LinearOpMode() {

	/* Declare OpMode members. */
	private var robot = Hardware(
			hardwareMap,
			this,
			Hardware.DriveMode.Mecanum,
			true,
			false,
			true
	)

	override fun runOpMode() {
		/*
		 * Initialize the hardware variables.
		 * The init() method of the hardware class does all the work here
		 */
		robot.init(hardwareMap, DcMotor.RunMode.RUN_TO_POSITION, { opModeIsActive() })

		waitForStart()

		robot.relicTrackables!!.activate()
		var vuMark: RelicRecoveryVuMark
		do {
			vuMark = robot.vuMark
			telemetry.addData("VuMark", "%s visible", vuMark)
			robot.update()
		} while (vuMark == RelicRecoveryVuMark.UNKNOWN)
		robot.relicTrackables!!.deactivate()

		robot.setDriveTargets(arrayOf(
				7500.0,
				7500.0,
				7500.0,
				7500.0
		), 0.7)

		robot.linearPositionAngle(.07, 0.0)
	}
}