package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor

/**
 * Created by ethan on 11/4/2017.
 */

@Autonomous(name = "Test", group = "Motors")
class TestMotors : LinearOpMode() {

	private var robot = Hardware(hardwareMap, this, Hardware.DriveMode.Mecanum, false, false) // use the class created to define a robot's hardware

	override fun runOpMode() {
		robot.init(hardwareMap, DcMotor.RunMode.RUN_WITHOUT_ENCODER)
		telemetry.addData(">", "Press Play to start")
		telemetry.update()
		waitForStart()

		while (opModeIsActive()) {
			robot.motors["drive"]!!.forEach {
				it.power = .3
				Thread.sleep(2000)
				it.power = 0.0
				Thread.sleep(2000)
			}

			Thread.sleep(4000)
			telemetry.update()
		}
	}
}