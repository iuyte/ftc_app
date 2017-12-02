package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import java.util.Locale

@TeleOp(name = "IMUtest", group = "Robot")
class IMUtest : LinearOpMode() {
	private var imu: BNO055IMU? = null

	private var angles: Orientation? = null
	private var gravity: Acceleration? = null

	override fun runOpMode() {

		val parameters = BNO055IMU.Parameters()
		parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES
		parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
		parameters.loggingEnabled = true
		parameters.useExternalCrystal = true
		parameters.mode = BNO055IMU.SensorMode.IMU
		parameters.calibrationDataFile = "BNO055IMUCalibration.json"
		parameters.loggingTag = "IMU"
		parameters.accelerationIntegrationAlgorithm = JustLoggingAccelerationIntegrator()
		imu = hardwareMap.get(BNO055IMU::class.java, "imu")

		imu!!.initialize(parameters)
		telemetry.msTransmissionInterval = 100
		waitForStart()

		while (opModeIsActive()) {
			angles = imu!!.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)
			gravity = imu!!.gravity
			sendTelemetry()
		}
	}

	internal fun sendTelemetry() {
		telemetry.addData("Status", imu!!.systemStatus.toString())
		telemetry.addData("Calib", imu!!.calibrationStatus.toString())
		telemetry.addData("Heading", formatAngle(angles!!.angleUnit, angles!!.firstAngle.toDouble()))
		telemetry.addData("Roll", formatAngle(angles!!.angleUnit, angles!!.secondAngle.toDouble()))
		telemetry.addData("Pitch", formatAngle(angles!!.angleUnit, angles!!.thirdAngle.toDouble()))
		telemetry.addData("Grav", gravity!!.toString())
		telemetry.addData(
				"Accel", "%.2f, %.2f, %.2f",
				imu!!.acceleration.xAccel,
				imu!!.acceleration.yAccel,
				imu!!.acceleration.zAccel)
		telemetry.update()
	}

	internal fun formatAngle(angleUnit: AngleUnit, angle: Double): String {
		return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle))
	}

	internal fun formatDegrees(degrees: Double): String {
		return String.format(Locale.getDefault(), "%.2f", AngleUnit.DEGREES.normalize(degrees))
	}
}