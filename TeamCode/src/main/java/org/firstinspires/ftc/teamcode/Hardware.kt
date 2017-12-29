package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables

/**
 * Created by ethan on 7/29/17.
 */

/* Makes moving with the robot easier */
class Hardware(
		private var hwMap: HardwareMap?,
		private var mode: OpMode,
		private var driveMode: DriveMode,
		private var newSensors: Boolean = false,
		private var useSensors: Boolean = false,
		var useVuforia: Boolean = false,
		private var useMotors: Boolean = true) {
	// Time elapsed
	val time = ElapsedTime()
	private var isActive: () -> Boolean = {
		false
	}
	// Color sensor
	var armColor: ColorSensor? = null
	var jewelColor: ColorSensor? = null
	var button: DigitalChannel? = null
	var revButton: DigitalChannel? = null
	var distance: DistanceSensor? = null

	// Vuforia members
	var vuforia: VuforiaLocalizer? = null
	// private var cameraMonitorViewId  Int? = null
	private var vuforiaParams: VuforiaLocalizer.Parameters? = null
	private val vuforiaKey =
			"AQ+eJzP/////AAAAGQ6Eyl3tQ0fqvcl2vinKWkR8t2wTESOSuHo32BhQvWxRBBxt+4C2BtrXTh" +
					"Ygcyd9fzaZ1tTjW8viQzNJAVYyL50GIG+hKJb6mHRq/iYJ53ve+oETR1t5ZBaVmq2l" +
					"qljNaxyRJURYSLp52UoNi2RpvW0xvVIZ4KBoClIUI4KRkiozACNg0GsxDIDnAiBx5i" +
					"iPfug5PklSyPnKHkhKf+mtRhoY8HcittFKh4lQkI5LtJOm6lH/K6CCa7RzVUkRbI9m" +
					"bxVybXiOLtpN/Yr+FDS0bM19czvflS+URyrO2J5APOipQ1XZ1wieP/wp75NFXkV0V3" +
					"hWcVAkWHQr9aJhZtkYUGjrb94QP7OOF7h6Ftl+ls2j"

	// VuMark specific
	var relicTrackables: VuforiaTrackables? = null
	private var relicTemplate: VuforiaTrackable? = null

	// Colors
	enum class COLORS {
		RED,
		BLUE,
		NONE,
	}

	// The IMU
	private var imu: BNO055IMU? = null
	private var orientation: Orientation? = null

	/* The motors */
	var motors: Map<String, Array<DcMotor>> = mapOf()

	var jewel: CRServo? = null
	var roller: List<CRServo> = listOf()

	/* The different drive modes */
	enum class DriveMode {
		Tank,
		Holonomic,
		Mecanum,
	}

	/* Initialize standard Hardware interfaces */
	fun init(awMap: HardwareMap?, motorMode: DcMotor.RunMode = DcMotor.RunMode.RUN_USING_ENCODER, active: () -> Boolean = { false }) {
		val thwMap = awMap!!
		try {
			hwMap = awMap
			isActive = active
			mode.telemetry!!.addData("Robot", "Initializing")

			button = thwMap.digitalChannel["button"]
			button!!.mode = DigitalChannel.Mode.INPUT

			if (useSensors) {
				// Initialize sensors
				armColor = thwMap.get(ColorSensor::class.java, "rev color")
				button = thwMap.get(DigitalChannel::class.java, "button")
				revButton = thwMap.get(DigitalChannel::class.java, "rButton")
			}

			if (newSensors) {
				armColor = thwMap.colorSensor["arm color"]
				distance = thwMap.get(DistanceSensor::class.java, "arm color")
				jewelColor = thwMap.colorSensor["jewel color"]

				val parameters = BNO055IMU.Parameters()
				parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES
				parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
				parameters.loggingEnabled = true
				parameters.useExternalCrystal = true
				parameters.mode = BNO055IMU.SensorMode.IMU
				parameters.calibrationDataFile = "BNO055IMUCalibration.json"
				parameters.loggingTag = "IMU"
				parameters.accelerationIntegrationAlgorithm = JustLoggingAccelerationIntegrator()
				imu = thwMap.get(BNO055IMU::class.java, "imu")
				imu!!.initialize(parameters)
			}
		} catch (_: KotlinNullPointerException) {
			mode.telemetry!!.addData("Sensors", "Error initializing")
		}

		try {
			if (useMotors) {
				// Define and Initialize Motors
				motors = mapOf(
						Pair("drive", arrayOf<DcMotor>(
								thwMap.dcMotor["front left drive"]!!,
								thwMap.dcMotor["front right drive"]!!,
								thwMap.dcMotor["back left drive"]!!,
								thwMap.dcMotor["back right drive"]!!
						)),

						Pair("arm", arrayOf(
								thwMap.dcMotor["left lift motor"]!!,
								thwMap.dcMotor["right lift motor"]!!,
								thwMap.dcMotor["claw control motor"]!!
						))
				)

				jewel = thwMap.crservo["jewel vex"]
				roller = listOf(
						thwMap.crservo["left roller vex"]!!,
						thwMap.crservo["right roller vex"]!!
				)

				roller[1].direction = DcMotorSimple.Direction.REVERSE

				// Set all motors to zero power and to run without encoders
				for ((_, motorList) in motors) {
					motorList.forEach { motor ->
						motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
					}
				}

				Thread.sleep(50)

				for ((_, motorList) in motors) {
					motorList.forEach { motor ->
						motor.mode = motorMode
						motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
						motor.power = 0.0
					}
				}

				motors["drive"]!![0].direction = DcMotorSimple.Direction.REVERSE
				motors["drive"]!![1].direction = DcMotorSimple.Direction.FORWARD
				motors["drive"]!![2].direction = DcMotorSimple.Direction.REVERSE
				motors["drive"]!![3].direction = DcMotorSimple.Direction.FORWARD
				motors["arm"]!![0].direction = DcMotorSimple.Direction.REVERSE
				motors["arm"]!![1].direction = DcMotorSimple.Direction.FORWARD
				motors["arm"]!![2].direction = DcMotorSimple.Direction.REVERSE
			}
		} catch (_: KotlinNullPointerException) {
			mode.telemetry!!.addData("Motors", "Error initializing")
		}

		if (useVuforia) {
			// Vuforia initialization, setup
			// cameraMonitorViewId = thwMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", thwMap.appContext.packageName)
			vuforiaParams = VuforiaLocalizer.Parameters(/* cameraMonitorViewId!! */)
			vuforiaParams!!.vuforiaLicenseKey = vuforiaKey
			vuforiaParams!!.cameraDirection = VuforiaLocalizer.CameraDirection.BACK
			this.vuforia = ClassFactory.createVuforiaLocalizer(vuforiaParams)

			relicTrackables = this.vuforia!!.loadTrackablesFromAsset("RelicVuMark")
			relicTemplate = relicTrackables!![0]
			relicTemplate!!.name = "relicVuMarkTemplate"
		}

		mode.telemetry.addData("Robot", "Initialized!")
	}

	fun setDrive(upLeft: Double, upright: Double, downLeft: Double, downright: Double) {
		val drive = motors["drive"]!!
		drive[0].power = upLeft
		drive[1].power = upright
		drive[2].power = downLeft
		drive[3].power = downright
	}

	fun holonomic(x: Double, y: Double, θ: Double, index: Int): Double = when (index) {
			0 -> -x + y - θ
			1 ->  x + y + θ
			2 ->  x + y - θ
			3 ->  -x + y + θ
			else ->  0.0
		}

	fun mecanum(x: Double, y: Double, θ: Double, index: Int): Double = when (index) {
		0 ->  x + y - θ
		1 -> -x + y + θ
		2 -> -x + y - θ
		3 ->  x + y + θ
		else ->  0.0
	}

	/*
	fun drive(x: Number, y: Number, θ: Number = 0) {
		val xx = x.toDouble()
		val yy = y.toDouble()
		val theta = θ.toDouble()

		when (driveMode) {
			DriveMode.Holonomic -> holonomic(xx, yy, theta)
			DriveMode.Mecanum -> mecanum(xx, yy, theta)
			DriveMode.Tank -> setDrive(xx, xx, yy, yy)
		}
	}
	*/

	fun setDriveTargets(positions: Array<Double>, power: Double? = null) {
		val drive = motors["drive"]!!

		for (i in 0 until motors["drive"]!!.size) {
			drive[i].targetPosition = positions[i].toInt()
			if (power != null) {
				drive[i].power = power
			}
		}
	}

	fun linearPositionAngle(multiplier: Double, target: Double, tolerance: Long = 7) {
		val change = multiplier * (target - angle)
		val drive = motors["drive"]!!

		drive[0].power -= change
		drive[1].power += change
		drive[2].power -= change
		drive[3].power += change

		drive.forEach { motor ->
			while (Math.abs(motor.targetPosition - motor.currentPosition) > tolerance && isActive()) {
				update()
				linearPositionAngle(multiplier, target)
			}

			if (!isActive()) {
				return@linearPositionAngle
			}
		}
	}

	fun color(c: ColorSensor? = armColor): COLORS {
		val rgb = arrayOf(
				c!!.red(),
				c.blue(),
				c.green())
		if (rgb[0] > rgb[2] + 25) {
			return COLORS.RED
		} else if (rgb[2] > rgb[0] + 15) {
			return COLORS.BLUE
		}
		return COLORS.NONE
	}

	val vuMark: RelicRecoveryVuMark
		get() {
			return RelicRecoveryVuMark.from(relicTemplate)
		}
	val angle: Double
		get() {
			return orientation!!.firstAngle.toDouble()
		}
	val batteryVoltage: Double
		get() {
			var result = java.lang.Double.POSITIVE_INFINITY
			hwMap!!.voltageSensor.forEach { sensor ->
				val voltage = sensor.voltage
				if (voltage > 0) {
					result = Math.min(result, voltage)
				}
			}
			return result
		}

	fun update(ms: Number = 25) {
		if (false && newSensors) {
			orientation = imu!!.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)!!
			mode.telemetry
					.addData("Angle", angle)
					.addData("Jewel", color(jewelColor))
					.addData("Glyph", color())
		}

		var i: Int
		for ((key, list) in motors) {
			i = 0
			list.forEach { motor ->
				mode.telemetry.addData("$key[${i++}]",
						"${motor.currentPosition}")
			}
		}

		// mode.telemetry.addData("Button", button!!.state)
		mode.telemetry.update()
		waitForTick(ms.toLong())
	}

	/***

	 * waitForTick implements a timeic delay. However, this acts like a metronome with a regular
	 * timeic tick.  This is used to compensate for varying processing times for each cycle.
	 * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.

	 * @param timeMs Length of wait cycle in mSec.
	 */
	fun waitForTick(timeMs: Long) {
		val remaining = timeMs - time.milliseconds().toLong()

		// sleep for the remaining portion of the regular cycle time.
		if (remaining > 0) {
			try {
				Thread.sleep(remaining)
			} catch (e: InterruptedException) {
				Thread.currentThread().interrupt()
			}
		}

		// Reset the cycle clock for the next pass.
		time.reset()
	}
}