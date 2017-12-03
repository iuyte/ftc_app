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
		public  var useVuforia: Boolean = false,
		private var useMotors: Boolean = true) {
	// Time elapsed
	private val period = ElapsedTime()
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
	// private var cameraMonitorViewId : Int? = null
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

	// Motor power multiplier for the front motors when strafing (helps to correct heavy back)
	private val frontStrafe = 1.0

	/* The different drive modes */
	enum class DriveMode {
		Tank,
		Holonomic,
		Mecanum,
	}

	/* Initialize standard Hardware interfaces */
	fun init(awMap: HardwareMap?, motorMode: DcMotor.RunMode = DcMotor.RunMode.RUN_USING_ENCODER, active: () -> Boolean = { false }) {
		try {
			hwMap = awMap
			isActive = active
			mode.telemetry!!.addData("Robot", "Initializing")

			button = hwMap!!.digitalChannel["button"]
			button!!.mode = DigitalChannel.Mode.INPUT

			if (useSensors) {
				// Initialize sensors
				armColor = hwMap!!.get(ColorSensor::class.java, "rev color")
				button = hwMap!!.get(DigitalChannel::class.java, "button")
				revButton = hwMap!!.get(DigitalChannel::class.java, "rButton")
			}

			if (newSensors) {
				armColor = hwMap!!.colorSensor["arm color"]
				distance = hwMap!!.get(DistanceSensor::class.java, "arm color")
				jewelColor = hwMap!!.colorSensor["jewel color"]

				val parameters = BNO055IMU.Parameters()
				parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES
				parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
				parameters.loggingEnabled = true
				parameters.useExternalCrystal = true
				parameters.mode = BNO055IMU.SensorMode.IMU
				parameters.calibrationDataFile = "BNO055IMUCalibration.json"
				parameters.loggingTag = "IMU"
				parameters.accelerationIntegrationAlgorithm = JustLoggingAccelerationIntegrator()
				imu = hwMap!!.get(BNO055IMU::class.java, "imu")
				imu!!.initialize(parameters)
			}
		} catch (_: KotlinNullPointerException) {
			mode.telemetry!!.addData("Sensors", "Error initializing")
		}

		try {
			if (useMotors) {
				// Define and Initialize Motors
				motors = mapOf(
						Pair("drive", arrayOf(
								hwMap!!.dcMotor["front left drive"]!!,
								hwMap!!.dcMotor["front right drive"]!!,
								hwMap!!.dcMotor["back left drive"]!!,
								hwMap!!.dcMotor["back right drive"]!!
						)),

						Pair("arm", arrayOf(
								hwMap!!.dcMotor["left lift motor"]!!,
								hwMap!!.dcMotor["right lift motor"]!!,
								hwMap!!.dcMotor["claw control motor"]!!
						))
				)

				jewel = hwMap!!.crservo["jewel vex"]
				roller = listOf(
						hwMap!!.crservo["left roller vex"]!!,
						hwMap!!.crservo["right roller vex"]!!
				)

				roller[1].direction = DcMotorSimple.Direction.REVERSE

				// Set all motors to zero power and to run without encoders
				for ((_, motorList) in motors) {
					motorList.forEach {
						it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
					}
				}

				Thread.sleep(50)

				for ((_, motorList) in motors) {
					motorList.forEach {
						it.mode = motorMode
						it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
						it.power = 0.0
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
			// cameraMonitorViewId = hwMap!!.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hwMap!!.appContext.packageName)
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
		motors["drive"]!![0].power = upLeft
		motors["drive"]!![1].power = upright
		motors["drive"]!![2].power = downLeft
		motors["drive"]!![3].power = downright
	}

	private fun holonomic(x: Double, y: Double, θ: Double) {
		setDrive(
				-x + y - θ,
				x + y + θ,
				x + y - θ,
				-x + y + θ
		)
	}

	private fun mecanum(x: Double, y: Double, θ: Double) {
		val fl = x * frontStrafe + y - θ
		val fr = -x * frontStrafe + y + θ
		val bl = -x + y - θ
		val br = x + y + θ
		setDrive(fl, fr, bl, br)
	}

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

	fun setDriveTargets(positions: Array<Double>, power : Double? = null) {
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
		if (newSensors) {
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

		mode.telemetry.addData("Button", button!!.state)
		mode.telemetry.update()
		waitForTick(ms.toLong())
	}

	/***

	 * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
	 * periodic tick.  This is used to compensate for varying processing times for each cycle.
	 * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.

	 * @param periodMs Length of wait cycle in mSec.
	 */
	fun waitForTick(periodMs: Long) {
		val remaining = periodMs - period.milliseconds().toLong()

		// sleep for the remaining portion of the regular cycle period.
		if (remaining > 0) {
			try {
				Thread.sleep(remaining)
			} catch (e: InterruptedException) {
				Thread.currentThread().interrupt()
			}
		}

		// Reset the cycle clock for the next pass.
		period.reset()
	}
}