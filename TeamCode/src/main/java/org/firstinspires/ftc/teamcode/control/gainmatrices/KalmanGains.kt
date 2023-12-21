package org.firstinspires.ftc.teamcode.control.gainmatrices

data class KalmanGains @JvmOverloads constructor(
    @JvmField var sensorGain: Double = 0.0,
    @JvmField var modelGain: Double = 0.0,
    @JvmField var count: Int = 2,
)
