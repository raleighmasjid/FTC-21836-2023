package org.firstinspires.ftc.teamcode.control.gainmatrices

import kotlin.Double.Companion.POSITIVE_INFINITY
import kotlin.math.*

data class PIDGains

@JvmOverloads
constructor(
    @JvmField var kP: Double = 0.0,
    @JvmField var kI: Double = 0.0,
    @JvmField var kD: Double = 0.0,
    @JvmField var maxOutputWithIntegral: Double = POSITIVE_INFINITY,
) {

    @JvmOverloads
    fun criticallyDamp(gains: FeedforwardGains, percentOver: Double = 0.0): PIDGains {
        kP = max(gains.kV.pow(2) / (4 * gains.kA), kP)
        val overshoot: Double = percentOver / 100.0
        val zeta: Double = if (overshoot <= 0.0) 1.0 else -ln(overshoot) / sqrt(PI.pow(2) + ln(overshoot).pow(2))
        kD = 2 * zeta * sqrt(gains.kA * kP) - gains.kV
        return this
    }

    fun copyFrom(other:PIDGains): PIDGains {
        kP = other.kP
        kI = other.kI
        kD = other.kD
        maxOutputWithIntegral = other.maxOutputWithIntegral
        return this
    }
}
