package com.acmerobotics.roadrunner.followers

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.drive.Kinematics
import com.acmerobotics.roadrunner.drive.TankDrive
import com.acmerobotics.roadrunner.drive.TankKinematics
import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.profile.SimpleMotionConstraints
import com.acmerobotics.roadrunner.util.Angle
import com.acmerobotics.roadrunner.util.NanoClock
import kotlin.math.sqrt

/**
 * State-of-the-art path follower based on the [GuidingVectorField].
 *
 * @param drive tank drive
 * @param constraints robot motion constraints
 * @param admissibleError admissible/satisfactory pose error at the end of each move
 * @param kN normal vector weight (see [GuidingVectorField])
 * @param kOmega proportional heading gain
 * @param kV feedforward velocity gain
 * @param kA feedforward acceleration gain (currently unused)
 * @param kStatic additive feedforward constant (used to overcome static friction)
 * @param errorMapFunc error map function (see [GuidingVectorField])
 * @param clock clock
 */
class GVFFollower @JvmOverloads constructor(
        private val drive: TankDrive,
        private val constraints: SimpleMotionConstraints,
        admissibleError: Pose2d,
        private val kN: Double,
        private val kOmega: Double,
        private val kV: Double,
        private val kA: Double,
        private val kStatic: Double,
        private val errorMapFunc: (Double) -> Double = { it },
        clock: NanoClock = NanoClock.system()
) : PathFollower(admissibleError, clock) {
    private lateinit var gvf: GuidingVectorField
    private var lastUpdateTimestamp: Double = 0.0
    private var lastVelocity: Double = 0.0
    private var lastProjectionDisplacement: Double = 0.0

    override var lastError: Pose2d = Pose2d()

    override fun followPath(path: Path) {
        gvf = GuidingVectorField(path, kN, errorMapFunc)
        lastUpdateTimestamp = clock.seconds()
        lastVelocity = 0.0
        lastProjectionDisplacement = 0.0
        super.followPath(path)
    }

    override fun update(currentPose: Pose2d) {
        super.update(currentPose)

        if (!isFollowing()) {
            drive.setMotorPowers(0.0, 0.0)
            return
        }

        val gvfResult = gvf.getExtended(currentPose.x, currentPose.y, lastProjectionDisplacement)

        val desiredHeading = Math.atan2(gvfResult.vector.y, gvfResult.vector.x)
        val headingError = Angle.norm(currentPose.heading - desiredHeading)

        // TODO: implement this or nah? ref eqs. (18), (23), and (24)
        val desiredOmega = 0.0
        val omega = desiredOmega - kOmega * headingError

        // basic online motion profiling
        val timestamp = clock.seconds()
        val dt = timestamp - lastUpdateTimestamp
        val remainingDistance = currentPose.pos() distanceTo path.end().pos()
        val maxVelToStop = sqrt(2 * constraints.maximumAcceleration * remainingDistance)
        val maxVelFromLast = lastVelocity + constraints.maximumAcceleration * dt
        val velocity = minOf(maxVelFromLast, maxVelToStop, constraints.maximumVelocity)

        val wheelVelocities = TankKinematics.robotToWheelVelocities(Pose2d(velocity, 0.0, omega), drive.trackWidth)

        val motorPowers = Kinematics.calculateMotorFeedforward(wheelVelocities, wheelVelocities.map { 0.0 }, kV, kA, kStatic)

        drive.setMotorPowers(motorPowers[0], motorPowers[1])

        lastUpdateTimestamp = timestamp
        lastVelocity = velocity
        lastProjectionDisplacement = gvfResult.displacement

        val targetPose = path[gvfResult.displacement]

        lastError = Kinematics.calculatePoseError(targetPose, currentPose)
    }

}