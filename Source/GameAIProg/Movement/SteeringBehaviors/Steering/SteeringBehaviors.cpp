#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"


//SEEK
//*******
// TODO: Do the Week01 assignment :^)
SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput output{};

	const FVector2D agentPos = FVector2D(Agent.GetActorLocation());
	const FVector2D targetPos = Target.Position;
	FVector2D toTarget = targetPos - agentPos;

	if (toTarget.IsNearlyZero())
	{
		output.LinearVelocity = FVector2D::ZeroVector;
		output.AngularVelocity = 0.f;
		output.IsValid = true;
		return output;
	}

	toTarget.Normalize();
	output.LinearVelocity = toTarget;
	output.AngularVelocity = 0.f;
	output.IsValid = true;

	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugDirectionalArrow(
			Agent.GetWorld(),
			Agent.GetActorLocation(),
			Agent.GetActorLocation() + FVector(output.LinearVelocity, 0.f) * DeltaT,
			30.f,
			FColor::Green,
			false,
			-1.f,
			0,
			2.f
		);
	}

	return output;
}

// FLEE
// Opposite of Seek: move away from target at full speed (direction only)
SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput output{};
	const FVector2D agentPos = Agent.GetPosition();
	const FVector2D targetPos = Target.Position;

	FVector2D away = agentPos - targetPos;
	if (away.IsNearlyZero())
	{
		output.IsValid = false;
		return output;
	}

	output.LinearVelocity = away.GetSafeNormal(); // direction away from target
	output.AngularVelocity = 0.f;
	output.IsValid = true;

	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugDirectionalArrow(
			Agent.GetWorld(),
			Agent.GetActorLocation(),
			Agent.GetActorLocation() + FVector(output.LinearVelocity, 0.f) * (Agent.GetMaxLinearSpeed() * DeltaT),
			30.f,
			FColor::Red,
			false,
			-1.f,
			0,
			2.f
		);
	}

	return output;
}

// ARRIVE
// Slow down when inside slow radius, stop inside target radius.
SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput output{};
	const FVector2D agentPos = Agent.GetPosition();
	const FVector2D targetPos = Target.Position;

	FVector2D toTarget = targetPos - agentPos;
	const float dist = toTarget.Size();

	// If already at target -> stop
	if (dist <= m_TargetRadius || toTarget.IsNearlyZero())
	{
		output.LinearVelocity = FVector2D::ZeroVector;
		output.AngularVelocity = 0.f;
		output.IsValid = false; // no movement required
		return output;
	}

	toTarget.Normalize();

	// Outside slow radius -> full speed (direction = 1.0)
	if (dist >= m_SlowRadius)
	{
		output.LinearVelocity = toTarget; // full speed direction
	}
	else
	{
		// Inside slow radius -> scale speed proportionally
		const float speedFraction = FMath::Clamp(dist / m_SlowRadius, 0.f, 1.f);
		output.LinearVelocity = toTarget * speedFraction;
	}

	output.AngularVelocity = 0.f;
	output.IsValid = true;

	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugDirectionalArrow(
			Agent.GetWorld(),
			Agent.GetActorLocation(),
			Agent.GetActorLocation() + FVector(output.LinearVelocity, 0.f) * (Agent.GetMaxLinearSpeed() * DeltaT),
			30.f,
			FColor::Blue,
			false,
			-1.f,
			0,
			2.f
		);
	}

	return output;
}
// FACE
// Rotate towards the target, produce only angular velocity.
SteeringOutput Face::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput output{};
	const FVector2D agentPos = Agent.GetPosition();
	const FVector2D targetPos = Target.Position;

	const FVector2D toTarget = targetPos - agentPos;
	if (toTarget.IsNearlyZero())
	{
		output.IsValid = false;
		return output;
	}

	// Desired yaw (degrees) from agent to target
	const float desiredYaw = FMath::RadiansToDegrees(FMath::Atan2(toTarget.Y, toTarget.X));
	const float currentYaw = Agent.GetRotation();

	// Shortest signed angle difference in degrees (-180 .. 180)
	const float angleDiff = FMath::FindDeltaAngleDegrees(currentYaw, desiredYaw);

	// If within a small epsilon, consider finished
	if (FMath::Abs(angleDiff) < 1.0f)
	{
		output.LinearVelocity = FVector2D::ZeroVector;
		output.AngularVelocity = 0.f;
		output.IsValid = false;
		return output;
	}

	// Clamp requested angular velocity by agent's max angular speed
	const float clamped = FMath::Clamp(angleDiff, -Agent.GetMaxAngularSpeed(), Agent.GetMaxAngularSpeed());
	output.LinearVelocity = FVector2D::ZeroVector;
	output.AngularVelocity = clamped;
	output.IsValid = true;

	return output;
}
// PURSUIT
// Predict target future position and seek that position.
SteeringOutput Pursuit::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput output{};
	const FVector2D agentPos = Agent.GetPosition();
	const FVector2D targetPos = Target.Position;
	const FVector2D targetVel = Target.LinearVelocity;

	const FVector2D toTarget = targetPos - agentPos;
	const float dist = toTarget.Size();

	// If target is essentially on top of agent
	if (toTarget.IsNearlyZero())
	{
		output.IsValid = false;
		return output;
	}

	const float speed = Agent.GetMaxLinearSpeed();
	float lookAhead = 0.f;
	if (speed > KINDA_SMALL_NUMBER)
	{
		lookAhead = dist / speed;
	}

	const FVector2D predictedPos = targetPos + targetVel * lookAhead;

	// Seek predicted position
	FVector2D dir = predictedPos - agentPos;
	if (dir.IsNearlyZero())
	{
		output.IsValid = false;
		return output;
	}

	dir.Normalize();
	output.LinearVelocity = dir;
	output.AngularVelocity = 0.f;
	output.IsValid = true;

	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugDirectionalArrow(
			Agent.GetWorld(),
			Agent.GetActorLocation(),
			Agent.GetActorLocation() + FVector(output.LinearVelocity, 0.f) * (Agent.GetMaxLinearSpeed() * DeltaT),
			30.f,
			FColor::Cyan,
			false,
			-1.f,
			0,
			2.f
		);
	}

	return output;
}
// EVADE
// Predict target future position and flee from it.
SteeringOutput Evade::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput output{};
	const FVector2D agentPos = Agent.GetPosition();
	const FVector2D targetPos = Target.Position;
	const FVector2D targetVel = Target.LinearVelocity;

	const FVector2D toTarget = targetPos - agentPos;
	const float dist = toTarget.Size();

	// If target is far and outside evade radius, do nothing (optional optimization)
	if (dist > m_EvadeRadius && m_EvadeRadius > 0.f)
	{
		// Not evading right now
		output.IsValid = false;
		return output;
	}

	const float speed = Agent.GetMaxLinearSpeed();
	float lookAhead = 0.f;
	if (speed > KINDA_SMALL_NUMBER)
	{
		lookAhead = dist / speed;
	}

	const FVector2D predictedPos = targetPos + targetVel * lookAhead;

	FVector2D away = agentPos - predictedPos;
	if (away.IsNearlyZero())
	{
		output.IsValid = false;
		return output;
	}

	away.Normalize();
	output.LinearVelocity = away;
	output.AngularVelocity = 0.f;
	output.IsValid = true;

	if (Agent.GetDebugRenderingEnabled())
	{
		DrawDebugDirectionalArrow(
			Agent.GetWorld(),
			Agent.GetActorLocation(),
			Agent.GetActorLocation() + FVector(output.LinearVelocity, 0.f) * (Agent.GetMaxLinearSpeed() * DeltaT),
			30.f,
			FColor::Orange,
			false,
			-1.f,
			0,
			2.f
		);
	}

	return output;
}

// HIDE (placeholder)
SteeringOutput Hide::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput output{};
	// Not implemented: return invalid so caller knows nothing to apply.
	output.IsValid = false;
	return output;
}

// AVOID OBSTACLE (placeholder)
SteeringOutput AvoidObstacle::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput output{};
	// Not implemented yet
	output.IsValid = false;
	return output;
}

// WANDER
// Random point on circle in front of agent becomes target; then Seek to it.
SteeringOutput Wander::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	// Agent position & forward
	const FVector2D agentPos = Agent.GetPosition();
	const float agentYawRad = FMath::DegreesToRadians(Agent.GetRotation());
	const FVector2D forward = FVector2D(FMath::Cos(agentYawRad), FMath::Sin(agentYawRad));

	// Circle center in front of agent
	const FVector2D circleCenter = agentPos + forward * m_OffsetDistance;

	// Update wander angle with limited random change
	const float randomDelta = FMath::FRandRange(-m_MaxAngleChange, m_MaxAngleChange);
	m_WanderAngle += randomDelta;

	// Calculate wander point on circle
	const FVector2D wanderDir = FVector2D(FMath::Cos(m_WanderAngle), FMath::Sin(m_WanderAngle));
	const FVector2D wanderPoint = circleCenter + wanderDir * m_Radius;

	// Set as target and seek it
	Target.Position = wanderPoint;
	SteeringOutput output = Seek::CalculateSteering(DeltaT, Agent);

	if (Agent.GetDebugRenderingEnabled())
	{
		// Draw circle center and target point for visualization
		DrawDebugSphere(Agent.GetWorld(), FVector(circleCenter, 0.f), 10.f, 8, FColor::Purple, false, -1.f, 0, 1.f);
		DrawDebugDirectionalArrow(
			Agent.GetWorld(),
			Agent.GetActorLocation(),
			Agent.GetActorLocation() + FVector(output.LinearVelocity, 0.f) * (Agent.GetMaxLinearSpeed() * DeltaT),
			20.f,
			FColor::Purple,
			false,
			-1.f,
			0,
			1.5f
		);
	}

	return output;
}