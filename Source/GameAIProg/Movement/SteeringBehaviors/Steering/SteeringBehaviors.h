#pragma once

#include <Movement/SteeringBehaviors/SteeringHelpers.h>
#include "Kismet/KismetMathLibrary.h"

class ASteeringAgent;

// SteeringBehavior base, all steering behaviors should derive from this.
class ISteeringBehavior
{
public:
	ISteeringBehavior() = default;
	virtual ~ISteeringBehavior() = default;

	// Override to implement your own behavior
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent & Agent) = 0;

	void SetTarget(const FTargetData& NewTarget) { Target = NewTarget; }
	
	template<class T, std::enable_if_t<std::is_base_of_v<ISteeringBehavior, T>>* = nullptr>
	T* As()
	{ return static_cast<T*>(this); }

protected:
	FTargetData Target;
};
//SEEK
//****
class Seek : public ISteeringBehavior
{
public:
	Seek() = default;
	~Seek() override = default;

	//Seek Behaviour
	SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
};


///////////////////////////////////////
//FLEE
//****
class Flee : public ISteeringBehavior
{
public:
	Flee() = default;
	~Flee() override = default;

	//Flee Behaviour
	SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
};

///////////////////////////////////////
//ARRIVE
//****
class Arrive : public ISteeringBehavior
{
public:
	Arrive() = default;
	~Arrive() override = default;

	//Arrive Behaviour
	SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
	void SetTargetRadius(float radius) { m_TargetRadius = radius; }
	void SetSlowRadius(float radius) { m_SlowRadius = radius; }
private:
	float m_SlowRadius = 15.0f;
	float m_TargetRadius = 3.0f;
};

///////////////////////////////////////
//Face
//****
class Face : public ISteeringBehavior
{
public:
	Face() = default;
	~Face() override = default;

	//Face Behaviour
	SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
private:
};

///////////////////////////////////////
//Pursuit
//****
class Pursuit : public Seek
{
public:
	Pursuit() = default;
	~Pursuit() override = default;

	//Pursuit Behaviour
	SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
private:
};

///////////////////////////////////////
//Evade
//****
class Evade : public Flee
{
public:
	Evade(float evadeRadius = 15.f) : m_EvadeRadius(evadeRadius) {}
	~Evade() override = default;

	//Evade Behaviour
	SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
private:
	float m_EvadeRadius = 15.f;
};

///////////////////////////////////////
//Hide
//****
class Hide : public ISteeringBehavior
{
public:
	Hide() = default;
	~Hide() override = default;

	//Hide Behaviour
	SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
private:
};

///////////////////////////////////////
//AvoidObstacle
//****
class AvoidObstacle : public ISteeringBehavior
{
public:
	AvoidObstacle() = default;
	~AvoidObstacle() override = default;

	//AvoidObstacle Behaviour
	SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;
private:
};

///////////////////////////////////////
//AvoidObstacle
//****
class Wander : public Seek
{
public:
	Wander() = default;
	~Wander() override = default;

	//Wander Behaviour
	SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& Agent) override;

	void SetWanderOffset(float offset) { m_OffsetDistance = offset; }
	void SetWanderRadius(float radius) { m_Radius = radius; }
	void SetMaxAngleChange(float rad) { m_MaxAngleChange = rad; }

protected:
	float m_OffsetDistance = 6.f; //Offset (Agent Direction)
	float m_Radius = 4.f; //Circle Radius
	float m_MaxAngleChange = FMath::DegreesToRadians(45.f); //Max Wander Angle change per frame
	float m_WanderAngle = FMath::FRandRange(-PI, PI); //Internal
private:
};
