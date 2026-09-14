using System.Collections.Generic;

namespace SmartAutopilot.Navigation;

internal enum FlightPhase
{
    Cruise,
    Detour,
    Escape,
    Braking,
    Hold,
    Arrived
}

internal sealed class Obstacle
{
    public int    PrimaryIndex = -1;
    public Vector Position;
    public Vector Velocity;
    public Vector Acceleration;
    public double Radius;
    public double PhysicalRadius;
}

internal readonly struct ArrivalThrustLimit
{
    public readonly double Radius;
    public readonly double Thrust;

    public ArrivalThrustLimit(double radius, double thrust)
    {
        Radius = radius;
        Thrust = thrust;
    }
}

internal sealed class FlightState
{
    public Vector Position;
    public Vector Velocity;
    public Vector ExternalAcceleration;
    public Vector TargetPosition;
    public Vector TargetVelocity;
    public Vector TargetAcceleration;
    public double ArrivalRadius;
    public double Thrust;
    public double MaximumThrust;
    public int    TargetObstacleIndex = -1;
    public double SpeedLimit;
    public double Time;
    public double DeltaTime = 0.02;

    public readonly List<Obstacle>           Obstacles           = [];
    public readonly List<ArrivalThrustLimit> ArrivalThrustLimits = [];
}

internal readonly struct FlightCommand
{
    public readonly Vector      Acceleration;
    public readonly FlightPhase Phase;

    public FlightCommand(Vector acceleration, FlightPhase phase)
    {
        Acceleration = acceleration;
        Phase        = phase;
    }
}
