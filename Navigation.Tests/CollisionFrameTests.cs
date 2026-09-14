using System;
using SmartAutopilot.Navigation;

internal static class CollisionFrameTests
{
    public static void DistantRotation()
    {
        var offset = new Vector(10000, 0, 0);
        var bodyVelocity = new Vector(100, 0, -200);
        var spin = new Vector(0, 0.06, 0);
        var state = new FlightState
        {
            Position             = offset,
            Velocity             = bodyVelocity + new Vector(135, 0, 0),
            ExternalAcceleration = new Vector(-1.6, 0, -1.5),
            Thrust               = 50,
            MaximumThrust        = 50
        };
        var extendedSurface = CollisionFrame.Create(offset, bodyVelocity, default, spin, default, 10000);
        var frame = CollisionFrame.Create(offset, bodyVelocity, default, spin, default, 350);
        double oldDistance = CollisionBraking.ProbeDistance(state, extendedSurface.Velocity, extendedSurface.Acceleration);
        double newDistance = CollisionBraking.ProbeDistance(state, frame.Velocity, frame.Acceleration);
        Require(oldDistance > 10000 && newDistance < 500, "Fixture did not remove the spurious multi-kilometre rotating-frame probe");
        Require(frame.RotationWeight == 0 && (frame.Velocity - bodyVelocity).Length == 0 && frame.Acceleration.Length == 0,
            "A distant collision frame retained surface rotation");
        state.Velocity = bodyVelocity;
        state.ExternalAcceleration = default;
        Require(CollisionBraking.Stop(state, frame.Velocity, frame.Acceleration).Acceleration.Length == 0,
            "A distant hold accelerated a velocity-matched ship into a rotating surface frame");
        state.Velocity = bodyVelocity + new Vector(1200, 0, 0);
        state.Thrust = 20;
        Require(CollisionBraking.ProbeDistance(state, frame.Velocity, frame.Acceleration) > 36000,
            "Fixing distant rotation capped a genuine high-speed stopping distance");
        Console.WriteLine($"COLLISION FRAME: rotating probe={oldDistance:F1}m; translational probe={newDistance:F1}m");
    }

    public static void SurfaceFrame()
    {
        var velocity = new Vector(100, 20, -30);
        var acceleration = new Vector(2, -1, 0);
        var spin = new Vector(0, 0.02, 0);
        var angularAcceleration = new Vector(0, 0.001, 0);
        var near = CollisionFrame.Create(new Vector(380, 0, 0), velocity, acceleration, spin, angularAcceleration, 350);
        Require(near.RotationWeight == 1 && (near.Velocity - velocity - new Vector(0, 0, -7.6)).Length < 1e-7,
            "Nearby surface motion lost rotational velocity");
        Require((near.Acceleration - acceleration - new Vector(-0.152, 0, -0.38)).Length < 1e-7,
            "Nearby surface motion lost rotational acceleration");
        double previousWeight = 1;
        for (double distance = 400; distance <= 600; distance += 1)
        {
            var frame = CollisionFrame.Create(new Vector(distance, 0, 0), velocity, acceleration, spin, angularAcceleration, 350);
            Require(frame.Velocity.IsFinite && frame.Acceleration.IsFinite && frame.RotationWeight <= previousWeight
                && frame.RotationWeight >= 0, "Transition out of the surface frame was not bounded and smooth");
            previousWeight = frame.RotationWeight;
        }

        Require(previousWeight == 0, "Surface rotation persisted beyond the local region");
    }

    public static void ClearHold()
    {
        var release = new CollisionHoldRelease();
        Require(!release.Step(false, false, 0), "A newly obstructed path was allowed through");
        Require(!release.Step(true, true, 1) && !release.Step(true, true, 1.74), "Hold resumed before sustained clearance");
        Require(!release.Step(false, true, 1.75), "An obstruction did not reset clearance");
        Require(!release.Step(true, true, 2) && release.Step(true, true, 2.75), "Clear navigation could not resume from hold");
        Require(release.Step(true, false, 3), "A clear continuing flight acquired an unnecessary delay");
        Require(!release.Step(true, true, 10) && !release.Step(true, true, 0), "Clock rewind retained stale release timing");
        Require(release.Step(true, true, 0.75), "A new clock could not establish fresh clearance");
    }

    public static void DistantTurn()
    {
        var state = new FlightState
        {
            Velocity      = new Vector(600, 0, 0),
            Thrust        = 50,
            MaximumThrust = 50
        };
        state.Obstacles.Add(new Obstacle
        {
            Position       = new Vector(3000, 0, 0),
            Radius         = 200,
            PhysicalRadius = 100
        });
        Require(CollisionBraking.ProbeDistance(state, default, default) > 3000, "Fixture lost the distant straight-sweep hit");
        Require(CollisionEscape.PathClear(state, new Vector(0, 50, 0), default, default, (start, end, padding) => true),
            "A checked turning maneuver remained blocked by a straight-line forecast");
        Require(!CollisionEscape.PathClear(state, new Vector(50, 0, 0), default, default, (start, end, padding) => true),
            "A real distant collision passed the trajectory check");
        Require(!CollisionEscape.PathClear(state, new Vector(0, 50, 0), default, default, (start, end, padding) => false),
            "An obstructed or crowded hull query was ignored during a distant turn");
    }

    private static void Require(bool condition, string message)
    {
        if (!condition)
        {
            throw new Exception(message);
        }
    }
}
