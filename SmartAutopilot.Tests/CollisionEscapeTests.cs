using Xunit;
using System;
using SmartAutopilot.Navigation;

internal static class CollisionEscapeTests
{
    public static void CometNearSun()
    {
        var state = new FlightState
        {
            Position             = new Vector(-2322.17, 76.99, 5297.17),
            Velocity             = new Vector(273.73, -12.66, -46.76),
            ExternalAcceleration = new Vector(4.24, -0.57, -13.14),
            TargetPosition       = new Vector(17000, 0, 8000),
            ArrivalRadius        = 1800,
            Thrust               = 20,
            MaximumThrust        = 50
        };
        state.Obstacles.Add(new Obstacle
        {
            Radius         = 5246.7,
            PhysicalRadius = 2500
        });
        var comet = new Obstacle
        {
            Position       = state.Position - new Vector(39.9, 72.8, -500.9),
            Velocity       = state.Velocity - new Vector(-13.5, -12.6, 101.7),
            Acceleration   = new Vector(4.3, -0.04, -9.76),
            Radius         = 390,
            PhysicalRadius = 110
        };
        state.Obstacles.Add(comet);

        var computer  = new FlightComputer();
        var requested = computer.Step(state);

        Assert.True(requested.Phase == FlightPhase.Escape && computer.AvoidanceBodyIndex == 0, "Fixture lost the logged solar escape conflict");

        var escaped = CollisionEscape.TrySelect(state, requested, 0, comet.Velocity, comet.Acceleration, (start, end, padding) => SphereClear(start, end, comet.Position - state.Position, 125 + padding), out var selected);
        Assert.True(escaped && selected.Phase == FlightPhase.Escape, "Comet hold did not yield a checked escape maneuver");
        Assert.True(Vector.Dot(selected.Acceleration, state.Position.Unit) > 0, "Escape thrust pointed toward the Sun");

        var minimumSunDistance = state.Position.Length;
        var manual             = false;
        for (var step = 0; step < 2000; step++)
        {
            requested = computer.Step(state);
            if (requested.Phase != FlightPhase.Escape)
            {
                break;
            }

            escaped = CollisionEscape.TrySelect(state, requested, computer.AvoidanceBodyIndex, comet.Velocity, comet.Acceleration, (start, end, padding) => SphereClear(start, end, comet.Position - state.Position, 125 + padding), out selected);
            if (!escaped)
            {
                manual = true;
                break;
            }

            state.Velocity             += (selected.Acceleration + state.ExternalAcceleration) * state.DeltaTime;
            state.Position             += state.Velocity                                       * state.DeltaTime;
            comet.Velocity             += comet.Acceleration                                   * state.DeltaTime;
            comet.Position             += comet.Velocity                                       * state.DeltaTime;
            state.ExternalAcceleration =  -state.Position.Unit                                 * (4e8 / state.Position.LengthSquared);
            comet.Acceleration         =  -comet.Position.Unit                                 * (4e8 / comet.Position.LengthSquared);
            state.Time                 += state.DeltaTime;
            minimumSunDistance         =  Math.Min(minimumSunDistance, state.Position.Length);

            Assert.True((state.Position - comet.Position).Length > 125, "Escape hit the comet");
            Assert.True(minimumSunDistance > 3000, "Collision recovery continued into the physical solar hazard");
        }

        Console.WriteLine($"COLLISION comet: minimum Sun distance={minimumSunDistance:F1}m; manual handoff={manual}; elapsed={state.Time:F2}s");
    }

    public static void ObstructedEscape()
    {
        var state = new FlightState
        {
            Position      = new Vector(6000, 0, 0),
            Thrust        = 20,
            MaximumThrust = 50
        };
        state.Obstacles.Add(new Obstacle
        {
            Radius         = 5246,
            PhysicalRadius = 2500
        });
        var requested = new FlightCommand(new Vector(20, 0, 0), FlightPhase.Escape);
        var clear     = CollisionEscape.TrySelect(state, requested, 0, default, default, (start, end, padding) => SphereClear(start, end, new Vector(35, 0, 0), 8 + padding), out var selected);

        Assert.True(clear && Math.Abs(selected.Acceleration.Y) + Math.Abs(selected.Acceleration.Z) > 1, "Blocked straight escape did not select a clear lateral alternative");
        Assert.True(selected.Acceleration.X > 0 && selected.Acceleration.Length <= state.Thrust + 1e-7, "Alternative lost outward thrust or exceeded authority");
        Assert.False(CollisionEscape.TrySelect(state, requested, 0, default, default, (start, end, padding) => false, out _));

        state.Position = new Vector(2600, 0, 0);
        state.Velocity = new Vector(-200, 0, 0);
        Assert.False(CollisionEscape.TrySelect(state, requested, 0, default, default, (start, end, padding) => true, out _));

        state.Position = new Vector(6000, 0, 0);
        state.Velocity = default;
        state.ExternalAcceleration = new Vector(-100, 0, 0);
        Assert.False(CollisionEscape.PathClear(state, requested.Acceleration, default, default, (start, end, padding) => true));
    }

    private static bool SphereClear(Vector start, Vector end, Vector center, double radius)
    {
        var offset   = start - center;
        var delta    = end   - start;
        var fraction = delta.LengthSquared > 1e-9 ? Math.Max(0, Math.Min(1, -Vector.Dot(offset, delta) / delta.LengthSquared)) : 0;

        return (offset + delta * fraction).Length > radius;
    }
}
