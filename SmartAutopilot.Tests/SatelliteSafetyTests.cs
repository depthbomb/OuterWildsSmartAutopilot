using Xunit;
using System;
using SmartAutopilot.Navigation;

internal static class SatelliteSafetyTests
{
    public static void MovingPrimaryForecast()
    {
        // Reproduce the logged relative approach in a representative 600m satellite orbit.
        // The old log did not include the primary-relative state needed for an exact orbital replay.
        var state = new FlightState
        {
            Thrust               = 50,
            MaximumThrust        = 50,
            ExternalAcceleration = new Vector(8.24, 0.01, 2.73),
            TargetPosition       = new Vector(3000, 0, 0),
            ArrivalRadius        = 300
        };
        var primary = new Obstacle
        {
            Velocity     = new Vector(150, 0, 0),
            Acceleration = new Vector(1, 0, 0),
            Radius       = 441.1
        };
        var localAcceleration = new Vector(-5.79, 0, -8.69) - primary.Acceleration;
        var radial            = -localAcceleration.Unit;
        var tangent           = new Vector(-radial.Z, 0, radial.X);
        var satellite = new Obstacle
        {
            PrimaryIndex   = 0,
            Position       = radial * 600,
            Velocity       = primary.Velocity + tangent * Math.Sqrt(localAcceleration.Length * 600),
            Acceleration   = localAcceleration + primary.Acceleration,
            Radius         = 200.7,
            PhysicalRadius = 30
        };
        state.Obstacles.Add(primary);
        state.Obstacles.Add(satellite);
        state.Position = satellite.Position + new Vector(-3631.45, -8.35, -10287.63);
        state.Velocity = satellite.Velocity + new Vector(222.38, 3.16, 675.37);

        Assert.True(OrbitalMotion.TryCreate(state, 1, out var orbit), "Moving-primary fixture is not recognized");
        Assert.True(!orbit.ExcludesLinearPath(state.Position, state.Velocity, 235.7, 33), "Fixture lost the accelerating-primary limitation");
        Assert.True(OrbitalMotion.ExcludesCollision(state, 1, 235.7, 33), "A curved satellite flyby still causes a distant tangent correction");

        var computer = new FlightComputer();
        computer.Step(state);
        Assert.True(computer.AvoidanceBodyIndex != 1, "The navigation controller still selects the excluded satellite");

        var translation = new Vector(1e7, -2e7, 3e7);
        var drift = new Vector(40, 20, -100);

        state.Position += translation;
        state.Velocity += drift;

        foreach (var body in state.Obstacles)
        {
            body.Position += translation;
            body.Velocity += drift;
        }

        Assert.True(OrbitalMotion.ExcludesCollision(state, 1, 235.7, 33), "Satellite forecast changed with the floating origin or reference-frame velocity");
        Assert.True(OrbitalMotion.TryCreate(state, 1, out orbit), "Translated orbit was not recognized");

        var encounter = (double)7;
        state.Position = satellite.Position + new Vector(-2500, 0, 0);
        state.Velocity = (orbit.Position(encounter) - state.Position) / encounter;
        Assert.True(!OrbitalMotion.ExcludesCollision(state, 1, 235.7, 20), "A real orbital interception was suppressed");

        // Contact halfway between prediction samples must remain protected too.
        encounter = 6.25;
        state.Velocity = (orbit.Position(encounter) - state.Position) / encounter;
        Assert.True(!OrbitalMotion.ExcludesCollision(state, 1, 235.7, 20), "An encounter between samples escaped the swept envelope");

        satellite.Velocity += radial * 20;
        Assert.True(!OrbitalMotion.ExcludesCollision(state, 1, 235.7, 20), "Unstable radial motion gained optimistic orbital clearance");

        primary.PrimaryIndex = 2;
        primary.Position = default;
        primary.Velocity = new Vector(Math.Sqrt(4 * 8600), 0, 0);
        primary.Acceleration = new Vector(0, 0, -4);
        state.Obstacles.Add(new Obstacle
        {
            Position = new Vector(0, 0, -8600),
            Radius   = 5246
        });
        satellite.Position = radial * 600;
        satellite.Velocity = primary.Velocity + tangent * Math.Sqrt(localAcceleration.Length * 600);
        satellite.Acceleration = primary.Acceleration + localAcceleration;
        state.Position = satellite.Position + new Vector(-3631.45, -8.35, -10287.63);
        state.Velocity = satellite.Velocity + new Vector(222.38, 3.16, 675.37);

        Assert.True(OrbitalMotion.TryCreate(state, 0, out var primaryOrbit), "Primary's stellar orbit was not recognized");
        Assert.True(OrbitalMotion.ExcludesCollision(state, 1, 235.7, 33), "Nested orbital motion lost a clear satellite flyby");
        Assert.True(OrbitalMotion.TryCreate(state, 1, out orbit), "Nested satellite orbit was not recognized");

        state.Position = satellite.Position + new Vector(-2500, 0, 0);
        var encounterPosition = orbit.Position(7) + primaryOrbit.Position(7) - primary.Position - primary.Velocity * 7 - primary.Acceleration * 24.5;
        state.Velocity = (encounterPosition - state.Position) / 7;
        Assert.True(!OrbitalMotion.ExcludesCollision(state, 1, 235.7, 20), "Following the primary's curved orbit hid a true encounter");
    }

    public static void ProtectedStation()
    {
        var acceleration = new Vector(72.26, 0, -23.78);
        var radial       = -acceleration.Unit;
        var tangent      = new Vector(-radial.Z, 0, radial.X);
        var station = new Obstacle
        {
            PrimaryIndex   = 0,
            Position       = radial * 2300,
            Velocity       = tangent * Math.Sqrt(acceleration.Length * 2300),
            Acceleration   = acceleration,
            Radius         = 310,
            PhysicalRadius = 30
        };
        var state = new FlightState
        {
            Position             = station.Position + new Vector(-2551.60, -304.40, -8521.23),
            Velocity             = station.Velocity + new Vector(275.78, 10.58, 828.90),
            ExternalAcceleration = new Vector(2.55, 0.26, 3.10),
            TargetPosition       = new Vector(12000, 0, 0),
            ArrivalRadius        = 400,
            Thrust               = 50,
            MaximumThrust        = 50
        };
        state.Obstacles.Add(new Obstacle
        {
            Radius         = 5246.7,
            PhysicalRadius = 2500
        });
        state.Obstacles.Add(station);
        Assert.True(OrbitalMotion.TryCreate(state, 1, out var orbit), "Station fixture is not a stable orbit");
        Assert.True(!orbit.ExcludesLinearPath(state.Position, state.Velocity, 345, 251), "Fixture does not reproduce the whole-orbit exclusion limitation");
        Assert.True(new FlightComputer().Step(state).Phase != FlightPhase.Escape, "An inner station caused a distant escape despite the enclosing Sun protection");

        station.PrimaryIndex = -1;
        Assert.True(new FlightComputer().Step(state).Phase != FlightPhase.Escape, "An unlinked station part lost the same enclosing protection");

        station.PrimaryIndex = 99;
        Assert.True(new FlightComputer().Step(state).Phase == FlightPhase.Escape, "Invalid orbital context silently suppressed conservative avoidance");

        station.PrimaryIndex = 0;
        state.Position       = new Vector(-5800, 0, 0);
        state.Velocity       = new Vector(500, 0, 0);

        var computer = new FlightComputer();
        Assert.True(computer.Step(state).Phase == FlightPhase.Escape && computer.AvoidanceBodyIndex == 0, "Suppressing a nested station also suppressed an urgent Sun approach");

        state.Position = station.Position + radial * 450;
        Assert.True(!OrbitalMotion.ExcludesCollision(state, 1, 345, 251), "Station protection remained active after entering the enclosing exclusion zone");

        state.Position = new Vector(-9000, 0, 0);
        state.Velocity = new Vector(500, 0, 0);
        state.Obstacles[0].Radius = 2700;

        Assert.True(!OrbitalMotion.ExcludesCollision(state, 1, 345, 251), "A station orbit extending outside its primary clearance was ignored");

        state.Obstacles[0].Radius = 5246.7;
        station.PrimaryIndex = -1;
        station.Acceleration = default;
        Assert.True(!OrbitalMotion.ExcludesCollision(state, 1, 345, 251), "Unbound debris was mistaken for a protected circular orbit");
    }
}
