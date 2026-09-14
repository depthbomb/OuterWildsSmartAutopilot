using System;
using System.Diagnostics;
using Xunit;
using SmartAutopilot.Navigation;
using Xunit.Abstractions;

internal sealed class Scenario
{
    public const double BudgetMilliseconds = 10;

    public readonly string Name;
    public readonly FlightState State;
    public readonly bool ExpectedRoute;
    public readonly RoutePlanner Planner = new();

    public Scenario(string name, Vector start, Vector target, double sunRadius, bool expectedRoute = true)
    {
        Name          = name;
        ExpectedRoute = expectedRoute;
        State = new FlightState
        {
            Position             = start,
            TargetPosition       = target,
            ArrivalRadius        = 1000,
            Thrust               = 30,
            MaximumThrust        = 50,
            DeltaTime            = 0.02,
            Velocity             = default,
            ExternalAcceleration = default,
            TargetVelocity       = default,
            TargetAcceleration   = default,
            SpeedLimit           = 0
        };
        AddBody(default, sunRadius);
        AddBody(new Vector(4900, 0, 0), 350);
        AddBody(new Vector(5350, 500, 0), 400);
        var random = new Random(8517);
        for (var i = 3; i < 29; i++)
        {
            var position = new Vector(random.NextDouble() * 70000 - 35000, 18000 + random.NextDouble() * 40000, random.NextDouble() * 50000 - 25000);
            AddBody(position, 150 + random.NextDouble() * 1000);
        }
    }

    public void Plan()
    {
        Planner.TryPlanToTarget(State.Position, State.TargetPosition, State.ArrivalRadius, State.Obstacles, default, out _, out _, out _, out _);
    }

    public void Verify()
    {
        var found = Planner.TryPlanToTarget(State.Position, State.TargetPosition, State.ArrivalRadius, State.Obstacles, default, out var goal, out var next, out _, out double distance);
        Assert.True(found == ExpectedRoute, Name + ": unexpected route availability");

        if (found)
        {
            Assert.True(goal.IsFinite && next.Position.IsFinite && !double.IsInfinity(distance) && distance >= 0, Name + ": invalid route output");
            Assert.True(RoutePlanner.SegmentClear(State.Position, next.Position, State.Obstacles), Name + ": unsafe first leg");
        }
        else
        {
            Assert.True(!string.IsNullOrEmpty(Planner.FailureReason), Name + ": missing blocked-route explanation");
        }
    }

    private void AddBody(Vector position, double radius)
    {
        State.Obstacles.Add(new Obstacle
        {
            Position       = position,
            Velocity       = default,
            Acceleration   = default,
            PhysicalRadius = radius * 0.6,
            Radius         = radius
        });
    }
}

public sealed class PerformanceTests
{
    private readonly ITestOutputHelper _testOutputHelper;

    public PerformanceTests(ITestOutputHelper testOutputHelper)
    {
        _testOutputHelper = testOutputHelper;
    }

    [Fact]
    public void Budgets()
    {
        var scenarios = new[]
        {
            new Scenario("Clear route, 29 bodies", new Vector(-14000, -12000, 0), new Vector(6000, -12000, 0), 5500),
            new Scenario("Solar detour, 29 bodies", new Vector(-14000, 0, 0), new Vector(9000, 0, 0), 5500),
            new Scenario("Alternate arrival, 29 bodies", new Vector(-14000, 0, 0), new Vector(4900, 0, 0), 5500),
            new Scenario("Blocked arrival, 29 bodies", new Vector(-14000, 0, 0), new Vector(4900, 0, 0), 8400, false)
        };

        _testOutputHelper.WriteLine("Runtime: " + Environment.Version + "; " + (Environment.Is64BitProcess ? "64-bit" : "32-bit") + "; CPU count: " + Environment.ProcessorCount);
        _testOutputHelper.WriteLine("101 individual warmed plans per scenario; latency in milliseconds. Unity collision queries and HUD work require in-game profiling.");

        foreach (var scenario in scenarios)
        {
            scenario.Verify();
            for (var warmup = 0; warmup < 16; warmup++)
            {
                scenario.Plan();
            }

            var samples = new double[101];
            for (var sample = 0; sample < samples.Length; sample++)
            {
                var start = Stopwatch.GetTimestamp();
                scenario.Plan();
                samples[sample] = (Stopwatch.GetTimestamp() - start) * 1000.0 / Stopwatch.Frequency;
            }

            var before = GC.GetAllocatedBytesForCurrentThread();
            for (var i = 0; i < 100; i++)
            {
                scenario.Plan();
            }

            var allocated = GC.GetAllocatedBytesForCurrentThread() - before;

            Array.Sort(samples);
            var p95 = samples[95];
            _testOutputHelper.WriteLine($"{scenario.Name}: median={samples[50]:F3}ms; p95={p95:F3}ms; max={samples[100]:F3}ms; allocated={allocated / 100.0:F0} bytes/plan; p95 budget={Scenario.BudgetMilliseconds:F0}ms");

            Assert.True(p95 <= Scenario.BudgetMilliseconds, scenario.Name + ": planning latency exceeded its budget");
            Assert.True(allocated == 0, scenario.Name + ": warmed route planning allocated managed memory");
        }

        MeasureControl();
        MeasureControl(true);
        MeasureStationPrediction();
        MeasureSafetyForecasts();
        MeasureArrivalForecast();
        MeasureCollisionFrame();
        MeasureApproachControl();

        _testOutputHelper.WriteLine("PASS: standalone navigation performance budgets and allocation checks.");
    }

    private static void MeasureControl(bool orbital = false)
    {
        var scenario = orbital
            ? new Scenario("Orbital control", new Vector(-14000, 0, 0), new Vector(4900, 0, 0), 5246.7)
            : new Scenario("Control", new Vector(-14000, -12000, 0), new Vector(6000, -12000, 0), 5500);
        if (orbital)
        {
            scenario.State.TargetObstacleIndex = 1;
            scenario.State.TargetVelocity = new Vector(0, 0, 280);
            scenario.State.TargetAcceleration = new Vector(-16, 0, 0);
            scenario.State.Obstacles[1].PrimaryIndex = 0;
            scenario.State.Obstacles[1].Velocity = scenario.State.TargetVelocity;
            scenario.State.Obstacles[1].Acceleration = scenario.State.TargetAcceleration;
        }

        var computer = new FlightComputer();
        var recorder = new FlightRecorder();
        for (var i = 0; i < 100; i++)
        {
            computer.Step(scenario.State);
            recorder.Add(new FlightSample(scenario.State, default, 1, "Cruise"));
        }

        var before = GC.GetAllocatedBytesForCurrentThread();
        var start  = Stopwatch.GetTimestamp();
        for (var i = 0; i < 10000; i++)
        {
            scenario.State.Time += scenario.State.DeltaTime;
            var command = computer.Step(scenario.State);
            Assert.True(command.Acceleration.IsFinite && command.Acceleration.Length <= scenario.State.Thrust + 1e-7, "Invalid control command during performance run");

            if (i % 25 == 0)
            {
                recorder.Add(new FlightSample(scenario.State, command.Acceleration, 1, "Cruise"));
            }
        }

        var average   = (Stopwatch.GetTimestamp() - start) * 1000.0 / Stopwatch.Frequency / 10000;
        var allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        Console.WriteLine($"{scenario.Name} with periodic replanning and buffered samples: mean={average:F4}ms/step; allocated={allocated} bytes/10000 steps");

        Assert.True(average <= 0.2, "Steady control exceeded 0.2ms per step");
        Assert.True(allocated == 0, "Warmed control or flight recording allocated managed memory");
    }

    private static void MeasureStationPrediction()
    {
        var scenario = new Scenario("Station prediction", new Vector(4906.14, 346.43, 2201.37), new Vector(12000, 0, 0), 5246.7);
        var station  = scenario.State.Obstacles[1];

        station.Position        = new Vector(2300, 0, 0);
        station.Velocity        = new Vector(0, 0, Math.Sqrt(75 * 2300));
        station.Radius          = 310;
        scenario.State.Velocity = station.Velocity + new Vector(-156.22, -23.07, -108.30);

        foreach (var stable in new[] { true, false })
        {
            station.Acceleration = stable ? new Vector(-75, 0, 0) : default;
            for (var warmup = 0; warmup < 100; warmup++)
            {
                OrbitalMotion.ExcludesCollision(scenario.State, 1, 345, 251);
            }

            var before   = GC.GetAllocatedBytesForCurrentThread();
            var start    = Stopwatch.GetTimestamp();
            var excluded = 0;
            for (var step = 0; step < 10000; step++)
            {
                if (OrbitalMotion.ExcludesCollision(scenario.State, 1, 345, 251))
                {
                    excluded++;
                }
            }

            var average   = (Stopwatch.GetTimestamp() - start) * 1000.0 / Stopwatch.Frequency / 10000;
            var allocated = GC.GetAllocatedBytesForCurrentThread() - before;

            Console.WriteLine($"Station prediction, stable={stable}: mean={average:F4}ms/check; allocated={allocated} bytes/10000 checks");

            Assert.True(excluded == (stable ? 10000 : 0), "Station prediction benchmark produced an incorrect safety decision");
            Assert.True(average <= 0.2 && allocated == 0, "Station prediction exceeded its time or allocation budget");
        }
    }

    private static void MeasureSafetyForecasts()
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
        var command = new FlightCommand(new Vector(20, 0, 0), FlightPhase.Escape);
        Func<Vector, Vector, double, bool> clear = (start, end, padding) => true;

        MeasureSafety("Collision escape, numerical checks only", Escape);

        var primary = new Obstacle
        {
            Velocity     = new Vector(150, 0, 0),
            Acceleration = new Vector(1, 0, 0),
            Radius       = 441.1
        };
        var localAcceleration = new Vector(-6.79, 0, -8.69);
        var radial = -localAcceleration.Unit;
        var satellite = new Obstacle
        {
            PrimaryIndex   = 0,
            Position       = radial * 600,
            Velocity       = primary.Velocity + new Vector(-radial.Z, 0, radial.X) * Math.Sqrt(localAcceleration.Length * 600),
            Acceleration   = primary.Acceleration + localAcceleration,
            Radius         = 200.7,
            PhysicalRadius = 30
        };
        state.Obstacles.Clear();
        state.Obstacles.Add(primary);
        state.Obstacles.Add(satellite);
        state.Position = satellite.Position + new Vector(-3631.45, -8.35, -10287.63);
        state.Velocity = satellite.Velocity + new Vector(222.38, 3.16, 675.37);

        MeasureSafety("Curved satellite forecast", SatelliteCheck);
        return;

        void SatelliteCheck()
        {
            Assert.True(OrbitalMotion.ExcludesCollision(state, 1, 235.7, 33), "Satellite benchmark failed its bounded flyby");
        }

        void Escape()
        {
            Assert.True(CollisionEscape.TrySelect(state, command, 0, default, default, clear, out _), "Escape benchmark rejected an open corridor");
        }
    }

    private static void MeasureArrivalForecast()
    {
        var state = new FlightState
        {
            Position             = new Vector(1500, 0, 0),
            ExternalAcceleration = new Vector(-1, 0, 0),
            ArrivalRadius        = 400,
            Thrust               = 50,
            MaximumThrust        = 50
        };
        state.ArrivalThrustLimits.Add(new ArrivalThrustLimit(515, 20));
        state.ArrivalThrustLimits.Add(new ArrivalThrustLimit(300, 10));

        MeasureSafety("Arrival thrust forecast", Check);
        return;

        void Check()
        {
            var speed = ArrivalBraking.LimitSpeed(state, 1000);
            Assert.True(speed is > 100 and < 250, "Arrival forecast benchmark lost its local speed bound");
        }
    }

    private static void MeasureCollisionFrame()
    {
        var velocity = new Vector(100, 20, -30);
        var spin     = new Vector(0, 0.06, 0);
        var release  = new CollisionHoldRelease();
        var time     = (double)0;

        MeasureSafety("Collision frame and hold clearance", Check);
        return;

        void Check()
        {
            var far  = CollisionFrame.Create(new Vector(10000, 0, 0), velocity, default, spin, default, 350);
            var near = CollisionFrame.Create(new Vector(380, 0, 0), velocity, default, spin, default, 350);
            Assert.True(far.RotationWeight == 0 && near.RotationWeight == 1, "Collision frame benchmark lost near/far separation");

            release.Step(true, true, time);
            time += 0.02;
        }
    }

    private static void MeasureApproachControl()
    {
        var scenario = new Scenario("Nearby approach", new Vector(820, 26, -513), new Vector(-899, 0, -438), 500);
        var state    = scenario.State;

        state.Thrust               = 50;
        state.ArrivalRadius        = 600;
        state.Velocity             = new Vector(44.3, 5.7, 41.8);
        state.TargetVelocity       = new Vector(24.1, 0, -49.5);
        state.TargetAcceleration   = new Vector(2.72, 0, 1.32);
        state.ExternalAcceleration = -state.Position.Unit * (3025000 / state.Position.LengthSquared);

        var computer = new FlightComputer();

        MeasureSafety("Nearby approach with 29 bodies and periodic replanning", Check);
        return;

        void Check()
        {
            state.Time += state.DeltaTime;
            var command = computer.Step(state);
            Assert.True(command.Phase != FlightPhase.Escape && command.Acceleration.IsFinite && command.Acceleration.Length <= state.Thrust + 1e-7, "Nearby approach benchmark lost valid steering");
        }
    }

    private static void MeasureSafety(string name, Action check)
    {
        for (var warmup = 0; warmup < 100; warmup++)
        {
            check();
        }

        var before = GC.GetAllocatedBytesForCurrentThread();
        var start  = Stopwatch.GetTimestamp();
        for (var sample = 0; sample < 10000; sample++)
        {
            check();
        }

        var average   = (Stopwatch.GetTimestamp() - start) * 1000.0 / Stopwatch.Frequency / 10000;
        var allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        Console.WriteLine($"{name}: mean={average:F4}ms/check; allocated={allocated} bytes/10000 checks");
        Assert.True(average < 0.2 && allocated == 0, name + " exceeded its time or allocation budget");
    }
}
