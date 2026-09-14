using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Reflection;
using System.Reflection.Metadata;
using System.Reflection.PortableExecutable;
using SmartAutopilot.Navigation;

internal readonly struct FlightResult
{
    public readonly double Time;
    public readonly double PeakSpeed;
    public readonly int RapidAvoidanceChanges;

    public FlightResult(double time, double peakSpeed, int rapidAvoidanceChanges)
    {
        Time                  = time;
        PeakSpeed             = peakSpeed;
        RapidAvoidanceChanges = rapidAvoidanceChanges;
    }
}

internal static class Program
{
    private static int _passed;

    private static void Main(string[] args)
    {
        if (args.Length == 1 && args[0] == "--moon-profile")
        {
            MoonApproachTests.Profile();

            return;
        }

        if (args.Length == 1 && args[0] == "--orbital-profile")
        {
            OrbitalApproachTests.Profile();

            return;
        }

        if (args.Length == 1 && args[0] == "--detour-braking-profile")
        {
            Simulate(new Vector(-3500, 0, 0), default, new Vector(3500, 0, 0), default, Bodies(), false, trace: true);

            return;
        }

        if (args.Length == 1 && args[0] == "--braking-profile")
        {
            BrakingTests.Profile();

            return;
        }

        if (args.Length == 2)
        {
            VerifyGameBindings(args[0], args[1]);

            return;
        }

        if (args.Length == 1 && args[0] == "--flight-baseline")
        {
            Simulate(default, default, new Vector(20000, 0, 0), default, [], false);
            Simulate(new Vector(-3500, 0, 0), default, default, default, Bodies(), true, 900);

            return;
        }

        Run("Distant collision checks exclude fictitious surface rotation and retain real braking", CollisionFrameTests.DistantRotation);
        Run("Automation requests reject stale, unbounded and invalid scenarios", AutomationRequestTests.ValidateRequests);
        Run("Arrival restores distance after inward avoidance across thrust levels and timesteps", ArrivalRecoveryTests.RecoverDistance);
        Run("Arrival distance recovery yields to collision avoidance", ArrivalRecoveryTests.AvoidancePriority);
        Run("Velocity-matched satellite arrivals finish across the braking boundary", ArrivalRecoveryTests.SatelliteArrivalBoundary);
        Run("Nearby collision frames preserve rotating-surface motion", CollisionFrameTests.SurfaceFrame);
        Run("Collision holds resume only after sustained fresh clearance", CollisionFrameTests.ClearHold);
        Run("Distant straight-sweep hits require a checked turning and stopping path", CollisionFrameTests.DistantTurn);
        Run("Arrival braking anticipates the destination's reduced-thrust region", ArrivalThrustTests.Transition);
        Run("Arrival thrust limits respect geometry, overlap and invalid data", ArrivalThrustTests.BoundsAndValidation);
        Run("A comet collision hold yields to checked solar escape or manual control", CollisionEscapeTests.CometNearSun);
        Run("Emergency escape checks lateral corridors, braking and celestial hazards", CollisionEscapeTests.ObstructedEscape);
        Run("An inaccessible comet does not pull a holding ship toward the Sun", PlaytestRegressionTests.BlockedCometHold);
        Run("Moving moon approaches shed lateral momentum and arrive promptly", MoonApproachTests.Verify);
        Run("Recovered moon approach avoids another escape across physics timesteps", MoonApproachTests.Recovery);
        Run("Nested station orbits preserve primary and local collision protection", SatelliteSafetyTests.ProtectedStation);
        Run("Moving-primary satellite forecasts exclude clear flybys and retain real encounters", SatelliteSafetyTests.MovingPrimaryForecast);
        Run("Orbital target approaches finish without repeated solar escapes", OrbitalApproachTests.Verify);
        Run("Orbital predictions respect moving frames and invalid data", OrbitalApproachTests.Prediction);
        Run("Satellite tangent forecasts cannot leave their whole orbit", OrbitalApproachTests.SatelliteTangent);
        Run("Blocked orbital forecasts preserve a valid current approach", OrbitalApproachTests.ForecastFallback);
        Run("Blocked-route warnings survive emergency clearance", PlaytestRegressionTests.WarningAcrossEscape);
        Run("Avoidance priorities resist jitter without delaying immediate threats", PlaytestRegressionTests.AvoidancePriority);
        Run("Irregular hull bounds cover rotated and offset geometry and exterior arrivals", PlaytestRegressionTests.IrregularHullEnvelope);
        Run("Flights detour around an extended hull from rest and at speed", PlaytestRegressionTests.IrregularHullFlight);
        Run("Detours identify their obstacle and recover to cruise", BetaSafetyTests.DetourStatus);
        Run("Collision braking stops before a swept obstacle", BetaSafetyTests.CollisionStop);
        Run("Collision holds track accelerating reference frames", BetaSafetyTests.MovingCollisionHold);
        Run("Fault retries are deliberate and bounded", BetaSafetyTests.FaultRecovery);
        Run("Prolonged holds warn on time and fuel consumption", BetaSafetyTests.HoldWarnings);
        Run("Flight diagnostics retain only the latest minute", BetaSafetyTests.FlightHistory);
        Run("Invalid game data cannot reach collision queries", BetaSafetyTests.InvalidState);
        Run("Interrupted braking resumes after avoidance", () => InterruptedArrival(true));
        Run("A stalled distant approach resumes with a moving target", () => InterruptedArrival(false));
        Run("Local thrust restrictions do not obstruct distant twin routes", LocalThrustClearance);
        Run("Clear route and arrival", () => Simulate(new Vector(-3500, 0, 0), default, new Vector(3500, 0, 0), default, [], false));
        Run("Head-on planet detour", () => Simulate(new Vector(-3500, 0, 0), default, new Vector(3500, 0, 0), default, Bodies(), false));
        Run("Detour with inverse-square gravity", () => Simulate(new Vector(-3500, 0, 0), default, new Vector(3500, 0, 0), default, Bodies(), true));
        Run("Moving obstacle and moving destination", () => Simulate(new Vector(-3500, 0, 0), default, new Vector(3500, 0, 0), new Vector(0, 8, 0), Bodies(new Vector(0, 10, 0)), true));
        Run("Initial lateral momentum", () => Simulate(new Vector(-3500, 0, 0), new Vector(100, 65, 0), new Vector(3500, 0, 0), default, Bodies(), true));
        Run("Departure inside clearance zone", () => Simulate(new Vector(-650, 0, 0), default, new Vector(3500, 0, 0), default, Bodies(), true));
        Run("Two blocking planets", () =>
        {
            var bodies = Bodies();
            bodies.Add(new Obstacle
            {
                Position       = new Vector(1800, -250, 0),
                Radius         = 700,
                PhysicalRadius = 450
            });
            Simulate(new Vector(-3500, 0, 0), default, new Vector(4500, 0, 0), default, bodies, true);
        });
        Run("Planet arrival respects target clearance", () =>
        {
            var bodies = Bodies();
            Simulate(new Vector(-3500, 0, 0), default, default, default, bodies, true, 900);
        });
        Run("No path is reported explicitly", NoPath);
        Run("Gravity compensation uses measured force", GravityCompensation);
        Run("Floating-origin translation leaves control unchanged", TranslationInvariant);
        Run("Emergency braking intervenes", EmergencyBraking);
        Run("Inverse-distance gravity", () => Simulate(new Vector(-3500, 0, 0), default, new Vector(3500, 0, 0), default, Bodies(), true, 300, 1));
        Run("Sun-scale obstacle", () =>
        {
            var bodies = Bodies();
            bodies[0].Radius         = 3200;
            bodies[0].PhysicalRadius = 2000;
            Simulate(new Vector(-9000, 0, 0), default, new Vector(9000, 0, 0), default, bodies, true);
        });
        Run("Accelerating reference-frame compensation", AcceleratingFrame);
        Run("Twenty-four 3D approaches", RandomApproaches);
        Run("Automatic long-distance acceleration and braking", AutomaticSpeed);
        Run("Avoidance remains engaged until recovery settles", AvoidanceRecovery);
        Run("A clear flyby does not trigger radial emergency braking", ClearFlyby);
        Run("Planet approach has no rapid avoidance switching", () =>
        {
            var result = Simulate(new Vector(-3500, 0, 0), default, default, default, Bodies(), true, 900);
            Require(result.RapidAvoidanceChanges == 0, "Emergency avoidance is chattering during planet arrival");
        });
        Run("Gentle corrections steer while accelerating", SteeringWhileAccelerating);
        Run("Small arrival overshoot matches velocity without reversing", () => ArrivalWithoutReversing(0.02, default, default));
        Run("Moving target arrival with gravity and a different timestep", () => ArrivalWithoutReversing(1.0 / 60, new Vector(150, 20, 0), new Vector(0, -4, 0)));
        Run("Optional speed limit does not start final braking early", OptionalSpeedLimit);
        Run("Initial flight away from the target recovers", () => Simulate(new Vector(-3500, 0, 0), new Vector(-100, 0, 0), new Vector(3500, 0, 0), default, [], false));
        Run("Distant twin-like target with an obstructed near-side arrival", () => TwinApproach(false));
        Run("Other twin with overlapping stellar and sibling clearance", () => TwinApproach(true));
        Run("Fully obstructed arrival region still holds", CoveredArrival);
        Run("Cold launch from a moving surface", () => DepartureTests.Launch(true, 0.02));
        Run("Warm launch at a different physics timestep", () => DepartureTests.Launch(false, 1.0 / 60));
        Run("Blocked and interrupted launch ignition", DepartureTests.BlockedIgnition);
        Run("Obstructed climb brakes without repeated restarts", DepartureTests.ObstructionDuringClimb);
        Run("Launch respects local thrust and detects a stuck ship", DepartureTests.ThrustAndProgressLimits);
        Run("Cancelled launch starts with fresh ignition", DepartureTests.CancelAndRestart);
        Run("Tilted launch clears the departure sphere", DepartureTests.ClearanceGeometry);
        Run("Launch hands off to obstacle-avoiding navigation", DepartureTests.NavigationHandoff);
        Run("Interrupted climb resumes and completes", DepartureTests.ResumeInterruptedClimb);
        Run("Unrelated or expired launch context is not resumed", DepartureTests.RejectUnrelatedResume);
        Run("Resumed climb checks overhead clearance again", DepartureTests.ResumeRechecksObstruction);
        Run("Launch matrix across gravity, rotation, slope and timesteps", DepartureReliabilityTests.SurfaceMatrix);
        Run("Descending interrupted launch recovers before the surface", DepartureReliabilityTests.ResumeDuringDescent);
        Run("Trapped airborne launch fails without restarting itself", DepartureReliabilityTests.AirborneStall);
        Run("Resume follows moving surface and rejects rewound time", DepartureReliabilityTests.RotatingResume);
        Run("Moving satellite near a large planet arrival", () =>
        {
            var bodies = new List<Obstacle>
            {
                new()
                {
                    Radius         = 2200,
                    PhysicalRadius = 1800
                },
                new()
                {
                    Position       = new Vector(-3500, 1500, 0),
                    Velocity       = new Vector(0, -80, 0),
                    Radius         = 250,
                    PhysicalRadius = 100
                }
            };
            Simulate(new Vector(-9557, 0, 0), new Vector(72.5, 0, 0), default, default, bodies, true, 2500);
        });
        Run("Clear final braking is shorter with stable arrival", BrakingTests.VerifyRefinement);
        Run("Faster arrival respects target-body emergency clearance", BrakingTests.TargetClearance);
        Run("Braking preserves stopping thrust while removing lateral momentum", BrakingTests.LateralMomentum);
        Console.WriteLine($"PASS: {_passed} navigation checks.");
    }

    private static void Run(string name, Action test)
    {
        var timer = Stopwatch.StartNew();
        test();
        _passed++;
        Console.WriteLine($"PASS {name} ({timer.ElapsedMilliseconds} ms)");
    }

    private static List<Obstacle> Bodies(Vector velocity = default) =>
    [

        new()
        {
            Position       = default,
            Velocity       = velocity,
            Acceleration   = default,
            Radius         = 800,
            PhysicalRadius = 500
        }
    ];

    private static FlightResult Simulate(Vector start, Vector velocity, Vector target, Vector targetVelocity, List<Obstacle> obstacles, bool gravity, double arrival = 300, double exponent = 2, bool trace = false)
    {
        var state = new FlightState
        {
            Position       = start,
            Velocity       = velocity,
            TargetPosition = target,
            TargetVelocity = targetVelocity,
            TargetAcceleration = default,
            ArrivalRadius  = arrival,
            Thrust         = 30,
            DeltaTime      = 0.05,
            SpeedLimit     = 0
        };
        state.Obstacles.AddRange(obstacles);
        var computer = new FlightComputer();
        var phases = new HashSet<FlightPhase>();
        double minimumClearance = double.PositiveInfinity;
        double peakSpeed = 0;
        double lastAvoidanceChange = -100;
        int rapidAvoidanceChanges = 0;
        bool wasAvoiding = false;
        bool finalBrakingStarted = false;
        const double dt = 0.05;
        for (int step = 0; step < 12000; step++)
        {
            state.Time = step * dt;
            state.ExternalAcceleration = default;
            foreach (var body in obstacles)
            {
                var offset = body.Position - state.Position;
                minimumClearance = Math.Min(minimumClearance, offset.Length - body.PhysicalRadius);
                Require(offset.Length > body.PhysicalRadius + 10, "Collision at " + state.Time);
                if (gravity)
                {
                    state.ExternalAcceleration += offset.Unit * (6 * Math.Pow(body.Radius / offset.Length, exponent));
                }
            }

            var command = computer.Step(state);
            if (trace && step % 10 == 0)
            {
                var relative = state.Velocity - state.TargetVelocity;
                var direction = (state.TargetPosition - state.Position).Unit;
                double closing = Vector.Dot(relative, direction);
                Console.WriteLine($"TRACE t={state.Time:F2} phase={command.Phase} remaining={(state.Position - state.TargetPosition).Length - arrival:F1} speed={relative.Length:F1} closing={closing:F1} lateral={(relative - direction * closing).Length:F1} thrust={command.Acceleration.Length:F1}");
            }

            bool interruptedFarAway = command.Phase == FlightPhase.Escape && (state.Position - state.TargetPosition).Length > arrival + Math.Max(60, arrival * 0.1);
            if (interruptedFarAway)
            {
                finalBrakingStarted = false;
            }

            if (finalBrakingStarted)
            {
                Require(command.Phase != FlightPhase.Cruise && command.Phase != FlightPhase.Detour, "Resumed approach after starting final braking");
            }

            finalBrakingStarted |= command.Phase == FlightPhase.Braking;
            bool avoiding = command.Phase == FlightPhase.Escape;
            if (avoiding != wasAvoiding)
            {
                if (state.Time - lastAvoidanceChange < 0.5)
                {
                    rapidAvoidanceChanges++;
                }

                lastAvoidanceChange = state.Time;
                wasAvoiding        = avoiding;
            }

            peakSpeed = Math.Max(peakSpeed, (state.Velocity - targetVelocity).Length);
            phases.Add(command.Phase);
            Require(command.Acceleration.IsFinite, "Non-finite thrust");
            Require(command.Acceleration.Length <= state.Thrust + 1e-7, "Exceeded available thrust");
            if (command.Phase == FlightPhase.Arrived)
            {
                Require((state.Velocity - targetVelocity).Length < 1, "Did not match target velocity");
                Require(Math.Abs((state.Position - state.TargetPosition).Length - arrival) < Math.Max(60, arrival * 0.15), "Stopped outside the arrival zone: " + (state.Position - state.TargetPosition).Length + " at " + state.Time);
                Console.WriteLine($"  arrived at {state.Time:F1}s; peak {peakSpeed:F1}m/s; minimum physical clearance {minimumClearance:F1}m; rapid avoidance changes {rapidAvoidanceChanges}; phases {string.Join(", ", phases)}");

                return new FlightResult(state.Time, peakSpeed, rapidAvoidanceChanges);
            }

            state.Velocity       += (command.Acceleration + state.ExternalAcceleration) * dt;
            state.Position       += state.Velocity * dt;
            state.TargetPosition += targetVelocity * dt;
            foreach (var body in obstacles)
            {
                body.Position += body.Velocity * dt;
            }
        }

        throw new Exception("Failed to arrive within 600 simulated seconds; distance " + (state.Position - state.TargetPosition).Length + "; phases " + string.Join(", ", phases));
    }

    private static void NoPath()
    {
        var planner = new RoutePlanner();
        bool found = planner.TryPlan(new Vector(-2000, 0, 0), default, Bodies(), default, out _);
        Require(!found, "Planner accepted a goal inside an obstacle");
    }

    private static void GravityCompensation()
    {
        var state = new FlightState
        {
            Position             = default,
            Velocity             = new Vector(10, 0, 0),
            TargetPosition       = new Vector(1000, 0, 0),
            ExternalAcceleration = new Vector(0, -4, 0),
            Thrust               = 30,
            SpeedLimit           = 10,
            ArrivalRadius        = 100
        };
        var command = new FlightComputer().Step(state);
        Require(Math.Abs(command.Acceleration.Y - 4) < 1e-8, "Did not counteract measured gravity");
    }

    private static void TranslationInvariant()
    {
        var offset = new Vector(100000, -20000, 60000);
        var state = new FlightState
        {
            Position       = new Vector(-3500, 0, 0),
            TargetPosition = new Vector(3500, 0, 0),
            Thrust         = 30,
            ArrivalRadius  = 300
        };
        state.Obstacles.AddRange(Bodies());
        var first = new FlightComputer().Step(state);
        state.Position       += offset;
        state.TargetPosition += offset;
        foreach (var body in state.Obstacles)
        {
            body.Position += offset;
        }

        var second = new FlightComputer().Step(state);
        Require((first.Acceleration - second.Acceleration).Length < 1e-7, "Control depends on world origin");
    }

    private static void EmergencyBraking()
    {
        var state = new FlightState
        {
            Position       = new Vector(-1200, 0, 0),
            Velocity       = new Vector(150, 0, 0),
            TargetPosition = new Vector(3500, 0, 0),
            Thrust         = 30,
            ArrivalRadius  = 300
        };
        state.Obstacles.AddRange(Bodies());
        var command = new FlightComputer().Step(state);
        Require(command.Phase == FlightPhase.Escape && command.Acceleration.X < 0, "Did not brake an imminent collision");
    }

    private static void AcceleratingFrame()
    {
        var state = new FlightState
        {
            Position             = default,
            Velocity             = new Vector(10, 0, 0),
            TargetPosition       = new Vector(1000, 0, 0),
            TargetAcceleration   = new Vector(0, 2, 0),
            ExternalAcceleration = new Vector(0, -4, 0),
            Thrust               = 30,
            SpeedLimit           = 10,
            ArrivalRadius        = 100
        };
        var command = new FlightComputer().Step(state);
        Require(Math.Abs(command.Acceleration.Y - 6) < 1e-8, "Did not compensate relative acceleration");
    }

    private static void RandomApproaches()
    {
        var random = new Random(753640);
        for (int i = 0; i < 24; i++)
        {
            var direction = new Vector(random.NextDouble() - 0.5, random.NextDouble() - 0.5, random.NextDouble() - 0.5).Unit;
            var drift = new Vector(random.NextDouble() * 40, random.NextDouble() * 40, random.NextDouble() * 40);
            var result = Simulate(direction * -3500, drift, direction * 3500, default, Bodies(), true);
            Require(result.RapidAvoidanceChanges == 0, "Emergency avoidance is chattering on a 3D approach");
        }
    }

    private static void AutomaticSpeed()
    {
        var result = Simulate(default, default, new Vector(20000, 0, 0), default, [], false);
        Require(result.PeakSpeed > 600 && result.Time < 80, "A clear long route should exceed the old cruise limit and arrive sooner");
        var state = new FlightState
        {
            Position       = default,
            Velocity       = new Vector(250, 0, 0),
            TargetPosition = new Vector(20000, 0, 0),
            ArrivalRadius  = 300,
            Thrust         = 30
        };
        var accelerating = new FlightComputer().Step(state);
        Require(accelerating.Acceleration.X > 29 && accelerating.Phase == FlightPhase.Cruise, "Should keep accelerating above 200 m/s with ample stopping distance");
        state.TargetPosition = new Vector(2000, 0, 0);
        state.Velocity       = new Vector(400, 0, 0);
        var braking = new FlightComputer().Step(state);
        Require(braking.Acceleration.X < -29 && braking.Phase == FlightPhase.Braking, "Should brake at the stopping-distance threshold");
    }

    private static void AvoidanceRecovery()
    {
        var state = new FlightState
        {
            Position       = new Vector(-1200, 0, 0),
            Velocity       = new Vector(150, 0, 0),
            TargetPosition = new Vector(3500, 0, 0),
            ArrivalRadius  = 300,
            Thrust         = 30
        };
        state.Obstacles.AddRange(Bodies());
        var computer = new FlightComputer();
        Require(computer.Step(state).Phase == FlightPhase.Escape, "Imminent collision should engage avoidance immediately");
        state.Velocity = default;
        for (int step = 1; step <= 40; step++)
        {
            state.Time = step * 0.02;
            Require(computer.Step(state).Phase == FlightPhase.Escape, "Avoidance was released before recovery settled");
        }

        state.Time = 1.1;
        Require(computer.Step(state).Phase != FlightPhase.Escape, "Avoidance failed to release after recovery");
    }

    private static void ClearFlyby()
    {
        var state = new FlightState
        {
            Position       = new Vector(-1000, 1000, 0),
            Velocity       = new Vector(400, 0, 0),
            TargetPosition = new Vector(6000, 1000, 0),
            ArrivalRadius  = 300,
            Thrust         = 30
        };
        state.Obstacles.AddRange(Bodies());
        Require(new FlightComputer().Step(state).Phase != FlightPhase.Escape, "Radial closing speed falsely classified a clear flyby as a collision");
    }

    private static void SteeringWhileAccelerating()
    {
        var state = new FlightState
        {
            Position       = default,
            Velocity       = new Vector(250, 0, 0),
            TargetPosition = new Vector(20000, 1000, 0),
            ArrivalRadius  = 300,
            Thrust         = 30
        };
        var command = new FlightComputer().Step(state);
        Require(command.Acceleration.X > 20 && command.Acceleration.Y > 0, "Course correction should add lateral thrust while continuing forward acceleration");
        Require(command.Phase == FlightPhase.Cruise, "Routine steering should remain in the approach stage");
    }

    private static void ArrivalWithoutReversing(double dt, Vector targetVelocity, Vector gravity)
    {
        var state = new FlightState
        {
            Position             = new Vector(-1798, 0, 0),
            Velocity             = targetVelocity + new Vector(10, 0, 0),
            TargetPosition       = default,
            TargetVelocity       = targetVelocity,
            ExternalAcceleration = gravity,
            ArrivalRadius        = 1800,
            Thrust               = 30,
            DeltaTime            = dt
        };
        var computer = new FlightComputer();
        for (int step = 0; step < 200; step++)
        {
            state.Time = step * dt;
            var command = computer.Step(state);
            var velocity = state.Velocity - state.TargetVelocity;
            Require(velocity.X >= -0.01, "Reversed to chase the arrival radius");
            Require(command.Phase == FlightPhase.Braking || command.Phase == FlightPhase.Arrived, "Returned to approach during final velocity matching");
            if (command.Phase == FlightPhase.Arrived)
            {
                Require(state.Time < 1.5 && velocity.Length < 0.5, "Arrival matching was slow or incomplete");

                return;
            }

            state.Velocity       += (command.Acceleration + gravity) * dt;
            state.Position       += state.Velocity * dt;
            state.TargetPosition += targetVelocity * dt;
        }

        throw new Exception("Arrival velocity matching did not finish");
    }

    private static void OptionalSpeedLimit()
    {
        var state = new FlightState
        {
            Position       = default,
            Velocity       = new Vector(20, 0, 0),
            TargetPosition = new Vector(20000, 0, 0),
            ArrivalRadius  = 300,
            Thrust         = 30,
            SpeedLimit     = 10
        };
        var command = new FlightComputer().Step(state);
        Require(command.Phase == FlightPhase.Cruise && command.Acceleration.X < 0, "A configured speed cap was confused with final arrival braking");
    }

    private static void TwinApproach(bool otherTwin)
    {
        var start = new Vector(-14000, 0, 0);
        var bodies = new List<Obstacle>
        {
            new()
            {
                Position       = default,
                Radius         = 5500,
                PhysicalRadius = 2000
            },
            new()
            {
                Position       = new Vector(6000, 0, 0),
                Radius         = 700,
                PhysicalRadius = 300
            },
            new()
            {
                Position       = new Vector(5500, 600, 0),
                Radius         = 800,
                PhysicalRadius = 300
            }
        };
        var target = bodies[otherTwin ? 2 : 1].Position;
        var nearSide = target + (start - target).Unit * 1000;
        var planner = new RoutePlanner();
        Require(!planner.TryPlan(start, nearSide, bodies, default, out _), "Fixture must reproduce the blocked single-point approach");
        bool found = planner.TryPlanToTarget(start, target, 1000, bodies, default, out var goal, out var next, out _, out _);
        Require(found, "Failed to find an alternate approach to an accessible arrival region: " + planner.FailureReason);
        Require(Math.Abs((goal - target).Length - 1000) < 1e-6, "Alternate arrival point changed the arrival radius");
        Require(RoutePlanner.SegmentClear(start, next.Position, bodies), "First route segment crosses an avoidance zone");
        foreach (var body in bodies)
        {
            Require((goal - body.Position).Length >= body.Radius + 49.9, "Alternate arrival point reduced required clearance");
        }

        var state = new FlightState
        {
            Position       = start,
            TargetPosition = target,
            ArrivalRadius  = 1000,
            Thrust         = 30
        };
        state.Obstacles.AddRange(bodies);
        var command = new FlightComputer().Step(state);
        Require(command.Phase != FlightPhase.Hold && command.Acceleration.Length > 1, "Controller held instead of following the alternate route");
        Simulate(start, default, target, default, bodies, true, 1000);
    }

    private static void InterruptedArrival(bool avoidance)
    {
        var drift = new Vector(150, 20, 0);
        var state = new FlightState
        {
            Position             = new Vector(-6320, 0, 0),
            Velocity             = drift + new Vector(600, 0, 0),
            TargetVelocity       = drift,
            ExternalAcceleration = new Vector(0, -4, 0),
            ArrivalRadius        = 1000,
            Thrust               = 30,
            DeltaTime            = 0.02
        };
        var computer = new FlightComputer();
        Require(computer.Step(state).Phase == FlightPhase.Braking, "Fixture did not begin final braking");
        if (avoidance)
        {
            state.Obstacles.Add(new Obstacle
            {
                Position       = new Vector(-5120, 0, 0),
                Velocity       = drift,
                Radius         = 800,
                PhysicalRadius = 500
            });
            state.Time = 0.02;
            Require(computer.Step(state).Phase == FlightPhase.Escape, "Fixture did not interrupt braking with avoidance");
        }

        // Reproduce the reported post-avoidance state: matched velocity, still almost 3 km away.
        state.Position = new Vector(-2934, 0, 0);
        state.Velocity = drift;
        state.Time     = 1.1;
        if (avoidance)
        {
            Require(computer.Step(state).Phase == FlightPhase.Escape, "Avoidance released before its settling interval");
        }

        state.Time = 2.1;
        var resumed = computer.Step(state);
        Require(resumed.Phase == FlightPhase.Cruise && resumed.Acceleration.X > 0, "Distant matched velocity was mistaken for arrival instead of resuming approach");
        bool braking = false;
        for (int step = 0; step < 10000; step++)
        {
            var command = computer.Step(state);
            Require(command.Acceleration.IsFinite && command.Acceleration.Length <= state.Thrust + 1e-7, "Recovery exceeded available thrust");
            if (braking)
            {
                Require(command.Phase == FlightPhase.Braking || command.Phase == FlightPhase.Arrived, "Recovered final approach started oscillating");
            }

            braking |= command.Phase == FlightPhase.Braking;
            double distance = (state.Position - state.TargetPosition).Length;
            if (command.Phase == FlightPhase.Arrived)
            {
                Require(Math.Abs(distance - state.ArrivalRadius) < 60, "Recovery completed far from the destination");
                Require((state.Velocity - drift).Length < 0.5, "Recovery did not match target velocity");

                return;
            }

            foreach (var obstacle in state.Obstacles)
            {
                Require((state.Position - obstacle.Position).Length > obstacle.PhysicalRadius + 10, "Recovery collided with the avoided body");
                obstacle.Position += drift * state.DeltaTime;
            }

            state.Velocity       += (command.Acceleration + state.ExternalAcceleration) * state.DeltaTime;
            state.Position       += state.Velocity * state.DeltaTime;
            state.TargetPosition += drift * state.DeltaTime;
            state.Time           += state.DeltaTime;
        }

        throw new Exception("Recovered approach did not arrive");
    }

    private static void LocalThrustClearance()
    {
        var state = new FlightState
        {
            Position       = new Vector(-14000, 0, 0),
            TargetPosition = new Vector(4900, 0, 0),
            ArrivalRadius  = 1000,
            MaximumThrust  = 50,
            Thrust         = 50
        };
        Func<float, float> gravity = radius => (float)(7 * Math.Pow(8100 / radius, 2));
        double normalRadius = GravityClearance.CalculateRadius(2600, state, gravity);
        state.Thrust = 20;
        double limitedRadius = GravityClearance.CalculateRadius(2600, state, gravity);
        Require(Math.Abs(limitedRadius - normalRadius) < 1e-6, "A local thrust restriction inflated the distant Sun clearance zone");
        state.Obstacles.Add(new Obstacle
        {
            Radius         = limitedRadius,
            PhysicalRadius = 2500
        });
        var command = new FlightComputer().Step(state);
        Require(command.Phase == FlightPhase.Detour, "A local thrust restriction blocked the required twin-like detour");
        Require(command.Acceleration.Length > 1 && command.Acceleration.Length <= 20 + 1e-7, "Route planning bypassed the actual local thrust limit");

        state.Position             = new Vector(-limitedRadius - 300, 0, 0);
        state.Velocity             = new Vector(150, 0, 0);
        state.ExternalAcceleration = new Vector(7, 0, 0);
        var escape = new FlightComputer().Step(state);
        Require(escape.Phase == FlightPhase.Escape && escape.Acceleration.X < 0, "Local avoidance ignored reduced braking authority");
        Require(escape.Acceleration.Length <= 20 + 1e-7, "Emergency avoidance exceeded the local thrust limit");

        var weakerShip = new FlightState
        {
            MaximumThrust = 20,
            Thrust        = 20
        };
        double weakerRadius = GravityClearance.CalculateRadius(2600, weakerShip, gravity);
        Require(weakerRadius > normalRadius + 2000, "Clearance must still account for a genuinely weaker engine");
    }

    private static void CoveredArrival()
    {
        var bodies = new List<Obstacle>
        {
            new()
            {
                Position       = default,
                Radius         = 8000,
                PhysicalRadius = 2000
            }
        };
        var planner = new RoutePlanner();
        bool found = planner.TryPlanToTarget(new Vector(-14000, 0, 0), new Vector(6000, 0, 0), 1000, bodies, default, out _, out _, out _, out _);
        Require(!found, "Planner accepted a destination whose entire arrival region is obstructed");
        Require(planner.FailureReason.Contains("arrival point"), "Missing explanation for obstructed arrival region");
    }

    private static void VerifyGameBindings(string gamePath, string modPath)
    {
        using var gameStream = File.OpenRead(gamePath);
        using var modStream  = File.OpenRead(modPath);
        using var gamePe     = new PEReader(gameStream);
        using var modPe      = new PEReader(modStream);
        var game = gamePe.GetMetadataReader();
        var mod  = modPe.GetMetadataReader();
        var types = new Dictionary<string, TypeDefinitionHandle>();
        foreach (var handle in game.TypeDefinitions)
        {
            var type = game.GetTypeDefinition(handle);
            bool globalType = game.GetString(type.Namespace).Length == 0 && type.GetDeclaringType().IsNil;
            if (globalType)
            {
                types[game.GetString(type.Name)] = handle;
            }
        }

        int verified = 0;
        foreach (var handle in mod.MemberReferences)
        {
            var member = mod.GetMemberReference(handle);
            if (member.Parent.Kind != HandleKind.TypeReference)
            {
                continue;
            }

            var type = mod.GetTypeReference((TypeReferenceHandle)member.Parent);
            string typeName   = mod.GetString(type.Name);
            string memberName = mod.GetString(member.Name);
            bool isGameType = mod.GetString(type.Namespace).Length == 0 && types.ContainsKey(typeName);
            if (!isGameType)
            {
                continue;
            }

            bool found = false;
            var current = types[typeName];
            while (!current.IsNil)
            {
                var definition = game.GetTypeDefinition(current);
                if (member.GetKind() == MemberReferenceKind.Field)
                {
                    foreach (var field in definition.GetFields())
                    {
                        var fieldDefinition = game.GetFieldDefinition(field);
                        bool matches = game.GetString(fieldDefinition.Name) == memberName;
                        if (matches)
                        {
                            Require((fieldDefinition.Attributes & FieldAttributes.FieldAccessMask) == FieldAttributes.Public, "Direct access to non-public game field: " + typeName + "." + memberName);
                            found = true;
                        }
                    }
                }
                else
                {
                    foreach (var method in definition.GetMethods())
                    {
                        var methodDefinition = game.GetMethodDefinition(method);
                        found |= game.GetString(methodDefinition.Name) == memberName
                            && (methodDefinition.Attributes & MethodAttributes.MemberAccessMask) == MethodAttributes.Public;
                    }
                }

                current = definition.BaseType.Kind == HandleKind.TypeDefinition ? (TypeDefinitionHandle)definition.BaseType : default;
            }

            Require(found, "Missing game binding: " + typeName + "." + memberName);
            verified++;
        }

        foreach (string hook in new[] { "ReadTranslationalInput", "FlyToDestination", "OnDisable" })
        {
            bool found = false;
            foreach (var method in game.GetTypeDefinition(types["Autopilot"]).GetMethods())
            {
                found |= game.GetString(game.GetMethodDefinition(method).Name) == hook;
            }

            Require(found, "Missing patch hook: " + hook);
        }

        bool launchHook = false;
        foreach (var method in game.GetTypeDefinition(types["ShipCockpitController"]).GetMethods())
        {
            launchHook |= game.GetString(game.GetMethodDefinition(method).Name) == "IsAutopilotAvailable";
        }

        Require(launchHook, "Missing landed launch availability hook");
        Console.WriteLine($"PASS: {verified} referenced game members and all 4 Harmony hook names exist; no direct non-public game field access. Runtime patching still needs an in-game test.");
    }

    private static void Require(bool condition, string message)
    {
        if (!condition)
        {
            throw new Exception(message);
        }
    }
}
