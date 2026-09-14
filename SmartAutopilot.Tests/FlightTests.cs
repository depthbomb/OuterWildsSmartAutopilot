using System;
using System.Collections.Generic;
using SmartAutopilot.Navigation;
using Xunit;

[assembly: CollectionBehavior(DisableTestParallelization = true)]

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

public sealed class FlightTests
{
    [Fact(DisplayName = "Distant collision checks exclude fictitious surface rotation and retain real braking")]
    public void DistantCollisionChecksExcludeFictitiousSurfaceRotationAndRetainRealBraking()
    {
        CollisionFrameTests.DistantRotation();
    }

    [Fact(DisplayName = "Automation requests reject stale, unbounded and invalid scenarios")]
    public void AutomationRequestsRejectStaleUnboundedAndInvalidScenarios()
    {
        AutomationRequestTests.ValidateRequests();
    }

    [Fact(DisplayName = "Arrival restores distance after inward avoidance across thrust levels and timesteps")]
    public void ArrivalRestoresDistanceAfterInwardAvoidanceAcrossThrustLevelsAndTimesteps()
    {
        ArrivalRecoveryTests.RecoverDistance();
    }

    [Fact(DisplayName = "Arrival distance recovery yields to collision avoidance")]
    public void ArrivalDistanceRecoveryYieldsToCollisionAvoidance()
    {
        ArrivalRecoveryTests.AvoidancePriority();
    }

    [Fact(DisplayName = "Velocity-matched satellite arrivals finish across the braking boundary")]
    public void VelocityMatchedSatelliteArrivalsFinishAcrossTheBrakingBoundary()
    {
        ArrivalRecoveryTests.SatelliteArrivalBoundary();
    }

    [Fact(DisplayName = "Nearby collision frames preserve rotating-surface motion")]
    public void NearbyCollisionFramesPreserveRotatingSurfaceMotion()
    {
        CollisionFrameTests.SurfaceFrame();
    }

    [Fact(DisplayName = "Collision holds resume only after sustained fresh clearance")]
    public void CollisionHoldsResumeOnlyAfterSustainedFreshClearance()
    {
        CollisionFrameTests.ClearHold();
    }

    [Fact(DisplayName = "Distant straight-sweep hits require a checked turning and stopping path")]
    public void DistantStraightSweepHitsRequireACheckedTurningAndStoppingPath()
    {
        CollisionFrameTests.DistantTurn();
    }

    [Fact(DisplayName = "Arrival braking anticipates the destination's reduced-thrust region")]
    public void ArrivalBrakingAnticipatesTheDestinationSReducedThrustRegion()
    {
        ArrivalThrustTests.Transition();
    }

    [Fact(DisplayName = "Arrival thrust limits respect geometry, overlap and invalid data")]
    public void ArrivalThrustLimitsRespectGeometryOverlapAndInvalidData()
    {
        ArrivalThrustTests.BoundsAndValidation();
    }

    [Fact(DisplayName = "A comet collision hold yields to checked solar escape or manual control")]
    public void ACometCollisionHoldYieldsToCheckedSolarEscapeOrManualControl()
    {
        CollisionEscapeTests.CometNearSun();
    }

    [Fact(DisplayName = "Emergency escape checks lateral corridors, braking and celestial hazards")]
    public void EmergencyEscapeChecksLateralCorridorsBrakingAndCelestialHazards()
    {
        CollisionEscapeTests.ObstructedEscape();
    }

    [Fact(DisplayName = "An inaccessible comet does not pull a holding ship toward the Sun")]
    public void AnInaccessibleCometDoesNotPullAHoldingShipTowardTheSun()
    {
        PlaytestRegressionTests.BlockedCometHold();
    }

    [Fact(DisplayName = "Moving moon approaches shed lateral momentum and arrive promptly")]
    public void MovingMoonApproachesShedLateralMomentumAndArrivePromptly()
    {
        MoonApproachTests.Verify();
    }

    [Fact(DisplayName = "Recovered moon approach avoids another escape across physics timesteps")]
    public void RecoveredMoonApproachAvoidsAnotherEscapeAcrossPhysicsTimesteps()
    {
        MoonApproachTests.Recovery();
    }

    [Fact(DisplayName = "Nested station orbits preserve primary and local collision protection")]
    public void NestedStationOrbitsPreservePrimaryAndLocalCollisionProtection()
    {
        SatelliteSafetyTests.ProtectedStation();
    }

    [Fact(DisplayName = "Moving-primary satellite forecasts exclude clear flybys and retain real encounters")]
    public void MovingPrimarySatelliteForecastsExcludeClearFlybysAndRetainRealEncounters()
    {
        SatelliteSafetyTests.MovingPrimaryForecast();
    }

    [Fact(DisplayName = "Orbital target approaches finish without repeated solar escapes")]
    public void OrbitalTargetApproachesFinishWithoutRepeatedSolarEscapes()
    {
        OrbitalApproachTests.Verify();
    }

    [Fact(DisplayName = "Orbital predictions respect moving frames and invalid data")]
    public void OrbitalPredictionsRespectMovingFramesAndInvalidData()
    {
        OrbitalApproachTests.Prediction();
    }

    [Fact(DisplayName = "Satellite tangent forecasts cannot leave their whole orbit")]
    public void SatelliteTangentForecastsCannotLeaveTheirWholeOrbit()
    {
        OrbitalApproachTests.SatelliteTangent();
    }

    [Fact(DisplayName = "Blocked orbital forecasts preserve a valid current approach")]
    public void BlockedOrbitalForecastsPreserveAValidCurrentApproach()
    {
        OrbitalApproachTests.ForecastFallback();
    }

    [Fact(DisplayName = "Blocked-route warnings survive emergency clearance")]
    public void BlockedRouteWarningsSurviveEmergencyClearance()
    {
        PlaytestRegressionTests.WarningAcrossEscape();
    }

    [Fact(DisplayName = "Avoidance priorities resist jitter without delaying immediate threats")]
    public void AvoidancePrioritiesResistJitterWithoutDelayingImmediateThreats()
    {
        PlaytestRegressionTests.AvoidancePriority();
    }

    [Fact(DisplayName = "Irregular hull bounds cover rotated and offset geometry and exterior arrivals")]
    public void IrregularHullBoundsCoverRotatedAndOffsetGeometryAndExteriorArrivals()
    {
        PlaytestRegressionTests.IrregularHullEnvelope();
    }

    [Fact(DisplayName = "Flights detour around an extended hull from rest and at speed")]
    public void FlightsDetourAroundAnExtendedHullFromRestAndAtSpeed()
    {
        PlaytestRegressionTests.IrregularHullFlight();
    }

    [Fact(DisplayName = "Detours identify their obstacle and recover to cruise")]
    public void DetoursIdentifyTheirObstacleAndRecoverToCruise()
    {
        BetaSafetyTests.DetourStatus();
    }

    [Fact(DisplayName = "Collision braking stops before a swept obstacle")]
    public void CollisionBrakingStopsBeforeASweptObstacle()
    {
        BetaSafetyTests.CollisionStop();
    }

    [Fact(DisplayName = "Collision holds track accelerating reference frames")]
    public void CollisionHoldsTrackAcceleratingReferenceFrames()
    {
        BetaSafetyTests.MovingCollisionHold();
    }

    [Fact(DisplayName = "Fault retries are deliberate and bounded")]
    public void FaultRetriesAreDeliberateAndBounded()
    {
        BetaSafetyTests.FaultRecovery();
    }

    [Fact(DisplayName = "Prolonged holds warn on time and fuel consumption")]
    public void ProlongedHoldsWarnOnTimeAndFuelConsumption()
    {
        BetaSafetyTests.HoldWarnings();
    }

    [Fact(DisplayName = "Flight diagnostics retain only the latest minute")]
    public void FlightDiagnosticsRetainOnlyTheLatestMinute()
    {
        BetaSafetyTests.FlightHistory();
    }

    [Fact(DisplayName = "Invalid game data cannot reach collision queries")]
    public void InvalidGameDataCannotReachCollisionQueries()
    {
        BetaSafetyTests.InvalidState();
    }

    [Fact(DisplayName = "Interrupted braking resumes after avoidance")]
    public void InterruptedBrakingResumesAfterAvoidance()
    {
        InterruptedArrival(true);
    }

    [Fact(DisplayName = "A stalled distant approach resumes with a moving target")]
    public void AStalledDistantApproachResumesWithAMovingTarget()
    {
        InterruptedArrival(false);
    }

    [Fact(DisplayName = "Local thrust restrictions do not obstruct distant twin routes")]
    public void LocalThrustRestrictionsDoNotObstructDistantTwinRoutes()
    {
        LocalThrustClearance();
    }

    [Fact(DisplayName = "Clear route and arrival")]
    public void ClearRouteAndArrival()
    {
        Simulate(new Vector(-3500, 0, 0), default, new Vector(3500, 0, 0), default, [], false);
    }

    [Fact(DisplayName = "Detour with inverse-square gravity")]
    public void DetourWithInverseSquareGravity()
    {
        Simulate(new Vector(-3500, 0, 0), default, new Vector(3500, 0, 0), default, Bodies(), true);
    }

    [Fact(DisplayName = "Moving obstacle and moving destination")]
    public void MovingObstacleAndMovingDestination()
    {
        Simulate(new Vector(-3500, 0, 0), default, new Vector(3500, 0, 0), new Vector(0, 8, 0), Bodies(new Vector(0, 10, 0)), true);
    }

    [Fact(DisplayName = "Departure inside clearance zone")]
    public void DepartureInsideClearanceZone()
    {
        Simulate(new Vector(-650, 0, 0), default, new Vector(3500, 0, 0), default, Bodies(), true);
    }

    [Fact(DisplayName = "Two blocking planets")]
    public void TwoBlockingPlanets()
    {
        var bodies = Bodies();
        bodies.Add(new Obstacle
        {
            Position       = new Vector(1800, -250, 0),
            Radius         = 700,
            PhysicalRadius = 450
        });
        Simulate(new Vector(-3500, 0, 0), default, new Vector(4500, 0, 0), default, bodies, true);
    }

    [Fact(DisplayName = "No path is reported explicitly")]
    public void NoPathIsReportedExplicitly()
    {
        NoPath();
    }

    [Fact(DisplayName = "Floating-origin translation leaves control unchanged")]
    public void FloatingOriginTranslationLeavesControlUnchanged()
    {
        TranslationInvariant();
    }

    [Fact(DisplayName = "Emergency braking intervenes")]
    public void EmergencyBrakingIntervenes()
    {
        EmergencyBraking();
    }

    [Fact(DisplayName = "Inverse-distance gravity")]
    public void InverseDistanceGravity()
    {
        Simulate(new Vector(-3500, 0, 0), default, new Vector(3500, 0, 0), default, Bodies(), true, 300, 1);
    }

    [Fact(DisplayName = "Accelerating reference-frame compensation")]
    public void AcceleratingReferenceFrameCompensation()
    {
        AcceleratingFrame();
    }

    [Fact(DisplayName = "Four 3D approaches")]
    public void Four3DApproaches()
    {
        RandomApproaches();
    }

    [Fact(DisplayName = "Automatic long-distance acceleration and braking")]
    public void AutomaticLongDistanceAccelerationAndBraking()
    {
        AutomaticSpeed();
    }

    [Fact(DisplayName = "Avoidance remains engaged until recovery settles")]
    public void AvoidanceRemainsEngagedUntilRecoverySettles()
    {
        AvoidanceRecovery();
    }

    [Fact(DisplayName = "A clear flyby does not trigger radial emergency braking")]
    public void AClearFlybyDoesNotTriggerRadialEmergencyBraking()
    {
        ClearFlyby();
    }

    [Fact(DisplayName = "Planet approach has no rapid avoidance switching")]
    public void PlanetApproachHasNoRapidAvoidanceSwitching()
    {
        var result = Simulate(new Vector(-3500, 0, 0), default, default, default, Bodies(), true, 900);
        Assert.True(result.RapidAvoidanceChanges == 0, "Emergency avoidance is chattering during planet arrival");
    }

    [Fact(DisplayName = "Moving target arrival with gravity and a different timestep")]
    public void MovingTargetArrivalWithGravityAndADifferentTimestep()
    {
        ArrivalWithoutReversing(1.0 / 60, new Vector(150, 20, 0), new Vector(0, -4, 0));
    }

    [Fact(DisplayName = "Optional speed limit does not start final braking early")]
    public void OptionalSpeedLimitDoesNotStartFinalBrakingEarly()
    {
        OptionalSpeedLimit();
    }

    [Fact(DisplayName = "Initial flight away from the target recovers")]
    public void InitialFlightAwayFromTheTargetRecovers()
    {
        Simulate(new Vector(-3500, 0, 0), new Vector(-100, 0, 0), new Vector(3500, 0, 0), default, [], false);
    }

    [Fact(DisplayName = "Distant twin-like target with an obstructed near-side arrival")]
    public void DistantTwinLikeTargetWithAnObstructedNearSideArrival()
    {
        TwinApproach(false);
    }

    [Fact(DisplayName = "Other twin with overlapping stellar and sibling clearance")]
    public void OtherTwinWithOverlappingStellarAndSiblingClearance()
    {
        TwinApproach(true);
    }

    [Fact(DisplayName = "Fully obstructed arrival region still holds")]
    public void FullyObstructedArrivalRegionStillHolds()
    {
        CoveredArrival();
    }

    [Fact(DisplayName = "Cold launch from a moving surface")]
    public void ColdLaunchFromAMovingSurface()
    {
        DepartureTests.Launch(true, 0.02);
    }

    [Fact(DisplayName = "Warm launch at a different physics timestep")]
    public void WarmLaunchAtADifferentPhysicsTimestep()
    {
        DepartureTests.Launch(false, 1.0 / 60);
    }

    [Fact(DisplayName = "Blocked and interrupted launch ignition")]
    public void BlockedAndInterruptedLaunchIgnition()
    {
        DepartureTests.BlockedIgnition();
    }

    [Fact(DisplayName = "Obstructed climb brakes without repeated restarts")]
    public void ObstructedClimbBrakesWithoutRepeatedRestarts()
    {
        DepartureTests.ObstructionDuringClimb();
    }

    [Fact(DisplayName = "Launch respects local thrust and detects a stuck ship")]
    public void LaunchRespectsLocalThrustAndDetectsAStuckShip()
    {
        DepartureTests.ThrustAndProgressLimits();
    }

    [Fact(DisplayName = "Cancelled launch starts with fresh ignition")]
    public void CancelledLaunchStartsWithFreshIgnition()
    {
        DepartureTests.CancelAndRestart();
    }

    [Fact(DisplayName = "Tilted launch clears the departure sphere")]
    public void TiltedLaunchClearsTheDepartureSphere()
    {
        DepartureTests.ClearanceGeometry();
    }

    [Fact(DisplayName = "Launch hands off to obstacle-avoiding navigation")]
    public void LaunchHandsOffToObstacleAvoidingNavigation()
    {
        DepartureTests.NavigationHandoff();
    }

    [Fact(DisplayName = "Interrupted climb resumes and completes")]
    public void InterruptedClimbResumesAndCompletes()
    {
        DepartureTests.ResumeInterruptedClimb();
    }

    [Fact(DisplayName = "Unrelated or expired launch context is not resumed")]
    public void UnrelatedOrExpiredLaunchContextIsNotResumed()
    {
        DepartureTests.RejectUnrelatedResume();
    }

    [Fact(DisplayName = "Resumed climb checks overhead clearance again")]
    public void ResumedClimbChecksOverheadClearanceAgain()
    {
        DepartureTests.ResumeRechecksObstruction();
    }

    [Fact(DisplayName = "Launch matrix across gravity, rotation, slope and timesteps")]
    public void LaunchMatrixAcrossGravityRotationSlopeAndTimesteps()
    {
        DepartureReliabilityTests.SurfaceMatrix();
    }

    [Fact(DisplayName = "Descending interrupted launch recovers before the surface")]
    public void DescendingInterruptedLaunchRecoversBeforeTheSurface()
    {
        DepartureReliabilityTests.ResumeDuringDescent();
    }

    [Fact(DisplayName = "Trapped airborne launch fails without restarting itself")]
    public void TrappedAirborneLaunchFailsWithoutRestartingItself()
    {
        DepartureReliabilityTests.AirborneStall();
    }

    [Fact(DisplayName = "Resume follows moving surface and rejects rewound time")]
    public void ResumeFollowsMovingSurfaceAndRejectsRewoundTime()
    {
        DepartureReliabilityTests.RotatingResume();
    }

    [Fact(DisplayName = "Moving satellite near a large planet arrival")]
    public void MovingSatelliteNearALargePlanetArrival()
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
    }

    [Fact(DisplayName = "Clear final braking is shorter with stable arrival")]
    public void ClearFinalBrakingIsShorterWithStableArrival()
    {
        BrakingTests.VerifyRefinement();
    }

    [Fact(DisplayName = "Faster arrival respects target-body emergency clearance")]
    public void FasterArrivalRespectsTargetBodyEmergencyClearance()
    {
        BrakingTests.TargetClearance();
    }

    [Fact(DisplayName = "Braking preserves stopping thrust while removing lateral momentum")]
    public void BrakingPreservesStoppingThrustWhileRemovingLateralMomentum()
    {
        BrakingTests.LateralMomentum();
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
        var computer              = new FlightComputer();
        var phases                = new HashSet<FlightPhase>();
        var minimumClearance      = double.PositiveInfinity;
        var peakSpeed             = (double)0;
        var lastAvoidanceChange   = (double)-100;
        var rapidAvoidanceChanges = 0;
        var wasAvoiding           = false;
        var finalBrakingStarted   = false;

        const double dt = 0.05;

        for (int step = 0; step < 12000; step++)
        {
            state.Time = step * dt;
            state.ExternalAcceleration = default;
            foreach (var body in obstacles)
            {
                var offset = body.Position - state.Position;
                minimumClearance = Math.Min(minimumClearance, offset.Length - body.PhysicalRadius);
                Assert.True(offset.Length > body.PhysicalRadius + 10, "Collision at " + state.Time);
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
                Assert.True(command.Phase != FlightPhase.Cruise && command.Phase != FlightPhase.Detour, "Resumed approach after starting final braking");
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
            Assert.True(command.Acceleration.IsFinite, "Non-finite thrust");
            Assert.True(command.Acceleration.Length <= state.Thrust + 1e-7, "Exceeded available thrust");

            if (command.Phase == FlightPhase.Arrived)
            {
                Assert.True((state.Velocity - targetVelocity).Length < 1, "Did not match target velocity");
                Assert.True(Math.Abs((state.Position - state.TargetPosition).Length - arrival) < Math.Max(60, arrival * 0.15), "Stopped outside the arrival zone: " + (state.Position - state.TargetPosition).Length + " at " + state.Time);

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
        Assert.True(!found, "Planner accepted a goal inside an obstacle");
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
        Assert.True((first.Acceleration - second.Acceleration).Length < 1e-7, "Control depends on world origin");
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
        Assert.True(command.Phase == FlightPhase.Escape && command.Acceleration.X < 0, "Did not brake an imminent collision");
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
        Assert.True(Math.Abs(command.Acceleration.Y - 6) < 1e-8, "Did not compensate relative acceleration");
    }

    private static void RandomApproaches()
    {
        var random = new Random(753640);
        for (int i = 0; i < 4; i++)
        {
            var direction = new Vector(random.NextDouble() - 0.5, random.NextDouble() - 0.5, random.NextDouble() - 0.5).Unit;
            var drift = new Vector(random.NextDouble() * 40, random.NextDouble() * 40, random.NextDouble() * 40);
            var result = Simulate(direction * -3500, drift, direction * 3500, default, Bodies(), true);
            Assert.True(result.RapidAvoidanceChanges == 0, "Emergency avoidance is chattering on a 3D approach");
        }
    }

    private static void AutomaticSpeed()
    {
        var result = Simulate(default, default, new Vector(20000, 0, 0), default, [], false);
        Assert.True(result.PeakSpeed > 600 && result.Time < 80, "A clear long route should exceed the old cruise limit and arrive sooner");

        var state = new FlightState
        {
            Position       = default,
            Velocity       = new Vector(250, 0, 0),
            TargetPosition = new Vector(20000, 0, 0),
            ArrivalRadius  = 300,
            Thrust         = 30
        };
        var accelerating = new FlightComputer().Step(state);
        Assert.True(accelerating.Acceleration.X > 29 && accelerating.Phase == FlightPhase.Cruise, "Should keep accelerating above 200 m/s with ample stopping distance");

        state.TargetPosition = new Vector(2000, 0, 0);
        state.Velocity       = new Vector(400, 0, 0);
        var braking = new FlightComputer().Step(state);
        Assert.True(braking.Acceleration.X < -29 && braking.Phase == FlightPhase.Braking, "Should brake at the stopping-distance threshold");
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
        Assert.True(computer.Step(state).Phase == FlightPhase.Escape, "Imminent collision should engage avoidance immediately");

        state.Velocity = default;
        for (int step = 1; step <= 40; step++)
        {
            state.Time = step * 0.02;
            Assert.True(computer.Step(state).Phase == FlightPhase.Escape, "Avoidance was released before recovery settled");
        }

        state.Time = 1.1;
        Assert.True(computer.Step(state).Phase != FlightPhase.Escape, "Avoidance failed to release after recovery");
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
        Assert.True(new FlightComputer().Step(state).Phase != FlightPhase.Escape, "Radial closing speed falsely classified a clear flyby as a collision");
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
            Assert.True(velocity.X >= -0.01, "Reversed to chase the arrival radius");
            Assert.True(command.Phase == FlightPhase.Braking || command.Phase == FlightPhase.Arrived, "Returned to approach during final velocity matching");

            if (command.Phase == FlightPhase.Arrived)
            {
                Assert.True(state.Time < 1.5 && velocity.Length < 0.5, "Arrival matching was slow or incomplete");

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
        Assert.True(command.Phase == FlightPhase.Cruise && command.Acceleration.X < 0, "A configured speed cap was confused with final arrival braking");
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
        Assert.True(!planner.TryPlan(start, nearSide, bodies, default, out _), "Fixture must reproduce the blocked single-point approach");

        var found = planner.TryPlanToTarget(start, target, 1000, bodies, default, out var goal, out var next, out _, out _);
        Assert.True(found, "Failed to find an alternate approach to an accessible arrival region: " + planner.FailureReason);
        Assert.True(Math.Abs((goal - target).Length - 1000) < 1e-6, "Alternate arrival point changed the arrival radius");
        Assert.True(RoutePlanner.SegmentClear(start, next.Position, bodies), "First route segment crosses an avoidance zone");

        foreach (var body in bodies)
        {
            Assert.True((goal - body.Position).Length >= body.Radius + 49.9, "Alternate arrival point reduced required clearance");
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
        Assert.True(command.Phase != FlightPhase.Hold && command.Acceleration.Length > 1, "Controller held instead of following the alternate route");

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
        Assert.True(computer.Step(state).Phase == FlightPhase.Braking, "Fixture did not begin final braking");

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
            Assert.True(computer.Step(state).Phase == FlightPhase.Escape, "Fixture did not interrupt braking with avoidance");
        }

        // Reproduce the reported post-avoidance state: matched velocity, still almost 3 km away.
        state.Position = new Vector(-2934, 0, 0);
        state.Velocity = drift;
        state.Time     = 1.1;
        if (avoidance)
        {
            Assert.True(computer.Step(state).Phase == FlightPhase.Escape, "Avoidance released before its settling interval");
        }

        state.Time = 2.1;
        var resumed = computer.Step(state);
        Assert.True(resumed is { Phase: FlightPhase.Cruise, Acceleration.X: > 0 }, "Distant matched velocity was mistaken for arrival instead of resuming approach");

        var braking = false;
        for (int step = 0; step < 10000; step++)
        {
            var command = computer.Step(state);
            Assert.True(command.Acceleration.IsFinite && command.Acceleration.Length <= state.Thrust + 1e-7, "Recovery exceeded available thrust");

            if (braking)
            {
                Assert.True(command.Phase is FlightPhase.Braking or FlightPhase.Arrived, "Recovered final approach started oscillating");
            }

            braking |= command.Phase == FlightPhase.Braking;
            var distance = (state.Position - state.TargetPosition).Length;
            if (command.Phase == FlightPhase.Arrived)
            {
                Assert.True(Math.Abs(distance - state.ArrivalRadius) < 60, "Recovery completed far from the destination");
                Assert.True((state.Velocity - drift).Length < 0.5, "Recovery did not match target velocity");
                return;
            }

            foreach (var obstacle in state.Obstacles)
            {
                Assert.True((state.Position - obstacle.Position).Length > obstacle.PhysicalRadius + 10, "Recovery collided with the avoided body");

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
        var normalRadius = GravityClearance.CalculateRadius(2600, state, gravity);

        state.Thrust = 20;

        var limitedRadius = GravityClearance.CalculateRadius(2600, state, gravity);
        Assert.True(Math.Abs(limitedRadius - normalRadius) < 1e-6, "A local thrust restriction inflated the distant Sun clearance zone");

        state.Obstacles.Add(new Obstacle
        {
            Radius         = limitedRadius,
            PhysicalRadius = 2500
        });
        var command = new FlightComputer().Step(state);
        Assert.True(command.Phase == FlightPhase.Detour, "A local thrust restriction blocked the required twin-like detour");
        Assert.True(command.Acceleration.Length is > 1 and <= 20 + 1e-7, "Route planning bypassed the actual local thrust limit");

        state.Position             = new Vector(-limitedRadius - 300, 0, 0);
        state.Velocity             = new Vector(150, 0, 0);
        state.ExternalAcceleration = new Vector(7, 0, 0);

        var escape = new FlightComputer().Step(state);
        Assert.True(escape is { Phase: FlightPhase.Escape, Acceleration.X: < 0 }, "Local avoidance ignored reduced braking authority");
        Assert.True(escape.Acceleration.Length <= 20 + 1e-7, "Emergency avoidance exceeded the local thrust limit");

        var weakerShip = new FlightState
        {
            MaximumThrust = 20,
            Thrust        = 20
        };
        var weakerRadius = GravityClearance.CalculateRadius(2600, weakerShip, gravity);
        Assert.True(weakerRadius > normalRadius + 2000, "Clearance must still account for a genuinely weaker engine");
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
        var found   = planner.TryPlanToTarget(new Vector(-14000, 0, 0), new Vector(6000, 0, 0), 1000, bodies, default, out _, out _, out _, out _);
        Assert.True(!found, "Planner accepted a destination whose entire arrival region is obstructed");
        Assert.True(planner.FailureReason.Contains("arrival point"), "Missing explanation for obstructed arrival region");
    }


}
