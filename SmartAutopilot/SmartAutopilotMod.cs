using System.Reflection;
using HarmonyLib;
using OWML.Common;
using OWML.ModHelper;
using SmartAutopilot.Navigation;
using UnityEngine;
using Vector = SmartAutopilot.Navigation.Vector;

namespace SmartAutopilot;

public sealed class SmartAutopilotMod : ModBehaviour
{
    private Autopilot                   _pilot;
    private FlightComputer              _computer;
    private FlightPhase?                _lastPhase;
    private int                         _lastAvoidanceBody = -1;
    private string                      _lastCollisionReference;
    private float                       _speedLimit;
    private float                       _clearance  = 100;
    private bool                        _logChanges = true;
    private bool                        _logSamples;
    private bool                        _showDebug;
    private bool                        _debugUnavailable;
    private NavigationDebugOverlay      _debugOverlay;
    private float                       _debugRefreshRate = 60;
    private bool                        _debugHasCommand;
    private FlightCommand               _debugCommand;
    private bool                        _finishing;
    private bool                        _handlingFailure;
    private bool                        _holdNotified;
    private double                      _nextHoldUi;
    private string                      _flightContext;
    private ReferenceFrame              _flightTarget;
    private ShipCollisionGuard          _collision;
    private ShipArrivalLimits           _arrivalLimits;
    private bool                        _arrivalLimitsLogged;
    private FlightFaults                _faults = new();
    private HoldMonitor                 _hold   = new();
    private double                      _nextFlightSample;
    private Harmony                     _harmony;
    private ShipDepartureController     _departure;
    private OWRigidbody                 _departureShip;
    private bool                        _departureUnavailable;
    private bool                        _sceneLoading;
    private Automation.AutomationRunner _automation;

    private readonly List<AstroObject>    _bodies          = [];
    private readonly List<NavigationHull> _navigationHulls = [];
    private readonly FlightState          _state           = new();
    private readonly FlightRecorder       _recorder        = new();
    internal static  SmartAutopilotMod    Instance { get; private set; }

    public override void Configure(IModConfig config)
    {
        var oldSpeed     = _speedLimit;
        var oldClearance = _clearance;

        _speedLimit       = ReadNumber(config, "Optional speed limit (m/s, 0 = automatic)", 0, 0, 5000);
        _clearance        = ReadNumber(config, "Extra clearance (m)", 100, 40, 500);
        _logChanges       = config.GetSettingsValue<bool>("Log navigation changes");
        _logSamples       = config.GetSettingsValue<bool>("Log detailed flight samples");
        _showDebug        = config.GetSettingsValue<bool>("Show navigation debug overlay");
        _debugRefreshRate = ReadNumber(config, "Debug overlay refresh rate (Hz)", 60, 10, 120);
        _debugUnavailable = false;
        if (!_showDebug)
        {
            DisposeDebug();
        }

        var navigationChanged = oldSpeed != _speedLimit || oldClearance != _clearance;
        if (navigationChanged && _computer != null)
        {
            _computer = new FlightComputer();
        }
    }

    public void Awake()
    {
        GameFields.Validate();
        Instance = this;
        _harmony = new Harmony("Depthbomb.SmartAutopilot");
        _harmony.PatchAll(Assembly.GetExecutingAssembly());
        LoadManager.OnStartSceneLoad    += OnStartSceneLoad;
        LoadManager.OnCompleteSceneLoad += OnCompleteSceneLoad;
    }

    public void Start()
    {
        ModHelper.Console.WriteLine("Smart Autopilot " + ModHelper.Manifest.Version + " loaded; game " + Application.version + "; runtime field bindings validated.", MessageType.Success);
        _automation = Automation.AutomationRunner.TryStart(message => SafeLog(message, MessageType.Info));
    }

    public void Update()
    {
        _automation?.Update();
        if (_sceneLoading)
        {
            return;
        }

        try
        {
            if (_departure != null && _departure.Ship == null)
            {
                _departure.Dispose();
                _departure            = null;
                _departureUnavailable = false;
            }

            if (PlayerState.AtFlightConsole())
            {
                EnsureDeparture();
            }

            _departure?.UpdateUi();
        }
        catch (Exception exception)
        {
            DisableDeparture(exception);
        }
    }

    public void OnDestroy()
    {
        _automation?.Dispose();
        LoadManager.OnStartSceneLoad    -= OnStartSceneLoad;
        LoadManager.OnCompleteSceneLoad -= OnCompleteSceneLoad;
        try
        {
            _departure?.Dispose();
        }
        finally
        {
            DisposeDebug();
            _harmony?.UnpatchSelf();
            Instance = null;
        }
    }

    public void LateUpdate()
    {
        if (!_showDebug || _debugUnavailable || !_debugHasCommand || _sceneLoading || _computer == null)
        {
            return;
        }

        try
        {
            _debugOverlay ??= new NavigationDebugOverlay();
            _debugOverlay.Draw(_state, _computer, _debugCommand, _collision, _departure, _debugRefreshRate);
        }
        catch (Exception exception)
        {
            _debugUnavailable = true;
            DisposeDebug();
            SafeLog("Navigation debug overlay disabled: " + exception, MessageType.Error);
            Notify("Navigation debug overlay unavailable\nFlight controls remain active");
        }
    }

    internal void ResetFlight()
    {
        var hadFlight = _computer != null && _flightContext != null && _logChanges;
        if (hadFlight)
        {
            var phase = _departure?.Active == true ? "Departure/" + _departure.Phase : _lastPhase?.ToString() ?? "Starting";
            SafeLog($"Smart Autopilot control released: last phase={phase}; remaining={(_state.TargetPosition - _state.Position).Length - _state.ArrivalRadius:F1}m; relative speed={(_state.Velocity - _state.TargetVelocity).Length:F1}m/s; {_flightContext}", MessageType.Info);
        }

        _departure?.Cancel();
        _pilot                  = null;
        _computer               = null;
        _lastPhase              = null;
        _lastAvoidanceBody      = -1;
        _lastCollisionReference = null;
        _finishing              = false;
        _nextFlightSample       = 0;
        _flightTarget           = null;
        _collision              = null;
        _arrivalLimits          = null;
        _state.ArrivalThrustLimits.Clear();
        _hold            = new HoldMonitor();
        _holdNotified    = false;
        _nextHoldUi      = 0;
        _debugHasCommand = false;
        try
        {
            _debugOverlay?.Clear();
        }
        catch (Exception exception)
        {
            DisposeDebug();
            SafeLog("Navigation debug reset: " + exception, MessageType.Error);
        }
    }

    internal void OnFlightRequested(Autopilot pilot)
    {
        if (!_faults.TryEngage(Time.time))
        {
            StopFlight(pilot, _faults.Locked ? "Navigation unavailable until the next loop\nUse manual controls" : "Navigation recovering\nWait 5 seconds, then engage again");

            return;
        }

        ResetFlight();
    }

    internal bool TryReadInput(Autopilot pilot, out Vector3 input)
    {
        input = Vector3.zero;
        if (_sceneLoading)
        {
            return GameFields.IsShip.Get(pilot);
        }

        var eligible = GameFields.IsShip.Get(pilot) && pilot.IsFlyingToDestination() && !pilot.IsMatchingVelocity();
        if (!eligible || _finishing)
        {
            return false;
        }

        if (_faults.Faulted)
        {
            return true;
        }

        try
        {
            var usable = !pilot.IsDamaged() && GameFields.Resources.Get(pilot).AreThrustersUsable();
            if (!usable)
            {
                StopFlight(pilot, "Autopilot disengaged\nCheck fuel and ship damage; use manual controls");

                return true;
            }

            var frame = GameFields.Target.Get(pilot);
            if (frame == null || frame.GetOWRigidBody() == null)
            {
                StopFlight(pilot, "Autopilot disengaged\nDestination unavailable; use manual controls");

                return true;
            }

            var thrusters   = GameFields.Thrusters.Get(pilot);
            var banksUsable = thrusters.IsThrusterBankEnabled(ThrusterBank.Left) && thrusters.IsThrusterBankEnabled(ThrusterBank.Right);
            if (!banksUsable)
            {
                StopFlight(pilot, "Autopilot disengaged\nRepair the damaged thruster bank");

                return true;
            }

            if (_pilot != pilot || _computer == null || _flightTarget != frame)
            {
                BeginFlight(pilot);
            }

            if (!pilot.IsFlyingToDestination())
            {
                return true;
            }

            UpdateState(pilot, frame);
            _automation?.ApplyNavigationFault(_state);
            if (_state.Thrust is >= 0 and <= 0.01)
            {
                StopFlight(pilot, "No usable thrust in this area\nAutopilot disengaged; move to an area with usable thrust");

                return true;
            }

            FlightValidation.Validate(_state);

            var fuel = GameFields.Resources.Get(pilot).GetFractionalFuel();
            if (_departure != null && _departure.TryReadInput(pilot, _state, out input))
            {
                var departureAcceleration = FromUnity(pilot.transform.TransformDirection(input)) * _state.MaximumThrust;
                CaptureSample(departureAcceleration, fuel, null);
                _automation?.Observe(_state, "Departure", _bodies);
                DrawDebug(new FlightCommand(departureAcceleration, FlightPhase.Hold));

                return true;
            }

            var command = _computer.Step(_state);
            command = _collision.Step(_state, command, _bodies, _computer.AvoidanceBodyIndex);

            if (_collision.NeedsManualControl)
            {
                _lastPhase = FlightPhase.Hold;

                LogBlockedRoute(frame);

                if (_computer.AvoidanceBodyIndex >= 0)
                {
                    LogAvoidance(_computer.AvoidanceBodyIndex);
                }

                DumpFlight("Collision hold conflicts with emergency escape; no verified clear maneuver");
                StopFlight(pilot, "Autopilot disengaged\nNo safe escape; take manual control");

                return true;
            }

            if (!command.Acceleration.IsFinite)
            {
                throw new InvalidOperationException("Navigation produced a non-finite acceleration.");
            }

            var phaseChanged   = command.Phase != _lastPhase;
            var avoidanceIndex = command.Phase switch
            {
                FlightPhase.Detour => _computer.DetourBodyIndex,
                FlightPhase.Escape => _computer.AvoidanceBodyIndex,
                _                  => -1
            };

            var collisionReference = _collision.Active ? "holding near " + _collision.ReferenceName : _collision.Recovering ? "escaping near " + _collision.ReferenceName : null;
            var statusChanged      = phaseChanged || avoidanceIndex != _lastAvoidanceBody || collisionReference != _lastCollisionReference;
            if (statusChanged)
            {
                _lastPhase              = command.Phase;
                _lastAvoidanceBody      = avoidanceIndex;
                _lastCollisionReference = collisionReference;

                var avoiding    = avoidanceIndex >= 0 ? _bodies[avoidanceIndex] : null;
                var displayName = avoiding       == null ? "obstacle" : BodyDisplayName(avoiding);

                _departure?.ShowNavigation(command.Phase, displayName);

                if (_logChanges)
                {
                    var distance = (_state.Position - _state.TargetPosition).Length;
                    var speed    = (_state.Velocity - _state.TargetVelocity).Length;

                    var avoidanceBody    = avoidanceIndex >= 0 && _bodies[avoidanceIndex] != null ? BodyDisplayName(_bodies[avoidanceIndex]) + " [" + avoidanceIndex + ":" + _bodies[avoidanceIndex].name + "]" : "none";

                    var routeStatus = command.Phase == FlightPhase.Hold ? " Reason: " + (_collision.Active ? _collision.Reason : _computer.RouteStatus) + "." : string.Empty;
                    var holdIndex   = _computer.HoldBodyIndex;

                    var holdReference = command.Phase == FlightPhase.Hold && !_collision.Active ? "holding near " + (holdIndex >= 0 && _bodies[holdIndex] != null ? BodyDisplayName(_bodies[holdIndex]) : "local space") : collisionReference;

                    var controlReference = holdReference != null ? " Control: " + holdReference + "." : string.Empty;

                    ModHelper.Console.WriteLine($"Smart Autopilot: {command.Phase}; target distance {distance:F0}m; relative speed {speed:F1}m/s; arrival radius {_state.ArrivalRadius:F0}m; avoidance body {avoidanceBody}.{routeStatus}{controlReference}", MessageType.Info);

                    if (_logSamples && command.Phase == FlightPhase.Hold)
                    {
                        LogBlockedRoute(frame);
                    }

                    if (_logSamples && command.Phase == FlightPhase.Escape && avoidanceIndex >= 0)
                    {
                        LogAvoidance(avoidanceIndex);
                    }
                }
            }

            CaptureSample(command.Acceleration, fuel, command.Phase);

            _automation?.Observe(_state, command.Phase.ToString(), _bodies);

            var waitingForRoute = command.Phase == FlightPhase.Hold || _computer.RouteBlocked && command.Phase == FlightPhase.Escape;

            UpdateHold(waitingForRoute, fuel);
            DrawDebug(command);

            if (command.Phase == FlightPhase.Arrived)
            {
                // Let the original completion path fire its audio, HUD and arrival events.
                _finishing = true;
                pilot.StartMatchVelocity(frame, true);
                GameFields.StopMatching.Set(pilot, true);

                return true;
            }

            var liningUp = command.Phase == FlightPhase.Detour || command.Phase == FlightPhase.Escape || command.Phase == FlightPhase.Hold;

            GameFields.LiningUp.Set(pilot, liningUp);
            GameFields.Approaching.Set(pilot, command.Phase == FlightPhase.Cruise);

            var beganBraking = phaseChanged && command.Phase == FlightPhase.Braking;
            if (beganBraking)
            {
                GameFields.RetroRockets.Get(pilot)?.Invoke();
            }

            var acceleration = ToUnity(command.Acceleration);
            input = pilot.transform.InverseTransformDirection(acceleration / thrusters.GetMaxTranslationalThrust());
            return true;
        }
        catch (Exception exception)
        {
            HandleFailure(pilot, exception);
            return true;
        }
    }

    internal void HandleFailure(Autopilot pilot, Exception exception)
    {
        if (_handlingFailure)
        {
            return;
        }

        _handlingFailure = true;
        try
        {
            var first = _faults.Record(Time.time);

            StopFlight(pilot, _faults.Locked ? "Navigation fault; use manual controls\nAutopilot unavailable until the next loop" : "Navigation fault; use manual controls\nWait 5 seconds before trying again");

            if (first)
            {
                SafeLog("Smart Autopilot disengaged: " + exception, MessageType.Error);
                DumpFlight("Navigation fault");
            }
        }
        catch (Exception cleanupException)
        {
            SafeLog("Smart Autopilot failure cleanup: " + cleanupException, MessageType.Error);
        }
        finally
        {
            _handlingFailure = false;
        }
    }

    internal bool AllowLandedAutopilot(ShipCockpitController cockpit)
    {
        try
        {
            EnsureDeparture();
            return _departure != null && _departure.CanLaunch(cockpit);
        }
        catch (Exception exception)
        {
            DisableDeparture(exception);
            return false;
        }
    }

    private void StopFlight(Autopilot pilot, string message)
    {
        try
        {
            pilot.Abort();
            ResetFlight();
        }
        catch (Exception exception)
        {
            // A broken game abort path must not keep supplying autopilot thrust.
            _pilot    = null;
            _computer = null;
            _faults.Record(Time.time);

            SafeLog("Autopilot cleanup failed: " + exception, MessageType.Error);

            try
            {
                pilot.enabled = false;
            }
            catch (Exception disableException)
            {
                SafeLog("Autopilot disable failed: " + disableException, MessageType.Error);
            }
        }

        Notify(message);
    }

    private void Notify(string message)
    {
        _automation?.ObserveNotice(message);

        SafeLog("Smart Autopilot: " + message.Replace('\n', ' '), MessageType.Warning);

        try
        {
            var notifications = NotificationManager.SharedInstance;
            notifications?.PostNotification(new NotificationData(NotificationTarget.Ship, message, 10));
        }
        catch (Exception exception)
        {
            SafeLog("Autopilot notification unavailable: " + exception, MessageType.Error);
        }
    }

    private void SafeLog(string message, MessageType type)
    {
        try
        {
            ModHelper.Console.WriteLine(message, type);
        }
        catch (Exception)
        {
            System.Diagnostics.Trace.WriteLine(message);
        }
    }

    private void UpdateHold(bool blocked, double fuel)
    {
        _hold.Step(blocked, _state.Time, fuel);
        if (!blocked)
        {
            _holdNotified = false;

            return;
        }

        var holdBody = _computer.HoldBodyIndex;
        var holdName = holdBody >= 0 && _bodies[holdBody] != null ? BodyDisplayName(_bodies[holdBody]) : "local space";
        var reason   = _collision.Active ? _collision.Reason + "; holding near " + _collision.ReferenceName : "No clear route; waiting near " + holdName;
        var action   = _collision.Active ? "Rechecking path; cancel to fly manually" : "Select another target or cancel to fly manually";
        if (_state.Time >= _nextHoldUi)
        {
            _nextHoldUi = _state.Time + 0.25;
            _departure?.ShowHold(reason, _hold.Duration, fuel, action);
        }
        if (_hold.NeedsAttention && !_holdNotified)
        {
            _holdNotified = true;
            Notify(reason + "\n" + action + $"\nFuel remaining: {fuel:P0}");
            DumpFlight(reason);
        }
    }

    private void CaptureSample(Vector acceleration, double fuel, FlightPhase? phase)
    {
        if (_state.Time < _nextFlightSample)
        {
            return;
        }

        _nextFlightSample = _state.Time + 0.5;

        var status = !phase.HasValue ? "Departure" : _collision.Active ? "Collision hold (" + _collision.ReferenceName + ")" : _collision.Recovering  ? "Collision escape (" + _collision.ReferenceName + ")" : phase.Value.ToString();
        var sample = new FlightSample(_state, acceleration, fuel, status);
        _recorder.Add(sample);
        if (_logSamples)
        {
            ModHelper.Console.WriteLine("Smart Autopilot sample: " + sample, MessageType.Info);
        }
    }

    private void DumpFlight(string reason)
    {
        SafeLog("Smart Autopilot diagnostic: " + reason + "; " + _flightContext, MessageType.Info);
        _recorder.Dump(message => SafeLog("  " + message, MessageType.Info));
    }

    private void DrawDebug(FlightCommand command)
    {
        _debugCommand    = command;
        _debugHasCommand = true;
    }

    private void DisposeDebug()
    {
        var overlay = _debugOverlay;
        _debugOverlay = null;
        try
        {
            overlay?.Dispose();
        }
        catch (Exception exception)
        {
            SafeLog("Navigation debug cleanup: " + exception, MessageType.Error);
        }
    }

    private void EnsureDeparture()
    {
        if (_sceneLoading)
        {
            return;
        }

        var ship = Locator.GetShipBody();
        if (_departureShip != ship)
        {
            _departure?.Dispose();
            _departure            = null;
            _departureShip        = ship;
            _departureUnavailable = false;
        }

        var canCreate = !_departureUnavailable        &&
                        _departure == null            &&
                        ship       != null            &&
                        PlayerState.AtFlightConsole() &&
                        Locator.GetPromptManager() != null;
        if (canCreate)
        {
            _departure = new ShipDepartureController(ship, message =>
            {
                if (_logChanges)
                {
                    ModHelper.Console.WriteLine("Smart Autopilot: " + message, MessageType.Info);
                }
            }, message =>
            {
                Notify(message);
                DumpFlight(message);
            });
        }
    }

    private void DisableDeparture(Exception exception)
    {
        if (!_departureUnavailable)
        {
            _departureUnavailable = true;
            ModHelper.Console.WriteLine("Automatic launch disabled: " + exception, MessageType.Error);
            Notify("Automatic launch unavailable this loop\nUse manual launch controls");
        }

        var departure = _departure;

        _departure = null;

        if (departure != null)
        {
            if (departure.Active && departure.Ship != null)
            {
                departure.Ship.GetComponent<Autopilot>().Abort();
            }

            departure.Dispose();
        }
    }

    private void BeginFlight(Autopilot pilot)
    {
        _pilot               = pilot;
        _computer            = new FlightComputer();
        _flightTarget        = GameFields.Target.Get(pilot);
        _collision           = new ShipCollisionGuard(GameFields.Body.Get(pilot));
        _arrivalLimits       = new ShipArrivalLimits(_flightTarget.GetOWRigidBody());
        _arrivalLimitsLogged = false;
        _hold                = new HoldMonitor();
        _holdNotified        = false;
        _nextHoldUi          = 0;
        _recorder.Clear();
        _nextFlightSample = 0;
        _lastPhase        = null;
        _finishing        = false;
        _bodies.Clear();
        _navigationHulls.Clear();
        _state.Obstacles.Clear();

        foreach (var body in FindObjectsOfType<AstroObject>())
        {
            var supported = body.GetOWRigidbody() != null && body.GetAstroObjectType() != AstroObject.Type.None && body.GetAstroObjectName() != AstroObject.Name.DreamWorld;
            if (supported)
            {
                _bodies.Add(body);
                _state.Obstacles.Add(new Obstacle());

                var hull = body.GetAstroObjectName() == AstroObject.Name.RingWorld ? new NavigationHull(body.GetOWRigidbody()) : null;

                _navigationHulls.Add(hull);

                if (hull != null && _logChanges)
                {
                    var radius = hull.GetRadius(Time.time);
                    SafeLog($"Smart Autopilot navigation hull: {body.GetOWRigidbody().name}; physical envelope={radius:F1}m; colliders={hull.ColliderCount}.", MessageType.Info);
                }
            }
        }

        var targetName  = TargetDisplayName(_flightTarget);
        var primaryName = "none";
        for (var i = 0; i < _bodies.Count; i++)
        {
            var primary = _bodies[i].GetPrimaryBody();
            for (var depth = 0; primary != null && depth < 8; depth++)
            {
                var index = _bodies.IndexOf(primary);
                if (index >= 0 && index != i)
                {
                    _state.Obstacles[i].PrimaryIndex = index;
                    if (_bodies[i].GetOWRigidbody() == _flightTarget.GetOWRigidBody())
                    {
                        primaryName = _bodies[index].GetOWRigidbody().name;
                    }
                    break;
                }

                primary = primary.GetPrimaryBody();
            }
        }

        ModHelper.Console.WriteLine("Smart Autopilot engaged; target " + targetName + "; body " + _flightTarget.GetOWRigidBody().name + "; tracking " + _bodies.Count + " celestial bodies.", MessageType.Info);
        _flightContext = $"mod={ModHelper.Manifest.Version}; game={Application.version}; OWML assembly={typeof(IModHelper).Assembly.GetName().Version}; OWML requirement={ModHelper.Manifest.OWMLVersion}; target={targetName}; body={_flightTarget.GetOWRigidBody().name}; primary={primaryName}; speedLimit={_speedLimit}; clearance={_clearance}; bodies={_bodies.Count}";

        EnsureDeparture();

        _departure?.Begin(GameFields.Target.Get(pilot), _bodies);
        _flightContext += "; departure=" + (_departure?.SurfaceName ?? "space");
    }

    private void OnStartSceneLoad(OWScene originalScene, OWScene loadScene)
    {
        // Release prompts and launch context while the old scene's objects still exist.
        _sceneLoading = true;

        var departure = _departure;
        _departure            = null;
        _departureShip        = null;
        _departureUnavailable = false;
        _faults               = new FlightFaults();
        _flightContext        = null;
        _recorder.Clear();

        DisposeDebug();

        _debugUnavailable = false;

        ResetFlight();

        _bodies.Clear();
        _navigationHulls.Clear();
        _state.Obstacles.Clear();
        _state.TargetObstacleIndex = -1;

        departure?.Dispose();
    }

    private void OnCompleteSceneLoad(OWScene originalScene, OWScene loadScene)
    {
        _sceneLoading = false;
    }

    private void LogBlockedRoute(ReferenceFrame frame)
    {
        if (_collision.Active)
        {
            SafeLog("Smart Autopilot collision diagnostic: " + _collision.Diagnostic, MessageType.Info);
        }

        ModHelper.Console.WriteLine($"Smart Autopilot route diagnostic: target {TargetDisplayName(frame)}; available thrust {_state.Thrust:F1}m/s^2; maximum thrust {_state.MaximumThrust:F1}m/s^2.", MessageType.Info);

        for (var i = 0; i < _state.Obstacles.Count; i++)
        {
            var obstacle      = _state.Obstacles[i];
            var offset        = obstacle.Position                            - _state.TargetPosition;
            var shipClearance = (obstacle.Position - _state.Position).Length - obstacle.Radius;
            var relevant      = obstacle.Radius > 0 && (offset.Length < obstacle.Radius + _state.ArrivalRadius + 500 || shipClearance < 500);
            if (relevant && _bodies[i] != null)
            {
                var body = _bodies[i];
                var name = body.GetAstroObjectName() == AstroObject.Name.CustomString ? body.GetCustomName() : body.GetAstroObjectName().ToString();
                ModHelper.Console.WriteLine($"  Zone {name}: center relative to target ({offset.X:F1}, {offset.Y:F1}, {offset.Z:F1}); radius {obstacle.Radius:F1}m; physical radius {obstacle.PhysicalRadius:F1}m; ship clearance {shipClearance:F1}m.", MessageType.Info);
            }
        }
    }

    private void LogAvoidance(int index)
    {
        var obstacle        = _state.Obstacles[index];
        var offset          = _state.Position - obstacle.Position;
        var velocity        = _state.Velocity - obstacle.Velocity;
        var closing         = Math.Max(0, -Vector.Dot(velocity, offset.Unit));
        var braking         = Math.Max(_state.Thrust * 0.1, _state.Thrust - (_state.ExternalAcceleration - obstacle.Acceleration).Length) * 0.7;
        var stopping        = closing * closing                                                                                           / (2 * braking);
        var closestTime     = velocity.LengthSquared > 1e-9 ? Math.Max(0, Math.Min(velocity.Length / braking + 1, -Vector.Dot(offset, velocity) / velocity.LengthSquared)) : 0;
        var closestDistance = (offset + velocity * closestTime).Length;
        var primaryIndex    = obstacle.PrimaryIndex;
        var primaryName     = primaryIndex >= 0 && primaryIndex < _bodies.Count && _bodies[primaryIndex] != null ? _bodies[primaryIndex].GetOWRigidbody().name : "none";
        var recognizedOrbit = OrbitalMotion.TryCreate(_state, index, out _);

        ModHelper.Console.WriteLine($"Smart Autopilot avoidance model: body={_bodies[index].GetOWRigidbody().name}; primary={primaryName}; recognized orbit={recognizedOrbit}.", MessageType.Info);
        ModHelper.Console.WriteLine($"Smart Autopilot avoidance diagnostic: body {BodyDisplayName(_bodies[index])}; center distance {offset.Length:F1}m; zone radius {obstacle.Radius:F1}m; physical radius {obstacle.PhysicalRadius:F1}m; closing {closing:F1}m/s; stopping distance {stopping:F1}m; constant-velocity closest approach {closestDistance:F1}m in {closestTime:F1}s; available thrust {_state.Thrust:F1}m/s^2.", MessageType.Info);
        ModHelper.Console.WriteLine($"  Relative position ({offset.X:F2}, {offset.Y:F2}, {offset.Z:F2}); relative velocity ({velocity.X:F2}, {velocity.Y:F2}, {velocity.Z:F2}); body acceleration ({obstacle.Acceleration.X:F2}, {obstacle.Acceleration.Y:F2}, {obstacle.Acceleration.Z:F2}); ship external acceleration ({_state.ExternalAcceleration.X:F2}, {_state.ExternalAcceleration.Y:F2}, {_state.ExternalAcceleration.Z:F2}).", MessageType.Info);

        var childIndex = index;
        for (var depth = 0; depth < 2 && primaryIndex >= 0 && primaryIndex < _state.Obstacles.Count; depth++)
        {
            var child           = _state.Obstacles[childIndex];
            var primary         = _state.Obstacles[primaryIndex];
            var orbitalPosition = child.Position - primary.Position;
            var orbitalVelocity = child.Velocity - primary.Velocity;

            ModHelper.Console.WriteLine($"  Orbital frame: child={childIndex}; primary={primaryIndex}; relative position=({orbitalPosition.X:F2}, {orbitalPosition.Y:F2}, {orbitalPosition.Z:F2}); relative velocity=({orbitalVelocity.X:F2}, {orbitalVelocity.Y:F2}, {orbitalVelocity.Z:F2}); primary acceleration=({primary.Acceleration.X:F2}, {primary.Acceleration.Y:F2}, {primary.Acceleration.Z:F2}); primary radius={primary.Radius:F2}m.", MessageType.Info);

            childIndex   = primaryIndex;
            primaryIndex = primary.PrimaryIndex;
        }
    }

    private void UpdateState(Autopilot pilot, ReferenceFrame frame)
    {
        var ship = GameFields.Body.Get(pilot);

        _state.Position             = FromUnity(ship.GetWorldCenterOfMass());
        _state.Velocity             = FromUnity(ship.GetVelocity());
        _state.ExternalAcceleration = FromUnity(GameFields.Forces.Get(pilot).GetForceAcceleration());
        _state.TargetPosition       = FromUnity(frame.GetPosition());
        _state.TargetVelocity       = FromUnity(frame.GetVelocity());
        _state.TargetAcceleration   = FromUnity(frame.GetAcceleration());
        _state.ArrivalRadius        = frame.GetAutopilotArrivalDistance();
        _state.TargetObstacleIndex  = -1;
        _state.MaximumThrust        = GameFields.Thrusters.Get(pilot).GetMaxTranslationalThrust();
        _state.Thrust               = Math.Min(_state.MaximumThrust, GameFields.Rules.Get(pilot).GetThrustLimit());
        _state.SpeedLimit           = _speedLimit;
        _state.Time                 = Time.time;
        _state.DeltaTime            = Time.fixedDeltaTime;

        for (var i = 0; i < _bodies.Count; i++)
        {
            var astro    = _bodies[i];
            var obstacle = _state.Obstacles[i];
            if (astro == null || !astro.gameObject.activeInHierarchy)
            {
                obstacle.Radius   = 0;
                obstacle.Position = _state.Position + new Vector(1e8, 0, 0);
                continue;
            }

            var body = astro.GetOWRigidbody();

            obstacle.Position     = FromUnity(body.GetWorldCenterOfMass());
            obstacle.Velocity     = FromUnity(body.GetVelocity());
            obstacle.Acceleration = FromUnity(body.GetAcceleration());

            var physicalRadius = GetPhysicalRadius(astro);
            var hull           = _navigationHulls[i];
            if (hull != null)
            {
                physicalRadius = Math.Max(physicalRadius, hull.GetRadius(_state.Time));
            }

            var radius  = physicalRadius + _clearance;
            var gravity = astro.GetGravityVolume();
            if (gravity != null)
            {
                // Query the game's gravity law rather than assuming inverse-square gravity.
                radius = GravityClearance.CalculateRadius(radius, _state, gravity.CalculateGravityMagnitude);
            }

            var isTarget = body == frame.GetOWRigidBody();
            if (isTarget)
            {
                _state.TargetObstacleIndex = i;
                _state.ArrivalRadius       = Math.Max(_state.ArrivalRadius, radius + 80);
            }
            else
            {
                // Cover short-term body motion between route updates; the controller also checks relative closing speed.
                radius += Math.Min(180, (obstacle.Velocity - _state.TargetVelocity).Length * 0.75);
            }

            obstacle.PhysicalRadius = physicalRadius;
            obstacle.Radius         = radius;
        }

        _arrivalLimits?.Refresh(_state);
        if (_logSamples && !_arrivalLimitsLogged)
        {
            _arrivalLimitsLogged = true;
            ModHelper.Console.WriteLine($"Smart Autopilot arrival thrust forecast: target={TargetDisplayName(frame)}; volumes={_state.ArrivalThrustLimits.Count}.", MessageType.Info);
            foreach (var limit in _state.ArrivalThrustLimits)
            {
                ModHelper.Console.WriteLine($"  Arrival thrust limit: radius={limit.Radius:F1}m; thrust={limit.Thrust:F1}m/s^2; arrival radius={_state.ArrivalRadius:F1}m.", MessageType.Info);
            }
        }
    }

    internal static string TargetDisplayName(ReferenceFrame frame)
    {
        var display = frame.GetHUDDisplayName();
        if (!string.IsNullOrWhiteSpace(display))
        {
            return display;
        }

        var body = frame.GetOWRigidBody();
        return body != null ? body.name : "Unnamed destination";
    }

    private static string BodyDisplayName(AstroObject astro)
    {
        var name    = astro.GetAstroObjectName();
        var display = name == AstroObject.Name.CustomString ? astro.GetCustomName() : AstroObject.AstroObjectNameToString(name);

        return string.IsNullOrEmpty(display) ? name.ToString() : display;
    }

    private static double GetPhysicalRadius(AstroObject astro)
    {
        var gravity = astro.GetGravityVolume();
        var radius  = gravity != null ? GameFields.SurfaceRadius.Get(gravity) : 30;
        var frame   = astro.GetOWRigidbody().GetReferenceFrame();
        if (frame != null)
        {
            radius = Math.Max(radius, frame.GetBracketsRadius());
        }

        var isSun = astro.GetAstroObjectName() == AstroObject.Name.Sun;
        if (isSun && Locator.GetSunController() != null)
        {
            radius = Math.Max(radius, Locator.GetSunController().GetSurfaceRadius());
        }

        return radius;
    }

    private static float ReadNumber(IModConfig config, string key, float fallback, float minimum, float maximum)
    {
        var value = config.GetSettingsValue<float>(key);
        if (float.IsNaN(value) || float.IsInfinity(value))
        {
            return fallback;
        }

        return Mathf.Clamp(value, minimum, maximum);
    }

    private static Vector FromUnity(Vector3 value) => new(value.x, value.y, value.z);

    private static Vector3 ToUnity(Vector value) => new((float)value.X, (float)value.Y, (float)value.Z);
}

[HarmonyPatch]
internal static class AutopilotPatches
{
    [HarmonyPrefix]
    [HarmonyPatch(typeof(Autopilot), "ReadTranslationalInput")]
    private static bool ReadInput(Autopilot __instance, ref Vector3 __result)
    {
        var mod = SmartAutopilotMod.Instance;
        try
        {
            var  input   = Vector3.zero;
            bool handled = mod != null && mod.TryReadInput(__instance, out input);
            if (handled)
            {
                __result = input;
            }

            return !handled;
        }
        catch (Exception exception)
        {
            // This boundary also catches failures raised while compiling the called method.
            __result = Vector3.zero;
            mod.HandleFailure(__instance, exception);

            return false;
        }
    }

    [HarmonyPostfix]
    [HarmonyPatch(typeof(Autopilot), nameof(Autopilot.FlyToDestination))]
    private static void Begin(Autopilot __instance, bool __result)
    {
        if (__result && __instance.GetComponent<ShipResources>() != null)
        {
            SmartAutopilotMod.Instance?.OnFlightRequested(__instance);
        }
    }

    [HarmonyPostfix]
    [HarmonyPatch(typeof(Autopilot), "OnDisable")]
    private static void Disable(Autopilot __instance)
    {
        if (__instance.GetComponent<ShipResources>() != null)
        {
            SmartAutopilotMod.Instance?.ResetFlight();
        }
    }

    [HarmonyPostfix]
    [HarmonyPatch(typeof(ShipCockpitController), nameof(ShipCockpitController.IsAutopilotAvailable))]
    private static void AllowLaunch(ShipCockpitController __instance, ref bool __result)
    {
        if (!__result && SmartAutopilotMod.Instance != null)
        {
            __result = SmartAutopilotMod.Instance.AllowLandedAutopilot(__instance);
        }
    }
}
