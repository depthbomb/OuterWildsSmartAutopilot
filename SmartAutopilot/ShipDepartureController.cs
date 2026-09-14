using System;
using System.Collections.Generic;
using SmartAutopilot.Navigation;
using UnityEngine;
using Vector = SmartAutopilot.Navigation.Vector;

namespace SmartAutopilot
{
    internal sealed class ShipDepartureController : IDisposable
    {
        public OWRigidbody Ship { get; }
        public bool Active => _computer != null && _computer.Phase != DeparturePhase.Complete;
        public DeparturePhase? Phase => _computer?.Phase;
        public string SurfaceName => _surface != null ? _surface.name : "space";
        public DebugSweep DebugProbe { get; private set; }
        public Vector3 DebugClearancePoint { get; private set; }

        private OWRigidbody       _surface;
        private Vector3           _localOrigin;
        private Vector3           _localUp;
        private int               _departureBodyIndex = -1;
        private DepartureComputer _computer;
        private DepartureComputer _suspendedComputer;
        private double            _suspendedAt;
        private bool              _resuming;
        private DeparturePhase    _lastPhase;
        private bool              _ignitionActive;
        private float             _nextUiUpdate;
        private float             _messageUntil;
        private string            _message;
        private string            _navigationStatus;
        private bool              _holdNotified;
        private double            _fuel;
        private HoldMonitor       _hold = new();

        private readonly ShipCockpitController  _cockpit;
        private readonly LandingPadManager      _landing;
        private readonly ShipThrusterController _thrusters;
        private readonly Autopilot              _pilot;
        private readonly PromptManager          _prompts;
        private readonly ScreenPrompt           _launchPrompt;
        private readonly ScreenPrompt           _statusPrompt;
        private readonly Collider[]             _hull;
        private readonly RaycastHit[]           _hits     = new RaycastHit[32];
        private readonly Collider[]             _overlaps = new Collider[32];
        private readonly DepartureState         _state    = new();
        private readonly Action<string>         _log;
        private readonly Action<string>         _attention;

        public ShipDepartureController(OWRigidbody ship, Action<string> log, Action<string> attention)
        {
            Ship      = ship;
            _log      = log;
            _attention = attention;
            _cockpit  = ship.GetComponentInChildren<ShipCockpitController>();
            _landing  = ship.GetComponentInChildren<LandingPadManager>();
            _thrusters = ship.GetComponent<ShipThrusterController>();
            _pilot    = ship.GetComponent<Autopilot>();
            _hull     = ship.GetComponentsInChildren<Collider>();
            _prompts  = Locator.GetPromptManager();
            _launchPrompt = new ScreenPrompt(InputLibrary.autopilot, "<CMD>   Launch and navigate");
            _statusPrompt = new ScreenPrompt("SMART AUTOPILOT");
            _prompts.AddScreenPrompt(_launchPrompt, PromptPosition.UpperLeft);
            _prompts.AddScreenPrompt(_statusPrompt, PromptPosition.UpperRight);
        }

        public bool CanLaunch(ShipCockpitController cockpit)
        {
            bool ready = Ship && cockpit == _cockpit && _landing != null && _landing.IsLanded()
                && _cockpit.IsPlayerAtFlightConsole() && _thrusters.enabled && !_pilot.IsDamaged()
                && !GameFields.CockpitFailure.Get(_cockpit) && !GameFields.ControlsLocked.Get(_cockpit)
                && PlayerData.GetAutopilotEnabled() && GameFields.Resources.Get(_pilot).AreThrustersUsable();
            if (!ready)
            {
                return false;
            }

            bool probeConflict = OWInput.UsingGamepad() && Locator.GetProbe() != null && Locator.GetProbe().IsAnchored()
                && Locator.GetToolModeSwapper().IsInToolMode(ToolMode.Probe);
            if (probeConflict)
            {
                return false;
            }

            var frame = Locator.GetReferenceFrame();

            return frame != null             &&
                   frame.GetOWRigidBody()    &&
                   frame.GetAllowAutopilot() && Vector3.Distance(Ship.GetPosition(), frame.GetPosition()) > frame.GetAutopilotArrivalDistance();
        }

        public void Begin(ReferenceFrame target, IReadOnlyList<AstroObject> bodies)
        {
            if (_landing == null)
            {
                return;
            }

            if (!_landing.IsLanded())
            {
                if (_suspendedComputer != null && _surface != null && Time.time - _suspendedAt <= 15 && _surface != target.GetOWRigidBody())
                {
                    _computer          = _suspendedComputer;
                    _suspendedComputer = null;
                    _resuming          = true;
                    FindDepartureBody(bodies);
                }

                return;
            }

            _suspendedComputer = null;
            _resuming          = false;
            _surface = null;
            foreach (var sensor in _landing.GetComponentsInChildren<LandingPadSensor>())
            {
                _surface = sensor.GetContactBody();
                if (_surface != null)
                {
                    break;
                }
            }

            if (_surface == null || _surface == target.GetOWRigidBody())
            {
                Fail("Select a destination away from the launch surface");

                return;
            }

            var origin = Ship.GetWorldCenterOfMass();
            var up     = Ship.transform.up;
            var gravity = GameFields.Forces.Get(_pilot).GetForceAcceleration() - _surface.GetPointAcceleration(origin);
            if (gravity.sqrMagnitude > 1 && Vector3.Dot(up, -gravity.normalized) < 0.65f)
            {
                Fail("Level the ship before automatic launch");

                return;
            }

            _localOrigin = _surface.transform.InverseTransformPoint(origin);
            _localUp     = _surface.transform.InverseTransformDirection(up);
            FindDepartureBody(bodies);

            _computer  = new DepartureComputer(GameFields.IgnitionDuration.Get(_thrusters));
            _lastPhase = DeparturePhase.Checking;
            _messageUntil = 0;
            _log("Departure: checking launch clearance; surface " + _surface.name + "; target " + SmartAutopilotMod.TargetDisplayName(target) + ".");
        }

        public bool TryReadInput(Autopilot pilot, FlightState flight, out Vector3 input)
        {
            input = Vector3.zero;
            if (!Active || pilot != _pilot)
            {
                return false;
            }

            if (_surface == null || !_cockpit.IsPlayerAtFlightConsole() || !_thrusters.enabled)
            {
                Fail("Automatic launch cancelled");

                return true;
            }

            var position = Ship.GetWorldCenterOfMass();
            var origin   = _surface.transform.TransformPoint(_localOrigin);
            var up       = _surface.transform.TransformDirection(_localUp).normalized;

            _state.Up                   = FromUnity(up);
            _state.Offset               = FromUnity(position - origin);
            _state.Velocity             = flight.Velocity;
            _state.SurfaceVelocity      = FromUnity(_surface.GetPointVelocity(position));
            _state.SurfaceAcceleration  = FromUnity(_surface.GetPointAcceleration(position));
            _state.ExternalAcceleration = flight.ExternalAcceleration;
            _state.Thrust               = flight.Thrust;
            _state.Time                 = flight.Time;
            _state.Grounded             = _landing.IsLanded();
            _state.IgnitionRequired     = _thrusters.RequiresIgnition();
            _state.ClearanceAltitude    = 120;
            if (_departureBodyIndex >= 0 && _departureBodyIndex < flight.Obstacles.Count)
            {
                var obstacle = flight.Obstacles[_departureBodyIndex];
                _state.ClearanceAltitude = DepartureComputer.ClearanceAltitude(FromUnity(origin) - obstacle.Position, _state.Up, obstacle.Radius + 150);
            }

            if (_resuming)
            {
                _resuming = false;
                if (!_computer.TryResume(_state, _suspendedAt))
                {
                    Cancel(false);

                    return false;
                }

                _lastPhase = DeparturePhase.Checking;
                _log($"Departure: resuming interrupted climb at {_state.Altitude:F0}m.");
            }

            _state.PathClear = CheckOverhead(position, up, (float)_computer.GetProbeDistance(_state));
            DebugClearancePoint = origin + up * (float)_state.ClearanceAltitude;
            var acceleration = _computer.Step(_state);
            if (!acceleration.IsFinite)
            {
                throw new InvalidOperationException("Automatic launch produced a non-finite acceleration.");
            }

            var phase = _computer.Phase;
            if (phase == DeparturePhase.Failed)
            {
                Fail(_computer.FailureReason);

                return true;
            }

            _fuel = GameFields.Resources.Get(_pilot).GetFractionalFuel();
            _hold.Step(phase == DeparturePhase.Blocked, flight.Time, _fuel);
            if (_hold.NeedsAttention && !_holdNotified)
            {
                _holdNotified = true;
                if (_state.Grounded)
                {
                    Fail("Launch path blocked; reposition the ship before trying again");

                    return true;
                }

                _attention("Launch path blocked\nHolding above surface; cancel to fly manually" + $"\nFuel remaining: {_fuel:P0}");
            }

            if (phase != DeparturePhase.Blocked)
            {
                _holdNotified = false;
            }

            UpdateIgnition(phase);
            if (phase != _lastPhase)
            {
                _lastPhase    = phase;
                _nextUiUpdate = 0;
                _log($"Departure: {phase}; climb {_state.Altitude:F0}/{_state.ClearanceAltitude:F0}m.");
            }

            if (phase == DeparturePhase.Complete)
            {
                _message      = "Launch complete\nNavigating to target";
                _messageUntil = Time.time + 3;

                return false;
            }

            GameFields.LiningUp.Set(pilot, phase == DeparturePhase.Ignition || phase == DeparturePhase.Blocked);
            GameFields.Approaching.Set(pilot, phase == DeparturePhase.LiftOff || phase == DeparturePhase.Climb);
            input = pilot.transform.InverseTransformDirection(ToUnity(acceleration) / (float)flight.MaximumThrust);

            return true;
        }

        public void UpdateUi()
        {
            bool visible = Ship                               &&
                           _cockpit                           &&
                           _cockpit.IsPlayerAtFlightConsole() &&
                           OWInput.IsInputMode(InputMode.ShipCockpit | InputMode.LandingCam);

            _launchPrompt.SetVisibility(visible && (Active
                ? _landing.IsLanded() || OWInput.IsInputMode(InputMode.LandingCam)
                : !_pilot.IsFlyingToDestination()
                  && OWInput.IsInputMode(InputMode.ShipCockpit) && CanLaunch(_cockpit)));

            bool navigating = _pilot && _pilot.IsFlyingToDestination() && _navigationStatus != null;
            _statusPrompt.SetVisibility(visible && (Active || navigating || Time.time < _messageUntil));
            if (!visible || Time.time < _nextUiUpdate)
            {
                return;
            }

            _nextUiUpdate = Time.time + 0.25f;
            if (Active)
            {
                string status;
                switch (_computer.Phase)
                {
                    case DeparturePhase.Ignition:
                        status = "Igniting engines";
                        break;
                    case DeparturePhase.LiftOff:
                        status = "Lifting off";
                        break;
                    case DeparturePhase.Climb:
                        status = $"Climbing clear of surface\n{Math.Max(0, _state.Altitude):F0} / {_state.ClearanceAltitude:F0} m";
                        break;
                    case DeparturePhase.Blocked:
                        status = $"Launch path blocked ({_hold.Duration:F0}s)\nFuel: {_fuel:P0}; cancel to reposition";
                        break;
                    default:
                        status = "Checking launch clearance";
                        break;
                }

                _statusPrompt.SetText("SMART AUTOPILOT\n" + status);
                _launchPrompt.SetText("<CMD>   Cancel automatic launch");
            }
            else
            {
                _launchPrompt.SetText("<CMD>   Launch and navigate");
                _statusPrompt.SetText("SMART AUTOPILOT\n" + (Time.time < _messageUntil ? _message : _navigationStatus));
            }
        }

        public void ShowNavigation(FlightPhase phase, string avoidanceBody)
        {
            switch (phase)
            {
                case FlightPhase.Detour:
                    _navigationStatus = "Routing around " + avoidanceBody;
                    break;
                case FlightPhase.Escape:
                    _navigationStatus = "Avoiding " + avoidanceBody;
                    _messageUntil = 0;
                    break;
                case FlightPhase.Braking:
                    _navigationStatus = "Braking for arrival";
                    break;
                case FlightPhase.Hold:
                    _navigationStatus = "No clear route\nWaiting near the obstruction";
                    _messageUntil = 0;
                    break;
                case FlightPhase.Arrived:
                    _navigationStatus = "Arrived";
                    break;
                default:
                    _navigationStatus = "Navigating to target";
                    break;
            }

            _nextUiUpdate = 0;
        }

        public void ShowHold(string reason, double seconds, double fuel, string action)
        {
            _navigationStatus = $"{reason} ({seconds:F0}s)\nFuel: {fuel:P0}\n{action}";
            _messageUntil = 0;
        }

        public void Cancel(bool allowResume = true)
        {
            if (allowResume && Active && _computer.CanSuspend && !_state.Grounded && _surface != null)
            {
                _suspendedComputer = _computer;
                _suspendedAt       = Time.time;
            }

            if (!allowResume)
            {
                _suspendedComputer = null;
            }

            if (_ignitionActive && _thrusters)
            {
                GameFields.IsIgniting.Set(_thrusters, false);
                GameFields.LastThrustInput.Set(_thrusters, Vector3.zero);
                GlobalMessenger.FireEvent("CancelShipIgnition");
            }

            _ignitionActive = false;
            _computer      = null;
            _resuming      = false;
            _navigationStatus = null;
            _hold         = new HoldMonitor();
            _holdNotified = false;
            if (_suspendedComputer == null)
            {
                _surface = null;
            }

            _messageUntil  = 0;
            _nextUiUpdate  = 0;
        }

        public void Dispose()
        {
            Cancel(false);
            if (_prompts)
            {
                _prompts.RemoveScreenPrompt(_launchPrompt, PromptPosition.UpperLeft);
                _prompts.RemoveScreenPrompt(_statusPrompt, PromptPosition.UpperRight);
            }
        }

        private void FindDepartureBody(IReadOnlyList<AstroObject> bodies)
        {
            _departureBodyIndex = -1;
            var anchor = _surface;
            for (int depth = 0; anchor != null && depth < 8 && _departureBodyIndex < 0; depth++)
            {
                for (int i = 0; i < bodies.Count; i++)
                {
                    if (bodies[i] != null && bodies[i].GetOWRigidbody() == anchor)
                    {
                        _departureBodyIndex = i;
                        break;
                    }
                }

                anchor = anchor.GetOrigParentBody();
            }
        }

        private bool CheckOverhead(Vector3 position, Vector3 up, float distance)
        {
            var forward = Vector3.ProjectOnPlane(Ship.transform.forward, up).normalized;
            if (forward.sqrMagnitude < 0.5f)
            {
                forward = Vector3.ProjectOnPlane(Ship.transform.right, up).normalized;
            }

            var right = Vector3.Cross(up, forward).normalized;
            float top = 1;
            float width = 2;
            float depth = 2;
            foreach (var collider in _hull)
            {
                bool physicalHull = collider != null && collider.enabled && !collider.isTrigger
                    && collider.attachedRigidbody == Ship.GetRigidbody();
                if (!physicalHull)
                {
                    continue;
                }

                var bounds = collider.bounds;
                var offset = bounds.center - position;
                top   = Mathf.Max(top, Vector3.Dot(offset, up) + ProjectExtent(bounds.extents, up));
                width = Mathf.Max(width, Mathf.Abs(Vector3.Dot(offset, right)) + ProjectExtent(bounds.extents, right));
                depth = Mathf.Max(depth, Mathf.Abs(Vector3.Dot(offset, forward)) + ProjectExtent(bounds.extents, forward));
            }

            // Sweep the leading face above the hull; starting on the ground must not count as an overhead obstruction.
            var center = position + up * (top + 0.35f);
            var halfExtents = new Vector3(width + 0.5f, 0.25f, depth + 0.5f);
            var rotation = Quaternion.LookRotation(forward, up);
            DebugProbe = new DebugSweep(center, halfExtents, rotation, up, distance);
            int overlaps = Physics.OverlapBoxNonAlloc(center, halfExtents, _overlaps, rotation, OWLayerMask.physicalMask, QueryTriggerInteraction.Ignore);
            if (overlaps == _overlaps.Length)
            {
                return false;
            }

            for (int i = 0; i < overlaps; i++)
            {
                if (!IsSelf(_overlaps[i]))
                {
                    return false;
                }
            }

            int hits = Physics.BoxCastNonAlloc(center, halfExtents, up, _hits, rotation, distance, OWLayerMask.physicalMask, QueryTriggerInteraction.Ignore);
            if (hits == _hits.Length)
            {
                return false;
            }

            for (int i = 0; i < hits; i++)
            {
                if (!IsSelf(_hits[i].collider))
                {
                    return false;
                }
            }

            return true;
        }

        private bool IsSelf(Collider collider) => collider                   == null                ||
                                                  collider.attachedRigidbody == Ship.GetRigidbody() ||
                                                  collider.transform.IsChildOf(Ship.transform);

        private void UpdateIgnition(DeparturePhase phase)
        {
            if (phase == DeparturePhase.Ignition && !_ignitionActive)
            {
                if (GameFields.IsIgniting.Get(_thrusters))
                {
                    GlobalMessenger.FireEvent("CancelShipIgnition");
                }

                _ignitionActive = true;
                GameFields.IsIgniting.Set(_thrusters, true);
                GlobalMessenger.FireEvent("StartShipIgnition");
            }

            if (_ignitionActive && phase != DeparturePhase.Ignition)
            {
                _ignitionActive = false;
                GameFields.IsIgniting.Set(_thrusters, false);
                GameFields.LastThrustInput.Set(_thrusters, Vector3.zero);
                bool complete = phase is DeparturePhase.LiftOff or DeparturePhase.Climb or DeparturePhase.Complete;
                if (complete)
                {
                    GameFields.RequireIgnition.Set(_thrusters, false);
                    GlobalMessenger.FireEvent("CompleteShipIgnition");
                    RumbleManager.PlayShipIgnition();
                    RumbleManager.SetShipThrottleNormal();
                }
                else
                {
                    GlobalMessenger.FireEvent("CancelShipIgnition");
                }
            }
        }

        private void Fail(string reason)
        {
            _log($"Departure cancelled: {reason}; surface={SurfaceName}; phase={Phase}; altitude={_state.Altitude:F1}m; clearance={_state.ClearanceAltitude:F1}m; grounded={_state.Grounded}; thrust={_state.Thrust:F2}m/s^2; required acceleration={_state.RequiredAcceleration:F2}m/s^2; reserve={_state.ThrustReserve:F2}m/s^2 (minimum 2.00); surface acceleration=({_state.SurfaceAcceleration.X:F2},{_state.SurfaceAcceleration.Y:F2},{_state.SurfaceAcceleration.Z:F2}); external acceleration=({_state.ExternalAcceleration.X:F2},{_state.ExternalAcceleration.Y:F2},{_state.ExternalAcceleration.Z:F2}).");
            _pilot.Abort();
            Cancel(false);
            _message      = reason;
            _messageUntil = Time.time + 5;
            _attention(reason);
        }

        private static float ProjectExtent(Vector3 extents, Vector3 axis) => Mathf.Abs(axis.x) * extents.x + Mathf.Abs(axis.y) * extents.y + Mathf.Abs(axis.z) * extents.z;

        private static Vector FromUnity(Vector3 value) => new(value.x, value.y, value.z);

        private static Vector3 ToUnity(Vector value) => new((float)value.X, (float)value.Y, (float)value.Z);
    }
}
