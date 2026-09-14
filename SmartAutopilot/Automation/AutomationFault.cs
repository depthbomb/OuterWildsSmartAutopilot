using System;
using System.Collections.Generic;
using SmartAutopilot.Navigation;
using UnityEngine;

namespace SmartAutopilot.Automation
{
    [Serializable]
    internal sealed class AutomationFaultResult
    {
        public string       kind;
        public bool         applied;
        public bool         responseVerified;
        public bool         restored;
        public bool         recovered;
        public double       responseSeconds;
        public List<string> notices = [];
    }

    internal sealed class AutomationFault : IDisposable
    {
        public readonly AutomationFaultResult Result;

        private ShipAutopilotComponent _component;
        private ThrustRuleset          _volume;
        private Obstacle               _blockedTarget;
        private int                    _targetPrimaryIndex;
        private float                  _fuel;
        private float                  _started;
        private float                  _safeSince = -1;
        private float                  _restoredAt;
        private string                 _phase;
        private bool                   _engagedAfterRestore;

        private readonly Autopilot      _pilot;
        private readonly OWRigidbody    _ship;
        private readonly ShipResources  _resources;
        private readonly Action<string> _log;

        public AutomationFault(string kind, Autopilot pilot, OWRigidbody ship, Action<string> log)
        {
            Result = new AutomationFaultResult
            {
                kind = kind
            };
            _pilot = pilot;
            _ship = ship;
            _resources = GameFields.Resources.Get(pilot);
            _log = log;
        }

        public void Dispose()
        {
            Restore();
        }

        public void Apply()
        {
            _fuel = _resources.GetFuel();
            _started = Time.unscaledTime;
            Result.applied = true;
            _log("Injecting " + Result.kind);
            if (Result.kind == "fuel-exhaustion")
            {
                _resources.SetFuel(0);
            }
            else if (Result.kind == "autopilot-damage")
            {
                _component = _ship.GetComponentInChildren<ShipAutopilotComponent>();
                if (!_component || _component.isDamaged)
                {
                    throw new InvalidOperationException("A healthy autopilot component is required for the damage test.");
                }

                _component.SetDamaged(true);
                if (!_pilot.IsDamaged())
                {
                    throw new InvalidOperationException("Autopilot component damage did not reach the controller.");
                }
            }
            else if (Result.kind == "insufficient-thrust")
            {
                var volumeObject = new GameObject("SmartAutopilot_TestThrustRestriction");
                volumeObject.transform.SetParent(_ship.transform, false);
                volumeObject.layer = LayerMask.NameToLayer("BasicEffectVolume");
                var collider = volumeObject.AddComponent<SphereCollider>();
                collider.isTrigger = true;
                collider.enabled = false;
                _volume = volumeObject.AddComponent<ThrustRuleset>();
                AutomationBindings.ThrustLimit.Set(_volume, 0);
                GameFields.Rules.Get(_pilot).AddVolume(_volume);
                if (GameFields.Rules.Get(_pilot).GetThrustLimit() != 0)
                {
                    throw new InvalidOperationException("The test thrust restriction was not applied.");
                }
            }
        }

        public void ApplyNavigationState(FlightState state)
        {
            if (Result.applied && !Result.restored && Result.kind == "blocked-route" && state.TargetObstacleIndex >= 0)
            {
                // Keep the synthetic obstruction around the current arrival sphere. Orbital prediction
                // could otherwise find a future arrival sphere outside this test obstruction.
                if (_blockedTarget == null)
                {
                    _blockedTarget = state.Obstacles[state.TargetObstacleIndex];
                    _targetPrimaryIndex = _blockedTarget.PrimaryIndex;
                }

                _blockedTarget.PrimaryIndex = -1;
                _blockedTarget.Radius = state.ArrivalRadius + 1000;
            }
        }

        public void Observe(string phase)
        {
            _phase = phase;
        }

        public void Notice(string message)
        {
            if (Result.applied && !Result.recovered && !Result.notices.Contains(message))
            {
                Result.notices.Add(message);
            }
        }

        public bool Tick(ReferenceFrame target)
        {
            float elapsed = Time.unscaledTime - _started;
            bool blocked = Result.kind == "blocked-route";
            if (!Result.restored)
            {
                bool safe = blocked ? _pilot.IsFlyingToDestination() && _phase == "Hold"
                    : !_pilot.IsFlyingToDestination() && _ship.GetComponent<ShipThrusterController>().GetTranslationalInput().sqrMagnitude < 0.0001f;
                if (safe)
                {
                    if (_safeSince < 0)
                    {
                        _safeSince = Time.unscaledTime;
                    }

                    if (!Result.responseVerified && Time.unscaledTime - _safeSince >= 0.5f)
                    {
                        Result.responseVerified = true;
                        Result.responseSeconds = elapsed;
                        _log("Fault response verified: " + (blocked ? "holding" : "disengaged with zero ship input"));
                    }
                }
                else
                {
                    _safeSince = -1;
                    if (Result.responseVerified)
                    {
                        throw new InvalidOperationException("The fault response did not remain stable while the fault was active.");
                    }
                }

                if (elapsed > 5 && !Result.responseVerified)
                {
                    throw new InvalidOperationException("The expected fault response was not observed within five seconds.");
                }

                float observeDuration = blocked ? 26 : 6;
                if (elapsed >= observeDuration)
                {
                    bool needsNotice = Result.kind != "autopilot-damage";
                    string expectedNotice = blocked ? "No clear route" : Result.kind == "fuel-exhaustion" ? "fuel" : "thrust";
                    if (needsNotice && !Result.notices.Exists(message => message.IndexOf(expectedNotice, StringComparison.OrdinalIgnoreCase) >= 0))
                    {
                        throw new InvalidOperationException("The fault did not produce a user-facing warning.");
                    }

                    Restore();
                    _restoredAt = Time.unscaledTime;
                    _phase = null;
                    _log("Fault removed; checking recovery");
                }

                return false;
            }

            float recoveryTime = Time.unscaledTime - _restoredAt;
            if (!blocked && recoveryTime < 1)
            {
                if (_pilot.IsFlyingToDestination())
                {
                    throw new InvalidOperationException("Autopilot restarted without a new engagement request.");
                }

                return false;
            }

            if (!blocked && !_pilot.IsFlyingToDestination())
            {
                if (_engagedAfterRestore || !_pilot.FlyToDestination(target))
                {
                    throw new InvalidOperationException("Autopilot failed its single engagement after restoring the fault.");
                }

                _engagedAfterRestore = true;
            }

            bool resumed = _pilot.IsFlyingToDestination() && (_phase == "Cruise" || _phase == "Detour" || _phase == "Braking" || _phase == "Departure");
            if (resumed)
            {
                Result.recovered = true;
                _log("Navigation recovered after " + Result.kind);

                return true;
            }

            if (recoveryTime > 8)
            {
                throw new InvalidOperationException("Navigation did not recover after removing the fault.");
            }

            return false;
        }

        private void Restore()
        {
            if (!Result.applied || Result.restored)
            {
                return;
            }

            if (_resources && Result.kind == "fuel-exhaustion")
            {
                _resources.SetFuel(_fuel);
            }

            if (_component)
            {
                _component.SetDamaged(false);
            }

            if (_volume)
            {
                GameFields.Rules.Get(_pilot).RemoveVolume(_volume);
                UnityEngine.Object.Destroy(_volume.gameObject);
            }

            _blockedTarget?.PrimaryIndex = _targetPrimaryIndex;

            Result.restored = true;
        }
    }
}
