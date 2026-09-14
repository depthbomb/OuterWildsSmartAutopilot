using System;
using System.Linq;
using System.Reflection;

namespace SmartAutopilot
{
    internal sealed class GameField<TTarget, TValue>
    {
        private readonly FieldInfo _field;

        public GameField(string name)
        {
            var type = typeof(TTarget);
            while (type != null)
            {
                _field = type.GetField(name, BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.DeclaredOnly);
                if (_field != null)
                {
                    break;
                }

                type = type.BaseType;
            }

            if (_field == null || _field.FieldType != typeof(TValue))
            {
                throw new MissingFieldException(typeof(TTarget).FullName, name + " (expected " + typeof(TValue).Name + ")");
            }
        }

        public TValue Get(TTarget target) => (TValue)_field.GetValue(target);

        public void Set(TTarget target, TValue value)
        {
            _field.SetValue(target, value);
        }
    }

    internal static class GameFields
    {
        internal static readonly GameField<Autopilot, bool>                             IsShip           = new("_isShipAutopilot");
        internal static readonly GameField<Autopilot, ShipResources>                    Resources        = new("_shipResources");
        internal static readonly GameField<Autopilot, ReferenceFrame>                   Target           = new("_referenceFrame");
        internal static readonly GameField<Autopilot, ThrusterModel>                    Thrusters        = new("_thrusterModel");
        internal static readonly GameField<Autopilot, OWRigidbody>                      Body             = new("_owRigidbody");
        internal static readonly GameField<Autopilot, ForceDetector>                    Forces           = new("_forceDetector");
        internal static readonly GameField<Autopilot, RulesetDetector>                  Rules            = new("_rulesetDetector");
        internal static readonly GameField<Autopilot, bool>                             StopMatching     = new("_stopMatchingNextFrame");
        internal static readonly GameField<Autopilot, bool>                             LiningUp         = new("_isLiningUpDestination");
        internal static readonly GameField<Autopilot, bool>                             Approaching      = new("_isApproachingDestination");
        internal static readonly GameField<GravityVolume, float>                        SurfaceRadius    = new("_upperSurfaceRadius");
        internal static readonly GameField<Autopilot, Autopilot.FireRetroRocketsEvent>  RetroRockets     = new("OnFireRetroRockets");
        internal static readonly GameField<ShipCockpitController, bool>                 CockpitFailure   = new("_shipSystemFailure");
        internal static readonly GameField<ShipCockpitController, bool>                 ControlsLocked   = new("_controlsLocked");
        internal static readonly GameField<ShipThrusterController, bool>                RequireIgnition  = new("_requireIgnition");
        internal static readonly GameField<ShipThrusterController, bool>                IsIgniting       = new("_isIgniting");
        internal static readonly GameField<ShipThrusterController, float>               IgnitionDuration = new("_ignitionDuration");
        internal static readonly GameField<ShipThrusterController, UnityEngine.Vector3> LastThrustInput  = new("_lastTranslationalInput");
        internal static readonly GameField<RingWorldController, OWRigidbody>            StaticRingBody   = new("_staticRingBody");

        internal static void Validate()
        {
            // Touch every binding before installing patches, so missing fields fail once at startup.
            object[] bindings = [IsShip, Resources, Target, Thrusters, Body, Forces, Rules, StopMatching, LiningUp, Approaching, SurfaceRadius, RetroRockets, CockpitFailure, ControlsLocked, RequireIgnition, IsIgniting, IgnitionDuration, LastThrustInput, StaticRingBody];
            if (bindings.Any(binding => binding == null))
            {
                throw new InvalidOperationException("A game field binding was not initialized.");
            }
        }
    }
}
