using System;
using System.Collections;
using System.IO;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.Serialization;
using SmartAutopilot;

internal static class Program
{
    private static string _managedPath;
    private static string _owmlPath;

    private static void Main(string[] args)
    {
        _managedPath = Path.Combine(args[0], "OuterWilds_Data", "Managed");
        _owmlPath    = args[2];
        AppDomain.CurrentDomain.AssemblyResolve += ResolveGameAssembly;
        CheckBindings();
        CheckAutomationBindings();
        CheckLifecycle(args[1]);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void CheckBindings()
    {
        GameFields.Validate();
        // Allocate managed field storage only. Do not construct Unity components or run native game code.
        var pilot    = (Autopilot)FormatterServices.GetUninitializedObject(typeof(Autopilot));
        var thruster = (ShipThrusterModel)FormatterServices.GetUninitializedObject(typeof(ShipThrusterModel));
        var gravity  = (GravityVolume)FormatterServices.GetUninitializedObject(typeof(GravityVolume));
        var frame    = (ReferenceFrame)FormatterServices.GetUninitializedObject(typeof(ReferenceFrame));

        GameFields.IsShip.Set(pilot, true);
        Require(GameFields.IsShip.Get(pilot), "Private ship flag read/write failed");
        GameFields.Thrusters.Set(pilot, thruster);
        Require(ReferenceEquals(GameFields.Thrusters.Get(pilot), thruster), "Inherited protected field read/write failed");
        GameFields.Target.Set(pilot, frame);
        Require(ReferenceEquals(GameFields.Target.Get(pilot), frame), "Target field read/write failed");
        GameFields.StopMatching.Set(pilot, true);
        Require(GameFields.StopMatching.Get(pilot), "Arrival field read/write failed");
        GameFields.LiningUp.Set(pilot, true);
        Require(pilot.IsLiningUpDestination(), "Stock HUD lining-up flag did not change");
        GameFields.LiningUp.Set(pilot, false);
        GameFields.Approaching.Set(pilot, true);
        Require(!pilot.IsLiningUpDestination() && pilot.IsApproachingDestination(), "Stock HUD approach flags did not change");
        GameFields.SurfaceRadius.Set(gravity, 500);
        Require(GameFields.SurfaceRadius.Get(gravity) == 500, "Private gravity radius read/write failed");
        Require(ReferenceEquals(GameFields.Resources.Get(pilot), null), "Resources binding failed");
        Require(ReferenceEquals(GameFields.Body.Get(pilot), null), "Body binding failed");
        Require(ReferenceEquals(GameFields.Forces.Get(pilot), null), "Forces binding failed");
        Require(ReferenceEquals(GameFields.Rules.Get(pilot), null), "Rules binding failed");
        bool brakingNotification = false;
        pilot.OnFireRetroRockets += () =>
        {
            brakingNotification = true;
        };
        GameFields.RetroRockets.Get(pilot).Invoke();
        Require(brakingNotification, "Stock braking notification event did not fire");
        var cockpit = (ShipCockpitController)FormatterServices.GetUninitializedObject(typeof(ShipCockpitController));
        var controller = (ShipThrusterController)FormatterServices.GetUninitializedObject(typeof(ShipThrusterController));
        GameFields.CockpitFailure.Set(cockpit, true);
        Require(GameFields.CockpitFailure.Get(cockpit), "Cockpit failure guard binding failed");
        GameFields.ControlsLocked.Set(cockpit, true);
        Require(GameFields.ControlsLocked.Get(cockpit), "Cockpit control lock binding failed");
        GameFields.RequireIgnition.Set(controller, true);
        Require(controller.RequiresIgnition(), "Cold engine binding failed");
        GameFields.RequireIgnition.Set(controller, false);
        Require(!controller.RequiresIgnition(), "Ignition completion binding failed");
        GameFields.IsIgniting.Set(controller, true);
        Require(GameFields.IsIgniting.Get(controller), "Ignition event ownership binding failed");
        GameFields.IsIgniting.Set(controller, false);
        Require(!GameFields.IsIgniting.Get(controller), "Ignition cancellation binding failed");
        GameFields.IgnitionDuration.Set(controller, 1.25f);
        Require(GameFields.IgnitionDuration.Get(controller) == 1.25f, "Ignition duration binding failed");
        GameFields.LastThrustInput.Set(controller, new UnityEngine.Vector3(0, 1, 0));
        Require(GameFields.LastThrustInput.Get(controller).y == 1, "Manual ignition input binding failed");
        GameFields.LastThrustInput.Set(controller, new UnityEngine.Vector3(0, 0, 0));
        Require(GameFields.LastThrustInput.Get(controller).y == 0, "Manual ignition input reset failed");
        var ringController = (RingWorldController)FormatterServices.GetUninitializedObject(typeof(RingWorldController));
        var ringBody = (OWRigidbody)FormatterServices.GetUninitializedObject(typeof(OWRigidbody));
        GameFields.StaticRingBody.Set(ringController, ringBody);
        Require(ReferenceEquals(GameFields.StaticRingBody.Get(ringController), ringBody), "Stationary Stranger hull binding failed");
        Console.WriteLine("PASS: all 19 runtime field bindings exercised against the installed game assembly using .NET Framework. Unity flight and Mono runtime testing remain separate.");
    }

    private static Assembly ResolveGameAssembly(object sender, ResolveEventArgs args)
    {
        string candidate = Path.Combine(_managedPath, new AssemblyName(args.Name).Name + ".dll");

        if (File.Exists(candidate))
        {
            return Assembly.LoadFrom(candidate);
        }

        candidate = Path.Combine(_owmlPath, new AssemblyName(args.Name).Name + ".dll");

        return File.Exists(candidate) ? Assembly.LoadFrom(candidate) : null;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void CheckAutomationBindings()
    {
        SmartAutopilot.Automation.AutomationBindings.Validate();
        var effects = (PlayerCameraEffectController)FormatterServices.GetUninitializedObject(typeof(PlayerCameraEffectController));
        SmartAutopilot.Automation.AutomationBindings.WaitingForWake.Set(effects, true);
        Require(SmartAutopilot.Automation.AutomationBindings.WaitingForWake.Get(effects), "Automation wake flag binding failed");
        Require(SmartAutopilot.Automation.AutomationBindings.WakePrompt.Get(effects) == null, "Automation wake prompt binding failed");
        var title = (TitleScreenManager)FormatterServices.GetUninitializedObject(typeof(TitleScreenManager));
        Require(ReferenceEquals(SmartAutopilot.Automation.AutomationBindings.Resume.Get(title), null), "Automation resume action binding failed");
        Require(ReferenceEquals(SmartAutopilot.Automation.AutomationBindings.MenuBlocker.Get(title), null), "Automation menu blocker binding failed");
        var animation = (TitleScreenAnimation)FormatterServices.GetUninitializedObject(typeof(TitleScreenAnimation));
        SmartAutopilot.Automation.AutomationBindings.GamepadSplash.Set(animation, true);
        SmartAutopilot.Automation.AutomationBindings.GamepadFadingIn.Set(animation, true);
        SmartAutopilot.Automation.AutomationBindings.GamepadFadingOut.Set(animation, true);
        Require(SmartAutopilot.Automation.AutomationBindings.GamepadSplash.Get(animation)
            && SmartAutopilot.Automation.AutomationBindings.GamepadFadingIn.Get(animation)
            && SmartAutopilot.Automation.AutomationBindings.GamepadFadingOut.Get(animation), "Automation splash flags failed");
        Require(ReferenceEquals(SmartAutopilot.Automation.AutomationBindings.GamepadFade.Get(animation), null), "Automation splash fade binding failed");
        var thrustRules = (ThrustRuleset)FormatterServices.GetUninitializedObject(typeof(ThrustRuleset));
        SmartAutopilot.Automation.AutomationBindings.ThrustLimit.Set(thrustRules, 0.5f);
        Require(SmartAutopilot.Automation.AutomationBindings.ThrustLimit.Get(thrustRules) == 0.5f, "Automation thrust restriction binding failed");
        Console.WriteLine("PASS: nine optional automation fields and two method signatures validated. Native invocation needs the in-game runner.");
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void CheckLifecycle(string modPath)
    {
        var assembly = Assembly.LoadFrom(modPath);
        var type     = assembly.GetType("SmartAutopilot.SmartAutopilotMod", true);
        var mod      = FormatterServices.GetUninitializedObject(type);
        var pilot    = (Autopilot)FormatterServices.GetUninitializedObject(typeof(Autopilot));
        GameFields.IsShip.Set(pilot, true);
        const BindingFlags fields = BindingFlags.Instance | BindingFlags.NonPublic;
        var stateField = type.GetField("_state", fields);
        var state      = Activator.CreateInstance(stateField.FieldType, true);
        stateField.SetValue(mod, state);
        var recorderField = type.GetField("_recorder", fields);
        recorderField.SetValue(mod, Activator.CreateInstance(recorderField.FieldType, true));
        var bodiesField = type.GetField("_bodies", fields);
        var bodies      = (IList)Activator.CreateInstance(bodiesField.FieldType);
        bodiesField.SetValue(mod, bodies);
        var hullsField = type.GetField("_navigationHulls", fields);
        var hulls      = (IList)Activator.CreateInstance(hullsField.FieldType);
        hullsField.SetValue(mod, hulls);
        var obstacles = (IList)state.GetType().GetField("Obstacles").GetValue(state);
        var arrivalLimits = (IList)state.GetType().GetField("ArrivalThrustLimits").GetValue(state);
        var arrivalLimitType = assembly.GetType("SmartAutopilot.Navigation.ArrivalThrustLimit", true);
        var arrivalReaderField = type.GetField("_arrivalLimits", fields);
        var obstacleType = assembly.GetType("SmartAutopilot.Navigation.Obstacle", true);
        var phaseType    = assembly.GetType("SmartAutopilot.Navigation.FlightPhase", true);
        var sceneArgs    = new object[] { default(OWScene), default(OWScene) };
        for (int loop = 0; loop < 3; loop++)
        {
            var faultsField = type.GetField("_faults", fields);
            var faults = Activator.CreateInstance(faultsField.FieldType, true);
            faultsField.SetValue(mod, faults);
            faults.GetType().GetMethod("Record").Invoke(faults, [1.0]);
            bodies.Add(FormatterServices.GetUninitializedObject(typeof(AstroObject)));
            hulls.Add(FormatterServices.GetUninitializedObject(assembly.GetType("SmartAutopilot.NavigationHull", true)));
            obstacles.Add(Activator.CreateInstance(obstacleType, true));
            arrivalLimits.Add(Activator.CreateInstance(arrivalLimitType, new object[] { 515.0, 20.0 }));
            arrivalReaderField.SetValue(mod, FormatterServices.GetUninitializedObject(arrivalReaderField.FieldType));
            type.GetField("_pilot", fields).SetValue(mod, pilot);
            var computerField = type.GetField("_computer", fields);
            computerField.SetValue(mod, Activator.CreateInstance(computerField.FieldType, true));
            type.GetField("_lastPhase", fields).SetValue(mod, Enum.Parse(phaseType, "Braking"));
            type.GetField("_finishing", fields).SetValue(mod, true);
            type.GetField("_departureUnavailable", fields).SetValue(mod, true);
            type.GetField("_nextFlightSample", fields).SetValue(mod, 1000.0);
            state.GetType().GetField("TargetObstacleIndex").SetValue(state, 0);
            type.GetMethod("OnStartSceneLoad", fields).Invoke(mod, sceneArgs);
            Require(bodies.Count == 0 && obstacles.Count == 0 && hulls.Count == 0, "Scene transition retained old celestial bodies or hull geometry");
            Require(arrivalLimits.Count == 0 && arrivalReaderField.GetValue(mod) == null, "Scene transition retained the old destination's thrust limits");
            Require((int)state.GetType().GetField("TargetObstacleIndex").GetValue(state) == -1, "Scene transition retained the old target index");
            Require(type.GetField("_pilot", fields).GetValue(mod) == null && computerField.GetValue(mod) == null,
                "Scene transition retained a flight or braking controller");
            Require(type.GetField("_lastPhase", fields).GetValue(mod) == null
                && !(bool)type.GetField("_finishing", fields).GetValue(mod)
                && !(bool)type.GetField("_departureUnavailable", fields).GetValue(mod)
                && (double)type.GetField("_nextFlightSample", fields).GetValue(mod) == 0,
                "Scene transition retained completion, departure failure or telemetry timing");
            Require((bool)type.GetField("_sceneLoading", fields).GetValue(mod), "Scene transition did not set the loading guard");
            Require(!(bool)faultsField.FieldType.GetProperty("Faulted").GetValue(faultsField.GetValue(mod)), "New scene retained the navigation fault lockout");
            Require(type.GetField("_flightTarget", fields).GetValue(mod) == null
                && type.GetField("_collision", fields).GetValue(mod) == null
                && type.GetField("_debugOverlay", fields).GetValue(mod) == null, "Scene transition retained navigation or debug state");
            type.GetMethod("OnCompleteSceneLoad", fields).Invoke(mod, sceneArgs);
            Require(!(bool)type.GetField("_sceneLoading", fields).GetValue(mod), "New scene left navigation locked out");
        }

        Console.WriteLine("PASS: built mod clears flight state and sets/releases the loading guard across three simulated scene callbacks. Native input, prompt disposal and real scene loads still require Unity testing.");
    }

    private static void Require(bool condition, string message)
    {
        if (!condition)
        {
            throw new InvalidOperationException(message);
        }
    }
}
