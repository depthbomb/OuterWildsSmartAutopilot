using System.Reflection;

namespace SmartAutopilot.Automation;

internal static class AutomationBindings
{
    internal static readonly GameField<TitleScreenManager, SubmitActionLoadScene>       Resume           = new("_resumeGameAction");
    internal static readonly GameField<TitleScreenManager, UnityEngine.CanvasGroup>     MenuBlocker      = new("_titleMenuRaycastBlocker");
    internal static readonly GameField<PlayerCameraEffectController, bool>              WaitingForWake   = new("_waitForWakeInput");
    internal static readonly GameField<PlayerCameraEffectController, ScreenPrompt>      WakePrompt       = new("_wakePrompt");
    internal static readonly GameField<TitleScreenAnimation, bool>                      GamepadSplash    = new("_gamepadSplash");
    internal static readonly GameField<TitleScreenAnimation, bool>                      GamepadFadingIn  = new("_fadingInGamepad");
    internal static readonly GameField<TitleScreenAnimation, bool>                      GamepadFadingOut = new("_fadingOutGamepad");
    internal static readonly GameField<TitleScreenAnimation, CanvasGroupFadeController> GamepadFade      = new("_gamepadSplashController");
    internal static readonly GameField<ThrustRuleset, float>                            ThrustLimit      = new("_thrustLimit");
    internal static readonly MethodInfo                                                 Wake             = FindMethod(typeof(PlayerCameraEffectController), "WakeUp");
    internal static readonly MethodInfo                                                 EnterCockpit     = FindMethod(typeof(ShipCockpitController), "OnPressInteract");

    internal static void Validate()
    {
        if (Resume           == null ||
            MenuBlocker      == null ||
            WaitingForWake   == null ||
            WakePrompt       == null ||
            Wake             == null ||
            EnterCockpit     == null ||
            GamepadSplash    == null ||
            GamepadFadingIn  == null ||
            GamepadFadingOut == null ||
            GamepadFade      == null ||
            ThrustLimit      == null)
        {
            throw new InvalidOperationException("Automation game bindings are unavailable.");
        }
    }

    private static MethodInfo FindMethod(Type type, string name)
    {
        var method = type.GetMethod(name, BindingFlags.Instance | BindingFlags.NonPublic, null, Type.EmptyTypes, null);
        if (method == null || method.ReturnType != typeof(void))
        {
            throw new MissingMethodException(type.FullName, name);
        }

        return method;
    }
}
