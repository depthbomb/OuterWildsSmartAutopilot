using System;
using SmartAutopilot.Automation;

internal static class AutomationRequestTests
{
    public static void ValidateRequests()
    {
        var now = DateTime.UtcNow;
        Create(now).Validate(now);
        Reject(now, request => request.id = "../outside");
        Reject(now, request => request.protocol = 2);
        Reject(now, request => request.fault = "unknown");
        Reject(now, request => request.fault = "fuel-exhaustion");
        Reject(now, request =>
        {
            request.targets            = ["Moon_Body"];
            request.fault              = "autopilot-damage";
            request.cancelAfterSeconds = 5;
        });

        foreach (string fault in new[] { "fuel-exhaustion", "autopilot-damage", "insufficient-thrust", "blocked-route" })
        {
            var faultRequest = Create(now);
            faultRequest.targets = ["Moon_Body"];
            faultRequest.fault   = fault;
            faultRequest.Validate(now);
        }

        Reject(now, request => request.expiresUtc        = now.AddSeconds(-1).ToString("O"));
        Reject(now, request => request.expiresUtc        = now.AddDays(1).ToString("O"));
        Reject(now, request => request.targets           = []);
        Reject(now, request => request.targets           = [" "]);
        Reject(now, request => request.legTimeoutSeconds = double.NaN);
        Reject(now, request => request.launchAtSeconds   = double.PositiveInfinity);
        Reject(now, request => request.startBody         = "Sun_Body");
        Reject(now, request =>
        {
            request.startBody     = "Sun_Body";
            request.startOffset   = [float.NaN, 0, 0];
            request.startVelocity = new float[3];
        });

        var anchored = Create(now);
        anchored.startBody          = "BrittleHollow_Body";
        anchored.startOffset        = [2000, 500, 0];
        anchored.startVelocity      = [0, 0, 100];
        anchored.cancelAfterSeconds = 5;
        anchored.Validate(now);
    }

    private static AutomationRequest Create(DateTime now) => new()
    {
        id         = Guid.NewGuid().ToString("N"),
        expiresUtc = now.AddMinutes(30).ToString("O"),
        dllHash    = new string('A', 64),
        targets    = ["VolcanicMoon_Body", "Comet_Body"]
    };

    private static void Reject(DateTime now, Action<AutomationRequest> change)
    {
        var request = Create(now);
        change(request);
        try
        {
            request.Validate(now);
        }
        catch (InvalidOperationException)
        {
            return;
        }

        throw new Exception("An invalid automation request was accepted");
    }
}
