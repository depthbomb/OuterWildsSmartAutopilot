using System.Globalization;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using Newtonsoft.Json;
using SmartAutopilot.Navigation;
using UnityEngine;

namespace SmartAutopilot.Automation;

[Serializable]
internal sealed class AutomationLeg
{
    public string                target;
    public string                status = "running";
    public string                reason;
    public double                startedAtLoopSeconds;
    public double                flightSeconds;
    public double                arrivalError;
    public double                relativeSpeed;
    public double                arrivalRadius;
    public double                minimumModeledClearance = double.MaxValue;
    public double                fuelUsed;
    public double                startingFuel;
    public double                endingFuel;
    public string[]              damagedComponentsAtStart = [];
    public string[]              damagedComponentsAtEnd   = [];
    public int                   samples;
    public int                   escapes;
    public int                   holds;
    public int                   impacts;
    public double                maximumImpactSpeed;
    public bool                  cancelledAndReengaged;
    public bool                  stockArrivalEvent;
    public AutomationFaultResult fault;
}

[Serializable]
internal sealed class AutomationResult
{
    public int                 protocol = 1;
    public string              id;
    public string              dllHash;
    public string              gameVersion;
    public int                 gameProcessId;
    public string              status = "running";
    public string              stage  = "title";
    public string              reason;
    public string              updatedUtc;
    public string              shutdown = "pending";
    public int                 renderedFrames;
    public double              maximumFrameSeconds;
    public int                 framesOver100ms;
    public List<AutomationLeg> legs = [];
}

internal sealed class AutomationRunner : IDisposable
{
    private AutomationRequest _request;
    private AutomationResult  _result;
    private string            _directory;
    private StreamWriter      _samples;
    private StreamWriter      _events;
    private OWRigidbody       _ship;
    private Autopilot         _pilot;
    private ImpactSensor      _impacts;
    private ReferenceFrame    _target;
    private AutomationLeg     _leg;
    private string            _lastPhase;
    private float             _stageStarted;
    private float             _started;
    private float             _nextStatus;
    private float             _nextSample;
    private float             _legStarted;
    private float             _fuelStart;
    private bool              _cancelRequested;
    private bool              _finished;
    private bool              _disposed;
    private float             _splashSeen = -1;
    private float             _introSeen  = -1;
    private bool              _introSkipped;
    private AutomationFault   _fault;

    private readonly Action<string> _log;

    private static readonly string Root = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), "Codex", "ProjectNotes", "outer-wilds-smart-autopilot", "automation");

    private AutomationRunner(Action<string> log)
    {
        _log = log;
    }

    public void Dispose()
    {
        if (_disposed)
        {
            return;
        }

        _disposed = true;
        _fault?.Dispose();
        if (_pilot != null)
        {
            _pilot.OnArriveAtDestination -= OnArrival;
        }

        if (_impacts != null)
        {
            _impacts.OnImpact -= OnImpact;
        }

        _samples?.Dispose();
        _events?.Dispose();
    }

    public void Update()
    {
        if (_disposed)
        {
            return;
        }

        try
        {
            Tick();
        }
        catch (Exception exception)
        {
            Finish(false, exception.ToString());
        }
    }

    public void Observe(FlightState state, string phase, IReadOnlyList<AstroObject> bodies)
    {
        if (_leg is not { status: "running" } || _finished)
        {
            return;
        }

        _fault?.Observe(phase);
        _leg.arrivalError  = (state.Position - state.TargetPosition).Length - state.ArrivalRadius;
        _leg.arrivalRadius = state.ArrivalRadius;
        _leg.relativeSpeed = (state.Velocity - state.TargetVelocity).Length;
        if (_leg.samples == 0)
        {
            var initial = new
            {
                time                 = state.Time,
                deltaTime            = state.DeltaTime,
                position             = Components(state.Position),
                velocity             = Components(state.Velocity),
                externalAcceleration = Components(state.ExternalAcceleration),
                targetPosition       = Components(state.TargetPosition),
                targetVelocity       = Components(state.TargetVelocity),
                targetAcceleration   = Components(state.TargetAcceleration),
                targetIndex          = state.TargetObstacleIndex,
                arrivalRadius        = state.ArrivalRadius,
                thrust               = state.Thrust,
                maximumThrust        = state.MaximumThrust,
                obstacles = state.Obstacles.Select((body, index) => new
                {
                    name           = index < bodies.Count && bodies[index] != null ? bodies[index].name : "unknown",
                    primaryIndex   = body.PrimaryIndex,
                    position       = Components(body.Position),
                    velocity       = Components(body.Velocity),
                    acceleration   = Components(body.Acceleration),
                    radius         = body.Radius,
                    physicalRadius = body.PhysicalRadius
                }).ToArray()
            };
            File.WriteAllText(Path.Combine(_directory, "initial-state-" + _result.legs.Count + ".json"), JsonConvert.SerializeObject(initial, Formatting.Indented));
        }

        if (phase != _lastPhase)
        {
            switch (phase)
            {
                case "Escape":
                    _leg.escapes++;
                    break;
                case "Hold":
                    _leg.holds++;
                    break;
            }

            Event("phase=" + phase);
            _lastPhase = phase;
        }

        // This is modeled clearance, not a replacement for actual impact telemetry.
        foreach (var obstacle in state.Obstacles)
        {
            if (obstacle.PhysicalRadius > 0)
            {
                _leg.minimumModeledClearance = Math.Min(_leg.minimumModeledClearance, (state.Position - obstacle.Position).Length - obstacle.PhysicalRadius);
            }
        }

        if (Time.unscaledTime < _nextSample)
        {
            return;
        }

        _nextSample = Time.unscaledTime + 0.25f;
        _leg.samples++;

        var offset   = state.Position - state.TargetPosition;
        var velocity = state.Velocity - state.TargetVelocity;

        _samples.WriteLine(FormattableString.Invariant($"{_result.legs.Count},{state.Time:F3},{Time.unscaledTime:F3},{phase},{_leg.arrivalError:F3},{_leg.relativeSpeed:F3},{state.Thrust:F3},{offset.X:F3},{offset.Y:F3},{offset.Z:F3},{velocity.X:F3},{velocity.Y:F3},{velocity.Z:F3},{_leg.minimumModeledClearance:F3}"));
    }

    public void ApplyNavigationFault(FlightState state)
    {
        _fault?.ApplyNavigationState(state);
    }

    public void ObserveNotice(string message)
    {
        _fault?.Notice(message);
    }

    public static AutomationRunner TryStart(Action<string> log)
    {
        var path = Path.Combine(Root, "request.json");
        if (!File.Exists(path))
        {
            return null;
        }

        var runner = new AutomationRunner(log);
        try
        {
            if (new FileInfo(path).Length > 65536)
            {
                throw new InvalidOperationException("Automation request exceeds 64KB.");
            }

            var request = JsonConvert.DeserializeObject<AutomationRequest>(File.ReadAllText(path));
            request.Validate(DateTime.UtcNow);

            var directory = Path.Combine(Root, request.id);
            var lease     = Path.Combine(directory, "lease");
            if (!File.Exists(lease) || DateTime.UtcNow - File.GetLastWriteTimeUtc(lease) > TimeSpan.FromSeconds(30))
            {
                throw new InvalidOperationException("Automation controller lease is missing or stale.");
            }

            using (var sha = SHA256.Create())
            using (var assembly = File.OpenRead(typeof(AutomationRunner).Assembly.Location))
            {
                var hash = BitConverter.ToString(sha.ComputeHash(assembly)).Replace("-", "");
                if (!string.Equals(hash, request.dllHash, StringComparison.OrdinalIgnoreCase))
                {
                    throw new InvalidOperationException("Automation request identifies a different installed build.");
                }
            }

            File.Move(path, Path.Combine(directory, "claimed.json"));
            runner._request   = request;
            runner._directory = directory;
            runner._result = new AutomationResult
            {
                id            = request.id,
                dllHash       = request.dllHash,
                gameVersion   = Application.version,
                gameProcessId = System.Diagnostics.Process.GetCurrentProcess().Id
            };
            runner._events = new StreamWriter(Path.Combine(directory, "events.log"))
            {
                AutoFlush = true
            };
            runner._samples = new StreamWriter(Path.Combine(directory, "samples.csv"));
            runner._samples.WriteLine("leg,gameTime,realTime,phase,remaining,speed,availableThrust,offsetX,offsetY,offsetZ,velocityX,velocityY,velocityZ,minimumModeledClearance");
            AutomationBindings.Validate();
            Application.runInBackground = true;
            runner._started             = runner._stageStarted = Time.unscaledTime;
            runner.Event("Automation claimed; waiting for Resume Expedition");
            runner.WriteStatus();

            return runner;
        }
        catch (Exception exception)
        {
            if (runner._result != null)
            {
                runner.Finish(false, exception.ToString());

                return runner;
            }

            log("Automation request rejected: " + exception.Message);
            runner.Dispose();

            return null;
        }
    }

    private void Tick()
    {
        var elapsed = Time.unscaledTime - _stageStarted;
        if (Time.unscaledTime >= _nextStatus)
        {
            _nextStatus = Time.unscaledTime + 2;
            WriteStatus();
            var disconnected = DateTime.UtcNow - File.GetLastWriteTimeUtc(Path.Combine(_directory, "lease")) > TimeSpan.FromSeconds(30);
            if (!_finished && (disconnected || File.Exists(Path.Combine(_directory, "stop"))))
            {
                Finish(false, "Controller stopped or disconnected");

                return;
            }
        }

        if (_finished)
        {
            if (_result.stage == "returning-title" && LoadManager.GetCurrentScene() == OWScene.TitleScreen && elapsed > 4)
            {
                _result.shutdown = "returned-to-title";
                Screenshot("title-after");
                SetStage("quitting");
                return;
            }

            if (_result.stage == "quitting" && elapsed > 2 || elapsed > 25)
            {
                WriteFinal();
                Application.Quit();
            }

            return;
        }

        bool setup = _result.stage != "flying" && _result.stage != "between-legs" && _result.stage != "reengaging" && _result.stage != "fault-observe";
        if (setup && Time.unscaledTime - _started > _request.setupTimeoutSeconds)
        {
            Finish(false, "Setup timed out in " + _result.stage);

            return;
        }

        switch (_result.stage)
        {
            case "title":
                SkipSplashes();
                var title = UnityEngine.Object.FindObjectOfType<TitleScreenManager>();
                if (title && elapsed > 5)
                {
                    var resume    = AutomationBindings.Resume.Get(title);
                    var blocker   = AutomationBindings.MenuBlocker.Get(title);
                    var menuReady = StandaloneProfileManager.SharedInstance.isInitialized && MenuStackManager.SharedInstance.GetMenuCount() == 0;
                    if (resume && blocker && !blocker.blocksRaycasts && menuReady && resume.gameObject.activeInHierarchy)
                    {
                        if (PlayerData.GetWarpedToTheEye())
                        {
                            throw new InvalidOperationException("The active save resumes at the Eye, not the solar system.");
                        }

                        Screenshot("title-before");
                        resume.Submit();
                        SetStage("waking");
                    }
                }
                break;
            case "waking":
                if (LoadManager.GetCurrentScene() == OWScene.SolarSystem && LateInitializerManager.isDoneInitializing && Locator.GetPlayerCamera())
                {
                    var effects = Locator.GetPlayerCamera().GetComponent<PlayerCameraEffectController>();
                    if (effects && AutomationBindings.WaitingForWake.Get(effects))
                    {
                        AutomationBindings.WaitingForWake.Set(effects, false);
                        LateInitializerManager.pauseOnInitialization = false;
                        Locator.GetPauseCommandListener().RemovePauseCommandLock();
                        Locator.GetPromptManager().RemoveScreenPrompt(AutomationBindings.WakePrompt.Get(effects));
                        OWTime.Unpause(OWTime.PauseType.Sleeping);
                        AutomationBindings.Wake.Invoke(effects, null);
                        SetStage("wake-animation");
                    }
                }
                break;
            case "wake-animation":
                if (elapsed > 8 && !OWTime.IsPaused())
                {
                    Screenshot("awake");
                    var spawner = UnityEngine.Object.FindObjectOfType<PlayerSpawner>();
                    var spawn   = spawner.GetSpawnPoint(SpawnLocation.Ship);
                    if (!spawn)
                    {
                        throw new InvalidOperationException("Player ship spawn point was not found.");
                    }

                    spawner.DebugWarp(spawn);
                    SetStage("boarding");
                }
                break;
            case "boarding":
                if (elapsed > 2)
                {
                    _ship = Locator.GetShipBody();
                    var cockpit = _ship.GetComponentInChildren<ShipCockpitController>();
                    AutomationBindings.EnterCockpit.Invoke(cockpit, null);
                    _pilot                       =  _ship.GetComponent<Autopilot>();
                    _pilot.OnArriveAtDestination += OnArrival;
                    _impacts                     =  _ship.GetComponent<ImpactSensor>();
                    if (_impacts)
                    {
                        _impacts.OnImpact += OnImpact;
                    }

                    SetStage("cockpit");
                }
                break;
            case "cockpit":
                if (elapsed > 3 && PlayerState.AtFlightConsole() && OWInput.IsInputMode(InputMode.ShipCockpit))
                {
                    Screenshot("cockpit");
                    SetStage("ready");
                }
                break;
            case "ready":
                if (elapsed > 2 && TimeLoop.GetSecondsElapsed() >= _request.launchAtSeconds)
                {
                    if (!string.IsNullOrEmpty(_request.startBody))
                    {
                        var anchor   = FindBody(_request.startBody);
                        var offset   = new Vector3(_request.startOffset[0], _request.startOffset[1], _request.startOffset[2]);
                        var velocity = new Vector3(_request.startVelocity[0], _request.startVelocity[1], _request.startVelocity[2]);
                        _ship.Unsuspend();
                        _ship.WarpToPositionRotation(anchor.GetWorldCenterOfMass() + offset, _ship.GetRotation());
                        _ship.SetVelocity(anchor.GetVelocity()                     + velocity);
                        _ship.SetAngularVelocity(Vector3.zero);
                        Event("Ship positioned relative to " + _request.startBody);
                        SetStage("positioning");

                        return;
                    }

                    StartLeg();
                }
                break;
            case "positioning":
                // Let physics refresh the launch-pad contacts and detector volumes after the warp.
                if (elapsed > 0.25f)
                {
                    StartLeg();
                }
                break;
            case "between-legs":
                if (elapsed > 1)
                {
                    if (_result.legs.Count == _request.targets.Length)
                    {
                        Finish(true, "All requested routes arrived within tolerance");
                    }
                    else
                    {
                        StartLeg();
                    }
                }
                break;
            case "reengaging":
                if (elapsed >= _request.reengageDelaySeconds)
                {
                    if (!_pilot.FlyToDestination(_target))
                    {
                        throw new InvalidOperationException("Re-engagement was refused.");
                    }

                    _leg.cancelledAndReengaged = true;
                    SetStage("flying");
                }
                break;
            case "flying":
                CheckFlight();
                break;
            case "fault-observe":
                _leg.flightSeconds = Time.unscaledTime - _legStarted;
                if (_leg.flightSeconds > _request.legTimeoutSeconds || Locator.GetDeathManager().IsPlayerDead() || _leg.maximumImpactSpeed >= 10)
                {
                    Finish(false, "Fault test timed out, the player died, or the ship suffered an impact");
                }
                else if (_fault.Tick(_target))
                {
                    Screenshot("fault-recovered");
                    SetStage("flying");
                }
                break;
        }
    }

    private void StartLeg()
    {
        var targetName = _request.targets[_result.legs.Count];
        var body       = FindBody(targetName);
        if (targetName == "RingWorld_Body" && Locator.GetCloakFieldController() != null)
        {
            // Selecting a location inside the Stranger in the ship log enables this same volume.
            Locator.GetCloakFieldController().SetReferenceFrameVolumeActive(true);
            Event("Enabled Stranger targeting through its ship-log reference volume");
        }

        _target = body.GetReferenceFrame();
        if (_target == null || !_target.GetAllowAutopilot())
        {
            throw new InvalidOperationException("Requested body has no autopilot-enabled reference frame: " + targetName);
        }

        Locator.GetPlayerBody().GetComponent<ReferenceFrameTracker>().TargetReferenceFrame(_target);
        _leg = new AutomationLeg
        {
            target               = _request.targets[_result.legs.Count],
            startedAtLoopSeconds = TimeLoop.GetSecondsElapsed()
        };
        _result.legs.Add(_leg);
        _legStarted                   = Time.unscaledTime;
        _fuelStart                    = GameFields.Resources.Get(_pilot).GetFractionalFuel();
        _leg.startingFuel             = _fuelStart;
        _leg.damagedComponentsAtStart = GetDamagedComponents(_ship);
        _lastPhase                    = null;
        _nextSample                   = 0;
        _cancelRequested              = false;
        SetStage("flying");
        if (_leg.damagedComponentsAtStart.Length > 0)
        {
            Finish(false, "Ship was already damaged before engagement: " + string.Join(", ", _leg.damagedComponentsAtStart));

            return;
        }

        if (!string.IsNullOrEmpty(_request.fault))
        {
            _fault     = new AutomationFault(_request.fault, _pilot, _ship, Event);
            _leg.fault = _fault.Result;
        }

        if (!_pilot.FlyToDestination(_target))
        {
            Finish(false, "Autopilot refused target " + _leg.target);
        }
    }

    private void SkipSplashes()
    {
        var animation = UnityEngine.Object.FindObjectOfType<TitleScreenAnimation>();
        if (!animation || animation.IsPaused())
        {
            return;
        }

        if (AutomationBindings.GamepadSplash.Get(animation))
        {
            if (_splashSeen < 0)
            {
                _splashSeen = Time.unscaledTime;
            }

            var skippable = Time.unscaledTime - _splashSeen >= 1 && AutomationBindings.GamepadFadingIn.Get(animation)
                                                                 && !AutomationBindings.GamepadFadingOut.Get(animation);
            if (skippable)
            {
                // Use the same fade and state transition as the splash's input handler.
                AutomationBindings.GamepadFadingOut.Set(animation, true);
                AutomationBindings.GamepadFade.Get(animation).FadeTo(0f, 0.5f);
                Event("Skipped controller splash");
            }

            return;
        }

        if (_introSeen < 0)
        {
            _introSeen = Time.unscaledTime;
        }

        if (!_introSkipped && Time.unscaledTime - _introSeen >= 1 && animation.IsPlayingIntroAnimation() && !animation.IsFadingIn())
        {
            animation.SkipToTitle();
            _introSkipped = true;
            Event("Skipped logo animation");
        }
    }

    private void CheckFlight()
    {
        _result.renderedFrames++;
        _result.maximumFrameSeconds = Math.Max(_result.maximumFrameSeconds, Time.unscaledDeltaTime);
        if (Time.unscaledDeltaTime > 0.1f)
        {
            _result.framesOver100ms++;
        }

        if (Locator.GetDeathManager().IsPlayerDead() || Locator.GetDeathManager().IsPlayerDying())
        {
            Finish(false, "Player died during the route");

            return;
        }

        if (!PlayerState.AtFlightConsole() || _pilot.IsDamaged() || !GameFields.Resources.Get(_pilot).AreThrustersUsable())
        {
            Finish(false, "Cockpit, autopilot or thrusters became unavailable");

            return;
        }

        _leg.flightSeconds = Time.unscaledTime - _legStarted;
        if (_leg.flightSeconds > _request.legTimeoutSeconds)
        {
            Finish(false, "Route timed out");

            return;
        }

        if (!_pilot.IsFlyingToDestination() && Time.unscaledTime - _stageStarted > 1)
        {
            Finish(false, "Autopilot disengaged without a successful arrival");

            return;
        }

        if (_fault != null && !_fault.Result.applied && _leg.flightSeconds >= 5)
        {
            SetStage("fault-observe");
            _fault.Apply();
            Screenshot("fault-applied");

            return;
        }

        if (!_cancelRequested && _request.cancelAfterSeconds >= 0 && _leg.flightSeconds >= _request.cancelAfterSeconds)
        {
            _cancelRequested = true;
            SetStage("reengaging");
            _pilot.Abort();
            Event("Requested cancellation; waiting to re-engage");
        }
    }

    private void OnArrival(float stockError)
    {
        if (_finished || _leg is not { status: "running" })
        {
            return;
        }

        _leg.stockArrivalEvent      = true;
        _leg.flightSeconds          = Time.unscaledTime - _legStarted;
        _leg.endingFuel             = GameFields.Resources.Get(_pilot).GetFractionalFuel();
        _leg.fuelUsed               = _fuelStart - _leg.endingFuel;
        _leg.damagedComponentsAtEnd = GetDamagedComponents(_ship);

        string failure = null;

        if (_leg.samples == 0)
        {
            failure = "Arrival had no navigation samples";
        }
        else if (_leg.arrivalError < -10 || _leg.arrivalError > Math.Max(60, _leg.arrivalRadius * 0.1))
        {
            failure = FormattableString.Invariant($"Arrival distance error {_leg.arrivalError:F2}m is outside [-10, {Math.Max(60, _leg.arrivalRadius * 0.1):F2}]m");
        }
        else if (_leg.relativeSpeed >= 1)
        {
            failure = FormattableString.Invariant($"Arrival relative speed {_leg.relativeSpeed:F2}m/s exceeds the limit");
        }
        else if (_leg.maximumImpactSpeed >= 10 || _leg.damagedComponentsAtEnd.Length > 0)
        {
            failure = "Ship impact or component damage occurred during the route";
        }
        else if (_request.cancelAfterSeconds >= 0 && !_leg.cancelledAndReengaged)
        {
            failure = "Requested cancellation and re-engagement did not occur";
        }
        else if (_fault != null && (!_fault.Result.responseVerified || !_fault.Result.restored || !_fault.Result.recovered))
        {
            failure = "Requested fault handling and recovery were not verified";
        }

        var passed = failure == null;
        _leg.status = passed ? "passed" : "failed";
        _leg.reason = passed ? "Stock arrival event with acceptable speed and standoff" : failure;
        Screenshot("arrival-" + _result.legs.Count);
        Event("Arrival: "     + JsonConvert.SerializeObject(_leg));
        if (!passed)
        {
            Finish(false, _leg.reason);

            return;
        }

        SetStage("between-legs");
    }

    private void OnImpact(ImpactData impact)
    {
        if (_leg is not { status: "running" } || _finished)
        {
            return;
        }

        _leg.impacts++;
        _leg.maximumImpactSpeed = Math.Max(_leg.maximumImpactSpeed, impact.speed);
        Event("Impact: speed=" + impact.speed.ToString("F2", CultureInfo.InvariantCulture) + "; body=" + (impact.otherBody != null ? impact.otherBody.name : "none"));
    }

    private void Finish(bool passed, string reason)
    {
        if (_finished)
        {
            return;
        }

        _finished      = true;
        _result.status = passed ? "passed" : "failed";
        _result.reason = reason;
        if (_leg is { status: "running" })
        {
            _leg.status = "failed";
            _leg.reason = reason;
        }

        Event(_result.status + ": " + reason);
        WriteFinal();
        try
        {
            _fault?.Dispose();
            _pilot?.Abort();
            Screenshot("finished");
            if (LoadManager.GetCurrentScene() == OWScene.SolarSystem)
            {
                var player = Locator.GetPlayerBody();
                if (player)
                {
                    player.GetComponent<ReferenceFrameTracker>()?.UntargetReferenceFrame(false);
                }

                SetStage("returning-title");
                LoadManager.LoadScene(OWScene.TitleScreen, LoadManager.FadeType.ToBlack, 1f);
            }
            else
            {
                SetStage("quitting");
            }
        }
        catch (Exception exception)
        {
            _result.shutdown = "cleanup-error: " + exception.Message;
            SetStage("quitting");
        }
    }

    private void SetStage(string stage)
    {
        _result.stage = stage;
        _stageStarted = Time.unscaledTime;
        Event("stage=" + stage);
        WriteStatus();
    }

    private void Event(string message)
    {
        _events?.WriteLine(DateTime.UtcNow.ToString("O") + " " + message);
        _log("Automation: "                              + message);
    }

    private void Screenshot(string name)
    {
        ScreenCapture.CaptureScreenshot(Path.Combine(_directory, name + ".png"));
    }

    private void WriteStatus()
    {
        _result.updatedUtc = DateTime.UtcNow.ToString("O");
        _samples?.Flush();
        WriteJson("status.json");
    }

    private void WriteFinal()
    {
        WriteStatus();
        WriteJson("result.json");
    }

    private void WriteJson(string name)
    {
        var path = Path.Combine(_directory, name);
        File.WriteAllText(path + ".tmp", JsonConvert.SerializeObject(_result, Formatting.Indented));
        if (File.Exists(path))
        {
            File.Replace(path + ".tmp", path, null);
        }
        else
        {
            File.Move(path + ".tmp", path);
        }
    }

    private static OWRigidbody FindBody(string name)
    {
        foreach (var body in UnityEngine.Object.FindObjectsOfType<AstroObject>())
        {
            if (string.Equals(body.name, name, StringComparison.OrdinalIgnoreCase))
            {
                return body.GetOWRigidbody();
            }
        }

        throw new InvalidOperationException("Celestial body not found: " + name);
    }

    private static string[] GetDamagedComponents(OWRigidbody ship)
    {
        return [.. ship.GetComponentsInChildren<ShipComponent>().Where(component => component.isDamaged).Select(component => component.componentName.ToString()).Distinct()];
    }

    private static double[] Components(Vector vector) => [vector.X, vector.Y, vector.Z];
}
