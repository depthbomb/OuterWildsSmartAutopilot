# Smart Autopilot

Gives your ship's autopilot a little more sense. It can launch from suitable surfaces, plan routes around celestial bodies, and brake to match your destination's velocity. It also checks for nearby physical obstacles and can stop and wait when the way ahead is blocked.

## Video demonstration

<div align="left">
      <a href="https://www.youtube.com/watch?v=ZeWU5-fn1lY">
         <img src="https://i.ytimg.com/vi/ZeWU5-fn1lY/maxres1.jpg" style="width:100%;">
      </a>
</div>

## Install

Install the mod ZIP through [Outer Wilds Mod Manager](https://outerwildsmods.com/mod-manager/):

1. Open the manager and install OWML if prompted.
2. Use the manager's option to install from a ZIP file and select `Depthbomb.SmartAutopilot-0.1.0.zip`.
3. Enable **Smart Autopilot** and launch the game through the manager.

For a manual install on Windows, close the game and extract the mod ZIP into:

```text
%APPDATA%\OuterWildsModManager\OWML\Mods\Depthbomb.SmartAutopilot
```

Make sure `manifest.json` sits directly in that folder, then launch through the mod manager.

This beta targets Outer Wilds **1.1.16.1372** on Windows/Steam with **OWML 2.16.3**.

## Flying

Pick a destination and use the normal autopilot control. When you're landed, the prompt changes to **Launch and navigate**. The ship checks for overhead clearance, starts the engines, and climbs before heading toward your target. You'll need a reasonably level ship, usable thrusters, and enough fuel.

Use the same control to cancel and take over. Arrival leaves you near the destination with your velocity matched. Landing is still your job.

If a route or nearby obstacle blocks progress, the autopilot can hold position relative to a nearby body and warn you. Holding still uses fuel. You can select another target or cancel and move the ship yourself.

This is a beta, so stay at the controls. It doesn't navigate tunnels or interiors, find a path around every structure, or avoid every piece of moving debris. Late engagement, limited thrust, and unloaded geometry can still get you into trouble. Custom solar systems and other mods that change the autopilot or ship physics haven't been verified.

## Settings

The defaults use automatic speed selection and **100m of extra clearance**. You can change these in the mod settings:

- **Optional speed limit:** `0` lets the autopilot choose. A positive value caps normal navigation speed; emergency avoidance and launch may need to exceed it.
- **Extra clearance:** adds space around celestial bodies, adjustable from 40 to 500 metres.
- **Log navigation changes:** records launches, navigation phases, and disengagements. On by default.
- **Log detailed flight samples:** adds more frequent diagnostic information. Off by default.
- **Show navigation debug overlay:** draws the planned route, arrival boundary, avoidance zones, and clearance checks in the world. You can toggle it during flight.
- **Debug overlay refresh rate:** defaults to 60 Hz, adjustable from 10 to 120. The game's frame rate still limits how often it can draw.

If something looks wrong, turn on **Log detailed flight samples** and include the OWML log when reporting it. Mention your destination, where you launched from, other enabled mods, and whether you cancelled the autopilot yourself. A short recording with the debug overlay on can help, too. Logs and recordings may contain spoilers.

## Building

With the .NET 10 SDK specified in `global.json`, Outer Wilds, and OWML installed:

```powershell
.\build.ps1
```

That builds, runs the checks, creates the mod ZIP in `artifacts`, and installs the mod with your settings preserved. Close the game first. Custom install locations can be passed with `-GamePath` and `-OwmlPath`.

Use `-ChecksOnly` for the portable checks without installing or creating a package. These don't replace in-game testing.
