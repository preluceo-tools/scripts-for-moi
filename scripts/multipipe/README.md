<div align="center">

# MultiPipe

### Turn a network of curves into one solid pipe frame, inside MoI, in a single step.

<img src="https://img.shields.io/badge/version-1.0.0%20(final)-b1b9f9?style=flat-square&labelColor=0d1117" alt="Version 1.0.0, final" />
<img src="https://img.shields.io/badge/MoI-4-b1b9f9?style=flat-square&labelColor=0d1117" alt="MoI 4" />
<img src="https://img.shields.io/badge/platform-Windows-b1b9f9?style=flat-square&labelColor=0d1117" alt="Windows" />
<img src="https://img.shields.io/badge/license-GPL%20v3-b1b9f9?style=flat-square&labelColor=0d1117" alt="GPL v3" />

</div>

> [!IMPORTANT]
> **MultiPipe is no longer being developed.** Version 1.0.0 is frozen as its final version: it stays
> available and keeps working, but it gets no new features and no fixes. For new work use
> **[MultiPipe2](../multipipe2/README.md)**, which builds smooth organic joints and installs beside
> MultiPipe under its own name.

> [!NOTE]
> Unofficial. Not affiliated with, endorsed by, or supported by Triple Squid Software (MoI). You need
> your own licensed copy of MoI to run it.

Select any set of curves and run `MultiPipe`. It sweeps a pipe along every curve, puts a joint
wherever strut ends meet, and unions the lot into one closed solid. Your input curves are kept, and
one <kbd>Ctrl</kbd>+<kbd>Z</kbd> takes the whole result back.

---

## Contents

- [What it does](#what-it-does)
- [Options](#options)
- [Known limits](#known-limits)
- [Install](#install)
- [Version history](#version-history)
- [Full manual, license and built-with](#full-manual-license-and-built-with)

---

## What it does

- **Any node valency.** Three, four, five, six or more struts meeting at a point all get a joint.
  Corners inside a polyline count as nodes too.
- **Ball or filleted joints.** Ball joints are the fast, reliable default; filleted joints round the
  creases where struts meet.
- **Straight nodes stay straight.** Two struts meeting with a bend under 5° get no joint; the pipe
  runs through as one continuous strut.
- **Closed curves** become closed ring pipes. **Duplicate curves** are dropped and counted.
  **Crossings** (curves passing through each other mid-span) are warned about and left unjoined.
- **Capped or open ends.** Capped gives a closed solid ready for printing; open leaves tube ends.
- **Nothing is committed until you press Done.** Cancel at any point and the document is untouched.

---

## Options

| Option | Default | What it does |
|---|---|---|
| Strut radius | 0.5 | Pipe radius, as a distance in your document units. |
| Node size | 1.2 | Ball joint radius as a multiple of the strut radius. Values below 1.02 are raised to 1.02. |
| Joint style | Ball | Turn *Filleted* on to round the creases around each joint. |
| Fillet factor | 0.2 | Fillet radius as a fraction of the strut radius. Shown only when *Filleted* is on. |
| Cap | On | Closes free ends with a flat cap, so the result is a closed solid. |

---

## Known limits

- Crossings are not split for you; split them yourself where you want a node.
- If the fillet fails anywhere, the whole frame falls back to ball joints.
- Filleted joints can take minutes on a dense frame.
- The pipes are solid, not hollow.
- No live preview; you see the result after the build, before committing it.
- One radius applies to the whole frame.

MultiPipe2 addresses most of these: smooth joints that do not depend on a union or a fillet, a
preview, and a radius per curve style.

---

## Install

Copy `MultiPipe.js`, `MultiPipe.htm`, `MultiPipePlanner.js` and `MultiPipeBuilder.js` into your MoI
commands folder (`%APPDATA%\Moi\commands\` on Windows), restart MoI, and type `MultiPipe` on the
command line. Full steps and a shortcut-key tip are in the manual.

---

## Version history

The zip is `dist/MultiPipe-1.0.0.zip` in this repository, and on the
[Releases page](https://github.com/preluceo-tools/scripts-for-moi/releases).

| Version | What changed |
|---|---|
| **1.0.0** | First and final release: pipe frame from lines, polylines and curves, ball or filleted joints at any node valency, straight nodes kept straight, closed rings, capped or open ends. Development frozen at this version. |

---

## Full manual, license and built-with

The [manual](MultiPipe-manual.html) covers installing, every option, every message and warning,
known limits, what's tested and what isn't, the **Built with** table of everything the script
depends on, and the license (GPL v3 for the code, CC BY 4.0 for the docs). This page is the short
version; the manual is the reference.
