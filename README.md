<div align="center">

# scripts-for-moi

### Commands that add to MoI3D, installed by copying a few files.

<img src="https://img.shields.io/badge/MoI-4-b1b9f9?style=flat-square&labelColor=0d1117" alt="MoI 4" />
<img src="https://img.shields.io/badge/platform-Windows-b1b9f9?style=flat-square&labelColor=0d1117" alt="Windows" />
<img src="https://img.shields.io/badge/runtime-none-b1b9f9?style=flat-square&labelColor=0d1117" alt="No runtime needed" />
<img src="https://img.shields.io/badge/license-GPL%20v3-b1b9f9?style=flat-square&labelColor=0d1117" alt="GPL v3" />

</div>

This is a collection of native MoI commands. Each one is a handful of plain files that go into MoI's
commands folder: no server, no installer, no package manager, nothing running in the background.
You copy the files, restart MoI, and type the command's name on MoI's command line.

Everything here uses only what already ships with MoI — its scripting API and its own stock
geometry commands.

> [!NOTE]
> **Unofficial.** This project is not affiliated with, endorsed by, or supported by Triple Squid
> Software. MoI and MoI3D are trademarks of their owner and are used here only to say which
> program these scripts work with. You need your own licensed copy of MoI.

> [!WARNING]
> **Use with caution.** These are spare-time tools that change your model. Every script says what
> has been tested and what has not (see [Testing status](#testing-status)) — read that before you
> run one on work that matters. Save first, keep backups, and check the result. You use them at your
> own risk (see [Disclaimer](#disclaimer)).

---

## Contents

- [Download](#download)
- [The scripts](#the-scripts) — [MultiPipe](#multipipe)
- [Testing status](#testing-status)
- [Installing a script](#installing-a-script)
- [Requirements](#requirements)
- [What's in this repository](#whats-in-this-repository)
- [Developing and testing](#developing-and-testing)
- [Built with](#built-with)
- [Disclaimer](#disclaimer) · [License](#license)

---

## Download

| Script | Download | Manual |
|---|---|---|
| **MultiPipe** | **[MultiPipe.zip](https://github.com/preluceo-tools/scripts-for-moi/raw/main/dist/MultiPipe.zip)** (29 KB) | [Manual](https://htmlpreview.github.io/?https://github.com/preluceo-tools/scripts-for-moi/blob/main/scripts/multipipe/MultiPipe-manual.html) |

The zip holds the command files, the manual and the licence. Unzip it and follow
[Installing a script](#installing-a-script) — it takes about a minute.

You can also just clone the repository; the same files live under `scripts/`.

---

## The scripts

### MultiPipe

**Turns a network of curves into one solid pipe frame, in a single step.**

Select any set of curves and run `MultiPipe`. It sweeps a pipe along every curve, puts a joint
wherever strut ends meet, and unions the lot into one closed solid. Your input curves are kept, and
one <kbd>Ctrl</kbd>+<kbd>Z</kbd> takes the whole result back.

- **Any node valency.** Three, four, five, six or more struts meeting at a point all get a joint.
  Corners inside a polyline count as nodes too.
- **Ball or filleted joints.** Ball joints are the fast, reliable default; filleted joints round the
  creases where struts meet.
- **Straight nodes stay straight.** Two struts meeting with a bend under 5° get no joint — the pipe
  runs through as one continuous strut.
- **Closed curves** become closed ring pipes. **Duplicate curves** are dropped and counted.
  **Crossings** — curves passing through each other mid-span — are warned about and left unjoined.
- **Capped or open ends.** Capped gives a closed solid ready for printing; open leaves tube ends.
- **Nothing is committed until you press Done.** Cancel at any point and the document is untouched.

| Option | Default | What it does |
|---|---|---|
| Strut radius | 0.5 | Pipe radius, as a distance in your document units. |
| Node size | 1.2 | Ball joint radius as a multiple of the strut radius. Values below 1.02 are raised to 1.02. |
| Joint style | Ball | Turn *Filleted* on to round the creases around each joint. |
| Fillet factor | 0.2 | Fillet radius as a fraction of the strut radius. Shown only when *Filleted* is on. |
| Cap | On | Closes free ends with a flat cap, so the result is a closed solid. |

**Known limits:** crossings are not split for you; if the fillet fails anywhere the whole frame
falls back to ball joints; filleted joints can take minutes on a dense frame; the pipes are solid,
not hollow; there is no live preview; one radius applies to the whole frame.

The [manual](scripts/multipipe/MultiPipe-manual.html) documents every option, every message and every
warning in full, and [`sample-videos-scripts-in-action/`](sample-videos-scripts-in-action/) holds a
screen recording of a run.

---

## Testing status

One section per script, saying what is covered and what is not. Read the one for the script you are
about to use.

### MultiPipe

A first release. Its geometry is well covered; its command panel is not, because MoI's command
panel cannot be driven by a script — only by a person at the keyboard.

**Tested automatically, on every change:** the planning — which curve becomes which strut, where
nodes are, straight nodes, closed curves, duplicates, crossings (22 tests) — and the geometry, by
building 26 test frames inside a live MoI and checking each result: ball joints, the union and its
step-by-step retry, filleted joints and their fallback, capped and open ends, and a 300-strut frame.

**Not covered by any automated test:**

| Area | What is unverified |
|---|---|
| The command panel and run flow | Option fields, prompts, the summary area, *Done* and *Cancel* at each step, options remembered between runs, the *Fillet factor* row appearing with *Filleted*, the building message, and one <kbd>Ctrl</kbd>+<kbd>Z</kbd> removing the whole result. |
| Installing, and the command being found | Copying the files in, MoI finding the planner and builder beside the command, running it by name, and binding it to a shortcut key or toolbar button. |
| Curves picked in a live document | Turning a real selection into struts: ignoring non-curve objects, and splitting a joined multi-segment curve. The tests build their curves rather than picking them. |
| The panel's error messages | A zero radius or a zero fillet factor. The checks are tested; the panel showing the message is not. |
| The separate-pieces fallback | What you see when a union cannot be completed at all — reachable only by forcing the failure in code. |
| macOS | Written to be portable, never run there. Everything above was checked on Windows. |

Nothing reaches your document until the final *Done*, and one <kbd>Ctrl</kbd>+<kbd>Z</kbd> takes
back what was added. If you hit one of these, please open an issue — that is what turns this list
into a shorter one.

---

## Installing a script

Installing a MoI command is copying files. Nothing is downloaded and nothing runs in the background.

**1. Copy the script's files into your MoI commands folder.** For MultiPipe that is
`MultiPipe.js`, `MultiPipe.htm`, `MultiPipePlanner.js` and `MultiPipeBuilder.js`.

The commands folder is:

```text
Windows:  %APPDATA%\Moi\commands\
          e.g. ...\AppData\Roaming\Moi\commands\
macOS:    <MoI application support folder>/commands/
```

The commands folder inside the MoI install (`<MoI install folder>\commands\`) works too, but a
command kept in `%APPDATA%` survives a MoI upgrade and doesn't need reinstalling.

**2. Restart MoI.**

**3. Type the command's name** on MoI's command line (e.g. `MultiPipe`) and press <kbd>Enter</kbd>.

**Optional — a shortcut key.** In MoI, open *Options > Shortcut keys*, add an entry, put the key you
want in the left column and the command's name in the right column. The same name works on a custom
toolbar button.

> [!NOTE]
> MoI caches a command script the first time it runs. If you replace the files with a newer copy,
> restart MoI.

To uninstall, delete the files again.

---

## Requirements

- **MoI 4.** Written and tested against MoI 4; nothing here uses a version-specific API, but no
  other version has been checked.
- **A licensed copy of MoI.** Not included, and nothing here grants any right to it.
- **Nothing else.** No Node.js, no Python, no libraries — unless you want to run the tests, which
  need [Node.js](https://nodejs.org/) 18 or newer.

---

## What's in this repository

```text
scripts/<name>/      the files you copy into MoI's commands folder, and its manual
tests/<name>/        its tests
dist/<name>.zip      the packaged download
sample-videos-...   screen recordings of the scripts in action, one per script
```

Each script is self-contained: its folder under `scripts/` is everything a user needs.

---

## Developing and testing

MultiPipe's planning logic — node clustering, bend angles, straight nodes, crossings, duplicates —
is pure JavaScript with no MoI in it, and is covered by Node's own test runner:

```bash
node --test
```

The geometry itself can only be checked inside MoI. `tests/multipipe/builder/builder-suite.js`
builds every test scene through [mcp-bridge-for-moi](https://github.com/preluceo-tools/mcp-bridge-for-moi)
and checks what came out, adding nothing to the document. The bridge is wired up through an
`.mcp.json` in the repository root, which is deliberately **not** tracked: it names a path on your
own machine. Write your own:

```json
{ "mcpServers": { "moi": { "command": "node", "args": ["<mcp-bridge-for-moi folder>/dist/cli.js"] } } }
```

The command panel can't be driven from a script at all — see [Testing status](#testing-status).

Repackaging a script's zip after a change (PowerShell):

```powershell
Compress-Archive -Path scripts/multipipe/* -DestinationPath dist/MultiPipe.zip -Force
```

> [!NOTE]
> Scripts inside MoI run on **old JavaScript (ES5)**. No `let`, `const`, arrow functions or
> `Promise` — MoI's script engine rejects them.

---

## Built with

**Nothing has to be downloaded. These scripts use only MoI's own scripting API and MoI's stock
command factories — everything they need already ships with MoI.**

| Component | What the scripts use it for | Where it comes from | License |
|---|---|---|---|
| [MoI3D](https://moi3d.com/) script API — `moi.geometryDatabase`, `moi.ui`, `moi.command`, `moi.vectorMath` | Reading the selection and the document tolerance, the object picker, the command panel and its Done/Cancel loop, adding the result, and point and frame math. | Ships with MoI | **Commercial** (MoI is paid software; its licence covers the API) |
| [MoI3D](https://moi3d.com/) stock command factories — e.g. `sweep`, `sphere`, `booleanunion`, `fillet` | All the geometry: sweeping the struts, the ball joints, the union, the joint fillets and removing cap faces. | Ships with MoI | **Commercial** (part of MoI) |
| [MoI3D](https://moi3d.com/) command UI markup — `moi:DistanceInput`, `moi:NumericInput`, `moi:CheckButton`, `moi:CommandDoneCancel` | The command panel: the option fields, the Done and Cancel buttons, and MoI's own styling. | Ships with MoI | **Commercial** (part of MoI) |
| [JavaScript (ECMAScript 5)](https://ecma-international.org/publications-and-standards/standards/ecma-262/) built-ins — `Math`, `Date`, `Array`, `Number`, `String` | Geometry math, build timing and formatting the summary text. | MoI's built-in script engine | Open standard, free to implement and use |
| [Node.js](https://nodejs.org/) — `node:test`, `node:assert` | Runs the planner tests. Used for development only; no part of it is shipped or needed to use a script. | nodejs.org | MIT |

MoI is the only commercial component and the only thing you have to get yourself. There are no
third-party libraries, no package manager, no runtime downloads and no external process, and no
LGPL component is used or bundled.

---

## Disclaimer

These scripts are provided **"as is", without warranty of any kind**, express or implied, including
but not limited to the warranties of merchantability, fitness for a particular purpose and
non-infringement. You use them entirely at your own risk.

The author and contributors are not liable for any claim, damage or other loss arising from their
use or from being unable to use them: lost or corrupted models, damaged or deleted files, harm to
your system, or lost work or time. Checking the result, saving your work and keeping backups is
your responsibility alone.

The licences below say the same in their own terms (GPL v3, sections 15 and 16). Where this summary
and a licence differ, the licence applies.

## License

**The code is licensed under the [GNU GPL v3](https://www.gnu.org/licenses/gpl-3.0.html)** (see
[`LICENSE`](LICENSE)). You may use it, study it, change it and pass it on, including changed
versions, as long as those versions travel under the same terms with their source. A copy of the
full text ships beside each script and inside each download.

**The documentation — this page, the manuals and the other prose — is licensed
[CC BY 4.0](https://creativecommons.org/licenses/by/4.0/)**: copy it, translate it, adapt it, for
any purpose, as long as you credit the source.

MoI itself is separate commercial software, not distributed here; you need your own copy, and
nothing in this repository grants any right to it.

*This summary is not a license. The linked texts are the actual terms.*
