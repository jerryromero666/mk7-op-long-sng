# mk7_switch.sh: branch switcher for the GTI's comma 3X

This script switches the comma 3X between the tuning branches with a numbered menu. It
lives at `/data/mk7_switch.sh`, outside `/data/openpilot`, so switching branches never
deletes it.

## One-time install

SSH into the device (`ssh comma@192.168.0.195`) and paste this. It uses the device's
existing `sngrepo` git remote, so it works even though the repo is private:

```
cd /data/openpilot && git fetch sngrepo mk7_tools && git show sngrepo/mk7_tools:mk7_switch.sh > /data/mk7_switch.sh && chmod +x /data/mk7_switch.sh
```

If the last step says `invalid object name`, use this instead (it reads what was just
fetched):

```
cd /data/openpilot && git fetch sngrepo mk7_tools && git show FETCH_HEAD:mk7_switch.sh > /data/mk7_switch.sh && chmod +x /data/mk7_switch.sh
```

To get a newer version later, press `u` in the menu or run `/data/mk7_switch.sh update`.

## Use it (with the car OFF)

```
/data/mk7_switch.sh                         # menu: pick a number, then answer the questions
/data/mk7_switch.sh 2                       # switch to menu item 2, then it asks "Reboot now?"
/data/mk7_switch.sh t3_highway_aggressive   # same, by name
/data/mk7_switch.sh 7 -y                    # no questions: stash edits, switch, reboot
/data/mk7_switch.sh prev                    # back to the branch you were on before
/data/mk7_switch.sh status                  # last 10 switches, stop distance, saved edits
/data/mk7_switch.sh list                    # numbered list (curated first, then others on sngrepo)
```

Menu keys: a number switches to that branch, `p` goes back to the previous branch, `s`
shows status, `u` updates the script and `q` quits. Start the menu with
`/data/mk7_switch.sh --force` only if the car-is-on check is wrong.

To change the menu, edit the `BRANCHES=(...)` list near the top of the script. Each
line is `"branch|description"`.

## What happens on every switch

1. **Car check.** It refuses if the car is on (onroad). Add `--force` only if you are
   sure the car is off and the check is wrong.
2. **Git check.** It refuses if a half-finished git merge, rebase or cherry-pick is in
   the way, and tells you how to undo it.
3. **Your edits.** If you changed files that git tracks, it offers to put them in a
   "stash" (default yes) and prints the exact command to get them back. If you say no,
   nothing happens.
4. **Download.** It downloads the branch from sngrepo, retrying up to 3 times
   (waiting 2, 4, then 8 seconds) if Wi-Fi drops.
5. **Local commits.** If your local copy of that branch has commits that aren't on
   sngrepo, it warns you and makes you type `yes`. It saves them as a `backup/...`
   branch first. From the command line it refuses instead.
6. **Switch and check.** It switches, then checks that every changed Python file
   compiles. If one doesn't, it goes straight back to where you were, restores your
   edits, doesn't reboot, and says so loudly.
7. **Record and reboot.** It adds a line to `/data/mk7_branch_log.txt`, writes the
   branch name to `/data/mk7_current_branch`, shows what changed, and asks
   `Reboot now to apply? [Y/n]`.

The log shows which branch you drove when, so you can match drives to branches:

```
2026-10-14T18:02:11-0700  hms_latch_fix@d861e6b -> t2_lead2_phantom_fix@e558ce7
```

## Undo

- **Go back to the last branch:** run `/data/mk7_switch.sh prev` (or press `p`), then
  reboot.
- **Get stashed edits back:** the switch printed the command, for example:
  `cd /data/openpilot && git checkout hms_latch_fix && git stash pop`.
  If you've stashed more than once since, `/data/mk7_switch.sh status` lists every saved
  set with its exact `git stash pop stash@{N}` command.
- **Local commits that were replaced** are in a branch named
  `backup/<branch>-<date>`. To see them, run `cd /data/openpilot && git branch`.

## Good to know

- **t3_stop_distance_taper:** the stop gap is set in `/data/stop_distance` as two numbers
  in metres: the gap at 30 mph and below, then the gap at 60 mph and above, for example
  `echo "6.0 4.5" > /data/stop_distance`. Each is clamped to 4.5 to 9.0. If the file is
  missing, the defaults are 6.0 and 4.5. Changes apply within a second while driving, so
  you don't need to reboot.
- **Mount calibration is the same on every branch.** This build hard-locks camera pitch
  (0.0236 rad) and yaw (-0.0070 rad) in `selfdrive/locationd/calibrationd.py`, and that
  file is identical on every branch in the menu. Switching never touches `/data/params`,
  so the rest of the learned calibration (camera height, roll) and the learned car
  parameters carry over too, and A/B drives see the camera the same way. Don't press
  "Reset Calibration" between A/B drives; if you do, the first part of the next drive is
  spent re-calibrating. Because pitch is locked, nothing re-learns it if the mount moves
  or the car is loaded differently, so keep load similar between A/B drives.
- **Only Python changes take effect.** This checkout has a `prebuilt` marker, so
  openpilot doesn't recompile on boot. If a branch changes C/C++ source, the script
  warns you.
- **Exit codes:** 0 = OK, 1 = refused or failed (nothing changed), 2 = bad usage,
  3 = broken Python, already rolled back, 4 = the rollback failed (follow the printed
  command).
- **Testing off-device:** every path can be overridden: `OP_DIR`, `REMOTE`, `LOG`,
  `PARAMS_DIR`, `REBOOT_CMD`, `CURRENT_FILE`, `STOP_FILE` and `PYTHON`.
