#!/usr/bin/env bash
# If someone runs "sh mk7_switch.sh", hop over to bash before any bash-only syntax.
if [ -z "${BASH_VERSION:-}" ]; then exec bash "$0" "$@"; fi
# =============================================================================
#  mk7_switch.sh - easy openpilot branch switcher for the MK7 GTI comma 3X
# =============================================================================
#  Lives at /data/mk7_switch.sh (OUTSIDE /data/openpilot, so it survives branch
#  switches). Run it with no arguments for a numbered menu:
#
#      /data/mk7_switch.sh
#
#  Or straight from the command line:
#      /data/mk7_switch.sh 2                      switch to menu item 2
#      /data/mk7_switch.sh t2_lead2_phantom_fix   switch by name
#      /data/mk7_switch.sh 7 -y                   switch, then reboot without asking
#      /data/mk7_switch.sh prev                   back to the previous branch (from the log)
#      /data/mk7_switch.sh status                 recent switches, stop distance, saved edits
#      /data/mk7_switch.sh list                   the numbered branch list
#      /data/mk7_switch.sh update                 get the newest copy of this script
#
#  Options:  -y / --yes   don't ask: stash local edits and reboot automatically
#            --force      skip the "car is on" check (only if you're SURE it's off)
#
#  Exit codes: 0 ok or nothing to do | 1 refused / failed, nothing changed
#              2 bad usage | 3 new branch had broken Python, rolled back
#              4 the automatic roll-back itself failed (read the message!)
# =============================================================================

# -----------------------------------------------------------------------------
# EDIT ME: the tuning branches, in menu order.  Format:  "branch|description"
# -----------------------------------------------------------------------------
BRANCHES=(
  "claude/determined-hypatia-pnidyz|Task 1 baseline: grade taper removed (current recommended base)"
  "t2_lead2_phantom_fix|Ghost-car fix: stops a phantom parked car holding back launches (try first)"
  "t2_launch_floor_2p0|Firmer push when the brake hold releases"
  "t2_accel_cap_launch|Quicker 0-22 mph pickup"
  "t2_achange_cost_launch|Slightly quicker accel build-up below 7 mph"
  "t2_cruise_max_accel_2p0|Firmer takeoff when no car is ahead"
  "fix_brake_ramp_4p0|Restore normal brake build-up rate (safety; try before 8/9)"
  "t3_highway_aggressive|Aggressive following gap at highway speed, normal in town"
  "t3_stop_distance_taper|1.5 m closer at 60+ mph; stop gap tunable in /data/stop_distance"
  "hms_latch_fix|Previous daily driver"
)
# Branches on the remote that are never offered or switched to (not openpilot code).
HIDDEN_BRANCHES=("mk7_tools")
# Where "u" / "update" gets the newest copy of this script: <REMOTE>/<branch>:<file>
TOOLS_BRANCH="mk7_tools"
TOOLS_FILE="mk7_switch.sh"
# -----------------------------------------------------------------------------

set -u
set -o pipefail

# ---- settings (every path can be overridden with an environment variable) ----
OP_DIR="${OP_DIR:-/data/openpilot}"
REMOTE="${REMOTE:-sngrepo}"
LOG="${LOG:-/data/mk7_branch_log.txt}"
PARAMS_DIR="${PARAMS_DIR:-/data/params/d}"
REBOOT_CMD="${REBOOT_CMD:-sudo reboot}"
CURRENT_FILE="${CURRENT_FILE:-/data/mk7_current_branch}"
STOP_FILE="${STOP_FILE:-/data/stop_distance}"
PYTHON="${PYTHON:-}"                         # empty = auto (openpilot's venv python, else python3)
FETCH_RETRY_DELAYS="${FETCH_RETRY_DELAYS:-2 4 8}"   # seconds to wait before each retry
REFRESH_TIMEOUT="${REFRESH_TIMEOUT:-20}"     # seconds for the menu's "any new branches?" check

STOP_FILE_IN_CODE="/data/stop_distance"      # what the t3_stop_distance_taper code reads
STOP_DEFAULTS="6.0 4.5"
STOP_BRANCH="t3_stop_distance_taper"

SELF="$(readlink -f -- "${BASH_SOURCE[0]}" 2>/dev/null || printf '%s' "${BASH_SOURCE[0]}")"

YES=0
FORCE=0
REBOOTED=0
CHOICE=""
CUR_NAMES=()
CUR_DESCS=()
OTHER_NAMES=()

# Compiles every file given (relative to argv[1]) without writing anything into the repo.
PY_CHECK='
import os, sys, shutil, tempfile, py_compile
root, files = sys.argv[1], sys.argv[2:]
tmp = tempfile.mkdtemp(prefix="mk7_pyc_")
bad = 0
try:
    for i, rel in enumerate(files):
        try:
            py_compile.compile(os.path.join(root, rel), cfile=os.path.join(tmp, "%d.pyc" % i), doraise=True)
            if len(files) <= 15:
                print("      ok      " + rel)
        except py_compile.PyCompileError as e:
            bad += 1
            print("      BROKEN  " + rel)
            for line in str(e.msg).strip().splitlines():
                print("              " + line)
        except Exception as e:
            bad += 1
            print("      BROKEN  %s (%s)" % (rel, e))
finally:
    shutil.rmtree(tmp, ignore_errors=True)
print("      %d file(s) checked, %d broken" % (len(files), bad))
sys.exit(1 if bad else 0)
'

# =============================================================================
#  small helpers
# =============================================================================
say()   { printf '%s\n' "$*"; }
blank() { printf '\n'; }
hr()    { say "  ----------------------------------------------------------"; }
indent() { [ -n "$1" ] && printf '%s\n' "$1" | sed 's/^/      /'; return 0; }
trim()  { local s=$1; s="${s#"${s%%[![:space:]]*}"}"; s="${s%"${s##*[![:space:]]}"}"; printf '%s' "$s"; }
abs_path() { case $1 in /*) printf '%s' "$1" ;; *) printf '%s/%s' "$PWD" "$1" ;; esac; }

g() { git -C "$OP_DIR" --no-pager "$@"; }

with_timeout() {   # with_timeout SECONDS command...   (runs without a limit if 'timeout' is missing)
  local s=$1
  shift
  if command -v timeout >/dev/null 2>&1; then timeout "$s" "$@"; else "$@"; fi
}

in_list() {        # in_list needle item...
  local x=$1 y
  shift
  for y in "$@"; do [ "$x" = "$y" ] && return 0; done
  return 1
}

is_curated() { in_list "$1" ${CUR_NAMES[@]+"${CUR_NAMES[@]}"}; }
is_hidden()  { in_list "$1" ${HIDDEN_BRANCHES[@]+"${HIDDEN_BRANCHES[@]}"}; }

desc_for() {
  local i
  for ((i = 0; i < ${#CUR_NAMES[@]}; i++)); do
    if [ "${CUR_NAMES[i]}" = "$1" ]; then printf '%s' "${CUR_DESCS[i]}"; return 0; fi
  done
  return 0
}

ask_yn() {         # ask_yn "question" Y|N  -> 0 = yes, 1 = no.  End of input counts as no.
  local q=$1 def=$2 hint ans
  if [ "$def" = Y ]; then hint="[Y/n]"; else hint="[y/N]"; fi
  printf '%s %s ' "$q" "$hint"
  if ! IFS= read -r ans; then blank; return 1; fi
  [ -t 0 ] || printf '%s\n' "$ans"           # echo the answer when input is piped (logs)
  ans=$(trim "$ans")
  ans=${ans,,}
  case $ans in
    '')      [ "$def" = Y ] ;;
    y | yes) return 0 ;;
    *)       return 1 ;;
  esac
}

ask_typed_yes() {  # the user has to type the word yes
  local ans
  printf '  Type yes to continue (anything else cancels): '
  if ! IFS= read -r ans; then blank; return 1; fi
  [ -t 0 ] || printf '%s\n' "$ans"
  [ "$(trim "$ans")" = yes ]
}

pause() {
  local _x
  printf '\n  Press Enter for the menu (q to quit)... '
  IFS= read -r _x || { blank; return 1; }
  [ -t 0 ] || printf '%s\n' "$_x"
  case $(trim "$_x") in q | Q) return 1 ;; esac
  return 0
}

pick_python() {
  local p
  if [ -n "$PYTHON" ]; then printf '%s' "$PYTHON"; return 0; fi
  for p in /usr/local/venv/bin/python3 /usr/local/venv/bin/python; do
    if [ -x "$p" ]; then printf '%s' "$p"; return 0; fi
  done
  command -v python3 2>/dev/null || command -v python 2>/dev/null
}

current_branch() { g symbolic-ref --short -q HEAD 2>/dev/null; }   # empty when detached
head_sha()       { g rev-parse -q --verify 'HEAD^{commit}' 2>/dev/null; }
short()          { g rev-parse --short "$1" 2>/dev/null || printf '%.7s' "$1"; }
iso_now()        { date '+%Y-%m-%dT%H:%M:%S%z'; }

valid_branch_name() {
  case $1 in '' | -*) return 1 ;; esac
  git check-ref-format "refs/heads/$1" >/dev/null 2>&1
}

load_curated() {
  local e
  CUR_NAMES=()
  CUR_DESCS=()
  for e in ${BRANCHES[@]+"${BRANCHES[@]}"}; do
    case $e in
      *'|'*) CUR_NAMES+=("$(trim "${e%%|*}")"); CUR_DESCS+=("$(trim "${e#*|}")") ;;
      *)     CUR_NAMES+=("$(trim "$e")"); CUR_DESCS+=("") ;;
    esac
  done
}

load_other_branches() {   # remote-tracking branches for REMOTE, minus curated/hidden/HEAD/non-openpilot
  local ref name
  OTHER_NAMES=()
  while IFS= read -r ref; do
    name=${ref#"refs/remotes/$REMOTE/"}
    if [ -z "$name" ] || [ "$name" = HEAD ] || is_curated "$name" || is_hidden "$name"; then
      continue
    fi
    # only offer branches that actually contain openpilot
    g cat-file -e "$ref:launch_openpilot.sh" 2>/dev/null || continue
    OTHER_NAMES+=("$name")
  done < <(g for-each-ref --format='%(refname)' "refs/remotes/$REMOTE" 2>/dev/null)
}

refresh_remote() {        # best effort: learn about new/deleted branches on the remote
  printf '  Checking %s for new branches... ' "$REMOTE"
  if with_timeout "$REFRESH_TIMEOUT" env GIT_TERMINAL_PROMPT=0 \
      git -C "$OP_DIR" --no-pager fetch --prune --no-tags --quiet "$REMOTE" \
      "+refs/heads/*:refs/remotes/$REMOTE/*" </dev/null >/dev/null 2>&1; then
    say "ok"
  else
    say "couldn't reach it right now (list may be out of date)"
  fi
}

resolve_choice() {        # number or name -> CHOICE (returns 1 with a message if invalid)
  local arg=$1 n nc no
  CHOICE=""
  nc=${#CUR_NAMES[@]}
  no=${#OTHER_NAMES[@]}
  if [[ $arg =~ ^[0-9]+$ ]]; then
    n=$((10#$arg))
    if [ "$n" -ge 1 ] && [ "$n" -le "$nc" ]; then
      CHOICE=${CUR_NAMES[n - 1]}
    elif [ "$n" -gt "$nc" ] && [ "$n" -le $((nc + no)) ]; then
      CHOICE=${OTHER_NAMES[n - nc - 1]}
    else
      say "  There is no number $arg in the list (pick 1-$((nc + no)))."
      return 1
    fi
  else
    arg=${arg#refs/heads/}
    arg=${arg#"refs/remotes/$REMOTE/"}
    arg=${arg#"$REMOTE/"}
    CHOICE=$arg
  fi
  if ! valid_branch_name "$CHOICE"; then
    say "  '$CHOICE' isn't a valid branch name."
    CHOICE=""
    return 1
  fi
  if is_hidden "$CHOICE"; then
    say "  '$CHOICE' only holds this script's tools, not openpilot - you can't drive on it."
    CHOICE=""
    return 1
  fi
  return 0
}

# =============================================================================
#  safety checks
# =============================================================================
py_onroad() {             # ask openpilot's own Params; prints on|off, returns 1 if it can't
  local py out
  py=$(pick_python) || return 1
  [ -n "$py" ] || return 1
  out=$(cd / && PYTHONPATH="$(dirname -- "$OP_DIR")" PYTHONDONTWRITEBYTECODE=1 with_timeout 15 "$py" -c \
    'from openpilot.common.params import Params; print("1" if Params().get_bool("IsOnroad") else "0")' \
    2>/dev/null </dev/null) || return 1
  case $(trim "$out") in
    1) say on ;;
    0) say off ;;
    *) return 1 ;;
  esac
}

onroad_state() {          # prints on | off | unknown
  local f="$PARAMS_DIR/IsOnroad" v
  if [ -f "$f" ] && [ -r "$f" ]; then
    v=$(head -c 1 -- "$f" 2>/dev/null)
    case $v in
      1) say on; return 0 ;;
      0) say off; return 0 ;;
    esac           # anything else: don't guess, ask openpilot's Params below
  fi
  if py_onroad; then return 0; fi
  # Params directory is there but the key was never written -> openpilot never went onroad.
  if [ -d "$PARAMS_DIR" ] && [ -r "$PARAMS_DIR" ] && [ -x "$PARAMS_DIR" ] && [ ! -e "$f" ]; then
    say off
    return 0
  fi
  say unknown
}

git_busy_reason() {       # prints why git is busy and returns 0, or returns 1 if all clear
  local name p
  for name in MERGE_HEAD rebase-merge rebase-apply CHERRY_PICK_HEAD REVERT_HEAD index.lock; do
    p=$(g rev-parse --git-path "$name" 2>/dev/null) || continue
    case $p in /*) ;; *) p="$OP_DIR/$p" ;; esac
    [ -e "$p" ] || continue
    case $name in
      MERGE_HEAD)       say "a git merge is half-finished. To undo it: cd $OP_DIR && git merge --abort" ;;
      rebase-*)         say "a git rebase is half-finished. To undo it: cd $OP_DIR && git rebase --abort" ;;
      CHERRY_PICK_HEAD) say "a git cherry-pick is half-finished. To undo it: cd $OP_DIR && git cherry-pick --abort" ;;
      REVERT_HEAD)      say "a git revert is half-finished. To undo it: cd $OP_DIR && git revert --abort" ;;
      index.lock)       say "another git command is running (or one crashed). If nothing else is running: rm $p" ;;
    esac
    return 0
  done
  if [ -n "$(g ls-files -u 2>/dev/null | head -n 1)" ]; then
    say "there are unresolved merge conflicts ('cd $OP_DIR && git status' lists them)"
    return 0
  fi
  return 1
}

dirty_files() { g status --porcelain --untracked-files=no 2>/dev/null; }

fetch_branch() {          # fetch_branch NAME -> 0 ok, 1 failed (network etc.), 2 no such branch
  local b=$1 out attempt=1 tries d
  local -a delays=()
  read -r -a delays <<<"$FETCH_RETRY_DELAYS"
  tries=$((${#delays[@]} + 1))
  say "  Downloading '$b' from $REMOTE..."
  while :; do
    if out=$(LC_ALL=C git -C "$OP_DIR" --no-pager fetch --no-tags "$REMOTE" \
        "+refs/heads/$b:refs/remotes/$REMOTE/$b" 2>&1); then
      return 0
    fi
    case $out in
      *"ouldn't find remote ref"* | *"ould not find remote ref"*)
        say "  There is no branch called '$b' on $REMOTE."
        return 2 ;;
    esac
    if [ "$attempt" -ge "$tries" ]; then
      say "  Download failed after $attempt tries. Git said:"
      indent "$out"
      say "  (No internet on the device, or a GitHub login problem? Check Wi-Fi and try again.)"
      return 1
    fi
    d=${delays[attempt - 1]}
    say "  Download failed (try $attempt of $tries); trying again in ${d}s..."
    sleep "$d"
    attempt=$((attempt + 1))
  done
}

check_python() {          # check_python OLD NEW -> 0 all changed .py files compile, else 1
  local old=$1 new=$2 f py rc
  local -a files=()
  while IFS= read -r -d '' f; do
    [ -f "$OP_DIR/$f" ] && files+=("$f")
  done < <(g diff -z --name-only --diff-filter=d "$old" "$new" -- '*.py' 2>/dev/null)
  if [ "${#files[@]}" -eq 0 ]; then
    say "  No Python files changed - nothing to check."
    return 0
  fi
  py=$(pick_python)
  if [ -z "$py" ]; then
    say "  Can't find python3 to check the code."
    return 1
  fi
  say "  Checking that the ${#files[@]} changed Python file(s) compile..."
  "$py" -I -c "$PY_CHECK" "$OP_DIR" "${files[@]}" </dev/null
  rc=$?
  if [ "$rc" -ne 0 ] && [ "$rc" -ne 1 ]; then
    say "  (The Python check itself couldn't run with '$py' - exit code $rc.)"
  fi
  return $((rc != 0))
}

# =============================================================================
#  reporting
# =============================================================================
stop_distance_text() {
  local v re='^[+-]?([0-9]+\.?[0-9]*|\.[0-9]+)[[:space:]]+[+-]?([0-9]+\.?[0-9]*|\.[0-9]+)$'
  if [ ! -e "$STOP_FILE" ]; then
    printf 'not set (defaults %s)' "$STOP_DEFAULTS"
    return 0
  fi
  v=$(trim "$(head -c 200 -- "$STOP_FILE" 2>/dev/null | tr '\n\t' '  ')")
  if [ -z "$v" ]; then
    printf 'empty, so the defaults %s are used' "$STOP_DEFAULTS"
  elif [[ $v =~ $re ]]; then
    printf '"%s"' "$v"
  else
    printf '"%s" - not two numbers, so the defaults %s are used' "$v" "$STOP_DEFAULTS"
  fi
}

show_stop_reminder() {
  blank
  say "  REMINDER - this branch has the tunable stop distance:"
  say "    $STOP_FILE: $(stop_distance_text)"
  say "    Format: \"<gap at 30 mph and below> <gap at 60+ mph>\" in metres, each 4.5 to 9.0."
  say "    Example:  echo \"6.0 4.5\" > $STOP_FILE"
  say "    It is re-read every second while driving - no reboot needed for changes."
}

show_changes() {          # show_changes OLD NEW OLD_LABEL
  local old=$1 new=$2 label=$3 n m stat
  blank
  say "  What changed ($(short "$old") -> $(short "$new")):"
  n=$(g rev-list --count "$old..$new" 2>/dev/null || echo 0)
  m=$(g rev-list --count "$new..$old" 2>/dev/null || echo 0)
  if [ "$n" -gt 0 ]; then
    say "    Commits you now have that $label didn't:"
    g log --oneline -n 10 "$old..$new" | sed 's/^/      /'
    [ "$n" -gt 10 ] && say "      ... and $((n - 10)) more"
  fi
  if [ "$m" -gt 0 ]; then
    say "    Commits from $label that this branch does NOT have:"
    g log --oneline -n 10 "$new..$old" | sed 's/^/      /'
    [ "$m" -gt 10 ] && say "      ... and $((m - 10)) more"
  fi
  if [ "$n" -eq 0 ] && [ "$m" -eq 0 ]; then
    say "    Same commit as before - no code changes."
  fi
  stat=$(g diff --stat=72 "$old" "$new" 2>/dev/null | tail -n 12)
  if [ -n "$stat" ]; then
    say "    Files:"
    printf '%s\n' "$stat" | sed 's/^/    /'
  fi
}

show_prebuilt_note() {    # compiled code is NOT rebuilt on a 'prebuilt' checkout
  local old=$1 new=$2
  local -a src=()
  [ -e "$OP_DIR/prebuilt" ] || return 0
  mapfile -t src < <(g diff --name-only "$old" "$new" -- '*.c' '*.cc' '*.cpp' '*.h' '*.hpp' \
    '*.pyx' '*.pxd' '*SConscript' 'SConstruct' 2>/dev/null)
  [ "${#src[@]}" -gt 0 ] || return 0
  blank
  say "  Heads-up: this switch also changes ${#src[@]} C/C++/Cython source file(s), e.g.:"
  printf '      %s\n' "${src[@]:0:5}"
  say "  This device has a 'prebuilt' marker, so openpilot does NOT recompile on boot."
  say "  Only Python changes (and any committed .so files) take effect."
}

show_header() {
  local br sha subj d state n
  local -a dirty=()
  br=$(current_branch)
  sha=$(g log -1 --format='%h' 2>/dev/null)
  subj=$(g log -1 --format='%s' 2>/dev/null)
  mapfile -t dirty < <(dirty_files)
  state=$(onroad_state)
  hr
  say "  MK7 GTI branch switcher                 $(date '+%Y-%m-%d %H:%M')"
  hr
  say "  Branch : ${br:-(detached - not on a branch)}"
  say "  Commit : $sha $subj"
  d=$(desc_for "$br")
  [ -n "$d" ] && say "  About  : $d"
  n=${#dirty[@]}
  if [ "$n" -eq 0 ]; then
    say "  Edits  : none (no tracked files modified)"
  else
    say "  Edits  : YES - $n tracked file(s) modified (you'll be offered a stash):"
    printf '             %s\n' "${dirty[@]:0:5}"
    [ "$n" -gt 5 ] && say "             ... and $((n - 5)) more"
  fi
  case $state in
    on)  say "  Car    : ON (onroad) - switching is blocked until the car is off" ;;
    off) say "  Car    : off (offroad) - OK to switch" ;;
    *)   say "  Car    : unknown (can't read $PARAMS_DIR/IsOnroad) - make sure it's off" ;;
  esac
  hr
}

show_list() {
  local cur i n=0 name mark
  cur=$(current_branch)
  blank
  say "  Tuning branches:"
  for ((i = 0; i < ${#CUR_NAMES[@]}; i++)); do
    n=$((n + 1))
    name=${CUR_NAMES[i]}
    mark=""
    [ "$name" = "$cur" ] && mark="   <== you are on this"
    printf '  %2d) %s%s\n' "$n" "$name" "$mark"
    [ -n "${CUR_DESCS[i]}" ] && printf '        %s\n' "${CUR_DESCS[i]}"
  done
  blank
  say "  Other branches on $REMOTE:"
  if [ "${#OTHER_NAMES[@]}" -eq 0 ]; then
    say "      (none)"
  else
    for ((i = 0; i < ${#OTHER_NAMES[@]}; i++)); do
      n=$((n + 1))
      name=${OTHER_NAMES[i]}
      mark=""
      [ "$name" = "$cur" ] && mark="   <== you are on this"
      printf '  %2d) %s%s\n' "$n" "$name" "$mark"
    done
  fi
}

prev_branch() {           # newest branch in the log that isn't the current one
  local cur from to cand i
  local -a lines=()
  [ -r "$LOG" ] || return 1
  cur=$(current_branch)
  mapfile -t lines < <(grep -E '^[^ ]+  [^ ]+@[0-9a-f]+ -> [^ ]+@[0-9a-f]+$' "$LOG" 2>/dev/null)
  for ((i = ${#lines[@]} - 1; i >= 0; i--)); do
    read -r _ from _ to <<<"${lines[i]}"
    for cand in "${to%@*}" "${from%@*}"; do
      if [ "$cand" != "$cur" ] && [ "$cand" != "(detached)" ] && ! is_hidden "$cand" \
          && valid_branch_name "$cand"; then
        printf '%s' "$cand"
        return 0
      fi
    done
  done
  return 1
}

show_stashes() {
  local e ref msg from
  local -a st=()
  mapfile -t st < <(g stash list --format='%gd|%gs' 2>/dev/null | grep -F 'mk7_switch ')
  blank
  if [ "${#st[@]}" -eq 0 ]; then
    say "  Edits saved (stashed) by this script: none"
    return 0
  fi
  say "  Edits saved (stashed) by this script, newest first:"
  for e in "${st[@]}"; do
    ref=${e%%|*}
    msg=${e#*|}
    from=${msg##* from }
    say "    $ref  ${msg#*: }"
    say "      get it back: cd $OP_DIR && git checkout $from && git stash pop $ref"
  done
}

show_status() {
  local cf p
  show_header
  blank
  if [ -s "$CURRENT_FILE" ]; then cf=$(head -n 1 -- "$CURRENT_FILE"); else cf="(not set yet)"; fi
  say "  $CURRENT_FILE: $cf"
  say "  Stop distance ($STOP_FILE): $(stop_distance_text)"
  blank
  say "  Last 10 switches ($LOG):"
  if [ -s "$LOG" ]; then
    tail -n 10 -- "$LOG" | sed 's/^/    /'
  else
    say "    (none logged yet)"
  fi
  p=$(prev_branch) || p=""
  say "  Previous branch ('prev' goes here): ${p:-(none yet)}"
  show_stashes
}

# =============================================================================
#  the switch itself
# =============================================================================
log_line() { { printf '%s\n' "$1" >>"$LOG"; } 2>/dev/null || say "  (Couldn't write to $LOG)"; }

write_current() {
  { printf '%s\n' "$1" >"$CURRENT_FILE"; } 2>/dev/null || say "  (Couldn't write to $CURRENT_FILE)"
}

do_reboot() {
  sync 2>/dev/null
  say "  Rebooting: $REBOOT_CMD"
  if eval "$REBOOT_CMD"; then
    REBOOTED=1
  else
    say "  The reboot command failed. Reboot by hand with: sudo reboot"
  fi
}

offer_reboot() {
  blank
  if [ "$YES" = 1 ]; then
    say "  Reboot now to apply? [Y/n] y   (-y given)"
    do_reboot
  elif ask_yn "  Reboot now to apply?" Y; then
    do_reboot
  else
    say "  OK, not rebooting. The new branch takes effect at the next reboot (sudo reboot)."
  fi
}

rollback() {   # rollback OLD_BRANCH OLD_SHA TARGET TARGET_PREV_SHA STASHED -> 0 ok, 1 failed
  local ob=$1 os=$2 tg=$3 tps=$4 had_stash=$5 ok=1 now
  if [ -n "$ob" ]; then
    g checkout -q -B "$ob" "$os" 2>/dev/null || g checkout -q -f -B "$ob" "$os" || ok=0
  else
    g checkout -q --detach "$os" 2>/dev/null || g checkout -q -f --detach "$os" || ok=0
  fi
  now=$(head_sha)
  [ "$now" = "$os" ] || ok=0
  if [ "$ok" = 1 ] && [ "$tg" != "$ob" ]; then
    # put the target's local branch back the way it was, so nothing points at the broken code
    if [ -n "$tps" ]; then g branch -q -f "$tg" "$tps" 2>/dev/null; else g branch -q -D "$tg" 2>/dev/null; fi
  fi
  if [ "$ok" = 1 ] && [ "$had_stash" = 1 ]; then
    if g stash pop -q >/dev/null 2>&1; then
      say "  ##  Your stashed edits were put back too."
    else
      say "  ##  Your edits are still in the stash. Get them back with:"
      say "  ##      cd $OP_DIR && git stash pop"
    fi
  fi
  [ "$ok" = 1 ]
}

do_switch() {  # do_switch BRANCH MODE(menu|cli) -> exit code (see top of file)
  local target=$1 mode=$2
  local old_branch old_sha old_label old_ref new_sha state reason d rc out
  local need_stash=0 stashed=0 stash_msg="" stash_before stash_after
  local target_prev_sha="" ahead=0 backup=""
  local -a dirty=()

  old_branch=$(current_branch)
  if ! old_sha=$(head_sha); then
    say "  Can't read the current commit in $OP_DIR."
    return 1
  fi
  old_label=${old_branch:-(detached)}
  old_ref=${old_branch:-$old_sha}

  blank
  hr
  say "  Switching: $old_label  ->  $target"
  d=$(desc_for "$target")
  [ -n "$d" ] && say "  ($d)"
  hr

  # 1. the car must be off ----------------------------------------------------
  state=$(onroad_state)
  if [ "$state" != off ]; then
    if [ "$FORCE" = 1 ]; then
      say "  WARNING: car state is '$state', but --force was given. Carrying on."
    elif [ "$state" = on ]; then
      say "  STOP: the car is ON (openpilot is onroad). Switching now could break the drive."
      say "  Park, turn the car off, wait for the comma 3X to show offroad, then try again."
      say "  (Only if you're SURE the car is off and this is wrong: add --force.)"
      return 1
    else
      say "  STOP: couldn't tell whether the car is on ($PARAMS_DIR/IsOnroad isn't"
      say "  readable and openpilot's Params check didn't work either)."
      say "  If you're SURE the car is off, run again with --force."
      return 1
    fi
  fi

  # 2. no half-finished git operation ------------------------------------------
  if reason=$(git_busy_reason); then
    say "  STOP: $reason"
    say "  Nothing was changed."
    return 1
  fi

  # 3. local edits to tracked files -> offer a stash (done later, right before switching)
  mapfile -t dirty < <(dirty_files)
  if [ "${#dirty[@]}" -gt 0 ]; then
    blank
    say "  You have ${#dirty[@]} changed file(s) in $OP_DIR that git tracks:"
    printf '      %s\n' "${dirty[@]:0:10}"
    [ "${#dirty[@]}" -gt 10 ] && say "      ... and $((${#dirty[@]} - 10)) more"
    say "  They have to be put aside before switching. I can save them in a git"
    say "  \"stash\" (a safe side-pocket) - nothing is lost, and you can bring them back."
    if [ "$YES" = 1 ]; then
      say "  Stash them and continue? [Y/n] y   (-y given)"
    elif ! ask_yn "  Stash them and continue?" Y; then
      say "  OK - cancelled. Nothing was changed."
      return 1
    fi
    need_stash=1
  fi

  # 4. download the branch -----------------------------------------------------
  blank
  fetch_branch "$target"
  rc=$?
  if [ "$rc" -ne 0 ]; then
    say "  Nothing was changed."
    return 1
  fi
  if ! new_sha=$(g rev-parse -q --verify "refs/remotes/$REMOTE/$target^{commit}"); then
    say "  Downloaded, but can't find $REMOTE/$target afterwards. Nothing was changed."
    return 1
  fi
  if ! g cat-file -e "$new_sha:launch_openpilot.sh" 2>/dev/null; then
    say "  '$target' doesn't look like an openpilot branch (no launch_openpilot.sh)."
    say "  Refusing to switch to it. Nothing was changed."
    return 1
  fi
  say "  $REMOTE/$target is at $(short "$new_sha"): $(g log -1 --format='%s' "$new_sha")"

  # 5. a local copy of the branch with commits that aren't on the remote? --------
  if target_prev_sha=$(g rev-parse -q --verify "refs/heads/$target^{commit}" 2>/dev/null); then
    ahead=$(g rev-list --count "$new_sha..$target_prev_sha" 2>/dev/null || echo 0)
  else
    target_prev_sha=""
  fi
  if [ "$ahead" -gt 0 ]; then
    blank
    say "  WARNING: your local copy of '$target' has $ahead commit(s) that are NOT on $REMOTE:"
    g log --oneline -n 10 "$new_sha..$target_prev_sha" | sed 's/^/      /'
    [ "$ahead" -gt 10 ] && say "      ... and $((ahead - 10)) more"
    say "  Switching makes '$target' match $REMOTE exactly, so those commits would no"
    say "  longer be on it (a backup branch is saved first, so they are not deleted)."
    if [ "$mode" = cli ]; then
      say "  Refusing to do that from the command line. Run $SELF with no"
      say "  arguments and pick the branch from the menu to confirm. Nothing was changed."
      return 1
    fi
    if ! ask_typed_yes; then
      say "  OK - cancelled. Nothing was changed."
      return 1
    fi
  fi

  if [ "$old_branch" = "$target" ] && [ "$old_sha" = "$new_sha" ]; then
    blank
    say "  You're already on '$target' and it matches $REMOTE ($(short "$new_sha")). Nothing to do."
    [ "$need_stash" = 1 ] && say "  (Your edited files were left exactly as they are.)"
    return 0
  fi

  # ---- point of no return: don't let Ctrl-C or a dropped SSH session cut this in half
  trap '' INT HUP

  # 6. stash local edits ---------------------------------------------------------
  if [ "$need_stash" = 1 ]; then
    stash_msg="mk7_switch $(date '+%Y-%m-%d %H:%M:%S') from $old_label"
    stash_before=$(g rev-parse -q --verify refs/stash 2>/dev/null)
    if g var GIT_COMMITTER_IDENT >/dev/null 2>&1; then
      out=$(g stash push -m "$stash_msg" 2>&1)
    else   # git has no name/email configured on the device; give the stash one
      out=$(g -c user.name=mk7_switch -c user.email=mk7_switch@localhost stash push -m "$stash_msg" 2>&1)
    fi
    rc=$?
    stash_after=$(g rev-parse -q --verify refs/stash 2>/dev/null)
    if [ "$rc" -ne 0 ]; then
      say "  Couldn't stash your edits. Git said:"
      indent "$out"
      say "  Nothing was switched."
      trap - INT HUP
      return 1
    fi
    if [ -n "$stash_after" ] && [ "$stash_after" != "$stash_before" ]; then
      stashed=1
      blank
      say "  Your edits are saved in the stash as: \"$stash_msg\""
      say "  To get them back later (on the branch they came from), run:"
      say "      cd $OP_DIR && git checkout $old_ref && git stash pop"
      say "  ('$SELF status' shows the exact command if you've stashed more since.)"
    fi
  fi

  # 7. switch ------------------------------------------------------------------------
  if [ "$ahead" -gt 0 ]; then
    backup="backup/$target-$(date '+%Y%m%d-%H%M%S')"
    if g branch "$backup" "$target_prev_sha" 2>/dev/null; then
      say "  Saved your local '$target' commits as branch '$backup'."
    else
      say "  Couldn't save a backup branch. Your commits can still be recovered with:"
      say "      cd $OP_DIR && git branch rescued $target_prev_sha"
    fi
  fi
  blank
  if ! out=$(g checkout -B "$target" "$REMOTE/$target" 2>&1); then
    say "  Git couldn't switch. It said:"
    indent "$out"
    if [ "$stashed" = 1 ]; then
      if g stash pop -q >/dev/null 2>&1; then
        say "  (Your stashed edits were put back.)"
      else
        say "  Your edits are still in the stash: cd $OP_DIR && git stash pop"
      fi
    fi
    say "  You're still on $old_label. Nothing was switched."
    trap - INT HUP
    return 1
  fi
  indent "$out"

  # 8. make sure the new Python at least compiles; otherwise go straight back ------------
  blank
  if ! check_python "$old_sha" "$new_sha"; then
    blank
    say "  ############################################################"
    say "  ##  SWITCH CANCELLED: '$target' has Python that doesn't compile."
    say "  ##  openpilot would crash on it. Going back to $old_label..."
    say "  ############################################################"
    if rollback "$old_branch" "$old_sha" "$target" "$target_prev_sha" "$stashed"; then
      log_line "$(iso_now)  FAILED switch to $target@$(short "$new_sha"), rolled back to $old_label@$(short "$old_sha") (Python compile error)"
      write_current "$old_label"
      say "  ##  Back on $old_label @ $(short "$old_sha"). Nothing else changed."
      say "  ##  Do NOT reboot expecting the new branch - it was not installed."
      say "  ############################################################"
      trap - INT HUP
      return 3
    fi
    say "  ##  !!! THE AUTOMATIC ROLL-BACK FAILED !!!"
    say "  ##  Do NOT drive or reboot. Run this, then check with: $SELF status"
    if [ -n "$old_branch" ]; then
      say "  ##      cd $OP_DIR && git checkout -f -B $old_branch $old_sha"
    else
      say "  ##      cd $OP_DIR && git checkout -f --detach $old_sha"
    fi
    say "  ############################################################"
    trap - INT HUP
    return 4
  fi

  # 9. record it ----------------------------------------------------------------------
  log_line "$(iso_now)  $old_label@$(short "$old_sha") -> $target@$(short "$new_sha")"
  write_current "$target"
  trap - INT HUP

  blank
  hr
  say "  DONE: now on '$target' @ $(short "$new_sha"): $(g log -1 --format='%s' "$new_sha")"
  hr
  show_changes "$old_sha" "$new_sha" "$old_label"
  show_prebuilt_note "$old_sha" "$new_sha"
  if [ "$target" = "$STOP_BRANCH" ] || g grep -q -F "$STOP_FILE_IN_CODE" "$new_sha" -- '*.py' 2>/dev/null; then
    show_stop_reminder
  fi
  if [ "$stashed" = 1 ]; then
    blank
    say "  (Remember: your edits from $old_label are in the stash - see above.)"
  fi
  blank
  say "  Logged to $LOG"
  offer_reboot
  return 0
}

# =============================================================================
#  self update
# =============================================================================
self_update() {           # 0 = already current, 10 = updated, 1 = failed
  local tmp rc
  blank
  fetch_branch "$TOOLS_BRANCH"
  rc=$?
  if [ "$rc" -ne 0 ]; then
    say "  Couldn't get $REMOTE/$TOOLS_BRANCH - kept the current script."
    return 1
  fi
  if ! tmp=$(mktemp "$SELF.new.XXXXXX" 2>/dev/null); then
    say "  Can't write next to $SELF - kept the current script."
    return 1
  fi
  if ! g show "refs/remotes/$REMOTE/$TOOLS_BRANCH:$TOOLS_FILE" >"$tmp" 2>/dev/null; then
    rm -f -- "$tmp"
    say "  $REMOTE/$TOOLS_BRANCH has no $TOOLS_FILE - kept the current script."
    return 1
  fi
  if [ ! -s "$tmp" ] || ! head -n 1 -- "$tmp" | grep -q '^#!.*bash' || ! bash -n "$tmp" 2>/dev/null; then
    rm -f -- "$tmp"
    say "  The downloaded copy looks broken - kept the current script."
    return 1
  fi
  if cmp -s -- "$tmp" "$SELF"; then
    rm -f -- "$tmp"
    say "  This script is already the newest version."
    return 0
  fi
  chmod 755 -- "$tmp"
  # rename (not overwrite in place): the running copy keeps reading the old file safely
  if ! mv -f -- "$tmp" "$SELF"; then
    rm -f -- "$tmp"
    say "  Couldn't replace $SELF - kept the current script."
    return 1
  fi
  say "  Updated $SELF from $REMOTE/$TOOLS_BRANCH ($(short "refs/remotes/$REMOTE/$TOOLS_BRANCH"))."
  return 10
}

# =============================================================================
#  menu / command line
# =============================================================================
menu_switch() {
  if ask_yn "  Switch to '$1'?" Y; then
    do_switch "$1" menu
  else
    say "  OK, staying put."
  fi
}

menu() {
  local choice p rc
  local -a again=()
  blank
  refresh_remote
  while :; do
    load_other_branches
    blank
    show_header
    show_list
    p=$(prev_branch) || p=""
    blank
    say "   p) back to previous branch${p:+ ($p)}"
    say "   s) status    u) update this script    q) quit"
    printf '  Your choice: '
    if ! IFS= read -r choice; then blank; return 0; fi
    [ -t 0 ] || printf '%s\n' "$choice"
    choice=$(trim "$choice")
    case $choice in
      '') continue ;;
      q | Q | quit | exit) say "  Bye."; return 0 ;;
      p | P)
        if [ -z "$p" ]; then say "  No previous branch in $LOG yet."; else menu_switch "$p"; fi ;;
      s | S) blank; show_status ;;
      u | U)
        self_update
        rc=$?
        if [ "$rc" -eq 10 ]; then
          say "  Restarting the menu with the new version..."
          again=()
          [ "$FORCE" = 1 ] && again+=(--force)
          [ "$YES" = 1 ] && again+=(-y)
          exec bash "$SELF" ${again[@]+"${again[@]}"}
        fi ;;
      *)
        if resolve_choice "$choice"; then menu_switch "$CHOICE"; fi ;;
    esac
    [ "$REBOOTED" = 1 ] && return 0
    pause || return 0
  done
}

usage() {
  cat <<EOF
mk7_switch.sh - switch openpilot branches on the comma 3X

  mk7_switch.sh                      numbered menu
  mk7_switch.sh <number|branch>      switch (asks before rebooting)
  mk7_switch.sh <number|branch> -y   switch and reboot without asking
  mk7_switch.sh prev                 go back to the previous branch (from the log)
  mk7_switch.sh status               last switches, stop distance, saved edits
  mk7_switch.sh list                 numbered list of branches
  mk7_switch.sh update               get the newest copy of this script

Options:  -y, --yes   don't ask: stash local edits and reboot
          --force     skip the "car is on" check (only if you're SURE it's off)

Settings: OP_DIR=$OP_DIR  REMOTE=$REMOTE
          LOG=$LOG  CURRENT_FILE=$CURRENT_FILE
EOF
}

preflight() {
  local owner
  if ! command -v git >/dev/null 2>&1; then
    say "  git isn't installed?!"
    return 1
  fi
  if [ "$(id -u)" = 0 ] && [ -d "$OP_DIR" ]; then
    owner=$(stat -c %u -- "$OP_DIR" 2>/dev/null)
    if [ -n "$owner" ] && [ "$owner" != 0 ]; then
      say "  Please don't run this with sudo / as root - it would leave root-owned files in"
      say "  $OP_DIR. Run it as the normal user:  $SELF"
      return 1
    fi
  fi
  if [ ! -d "$OP_DIR" ] || ! g rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    say "  Can't find the openpilot git checkout at $OP_DIR."
    return 1
  fi
  if ! g remote get-url "$REMOTE" >/dev/null 2>&1; then
    say "  This checkout has no git remote called '$REMOTE'. Add it once with:"
    say "      cd $OP_DIR && git remote add $REMOTE https://github.com/jerryromero666/mk7-op-long-sng"
    return 1
  fi
  return 0
}

main() {
  local a cmd b rc
  local -a args=()
  for a in "$@"; do
    case $a in
      -y | --yes) YES=1 ;;
      --force) FORCE=1 ;;
      -h | --help | help) usage; return 0 ;;
      -*) say "Unknown option: $a"; usage; return 2 ;;
      *) args+=("$a") ;;
    esac
  done
  if [ "${#args[@]}" -gt 1 ]; then
    say "Too many arguments: ${args[*]}"
    usage
    return 2
  fi

  # make paths absolute, then step out of whatever directory we were started in
  # (a branch switch can delete the directory the shell is sitting in)
  OP_DIR=$(abs_path "$OP_DIR")
  LOG=$(abs_path "$LOG")
  PARAMS_DIR=$(abs_path "$PARAMS_DIR")
  CURRENT_FILE=$(abs_path "$CURRENT_FILE")
  STOP_FILE=$(abs_path "$STOP_FILE")
  cd / || return 1

  load_curated
  preflight || return 1

  if [ "${#args[@]}" -eq 0 ]; then
    menu
    return 0
  fi

  cmd=${args[0]}
  case $cmd in
    list)
      refresh_remote
      load_other_branches
      show_list
      return 0 ;;
    status)
      show_status
      return 0 ;;
    update)
      self_update
      rc=$?
      [ "$rc" -eq 10 ] && return 0
      return "$rc" ;;
    prev)
      if ! b=$(prev_branch); then
        say "  No previous branch found in $LOG."
        return 1
      fi
      say "  Previous branch (from $LOG): $b"
      do_switch "$b" cli
      return $? ;;
    *)
      load_other_branches
      resolve_choice "$cmd" || return 1
      [[ $cmd =~ ^[0-9]+$ ]] && say "  #$cmd = $CHOICE"
      do_switch "$CHOICE" cli
      return $? ;;
  esac
}

# Everything above is definitions only. main and exit sit on one line so bash has
# read all of it before anything runs ('update' also swaps the file by rename).
main "$@"; exit $?
