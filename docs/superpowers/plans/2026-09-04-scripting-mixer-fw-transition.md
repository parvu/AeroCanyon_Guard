# Scripting Mixer FW Transition Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the tricopter's `Q_FRAME_CLASS=7` (Tri) motor mixer with a Lua-scripted dynamic mixer (`Q_FRAME_CLASS=17`) that gives the rear motor an independent pure-throttle cruise role and the front pair a fold/stop cruise role, resolving the tiltrotor.cpp motor-role mismatch, then re-verify hover and the existing AUTO mission under the new mixer.

**Architecture:** A single Lua script (`Motors_dynamic`/`motor_factor_table` bindings) owns two motor-factor tables (hover, cruise) and slews the rear motor's existing tilt joint between them, triggered by `quadplane:in_vtol_mode()`. Config changes (`Q_FRAME_CLASS`, `SERVO14_FUNCTION`, `SCR_ENABLE`) wire it in. No new Gazebo/SDF geometry -- the rear tilt joint already exists.

**Tech Stack:** ArduPilot Lua scripting (`AP_Scripting`), the local WSL SITL stack (`gz sim` + `arduplane` + MAVROS) documented in the `local-wsl-sitl-setup` memory.

**Spec:** `docs/superpowers/specs/2026-09-04-scripting-mixer-fw-transition-design.md`

## Global Constraints

- Motor indices are 0-based and MUST match the existing physical convention already in `tricopter.parm`/`model.sdf`: motor 0 = front right, motor 1 = rear, motor 2 = front left.
- `Q_TILT_MASK`, `Q_TILT_TYPE=2`, `Q_TILT_MAX`, and all other front-pair tilt-yaw config stay UNCHANGED -- this plan only touches the rear motor's role and the overall motor mixer class.
- Every live-flight step arms at low altitude in calm air (no `wind_field_node` running) until explicitly noted otherwise.
- Before any live SITL launch, check `ps aux --sort=-%cpu | head` for stray `bfs`/`find` processes (see `check-for-stray-bfs-find` memory) -- this has caused multiple false "SITL crashed" diagnoses this project.

---

### Task 1: Write the Lua mixer script and wire the config

**Files:**
- Create: `src/aerocanyon/ardupilot/scripts/tricopter_mixer.lua`
- Modify: `src/aerocanyon/ardupilot/tricopter.parm` (append new section)
- Modify: `tricopter.param` (repo root, Windows-side dump -- keep in sync, same pattern as every other param change this project makes)

**Interfaces:**
- Consumes: `quadplane:in_vtol_mode()` (boolean, true=VTOL/hover), `Motors_dynamic:add_motor(motor_num, testing_order)`, `motor_factor_table()` with `:roll()/:pitch()/:yaw()/:throttle()` (each `(motor_num, factor)`), `Motors_dynamic:load_factors(table)`, `Motors_dynamic:init(num_motors)` (boolean), `SRV_Channels:set_output_scaled(function_num, value)`.
- Produces: nothing consumed by later tasks in this plan (this is the leaf script); Task 2 onward observe its *behavior* live, not its code.

- [ ] **Step 1: Write the script**

```lua
-- Dynamic scripting motor mixer for the tricopter-ap VTOL.
-- Hover: front pair (motors 0,2) do roll+pitch+yaw+throttle (matches the
-- old Q_FRAME_CLASS=7 Tri mixer's allocation as closely as possible);
-- rear (motor 1) does pitch+throttle only -- no roll, no yaw (yaw stays
-- the front pair's own Q_TILT_TYPE=2 VectoredYaw job, unchanged).
-- Cruise: front pair throttle zeroed (fold/stop); rear is throttle-only
-- (roll/pitch/yaw come from elevons/rudder in forward flight).
--
-- The rear motor's own tilt joint (SERVO14, reassigned from
-- k_tiltMotorRear to k_scripting1 by this same config change so this
-- script can own it independently of ArduPilot's own Tiltrotor class)
-- is slewed 0deg(down/hover)->90deg(horizontal/cruise) in lockstep with
-- the factor-table swap. The table swap only happens once the tilt has
-- actually reached its target -- a mid-tilt rear thrust vector combined
-- with the wrong table would put thrust into the wrong axis. See the
-- design spec's "Mode-detection + tilt coordination" section.

local UPDATE_HZ = 10
local TILT_SLEW_DURATION_S = 3.0
local REAR_TILT_FUNCTION = 94  -- k_scripting1, see SERVO14_FUNCTION in tricopter.parm

local MOTOR_FRONT_RIGHT = 0
local MOTOR_REAR = 1
local MOTOR_FRONT_LEFT = 2

local hover_factors = motor_factor_table()
hover_factors:roll(MOTOR_FRONT_RIGHT, -1.0)
hover_factors:pitch(MOTOR_FRONT_RIGHT, 1.0)
hover_factors:yaw(MOTOR_FRONT_RIGHT, 0.0)
hover_factors:throttle(MOTOR_FRONT_RIGHT, 1.0)

hover_factors:roll(MOTOR_FRONT_LEFT, 1.0)
hover_factors:pitch(MOTOR_FRONT_LEFT, 1.0)
hover_factors:yaw(MOTOR_FRONT_LEFT, 0.0)
hover_factors:throttle(MOTOR_FRONT_LEFT, 1.0)

hover_factors:roll(MOTOR_REAR, 0.0)
hover_factors:pitch(MOTOR_REAR, -1.0)
hover_factors:yaw(MOTOR_REAR, 0.0)
hover_factors:throttle(MOTOR_REAR, 1.0)

local cruise_factors = motor_factor_table()
cruise_factors:roll(MOTOR_FRONT_RIGHT, 0.0)
cruise_factors:pitch(MOTOR_FRONT_RIGHT, 0.0)
cruise_factors:yaw(MOTOR_FRONT_RIGHT, 0.0)
cruise_factors:throttle(MOTOR_FRONT_RIGHT, 0.0)

cruise_factors:roll(MOTOR_FRONT_LEFT, 0.0)
cruise_factors:pitch(MOTOR_FRONT_LEFT, 0.0)
cruise_factors:yaw(MOTOR_FRONT_LEFT, 0.0)
cruise_factors:throttle(MOTOR_FRONT_LEFT, 0.0)

cruise_factors:roll(MOTOR_REAR, 0.0)
cruise_factors:pitch(MOTOR_REAR, 0.0)
cruise_factors:yaw(MOTOR_REAR, 0.0)
cruise_factors:throttle(MOTOR_REAR, 1.0)

Motors_dynamic:add_motor(MOTOR_FRONT_RIGHT, 1)
Motors_dynamic:add_motor(MOTOR_REAR, 2)
Motors_dynamic:add_motor(MOTOR_FRONT_LEFT, 3)
Motors_dynamic:load_factors(hover_factors)
assert(Motors_dynamic:init(3), "tricopter_mixer: failed to init Motors_dynamic")
motors:set_frame_string("tricopter scripting mixer")

local current_tilt_deg = 0.0
local in_hover_table = true

local function update()
    local want_hover = quadplane:in_vtol_mode()
    local target_tilt_deg = want_hover and 0.0 or 90.0
    local step_deg = 90.0 / (TILT_SLEW_DURATION_S * UPDATE_HZ)

    if current_tilt_deg < target_tilt_deg then
        current_tilt_deg = math.min(current_tilt_deg + step_deg, target_tilt_deg)
    elseif current_tilt_deg > target_tilt_deg then
        current_tilt_deg = math.max(current_tilt_deg - step_deg, target_tilt_deg)
    end

    -- Scaled-output convention (0-1000 for "fully down" to "fully
    -- horizontal") matches tiltrotor.cpp's own k_tiltMotorRear write
    -- (tiltrotor.cpp:575, 1000*fraction) -- k_scripting1 is a different
    -- function so this needs live verification in Task 2 against actual
    -- SERVO14 PWM output; switch to SRV_Channels:set_output_pwm(...) with
    -- explicit 1000/2000 endpoints if this doesn't map as expected.
    SRV_Channels:set_output_scaled(REAR_TILT_FUNCTION, 1000.0 * (current_tilt_deg / 90.0))

    if want_hover and current_tilt_deg <= 0.01 and not in_hover_table then
        Motors_dynamic:load_factors(hover_factors)
        in_hover_table = true
    elseif not want_hover and current_tilt_deg >= 89.99 and in_hover_table then
        Motors_dynamic:load_factors(cruise_factors)
        in_hover_table = false
    end

    return update, 1000 / UPDATE_HZ
end

return update, 1000
```

- [ ] **Step 2: Append the config section to `tricopter.parm`**

Add after the existing `WP_RADIUS`/`ARSPD`/`Q_RTL_ALT` section (end of file):

```
# --- Dynamic scripting motor mixer -- FW transition, 2026-09-04 ---
# See docs/superpowers/specs/2026-09-04-scripting-mixer-fw-transition-design.md
# for why: Q_FRAME_CLASS=7 (Tri) has no way to give the rear motor an
# independent cruise-only role -- ArduPilot's own tiltrotor.cpp assumes
# the FRONT (tilting) pair becomes the cruise thruster, the opposite of
# this vehicle's real design. Q_FRAME_CLASS=17 (Dynamic Scripting
# Matrix) hands motor-factor allocation to scripts/tricopter_mixer.lua
# instead. Q_TILT_MASK/Q_TILT_TYPE/Q_TILT_MAX (front pair's own tilt-yaw)
# are UNCHANGED -- only the rear motor's role and the overall mixer
# class change here.
Q_FRAME_CLASS     17
SCR_ENABLE        1
# SERVO14 was k_tiltMotorRear (45, driven automatically by tiltrotor.cpp
# following the front pair's own tilt progress) -- reassigned to
# k_scripting1 (94) so tricopter_mixer.lua can own the rear tilt
# schedule independently, in lockstep with its own factor-table swap.
SERVO14_FUNCTION  94
```

- [ ] **Step 3: Mirror the same three param changes into `tricopter.param`**

```bash
cd /home/parvu/AeroCanyon_Guard
python3 -c "
path = 'tricopter.param'
with open(path) as f:
    lines = f.readlines()
out = []
for line in lines:
    if line.startswith('Q_FRAME_CLASS,'):
        out.append('Q_FRAME_CLASS,17\n')
    elif line.startswith('SCR_ENABLE,'):
        out.append('SCR_ENABLE,1\n')
    elif line.startswith('SERVO14_FUNCTION,'):
        out.append('SERVO14_FUNCTION,94\n')
    else:
        out.append(line)
with open(path, 'w') as f:
    f.writelines(out)
"
grep -n '^Q_FRAME_CLASS\|^SCR_ENABLE\|^SERVO14_FUNCTION' tricopter.param
```

Expected: all three lines present with the new values. If `SCR_ENABLE` isn't in the file at all (it may not be, if scripting was never touched before), append it manually instead of relying on the substitution.

- [ ] **Step 4: Commit**

```bash
cd /home/parvu/AeroCanyon_Guard
git add src/aerocanyon/ardupilot/scripts/tricopter_mixer.lua src/aerocanyon/ardupilot/tricopter.parm tricopter.param
git commit -m "$(cat <<'EOF'
Add dynamic scripting motor mixer for FW-transition rear-motor role

Q_FRAME_CLASS 7->17 (Dynamic Scripting Matrix), SERVO14_FUNCTION
45->94 (k_tiltMotorRear -> k_scripting1, so the script owns the rear
tilt schedule instead of ArduPilot's own Tiltrotor class), SCR_ENABLE=1.
Implements docs/superpowers/specs/2026-09-04-scripting-mixer-fw-transition-design.md.
Not yet live-verified -- see the matching plan's Task 2.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 2: Verify clean SITL boot with the new mixer

**Files:** none (verification-only task)

**Interfaces:**
- Consumes: Task 1's `tricopter.parm`, `tricopter_mixer.lua`.
- Produces: confirmation the script loads without a Lua error and SITL doesn't hit `config_error` -- the gate later tasks depend on.

- [ ] **Step 1: Check for stray processes before launching anything**

```bash
ps aux --sort=-%cpu | head -10
uptime
```

Expected: no `bfs`/`find` process consuming significant CPU, load average reasonable for the machine. Kill any stray full-filesystem scans found (`kill -9 <pid>`) before proceeding -- see `check-for-stray-bfs-find` memory for why this matters.

- [ ] **Step 2: Deploy the script into a fresh SITL apstate directory**

ArduPilot SITL looks for scripts in `./scripts` relative to its own CWD (confirmed via `AP_Scripting/lua_common_defs.h`: `SCRIPTING_DIRECTORY "./scripts"` for the SITL/ChibiOS-non-APM build). The apstate directory is recreated fresh on every launch (see `local-wsl-sitl-setup` memory's startup sequence), so the script has to be copied in each time, not just referenced once:

```bash
rm -rf /tmp/apstate_mixer_test && mkdir -p /tmp/apstate_mixer_test/scripts
cp /home/parvu/AeroCanyon_Guard/src/aerocanyon/ardupilot/scripts/tricopter_mixer.lua /tmp/apstate_mixer_test/scripts/
```

- [ ] **Step 3: Launch gz sim (if not already running)**

```bash
source /opt/ros/jazzy/setup.bash
export GZ_SIM_RESOURCE_PATH=$HOME/AeroCanyon_Guard/src/aerocanyon/models:$HOME/AeroCanyon_Guard/src/aerocanyon:$HOME/PX4-Autopilot/Tools/simulation/gz/models
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build
nohup gz sim -v 2 -s -r $HOME/AeroCanyon_Guard/src/aerocanyon/worlds/map_zone_ap.sdf > /tmp/gzsim_mixer_test.log 2>&1 &
disown
sleep 5
ps aux | grep "gz sim" | grep -v grep
```

Expected: `gz sim` process present, log has only the usual benign `gz_frame_id` warnings (no errors).

- [ ] **Step 4: Launch SITL from the apstate directory with the script present**

```bash
cd /tmp/apstate_mixer_test
nohup bash -c '/home/parvu/ardupilot/build/sitl/bin/arduplane --model JSON \
  --home 44.434424990487216,26.04781615647584,0,0 \
  --wipe --defaults /home/parvu/AeroCanyon_Guard/src/aerocanyon/ardupilot/tricopter.parm; \
  echo "EXIT CODE: $?"' > /tmp/sitl_mixer_test.log 2>&1 &
disown
sleep 15
tail -40 /tmp/sitl_mixer_test.log
```

Expected: no `config_error` message, no `Unsupported Q_FRAME_CLASS`, no Lua-related error (`lua_abort`, `Failed to load script`, or a Lua traceback). If SITL exits with a config_error, the log will say exactly which param is rejected -- fix it before proceeding, don't guess.

- [ ] **Step 5: Confirm the process is still alive (not just that it didn't print an error before exiting)**

```bash
ps aux | grep arduplane | grep -v grep
```

Expected: the `arduplane` process present and running. If it's gone despite no visible error in the log, check `ps aux --sort=-%cpu | head` again for stray-process contention (this project's most common cause of an unexplained SITL death) before assuming a real script/config bug.

- [ ] **Step 6: Connect MAVROS and confirm `mode` reports something other than an error/undefined state**

```bash
export GEOGRAPHICLIB_DATA=$HOME/.local/share/GeographicLib
source /opt/ros/jazzy/setup.bash
source /home/parvu/AeroCanyon_Guard/install/setup.bash
nohup ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=tcp://127.0.0.1:5760 -p system_id:=255 \
  -p gcs_url:=tcp-l://0.0.0.0:5761@ \
  > /tmp/mavros_mixer_test.log 2>&1 &
disown
sleep 20
timeout 6 ros2 topic echo /mavros/state --once
```

Expected: `connected: true`, `mode: CMODE(0)` (MANUAL) or similar valid mode -- NOT a connection failure. This confirms the vehicle firmware itself came up sane with the new frame class and script active, not just that the process didn't crash.

- [ ] **Step 7: No commit for this task** (verification-only; if Steps 1-6 required fixes to Task 1's files, go back and amend/re-commit Task 1's commit isn't necessary -- make a new fix commit instead, since Task 1 was already committed)

---

### Task 3: Verify the tilt-slew and table-swap runtime logic

**Files:** none (verification-only task)

**Interfaces:**
- Consumes: the live SITL instance from Task 2 (or a freshly relaunched one following the same steps).
- Produces: confirmation the rear tilt servo actually moves and the mixer swaps tables in response to `in_vtol_mode()`, before trusting it under a real flight.

- [ ] **Step 1: With SITL connected and disarmed, confirm the rear tilt servo starts at "down" (hover) position**

```bash
timeout 8 python3 -c "
from pymavlink import mavutil
conn = mavutil.mavlink_connection('tcp:127.0.0.1:5761', source_system=210)
conn.wait_heartbeat(timeout=6)
msg = conn.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=5)
print('servo14 (rear tilt):', msg.servo14_raw)
"
```

Expected: `servo14_raw` near the low end of its configured PWM range (representing 0 deg / down) -- SITL boots in a Q-mode-equivalent default state, so `in_vtol_mode()` should read true and the script's `target_tilt_deg` should be 0. If `set_output_scaled`'s 0-1000 convention from Task 1's Step 1 comment doesn't map to the expected PWM endpoints, this is where it'll show up wrong -- adjust the script to use `SRV_Channels:set_output_pwm` with explicit endpoints instead if so, and re-run Task 2 Steps 4-6 to confirm the fix.

- [ ] **Step 2: Arm into QSTABILIZE and confirm the tilt servo stays at "down" (still in a VTOL mode)**

```bash
timeout 15 python3 -c "
from pymavlink import mavutil
import time
conn = mavutil.mavlink_connection('tcp:127.0.0.1:5761', source_system=211)
conn.wait_heartbeat(timeout=6)
conn.mav.set_mode_send(conn.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 17)  # QSTABILIZE
time.sleep(1)
conn.mav.command_long_send(conn.target_system, conn.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0,0,0,0,0,0)
conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
time.sleep(2)
msg = conn.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=5)
print('servo14 (rear tilt) while armed QSTABILIZE:', msg.servo14_raw)
conn.mav.command_long_send(conn.target_system, conn.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0,0,0,0,0,0)
"
```

Expected: `servo14_raw` unchanged from Step 1 (still "down") -- QSTABILIZE is a VTOL mode, `in_vtol_mode()` stays true, no tilt/table swap should happen.

- [ ] **Step 3: Force a non-VTOL mode (disarmed, ground-safe) and confirm the tilt servo sweeps toward "horizontal" over ~3 seconds**

```bash
timeout 20 python3 -c "
from pymavlink import mavutil
import time
conn = mavutil.mavlink_connection('tcp:127.0.0.1:5761', source_system=212)
conn.wait_heartbeat(timeout=6)
conn.mav.set_mode_send(conn.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 5)  # FBWA, not a VTOL mode
for i in range(6):
    time.sleep(0.5)
    msg = conn.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=2)
    if msg:
        print(f't={i*0.5:.1f}s servo14={msg.servo14_raw}')
"
```

Expected: `servo14_raw` climbing steadily from the Step-1 "down" value toward the "horizontal" endpoint over roughly 3 seconds, matching `TILT_SLEW_DURATION_S`. If it jumps instantly instead of slewing, or doesn't move at all, that's a script bug to fix in `tricopter_mixer.lua` (check `update()`'s scheduling -- confirm the `return update, 1000/UPDATE_HZ` reschedule is actually being honored by re-reading `AP_Scripting`'s example scripts' own reschedule convention if this happens).

- [ ] **Step 4: Switch back to a VTOL mode and confirm the tilt servo sweeps back to "down"**

```bash
timeout 20 python3 -c "
from pymavlink import mavutil
import time
conn = mavutil.mavlink_connection('tcp:127.0.0.1:5761', source_system=213)
conn.wait_heartbeat(timeout=6)
conn.mav.set_mode_send(conn.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 17)  # QSTABILIZE
for i in range(6):
    time.sleep(0.5)
    msg = conn.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=2)
    if msg:
        print(f't={i*0.5:.1f}s servo14={msg.servo14_raw}')
"
```

Expected: `servo14_raw` sweeping back down to the Step-1 value over ~3 seconds.

- [ ] **Step 5: No commit** (verification-only; the script already committed in Task 1 is confirmed working as-is, or was fixed and re-committed during this task's Step 3 troubleshooting)

---

### Task 4: Re-tune hover stability under the new mixer

**Files:**
- Modify: `src/aerocanyon/ardupilot/tricopter.parm` (whatever gains change)
- Modify: `tricopter.param` (mirror the same changes)

**Interfaces:**
- Consumes: the verified-working script/config from Tasks 1-3.
- Produces: a hover-stable configuration for Task 5 to fly the AUTO mission against.

This task is empirical -- exact gain values can't be pre-written since they're determined live, the same way the original Phase 1 hover tuning was. Follow this process rather than guessing values upfront:

- [ ] **Step 1: Arm into QSTABILIZE at low altitude and observe attitude behavior**

Use the same live-flight monitoring approach as the rest of this project (watch `/mavros/state` for mode/armed, and pull `ATTITUDE` messages via a short pymavlink script, matching the pattern used throughout this session's earlier sanity checks). Give a small climb throttle and watch roll/pitch/yaw for oscillation, drift, or asymmetric response between the front pair and rear.

- [ ] **Step 2: If oscillation or asymmetric response is observed, adjust `Q_A_RAT_RLL_P/I/D`, `Q_A_RAT_PIT_P/I/D`, `Q_A_RAT_YAW_P/I/D` (currently 0.20/0.20/0.004, 0.20/0.20/0.004, 0.25/0.03/0.008 respectively per `tricopter.parm`)**

Since the hover factor table changes the rear motor's contribution (pitch+throttle only now, vs full RPYT before under the Tri mixer), pitch response in particular is likely to need retuning -- the rear motor's pitch authority is now working alongside a factor table that's structurally different from what these gains were tuned against. Push live param changes via the same `pymavlink` `param_set_send` + `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` pattern used earlier this project (see the `ARSPD_TYPE` live-tuning example from 2026-09-04's session), iterating until stable.

- [ ] **Step 3: If hover thrust feels wrong (climbs too fast/slow for a given throttle), check `Q_M_THST_HOVER` (currently 0.47)**

The hover factor table's throttle allocation across all 3 motors should be numerically similar to the old Tri mixer (all three still get throttle factor 1.0 in the hover table), so this is less likely to need a change than the rate gains -- but verify rather than assume.

- [ ] **Step 4: Once stable, persist the final gains to both `tricopter.parm` and `tricopter.param`, then commit**

```bash
cd /home/parvu/AeroCanyon_Guard
git add src/aerocanyon/ardupilot/tricopter.parm tricopter.param
git commit -m "$(cat <<'EOF'
Re-tune hover gains for the dynamic scripting mixer

Live-tuned after switching to Q_FRAME_CLASS=17 -- the hover factor
table's rear-motor allocation (pitch+throttle only, not full RPYT)
changed pitch response enough that the Tri-mixer-era gains no longer
applied cleanly. See docs/superpowers/plans/2026-09-04-scripting-mixer-fw-transition.md Task 4.

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

### Task 5: Re-verify the AUTO mission under the new mixer

**Files:** none (verification-only task)

**Interfaces:**
- Consumes: Task 4's stable hover config.
- Produces: confirmation the existing `map_zone_demo.json` mission still flies correctly (hover-only, no transition attempted) before any transition work begins.

- [ ] **Step 1: Launch the full local stack** (gz sim, SITL with the script deployed per Task 2 Step 2, MAVROS) following the `local-wsl-sitl-setup` memory's sequence.

- [ ] **Step 2: Launch `controller_node` against the existing mission**

```bash
source /opt/ros/jazzy/setup.bash && source /home/parvu/AeroCanyon_Guard/install/setup.bash
nohup ros2 run aerocanyon controller_node --ros-args \
  -p mode:=baseline -p world:=map_zone \
  -p mission_file:=/home/parvu/AeroCanyon_Guard/data/missions/map_zone_demo.json \
  > /tmp/controller_node_mixer_test.log 2>&1 &
disown
```

- [ ] **Step 3: Watch telemetry through the whole mission (arm -> AUTO -> all waypoints -> QRTL -> land -> disarm)**

Expected: same successful pattern as every other AUTO-mission flight this project has verified -- mission confirmed, armed into AUTO directly (no QSTABILIZE detour, per the earlier `controller_node.py` fix), waypoint sequence advancing, clean QRTL return. Since the whole mission stays in Q-modes (`in_vtol_mode()` true throughout), the new mixer should behave identically to the old Tri mixer's hover table here -- this task is confirming that equivalence holds under real flight, not just the Task 2/3 ground checks.

- [ ] **Step 4: No commit** (verification-only; if a bug surfaces, fix it in the relevant earlier task's files and re-commit there)

---

### Task 6: First real transition attempt

**Files:** none (verification-only task; this is the final checkpoint the whole plan builds toward)

**Interfaces:**
- Consumes: Task 5's verified-working AUTO mission under the new mixer.
- Produces: a go/no-go signal for whether the scripting-mixer approach actually resolves the original problem -- the answer this whole plan exists to get.

- [ ] **Step 1: At low altitude, calm air (no `wind_field_node`), manually command a mode that exits `in_vtol_mode()` (e.g. `FBWA` or `CRUISE`) while airborne and hovering stably**

- [ ] **Step 2: Watch `SERVO_OUTPUT_RAW` (front pair throttle should fall toward zero, rear should hold/rise) and `ATTITUDE`/`VFR_HUD` (airspeed should start climbing as the rear motor pushes forward) simultaneously**

Expected: front motors spin down as the cruise factor table takes over, rear motor provides forward thrust, vehicle begins accelerating forward under elevon/rudder-based attitude control instead of Q_-mode multicopter control. This is the actual test of everything this plan built -- if it doesn't work as expected, that's real new information (not a plan failure), and should go back into `ardupilot_phase2_notes.md` the same way every other finding this project has made did.

- [ ] **Step 3: Command back to a VTOL mode before the vehicle gets far/fast, confirm the reverse transition (rear throttle down, front pair spins back up, tilt sweeps back to down) completes cleanly**

- [ ] **Step 4: Record the outcome (success, partial, or failure with specifics) in `ardupilot_phase2_notes.md`, whatever it is**

This is a live finding, not a code change -- update memory directly rather than committing a code change for this step.
