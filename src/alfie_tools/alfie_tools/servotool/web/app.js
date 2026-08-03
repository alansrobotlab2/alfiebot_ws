// ServoTool3 - React front end for gen2 Alfie servo bring-up.
//
// The page shows ONE physical servo at a time, picked with the target + servo
// dropdowns: angle control down the left, that servo's register map on the
// right. Register access is addressed per physical servo (7 per arm), while
// commands are addressed per logical joint (6 per arm) - the selected servo's
// `joint_index` is what bridges the two.
//
// State model: the SSE snapshot is the truth for everything measured and for
// whether a subsystem is under tool control. What the operator is *asking for*
// lives in a local draft, seeded from the server the moment control is taken.
// Without that split, every slider would fight the 10 Hz snapshot echo.

import { React, ReactDOM, html, api, sendJoint, sendPatch,
         useSnapshot, useHeartbeat, useEffect, useMemo, useState } from './api.js';
import { TopBar } from './components/TopBar.js';
import { ServoSelector } from './components/ServoSelector.js';
import { AngleColumn } from './components/AngleColumn.js';
import { LiveStrip } from './components/LiveStrip.js';
import { MemoryPanel } from './components/MemoryPanel.js';
import { EyesPanel } from './components/EyesPanel.js';
import { BackPanel } from './components/BackPanel.js';

const SERVO_SUBSYSTEMS = ['left_arm', 'right_arm', 'head'];
const HEARTBEAT_MS = 400;

function App() {
  const [config, setConfig] = useState(null);
  const [snapshot, connected] = useSnapshot();
  const [draft, setDraft] = useState({});
  const [toast, setToast] = useState(null);

  useEffect(() => {
    api.config().then(setConfig).catch((err) => setToast(`config failed: ${err}`));
  }, []);

  useEffect(() => {
    if (!toast) return undefined;
    const id = setTimeout(() => setToast(null), 4000);
    return () => clearTimeout(id);
  }, [toast]);

  const control = (snapshot && snapshot.control) || {};
  const held = useMemo(
    () => Object.keys(control).filter((k) => control[k].owned),
    [snapshot]);

  useHeartbeat(held.length > 0, HEARTBEAT_MS);

  // Seed a draft when control is taken; drop it when control goes away (which
  // includes the server's deadman release, so a dropped browser leaves no
  // stale sliders behind).
  useEffect(() => {
    if (!snapshot) return;
    setDraft((prev) => {
      let changed = false;
      const next = { ...prev };
      for (const [name, state] of Object.entries(snapshot.control)) {
        if (state.owned && next[name] === undefined) {
          next[name] = SERVO_SUBSYSTEMS.includes(name)
            ? state.joints.map((j) => ({ ...j }))
            : name === 'eyes' ? [...state.eye_pwm] : { ...state.back };
          changed = true;
        } else if (!state.owned && next[name] !== undefined) {
          delete next[name];
          changed = true;
        }
      }
      return changed ? next : prev;
    });
  }, [snapshot]);

  const jointStates = useMemo(() => {
    const map = {};
    for (const joint of (snapshot ? snapshot.joints : [])) map[joint.key] = joint;
    return map;
  }, [snapshot]);

  const jointsByKey = useMemo(() => {
    const map = {};
    for (const info of (config ? config.joints : [])) map[info.key] = info;
    return map;
  }, [config]);

  // --- actions --------------------------------------------------------------

  const report = async (promise, what) => {
    const res = await promise;
    if (res && res.ok === false) setToast(`${what}: ${res.error || 'failed'}`);
    else if (res && res.message) setToast(res.message);
    return res;
  };

  const onJointChange = (subsystem, index, patch) => {
    setDraft((prev) => {
      const joints = prev[subsystem];
      if (!joints) return prev;
      const next = joints.slice();
      next[index] = { ...next[index], ...patch };
      return { ...prev, [subsystem]: next };
    });
    sendJoint(subsystem, index, patch);
  };

  const onEyesChange = (value) => {
    setDraft((prev) => ({ ...prev, eyes: value }));
    sendPatch('eyes', { eye_pwm: value });
  };

  const onBackChange = (patch) => {
    setDraft((prev) => {
      const merged = { ...(prev.back || {}), ...patch };
      sendPatch('back', { back: merged });
      return { ...prev, back: merged };
    });
  };

  const onTorqueOff = (subsystem) => {
    setDraft((prev) => {
      const joints = prev[subsystem];
      if (!joints) return prev;
      return { ...prev, [subsystem]: joints.map((j) => ({ ...j, enabled: false })) };
    });
    report(api.torqueOff(subsystem), 'torque off');
  };

  if (!config || !snapshot) {
    return html`<div class="boot">Loading ServoTool3…</div>`;
  }

  const estopped = !!snapshot.estop.engaged;
  const modules = snapshot.modules || {};
  const memory = snapshot.memory || {};
  const subsystem = memory.subsystem || 'left_arm';

  const servo = (config.bus_servos[subsystem] || [])
    .find((s) => s.bus_id === memory.bus_id) || null;
  const jointIndex = servo ? servo.joint_index : null;
  const joint = jointIndex === null || jointIndex === undefined
    ? null
    : jointsByKey[`${subsystem}.${jointIndex}`];
  const owned = !!(control[subsystem] && control[subsystem].owned);
  // The firmware refuses EEPROM writes while the servo holds torque. Read it
  // from the register map when we have one (it covers the derived servo too),
  // and fall back to joint feedback.
  const torqueOn = memory.values && memory.values.torqueswitch !== undefined
    ? memory.values.torqueswitch !== 0
    : !!(joint && jointStates[joint.key] && jointStates[joint.key].enabled);

  return html`
    <div class="app">
      <${TopBar}
        connected=${connected}
        snapshot=${snapshot}
        config=${config}
        held=${held}
        onEstop=${() => report(api.estop('engage'), 'e-stop')}
        onReset=${() => report(api.estop('reset'), 'e-stop reset')}
        onReleaseAll=${() => report(api.releaseAll(), 'release all')} />

      ${!connected
        ? html`<div class="banner banner-warn">
                 Lost the state stream - retrying. Held subsystems are released
                 after ${config.control_timeout}s without a heartbeat.
               </div>`
        : null}

      <${ServoSelector}
        targets=${SERVO_SUBSYSTEMS}
        busServos=${config.bus_servos}
        selection=${{ subsystem, bus_id: memory.bus_id }}
        module=${modules[subsystem]}
        onSelect=${(target, busId) =>
          report(api.select(target, busId), 'select servo')} />

      <main class="servo-page">
        <${AngleColumn}
          servo=${servo || { bus_id: memory.bus_id, mirrored: false }}
          joint=${joint}
          target=${owned && jointIndex !== null && draft[subsystem]
            ? draft[subsystem][jointIndex] : null}
          feedback=${joint ? jointStates[joint.key] : null}
          owned=${owned}
          limits=${config.limits}
          estopped=${estopped}
          moduleOnline=${!!(modules[subsystem] && modules[subsystem].online)}
          onTake=${() => report(api.take(subsystem), `take ${subsystem}`)}
          onRelease=${() => report(api.release(subsystem), `release ${subsystem}`)}
          onTorqueOff=${() => onTorqueOff(subsystem)}
          onChange=${(patch) => onJointChange(subsystem, jointIndex, patch)} />

        <div class="servo-detail">
          <${LiveStrip}
            joint=${joint}
            feedback=${joint ? jointStates[joint.key] : null}
            servo=${servo} />

          <${MemoryPanel}
            memory=${memory}
            groups=${config.register_groups}
            registers=${config.registers}
            writable=${config.memory_writable}
            faults=${memory.faults}
            torqueOn=${torqueOn}
            onWrite=${(register, value) => report(
              api.writeRegister(subsystem, memory.bus_id, register.address, value),
              register.label)}
            onLock=${(locked) => report(
              api.setLock(subsystem, memory.bus_id, locked),
              locked ? 'lock' : 'unlock')} />
        </div>
      </main>

      <section class="aux">
        <${EyesPanel}
          eyes=${snapshot.eyes}
          draft=${draft.eyes}
          owned=${!!(control.eyes && control.eyes.owned)}
          max=${config.limits.eye_pwm_max}
          estopped=${estopped}
          onTake=${() => report(api.take('eyes'), 'take eyes')}
          onRelease=${() => report(api.release('eyes'), 'release eyes')}
          onChange=${onEyesChange} />

        <${BackPanel}
          back=${snapshot.back}
          draft=${draft.back}
          owned=${!!(control.back && control.back.owned)}
          limits=${config.limits.back}
          estopped=${estopped}
          onTake=${() => report(api.take('back'), 'take back')}
          onRelease=${() => report(api.release('back'), 'release back')}
          onChange=${onBackChange}
          onCalibrate=${() => report(api.calibrateBack(), 'calibrate back')} />
      </section>

      <footer class="foot">
        ${''/* htm neither decodes entities nor keeps a newline-only gap between a
              tag and the text after it, so "<subsystem>" is an expression and the
              space before "at" is explicit. */}
        Commands go to <code>${config.prefix}cmd/${'<subsystem>'}/${config.source}</code>${' '}
        at ${config.forward_rate_hz} Hz through command_mux; registers are read
        from the module firmware at ${config.memory_poll_hz} Hz.
      </footer>

      ${toast ? html`<div class="toast">${toast}</div>` : null}
    </div>`;
}

ReactDOM.createRoot(document.getElementById('root')).render(html`<${App} />`);
