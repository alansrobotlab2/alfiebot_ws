// ServoTool3 - React front end for gen2 Alfie servo bring-up.
//
// State model: the SSE snapshot is the truth for everything measured, and for
// whether a subsystem is under tool control. What the operator is *asking for*
// lives in a local draft, seeded from the server the moment control is taken.
// Without that split, every slider would fight the 10 Hz snapshot echo.

import { React, ReactDOM, html, api, sendJoint, sendPatch,
         useSnapshot, useHeartbeat, useEffect, useMemo, useState } from './api.js';
import { TopBar } from './components/TopBar.js';
import { ServoPanel } from './components/ServoPanel.js';
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

  const jointsBySubsystem = useMemo(() => {
    const map = { left_arm: [], right_arm: [], head: [] };
    for (const info of (config ? config.joints : [])) {
      if (map[info.subsystem]) map[info.subsystem].push(info);
    }
    return map;
  }, [config]);

  // --- actions --------------------------------------------------------------

  const report = async (promise, what) => {
    const res = await promise;
    if (res && res.ok === false) setToast(`${what}: ${res.error || 'failed'}`);
    else if (res && res.message) setToast(res.message);
    return res;
  };

  const takeControl = (subsystem) => report(api.take(subsystem), `take ${subsystem}`);
  const releaseControl = (subsystem) => report(api.release(subsystem), `release ${subsystem}`);

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

  if (!config) {
    return html`<div class="boot">Loading configuration…</div>`;
  }

  const estopped = !!(snapshot && snapshot.estop.engaged);
  const modules = (snapshot && snapshot.modules) || {};

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

      <main class="grid">
        ${SERVO_SUBSYSTEMS.map((subsystem) => html`
          <${ServoPanel}
            key=${subsystem}
            subsystem=${subsystem}
            label=${subsystem === 'head' ? 'Head' : subsystem === 'left_arm' ? 'Left arm' : 'Right arm'}
            module=${modules[subsystem]}
            joints=${jointsBySubsystem[subsystem]}
            states=${jointStates}
            draft=${draft[subsystem]}
            owned=${!!(control[subsystem] && control[subsystem].owned)}
            limits=${config.limits}
            estopped=${estopped}
            onTake=${() => takeControl(subsystem)}
            onRelease=${() => releaseControl(subsystem)}
            onTorqueOff=${() => onTorqueOff(subsystem)}
            onJointChange=${(i, patch) => onJointChange(subsystem, i, patch)} />`)}

        <${EyesPanel}
          eyes=${snapshot && snapshot.eyes}
          draft=${draft.eyes}
          owned=${!!(control.eyes && control.eyes.owned)}
          max=${config.limits.eye_pwm_max}
          estopped=${estopped}
          onTake=${() => takeControl('eyes')}
          onRelease=${() => releaseControl('eyes')}
          onChange=${onEyesChange} />

        <${BackPanel}
          back=${snapshot && snapshot.back}
          draft=${draft.back}
          owned=${!!(control.back && control.back.owned)}
          limits=${config.limits.back}
          estopped=${estopped}
          onTake=${() => takeControl('back')}
          onRelease=${() => releaseControl('back')}
          onChange=${onBackChange}
          onCalibrate=${() => report(api.calibrateBack(), 'calibrate back')} />
      </main>

      <footer class="foot">
        ${''/* htm neither decodes entities nor keeps a newline-only gap between a
              tag and the text after it, so "<subsystem>" is an expression and the
              space before "at" is explicit. */}
        Commands go to <code>${config.prefix}cmd/${'<subsystem>'}/${config.source}</code>${' '}
        at ${config.forward_rate_hz} Hz through command_mux. Register-level servo
        configuration does not exist in gen2 - the module firmware owns the memory map.
      </footer>

      ${toast ? html`<div class="toast">${toast}</div>` : null}
    </div>`;
}

ReactDOM.createRoot(document.getElementById('root')).render(html`<${App} />`);
