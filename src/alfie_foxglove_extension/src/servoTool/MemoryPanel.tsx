// The selected servo's register map, grouped and editable.
//
// Three things this view has to keep honest:
//
//  * Which values are live. EEPROM registers only change when something writes
//    them; SRAM and feedback registers change under you constantly. They are
//    read in the same transaction, so nothing in the raw data distinguishes
//    them - the `kind` from /api/config does.
//  * Whether EEPROM is writable. Register 0x37 is 0 when UNLOCKED. A tool that
//    shows values but not the lock invites someone to conclude a write silently
//    failed, which is exactly what the servo does when locked.
//  * That an edit landed. The firmware re-reads after every write and the row
//    redraws from that read-back, so a value that snaps back was rejected by
//    the servo, not by the panel.

import { ReactElement, useEffect, useState } from "react";

import { Memory, Register, RegisterKind, fmt } from "./api";
import { Chip } from "./ui";

const KIND_LABEL: Record<RegisterKind, string> = {
  eeprom: "persistent",
  sram: "live · command",
  feedback: "live · measured",
};

/** Engineering value of a raw word: sign-magnitude registers carry the
 *  direction in `sign_bit`, not in two's complement. */
function decodeRaw(register: Register, raw: number): number {
  if (register.sign_bit == undefined || register.sign_bit === 0) {
    return raw;
  }
  const word = raw & 0xffff;
  const sign = 1 << register.sign_bit;
  return (word & sign) !== 0 ? -(word & (sign - 1)) : word;
}

function scaledText(register: Register, raw: number): string | undefined {
  if (register.scale == undefined || register.scale === 0) {
    return undefined;
  }
  const digits = register.scale >= 1 ? 0 : 1;
  return `${fmt(decodeRaw(register, raw) * register.scale, digits)} ${register.scaled_unit ?? ""}`;
}

function formatValue(register: Register, raw: number | undefined): string {
  if (raw == undefined) {
    return "--";
  }
  const scaled = scaledText(register, raw);
  if (scaled != undefined) {
    return `${raw} (${scaled})`;
  }
  const value = decodeRaw(register, raw);
  return value === raw ? String(raw) : `${raw} (${value} ${register.unit})`;
}

/** Editable register value. Commits on Enter or blur, reverts on Escape. */
function RegisterField({
  register,
  value,
  onWrite,
}: {
  register: Register;
  value: number | undefined;
  onWrite: (value: number) => Promise<void>;
}): ReactElement {
  const shown = value == undefined ? "" : String(value);
  const [text, setText] = useState(shown);
  const [editing, setEditing] = useState(false);
  const [busy, setBusy] = useState(false);

  useEffect(() => {
    if (!editing) {
      setText(shown);
    }
  }, [shown, editing]);

  const commit = async () => {
    setEditing(false);
    const parsed = parseInt(text, 10);
    if (Number.isNaN(parsed) || parsed === value) {
      setText(shown);
      return;
    }
    setBusy(true);
    await onWrite(parsed);
    setBusy(false);
  };

  return (
    <input
      className={`reg-input${busy ? " is-busy" : ""}`}
      type="text"
      inputMode="numeric"
      value={text}
      disabled={busy}
      title={`${register.vmin}..${register.vmax}`}
      onFocus={() => {
        setEditing(true);
      }}
      onChange={(e) => {
        setText(e.target.value);
      }}
      onBlur={() => void commit()}
      onKeyDown={(e) => {
        if (e.key === "Enter") {
          e.currentTarget.blur();
        } else if (e.key === "Escape") {
          setEditing(false);
          setText(shown);
        }
      }}
    />
  );
}

function RegisterRow({
  register,
  value,
  editable,
  onWrite,
}: {
  register: Register;
  value: number | undefined;
  editable: boolean;
  onWrite: (value: number) => Promise<void>;
}): ReactElement {
  const scaled = value == undefined ? undefined : scaledText(register, value);
  return (
    <div
      className={`reg reg-${register.kind}${editable ? " reg-editable" : ""}`}
      title={register.note ?? undefined}
    >
      <span className="reg-addr">
        0x{register.address.toString(16).toUpperCase().padStart(2, "0")}
      </span>
      <span className="reg-label">
        {register.label}
        {register.note != undefined && (
          <i className="reg-note-dot" aria-hidden="true">
            {" "}
            ·
          </i>
        )}
      </span>
      {editable ? (
        <RegisterField register={register} value={value} onWrite={onWrite} />
      ) : (
        <span className="reg-value">{formatValue(register, value)}</span>
      )}
      <span className="reg-unit">{editable && scaled != undefined ? scaled : register.unit}</span>
    </div>
  );
}

function LockControl({
  locked,
  busy,
  onToggle,
}: {
  locked: boolean | null;
  busy: boolean;
  onToggle: (locked: boolean) => void;
}): ReactElement {
  if (locked == undefined) {
    return <Chip tone="muted">lock unknown</Chip>;
  }
  return (
    <>
      <Chip
        tone={locked ? "muted" : "warn"}
        title={
          locked
            ? "register 0x37 = 1: the servo rejects EEPROM writes"
            : "register 0x37 = 0: EEPROM writes are accepted"
        }
      >
        EEPROM {locked ? "locked" : "UNLOCKED"}
      </Chip>
      <button
        type="button"
        className={`btn ${locked ? "btn-primary" : "btn-warn"}`}
        disabled={busy}
        title={
          locked
            ? "clear the write lock so EEPROM registers can be edited"
            : "restore the write lock"
        }
        onClick={() => {
          onToggle(!locked);
        }}
      >
        {locked ? "unlock" : "lock"}
      </button>
    </>
  );
}

export function MemoryPanel({
  memory,
  groups,
  registers,
  writable,
  torqueOn,
  onWrite,
  onLock,
}: {
  memory: Memory;
  groups: { key: string; label: string }[];
  registers: Register[];
  writable: boolean;
  torqueOn: boolean;
  onWrite: (register: Register, value: number) => Promise<void>;
  onLock: (locked: boolean) => Promise<void>;
}): ReactElement {
  const [lockBusy, setLockBusy] = useState(false);
  const values = memory.values ?? undefined;
  const fresh = values != undefined && memory.age != undefined && memory.age < 3;
  const locked = memory.locked;

  // Mirrors the firmware's own guards, so a row that cannot be written looks
  // that way instead of failing on submit.
  const canWrite = (register: Register) =>
    writable &&
    register.writable &&
    values != undefined &&
    register.kind === "eeprom" &&
    locked === false &&
    !torqueOn;

  const byGroup = new Map<string, Register[]>();
  for (const register of registers) {
    const rows = byGroup.get(register.group) ?? [];
    rows.push(register);
    byGroup.set(register.group, rows);
  }

  let blockedReason: string | undefined;
  if (values != undefined) {
    if (torqueOn) {
      blockedReason =
        "Torque is on for this servo - EEPROM writes are refused while it is holding.";
    } else if (locked !== false) {
      blockedReason = "EEPROM is locked. Unlock to edit persistent registers.";
    }
  }

  let readChip: string;
  if (values == undefined) {
    readChip = "no read yet";
  } else {
    readChip = fresh ? `read ${fmt(memory.age, 1)}s ago` : "stale";
  }

  return (
    <section className="memory">
      <header className="memory-head">
        <div>
          <h2>Register map</h2>
          <p className="memory-sub">
            bus {memory.bus_id} · {memory.label}
          </p>
        </div>
        <div className="memory-status">
          {memory.error != undefined && memory.error !== "" ? (
            <Chip tone="danger">{memory.error}</Chip>
          ) : (
            <Chip tone={fresh ? "ok" : "muted"}>{readChip}</Chip>
          )}
          <LockControl
            locked={locked}
            busy={lockBusy}
            onToggle={(next) => {
              setLockBusy(true);
              void onLock(next).finally(() => {
                setLockBusy(false);
              });
            }}
          />
        </div>
      </header>

      {memory.faults.length > 0 && (
        <div className="memory-faults">
          {memory.faults.map((f) => (
            <Chip key={f} tone="danger">
              {f}
            </Chip>
          ))}
        </div>
      )}

      {blockedReason != undefined && <p className="hint memory-note">{blockedReason}</p>}

      {values == undefined && (memory.error == undefined || memory.error === "") && (
        <p className="hint memory-empty">Waiting for the first register read…</p>
      )}

      <div className="memory-groups">
        {groups.map((group) => {
          const rows = byGroup.get(group.key);
          if (!rows || rows.length === 0) {
            return undefined;
          }
          const kind = rows[0]!.kind;
          return (
            <article className={`reg-group reg-group-${kind}`} key={group.key}>
              <header>
                <h3>{group.label}</h3>
                <Chip tone={kind === "eeprom" ? "muted" : "accent"}>{KIND_LABEL[kind]}</Chip>
              </header>
              <div className="reg-list">
                {rows.map((register) => (
                  <RegisterRow
                    key={register.field}
                    register={register}
                    value={values?.[register.field]}
                    editable={canWrite(register)}
                    onWrite={async (v) => {
                      await onWrite(register, v);
                    }}
                  />
                ))}
              </div>
            </article>
          );
        })}
      </div>

      <p className="hint memory-foot">
        EEPROM has a finite write endurance, so each field writes only on Enter or blur - never per
        keystroke. Every write is verified by read-back on the module; a value that snaps back was
        refused by the servo.
      </p>
    </section>
  );
}
