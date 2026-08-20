# ur-robot control-port lock-scope audit (plan B6)

Comparison target: `rtde_control_driver.cpp` `RTDEControl::poll()`
(:245-322) and its write handlers, which run device I/O under the one
asyn port lock. Our port splits that lock in two — the asyn parameter
table lock inside `PortDriverBase`/`PortHandle`, and the device mutex
`ControlDriver::inner` — so the audit question is whether any `inner`
hold exceeds the C port-lock hold over the same I/O.

Findings, per hold site in `drivers/control.rs`:

- Poll (`spawn_poll`): `inner` is held across `poll_once` only — the
  `isSteady` handshake, the motion state machine and the custom-script
  poll, exactly the I/O C holds its lock over. The parameter publish
  (`set_params_and_notify_blocking`) runs after the guard drops,
  whereas C's `callParamCallbacks()` still holds the port lock:
  narrower than C, not wider.
- The longest hold is the `isSteady` command handshake, bounded by the
  3 s ready wait; C holds its lock over the identical call. C skips
  `isSteady` while a custom script runs (the script that would answer
  the handshake has been displaced) — ported at `poll_once`
  (`inner.custom_script.is_some() → false`).
- Write handlers (`write_int32`/`write_float64` device arms,
  `run_custom_script`): `inner` held across `setTcp`, stops, teach
  mode, jog, the custom-script upload — all I/O C performs under the
  port lock in the same handlers. Parity.

No hold interval exceeds C, so per the plan rule ("측정 근거 없는
재구조화는 하지 않음") no code change. Follow-up candidates only if
measurement ever shows write-latency pressure: the `isSteady` poll
hold is the interval to shorten first.
