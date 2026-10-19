/** Causal selection for both streamed runs and arbitrary replay scrubbing. */
export function mpcAtTime(run, frame, time) {
  const solves = run.summary?.mpc_solves ?? [];
  const solve = solves.findLast(
    (s) => s.t <= time && (frame.mpc_id == null || s.id <= frame.mpc_id),
  );
  const owner = frame.controller ?? "";
  const active = !!solve?.success && owner.startsWith("ROADSTER SCvx MPC");
  return {
    solve,
    active,
    state: !run.config.use_mpc
      ? "disabled"
      : !solve
        ? "waiting"
        : !solve.success
          ? "rejected"
          : active
            ? "active"
            : "handoff",
    age: solve ? Math.max(0, time - solve.t) : 0,
    delivered: frame.delivered_force ?? [0, 0, 0],
    actual: solve
      ? run.frames.filter((f) => f.t >= solve.t && f.t <= time)
      : [],
  };
}
