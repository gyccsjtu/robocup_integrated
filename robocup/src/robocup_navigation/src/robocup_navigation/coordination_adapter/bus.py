"""Transport-independent plumbing around the Codex coordination core.

Deliberately free of rospy so the integration logic is host-testable without a
running ROS graph; ``node.py`` is the thin ROS wrapper over this module.

Two contracts from interface v0 are enforced here and must not be "optimised":

  * Single writer.  ``Coordinator`` is a single-writer state machine; every
    input goes through :meth:`CoreBus.submit` from ONE thread.
  * Full-stream fan-out.  Interface v0 (executor handshake #2): each
    :class:`ExecutorGate` must receive the COMPLETE coordinator output stream,
    including messages addressed to *other* UAVs.  Filtering per UAV before the
    gate makes its sequence check reject a gap.
"""
from ..coordination import Coordinator
from ..coordination.executor import ExecutorGate
from ..coordination.protocol import CoordinationError

# source_id the core stamps on every output; gates only accept this.
COORDINATOR_SOURCE = "coordinator_commands"


class InputSeq:
    """Monotonic per-source input sequence within one run (interface v0 §1).

    ``seq`` counts over the whole run *per producer*, not per topic, so a UAV's
    VEHICLE_STATE and COMMAND_ACK share one counter.
    """

    def __init__(self):
        self._n = {}

    def next(self, source_id):
        n = self._n.get(source_id, 0) + 1
        self._n[source_id] = n
        return n


class CoreBus:
    """Single-writer front end for :class:`Coordinator`."""

    def __init__(self, run_id, fleet_ids, tasks, limits, roles, test_case_id,
                 start_sim_s=0.0, start_wall_s=0.0):
        self.run_id = run_id
        self.fleet_ids = list(fleet_ids)
        self.core = Coordinator(run_id, self.fleet_ids, tasks, limits, roles,
                                test_case_id, start_sim_s=start_sim_s,
                                start_wall_s=start_wall_s)
        self.input_seq = InputSeq()

    def submit(self, source_id, kind, data, sim_s, wall_s):
        """Send one input message; return the core's freshly produced outputs.

        The core stamps its own output envelopes (source_id=COORDINATOR_SOURCE
        and a monotonic ``seq``), so outputs are passed through untouched.
        """
        msg = dict(schema_version=1, run_id=self.run_id, source_id=source_id,
                   seq=self.input_seq.next(source_id), sim_s=sim_s, kind=kind, data=data)
        self.core.receive(msg, wall_s)
        return self.core.drain_outputs()

    def drain_outputs(self):
        return self.core.drain_outputs()

    def drain_events(self):
        return self.core.drain_events()

    def finish(self, monitor_report=None):
        return self.core.finish(monitor_report)

    def snapshot(self):
        return self.core.snapshot()


class GateFanout:
    """Broadcast the complete coordinator output stream to every gate."""

    def __init__(self, run_id, fleet_ids, start_tolerance_m, heartbeat_timeout_s):
        self.run_id = run_id
        self.gates = {u: ExecutorGate(run_id, u, start_tolerance_m, heartbeat_timeout_s)
                      for u in fleet_ids}

    def deliver(self, commands, sim_s, wall_s, map_revision, position_of):
        """Feed one batch of outputs to all gates.

        ``position_of(uav_id)`` -> current world_xyz of that UAV.
        Returns ``{uav_id: [(kind, action_or_error), ...]}`` where each entry is
        either ``("ROUTE_GRANT", grant_dict)``, a stop request's kind with its
        data, or ``("GATE_ERROR", message)`` (fail closed for that UAV).
        Callers must pair an action with the kind it came from: the gate returns
        the message *data*, not the envelope.
        """
        out = {u: [] for u in self.gates}
        for m in commands:
            for u, gate in self.gates.items():
                try:
                    act = gate.receive(m, sim_s, wall_s, map_revision, position_of(u))
                except CoordinationError as exc:
                    out[u].append(("GATE_ERROR", str(exc)))
                    continue
                if act is not None:
                    out[u].append((m["kind"], act))
        return out

    def check(self, sim_s, wall_s, map_revision):
        """Call every executor tick -- not only when a network packet arrives."""
        result = {}
        for u, gate in self.gates.items():
            try:
                result[u] = gate.check(sim_s, wall_s, map_revision)
            except CoordinationError as exc:
                result[u] = ("GATE_ERROR", str(exc))
        return result

    def confirm_stopped(self, uav_id):
        """Call only after a MEASURED stop; this releases no resource."""
        self.gates[uav_id].confirm_stopped()
