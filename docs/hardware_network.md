# Hardware and Network

The physical nodes, the sensor-model distinction that matters for expansion,
campus networking and the durable fix for it, clock synchronisation, and the
NAS with its safety rules.

## Nodes

| Node | Host | IP | Sensor | Status |
|------|------|----|--------|--------|
| node1 | jetson-nano-01 | 172.26.42.167 | Livox MID-360 | Deployed, reference frame |
| node3 | jetson-nano-03 | 172.26.165.5 | Livox MID-360 | Deployed, calibrated to node1 |
| node2 | — | 172.26.42.1 | — | Config placeholder, not deployed |
| fused | (virtual) | via node1 | — | Virtual node, no hardware |

node1 and node3 are the two LiDARs in the current fused setup. node2 exists in
`nodes_config.yaml` as a partially-filled placeholder and is not deployed; do not
assume its values are correct until it is actually brought up. `fused` is
virtual and has no hardware of its own — see [architecture.md](architecture.md).

The Jetson login user is set per node in the config (`jetson_user`). The Jetsons
run the LiDAR driver, statistical background removal, and a rosbridge WebSocket
server, and nothing heavier — all inference is on the workstation.

## MID-360 vs MID-360S — read before adding nodes

The current nodes use the Livox **MID-360**. Planned expansion adds four
**MID-360S** sensors. **These are different sensor models and are not
drop-in interchangeable.** The `lidar_type` and the launch file differ between
them, so you cannot copy a MID-360 node's launch configuration for a MID-360S and
expect it to work.

When adding the MID-360S nodes:

- Use the MID-360S launch file and `lidar_type`, not node1/node3's MID-360 ones
  (node1 uses `msg_MID360_launch.py`; the 360S needs its own).
- Host-IP configuration for the MID-360S requires the newer **Livox Viewer 2.5.9**
  — the older 2.3.0 does not work for the 360S.

This distinction is the single biggest trap in scaling the node count, which is
why the planned "one-click LiDAR config script" must branch on sensor model
rather than assume one type. That script is intentionally deferred until the
manual process of bringing up a third and fourth node has been done by hand
enough times to be sure the steps are right.

## Campus networking and Tailscale

The Jetsons connect over CMU campus WiFi. Two practical problems shape the
networking:

- **You cannot set a static IP on the campus network**, and campus **DHCP
  reassigns addresses**, so a Jetson's IP drifts over time. node1 has dropped off
  the network this way mid-session.
- The fix on the campus side is to register each Jetson on **CMU-DEVICE** by MAC
  (node1's MAC `9cc7d3f6b407` is registered) rather than relying on the general
  CMU-SECURE association.

Because campus IPs are not stable, the durable fix is **Tailscale**: it gives each
machine a stable address regardless of what the campus DHCP hands out. The
workstation is on Tailscale at `100.113.199.85`. Rolling Tailscale out to every
Jetson — together with the CMU-DEVICE registration and WiFi auto-reconnect — is
the planned path to network robustness, so that an IP change or a brief WiFi drop
no longer breaks a session. Until that is in place, a node going unreachable will
fail fused operations by design; see the troubleshooting notes in
[operations.md](operations.md).

## Clock synchronisation

Fusing two LiDARs requires their timestamps to agree, so the nodes are
synchronised with **chrony**: **node1 acts as the time server, and node3 syncs to
node1**, giving sub-millisecond agreement (< 1 ms RMS).

The lesson behind this arrangement: **"each node is synced" is not the same as
"the nodes are synced to each other."** If each Jetson independently syncs to
whatever NTP source it can reach, they can drift relative to one another even
while both report being in sync. Chaining node3 to node1 as a common reference is
what actually guarantees the two clocks agree.

## The NAS

Recorded bags, tracklet CSVs, and background models are archived to a **Synology
DS1525+** at **172.24.72.224**, pushed over rsync. Authentication uses `sshpass`
with the password stored in `~/.nas_password` (keep it `chmod 600`); the archiver
is a no-op when that file is absent, so the pipeline runs fine without NAS access
configured.

### NAS safety rule — do not violate

A Synology holding millions of files freezes for roughly half an hour if asked to
walk a large directory. **Never run `ls`, `find`, `du`, `wc`, or `df` against NAS
directories.** This has caused ~30-minute system freezes.

Only these operations are safe against the NAS:

- `stat` on a **named** file path
- `test -d` / `test -f` on a named path
- `mkdir -p`
- `df` on the `/volume1` **mount point only** (not on directories under it)
- directory listing via **`os.scandir` pushed over SSH** — the only safe way to
  list a directory's contents
- **rsync push, local → NAS only**

The archiver scripts (`nas_archive.py`, `post_record_hook.py`) already restrict
themselves to these operations. Do not add ad-hoc NAS shell commands to any
script, and do not "just check" a NAS directory with `ls` — use `os.scandir` over
SSH if you must list it.

A known wrinkle: write permission. Confirm you are writing somewhere you actually
have permission (a personal home area under `/volume1/homes/…` is writable) rather
than the volume root, and resolve a shared-folder target with Kieran for the
long-term archive location rather than relying on a personal-home path.
