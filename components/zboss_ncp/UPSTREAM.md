# Upstream

The ZBOSS NCP serial protocol implementation in this component comes from
[tostmann/esp-coordinator](https://github.com/tostmann/esp-coordinator)
(busware.de), the maintained fork of
[andryblack/esp-coordinator](https://github.com/andryblack/esp-coordinator),
at commit `b292868d1ae5426dc048ee7749d7a710e1ccd789` (2026-10-19). It is
MIT-licensed; `LICENSE` here is upstream's and covers the files that came
from it.

Copied from upstream's `main/`, then changed for espOS. Every local change:

- **Host link over TCP, not USB or UART** (`transport`). One host on
  `CONFIG_ZBOSS_NCP_TCP_PORT` (6638, as upstream's Wi-Fi build). Every
  connection is sent the boot frames on accept and becomes the host once it
  sends a ZBOSS frame header with a valid CRC, as upstream's DE AD gate does
  for its interfaces; up to three connections wait at once. The switch is one
  app-task event (`EVENT_LINK_RESET`, tagged with a generation): the link
  state is reset, the route moves, and frames stamped for the previous host
  are dropped. Sends are bounded at 8 s per frame in 0.5 s slices.
- **NCP-to-host frames in a message buffer**, not a stream buffer, and each
  `EVENT_INPUT` drains all of them. A frame whose event was dropped on a full
  queue then goes out with the next drain instead of shifting every frame
  after it.
- **Link state reset** (`protocol::reset_link_state`): sequence numbers and
  the receive buffer back to boot state for each new host.
- **Boot frames per host.** Upstream sends the framing ACK and the
  boot-ready frame once at boot; here every connection gets them, the
  boot-ready frame only once the stack has loaded its NVRAM.
  `zb_ncp::send_boot_ready_frame` raises a request the app task sends from
  (`EVENT_BOOT_READY`, and polled in case that event is dropped), so each
  connection gets it exactly once.
- **Restarts send the host a FIN.** Every upstream `esp_restart()` is
  `transport::restart()`.
- **Its own NVS partition.** espOS keeps Wi-Fi credentials, the Signal K
  token and the device keys in `nvs`. Upstream's raw network backup reads
  and restores the whole `nvs` partition, and its factory reset erases it.
  Here those, the `restore_pend` namespace and the NVS init all use
  `zb_nvs`, a 24 KB partition of the coordinator's own, the size of
  upstream's `nvs`, so a backup image keeps upstream's layout.
- **espOS owns start-up.** Upstream's `main.cpp` is `ncp_boot.cpp`:
  `zboss_ncp_start()` replaces `app_main`, the event loop runs on its own
  `ncp_app` task, and a failed start disarms the boot guard rather than
  restart-looping. Upstream's board quirk (the Seeed XIAO antenna switch) is
  left out.
- **Version** comes from the app description rather than upstream's
  `version.txt`.
- **Green Power handler kept in the link** (`CMakeLists.txt`): an
  `INTERFACE` link option `-Wl,--undefined=zb_zcl_green_power_cluster_handler`.
  `libzboss_stack` calls back into `libesp_zb_api` for it, and in IDF 6's
  link order the linker has already passed `libesp_zb_api` by then. Upstream
  builds on IDF 5 without it.
