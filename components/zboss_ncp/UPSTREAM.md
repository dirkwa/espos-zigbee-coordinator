# Upstream

The ZBOSS NCP serial protocol implementation in this component comes from
[tostmann/esp-coordinator](https://github.com/tostmann/esp-coordinator)
(busware.de), the maintained fork of
[andryblack/esp-coordinator](https://github.com/andryblack/esp-coordinator),
at commit `b292868d1ae5426dc048ee7749d7a710e1ccd789` (2026-10-19). It is
MIT-licensed; `LICENSE` here is upstream's and covers the files that came
from it.

Copied from upstream's `main/`, then changed for espOS (see the git history
of this directory for the exact diff):

- **Host link over TCP, not USB or UART.** `transport` serves one client on
  a TCP port (`CONFIG_ZBOSS_NCP_TCP_PORT`, 6638 by default, as upstream's
  Wi-Fi build). espOS brings up Wi-Fi, so the USB-Serial/JTAG and UART
  backends are gone.
- **Its own NVS partition.** espOS keeps Wi-Fi credentials, the Signal K
  token and the device keys in `nvs`. Upstream's raw network backup reads
  and restores the whole `nvs` partition, and its factory reset erases it.
  Here both use `zb_nvs`, a 24 KB partition of the coordinator's own, the
  size of upstream's `nvs`, so a backup image keeps upstream's layout.
- **espOS owns start-up.** NVS, logging and the restart path are espOS's;
  `zboss_ncp_start()` runs the NCP once espOS is up. Upstream's board
  quirks (the Seeed XIAO antenna switch) are left out.
- **Version** comes from the app description rather than upstream's
  `version.txt`.
