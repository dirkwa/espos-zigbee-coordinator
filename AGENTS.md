# espos-zigbee-coordinator

Zigbee coordinator firmware on espOS for Zigbee2MQTT. An ESP32-C5 or C6 runs
the Zigbee stack on its own 802.15.4 radio and serves the ZBOSS NCP protocol,
which Zigbee2MQTT's `zboss` adapter drives, over TCP on the boat's Wi-Fi.
Wi-Fi, config, the web UI, mDNS and OTA are espOS (the `espos/` submodule);
the NCP is `components/zboss_ncp`, vendored from busware's esp-coordinator
(`UPSTREAM.md`).

## Rules

- **Fixes to WiFi, config, OTA, mDNS or the web UI belong in espOS**, with a
  host test where the logic is testable. Merge there first, then bump the
  submodule here. A workaround here names the espOS issue and goes when the
  bump lands.
- **The coordinator decodes nothing.** Devices, their values and Signal K
  paths are Zigbee2MQTT's and the Signal K plugin's.
- **The NCP never touches espOS's `nvs`.** It keeps its data in `zb_nvs`,
  `zb_storage` and `zb_fct`. A network backup exports `zb_nvs` and a restore
  overwrites it, so espOS's `nvs` (Wi-Fi, the Signal K token, the keys) must
  never be what those commands read or write.
- **Changes to `components/zboss_ncp` stay close to upstream.** Fix protocol
  bugs upstream as well, note every local change in `UPSTREAM.md`, and keep
  upstream's comments where the code is upstream's.
- **One host at a time.** The NCP has one link state. A connection becomes
  the host only once it sends a ZBOSS frame, and the switch happens on the
  app task, where the link reset, the route change and dropping the previous
  host's frames are one step (`transport.h` has the sequence). Do not let a
  bare connect replace a live host.
- **Partition tables are fixed at the first USB flash.** OTA never rewrites
  them, so a change reaches only newly flashed boards. The coordinator's
  partitions are the same size in every table.
- **Build through `scripts/build.sh`** (espOS's locked, nice'd wrapper), one
  build directory per target. Zero warnings.
- **Never commit boat configuration** (WiFi credentials, server addresses) or
  the signing key. Site config lives in NVS.

## Conventions

- Angular conventional commits and PR titles, `type(scope): subject`, subject
  lower case, at most 50 characters. PR titles become the release notes:
  release-please uses the notes GitHub generates, each PR with its author and
  link, sorted by the label `label-by-title.yml` sets from the title's type
  (`.github/release.yml`). Only feat, fix, perf, refactor, revert, a breaking
  change or a `Release-As:` footer refreshes the release PR; after a change
  to the release configuration, run `release-please.yml` by hand.
- Branch names `feat/…`, `fix/…`, `docs/…`; never commit to `main`.
- No AI attribution anywhere: no Co-Authored-By, no "Generated with".
- No version bumps or tags by hand; release-please cuts releases.
- Squash-merge every PR, the release PR included.
- Comments say why, not what.
