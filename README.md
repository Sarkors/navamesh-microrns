# navamesh-microrns

Experimenting with running microRNS on a RAK4631 to power-gate a more power-hungry radio using MPPT solar management.

## What's going on here

Part of the NavaMesh project — a mesh sensor network for agricultural monitoring in the Navajo region. The field nodes run on solar and LiPo, so keeping power draw low matters a lot.

The idea was to have the RAK4631 (nRF52840 + SX1262 LoRa) sit at ~5-10mA on the always-on rail, listening for a Reticulum command over LoRa. When it gets the right message, it pulls a GPIO pin high which enables the DFRobot Solar Manager's switched 5V rail, booting the Heltec WiFi HaLow dongle only when actually needed. HaLow peaks around 800mA so keeping it gated saves a lot on the energy budget.

The goal was to let the farmer send that trigger through Sideband instead of needing two separate apps — one Meshtastic to wake the radio and one Sideband for the actual data request.

## What actually works

- microReticulum running on the RAK4631 — announces, path routing, transport, the whole stack
- Bidirectional LoRa comms confirmed with Sideband via RNode
- Encrypted packet delivery end-to-end (X25519 key exchange, AES decrypt, IO1 trigger fires)
- LXMF-compatible announce with display name so the node shows up correctly in Reticulum tooling
- LXMF message parser on the RAK side (msgpack + signature strip) to handle Sideband messages
- Identity persists across reboots via LittleFS so the destination hash stays stable
- Hardware power gate working — IO1 → Solar Manager EN → HaLow boots

## The wall we hit

LoRa and WiFi HaLow are completely different PHY layers. Even though they both operate around 915MHz, the SX1262 can only demodulate LoRa chirp signals — a HaLow radio's 802.11ah OFDM transmissions are just noise to it. Bridging them would require an intermediate node with both radios active simultaneously, which starts looking like an SDR problem.

So the farmer still needs a LoRa RNode to talk to the RAK. The goal of cutting down to just one radio on the farmer's end isn't fully there yet.

## Hardware

| Part | Role |
|---|---|
| WisBlock RAK4631 (nRF52840 + SX1262) | Main controller, Reticulum LoRa node |
| DFRobot Solar Power Manager 5V V1.1 | MPPT solar + LiPo, switched 5V rail |
| Heltec WiFi HaLow Dongle V2 | The power-hungry radio being gated |
| RAKBox-UO150x100x45-Solar | Enclosure + 1.9W panel |

## Dependencies

Built with PlatformIO. Pulls in:
- [microReticulum](https://github.com/attermann/microReticulum) — C++ port of the Reticulum Network Stack
- [RadioLib](https://github.com/jgromes/RadioLib) — SX1262 radio driver
- [microStore](https://github.com/attermann/microStore) — filesystem abstraction for identity persistence
- ArduinoJson, MsgPack, Crypto (attermann fork)

The `lib/LoRaInterface/` folder is a local copy of the LoRaInterface from the microReticulum examples, adapted for the RAK4631's SX1262 pinout.

## Status

Proof of concept. Not going forward as the primary deployment strategy for NavaMesh right now, but keeping it here as a reference for anyone trying to get microReticulum running on nRF52 or the RAK4631 specifically — there isn't much out there on that combination.
