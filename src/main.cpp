// main.cpp — NavaMesh Reticulum Gatekeeper
//
// Replaces the Meshtastic GatekeeperModule with a Reticulum-native equivalent.
// The farmer carries one device running Sideband. A Sideband message over LoRa
// (via an RNode) reaches this node, which then gates the Heltec HaLow dongle
// via the DFRobot Solar Manager EN pin — exactly as the Meshtastic version did.
//
// What changed vs GatekeeperModule:
//   TRIGGER:  Meshtastic Channel 2 text packet → Reticulum Destination packet
//   LIBRARY:  Meshtastic stack          → microReticulum + RadioLib
//   RADIO:    Meshtastic LoRa config    → Reticulum LoRa config (via LoRaInterface)
//
// What did NOT change:
//   EN pin wiring, IO1, DFRobot Solar Manager — all identical
//   activateHeltec() / deactivateHeltec() — moved over unchanged
//   5-minute watchdog logic — identical

// ── Includes ──────────────────────────────────────────────────────────────────

// microStore filesystem abstraction — needed to persist the Identity keypair
// to flash so the node keeps the same Reticulum address across reboots.
#include <microStore/FileSystem.h>
#include <microStore/Adapters/UniversalFileSystem.h>

// LoRaInterface lives in lib/LoRaInterface/ — our local copy from the examples.
// It wraps RadioLib's SX1262 driver and presents it to microReticulum as an
// Interface the stack can send and receive packets through.
#include <LoRaInterface.h>

// Core microReticulum classes
#include <Reticulum.h>    // the stack itself
#include <Identity.h>     // cryptographic keypair — the node's permanent address
#include <Destination.h>  // named, addressable endpoint (what Sideband dials)
#include <Transport.h>    // routes packets between interfaces
#include <Packet.h>       // a single Reticulum packet
#include <Log.h>          // INFOF / ERRORF macros (serial output)
#include <Bytes.h>        // Reticulum's byte-buffer type
#include <Type.h>         // enums and constants
#include <Utilities/OS.h> // time helpers

// Arduino / nRF52 specifics
#include <Arduino.h>
#include <Adafruit_TinyUSB.h>  // required on nRF52 for USB serial to work

// LittleFS access for identity persistence.
// InternalFileSystem is a global provided by the Adafruit nRF52 BSP.
// microStore already calls InternalFS.begin() during filesystem.init(),
// so it is safe to use InternalFS directly after that call.
#include <InternalFileSystem.h>
using namespace Adafruit_LittleFS_Namespace;

// ── Pin and timing constants ───────────────────────────────────────────────────
//
// These are identical to GatekeeperModule.h. IO1 on the RAK4631 is pin 17
// in the Adafruit nRF52 BSP — it controls the Solar Manager EN pin, which
// switches the 5V rail that powers the Heltec HaLow dongle.

#ifndef WB_IO1
#define WB_IO1 17
#endif
#define GATEKEEPER_EN_PIN   WB_IO1

// Auto-shutoff: cut HaLow power after 5 minutes with no "Power Off" received.
// Protects the energy budget if the farmer forgets to shut down manually.
#define HELTEC_MAX_ON_MS    (5UL * 60UL * 1000UL)

// ── Command strings ────────────────────────────────────────────────────────────
//
// The farmer sends these as plain UTF-8 text via Sideband.
// Keeping them identical to the Meshtastic version means the same vocabulary
// works regardless of which radio the farmer uses to send the command.
#define CMD_POWER_ON    "Power On"
#define CMD_POWER_OFF   "Power Off"

// ── App identity strings ───────────────────────────────────────────────────────
//
// LXMF is the messaging layer Sideband uses. Announcing as "lxmf"/"delivery"
// is what makes this node show up in Sideband's contact list. Any node
// announcing under this namespace is treated as a messageable peer.
//
// DISPLAY_NAME is packed into the announce app_data as msgpack so Sideband
// shows a human-readable name instead of a raw destination hash.
//
// LXMF_SIG_LEN: every LXMF message ends with a 64-byte Ed25519 signature.
// We strip this before parsing the msgpack payload.
#define APP_NAME        "lxmf"
#define APP_ASPECT      "delivery"
#define DISPLAY_NAME    "NavaMesh Gatekeeper"
#define LXMF_SIG_LEN    64

// ── State ─────────────────────────────────────────────────────────────────────
//
// Identical to GatekeeperModule's private member variables, just as globals here.
static bool     _heltecOn      = false;
static uint32_t _activatedAtMs = 0;

// ── Reticulum objects ──────────────────────────────────────────────────────────
//
// These are declared with Type::NONE initially — they're empty shells until
// reticulum_setup() fills them in. This is microReticulum's pattern for
// declaring objects at global scope before they're fully initialized.
static RNS::Reticulum   reticulum({RNS::Type::NONE});
static RNS::Interface   lora_interface({RNS::Type::NONE});
static RNS::Identity    identity({RNS::Type::NONE});
static RNS::Destination destination({RNS::Type::NONE});

// ── Power control ─────────────────────────────────────────────────────────────
//
// These two functions are taken directly from GatekeeperModule.cpp.
// The only difference: LOG_INFO → INFOF (microReticulum's logging macro).
// The hardware logic — IO1 HIGH/LOW, _heltecOn flag, _activatedAtMs — is
// completely unchanged.

void activateHeltec() {
    if (_heltecOn) {
        // Already on — reset the timer so a repeat "Power On" extends
        // the on-time window rather than being silently ignored.
        INFO("Gatekeeper: Heltec already on, timer reset");
        _activatedAtMs = millis();
        return;
    }

    // IO1 HIGH → EN pin HIGH → Solar Manager enables 5V rail →
    // Heltec receives power via USB-C pigtail.
    digitalWrite(GATEKEEPER_EN_PIN, HIGH);
    _heltecOn      = true;
    _activatedAtMs = millis();

    INFOF("Gatekeeper: EN HIGH — Heltec ON (auto-off in %lu s)",
          (unsigned long)(HELTEC_MAX_ON_MS / 1000));
}

void deactivateHeltec() {
    if (!_heltecOn) {
        INFO("Gatekeeper: Heltec already off");
        return;
    }

    // IO1 LOW → EN LOW → 5V rail cut → Heltec loses power.
    // Hard power cut is intentional — the HaLow dongle has no OS to corrupt.
    digitalWrite(GATEKEEPER_EN_PIN, LOW);
    _heltecOn = false;

    uint32_t onTimeSec = (millis() - _activatedAtMs) / 1000;
    INFOF("Gatekeeper: EN LOW — Heltec OFF (was on %lu s)",
          (unsigned long)onTimeSec);
}

// ── LXMF message parsing ──────────────────────────────────────────────────────
//
// An LXMF message is a msgpack array of 5 elements:
//   [0] msgid     — 16 random bytes, unique message identifier
//   [1] timestamp — Unix time as float or int
//   [2] title     — bytes (UTF-8 string, may be nil/empty)
//   [3] content   — bytes (UTF-8 string) ← THIS is what the farmer typed
//   [4] fields    — dict of metadata, or nil
// Followed by a 64-byte Ed25519 signature appended after the msgpack.
//
// We only care about element [3]. The strategy is to skip [0], [1], [2],
// then read [3]. Each msgpack type has a known byte layout so we can
// advance through the buffer without needing a full msgpack library call.

// Advance pos past one msgpack element. Returns false if data is malformed.
static bool lxmf_skip(const uint8_t* buf, size_t len, size_t& pos) {
    if (pos >= len) return false;
    uint8_t b = buf[pos++];

    // Single-byte values: nil, bool, positive fixint, negative fixint
    if (b == 0xc0 || b == 0xc2 || b == 0xc3) return true;
    if (b <= 0x7f || b >= 0xe0)               return true;

    // fixstr: top 3 bits = 101, bottom 5 bits = length
    if ((b & 0xe0) == 0xa0) { pos += (b & 0x1f); return pos <= len; }

    switch (b) {
        case 0xca: pos += 4; break;  // float32
        case 0xcb: pos += 8; break;  // float64
        case 0xcc: pos += 1; break;  // uint8
        case 0xcd: pos += 2; break;  // uint16
        case 0xce: pos += 4; break;  // uint32
        case 0xcf: pos += 8; break;  // uint64
        case 0xd0: pos += 1; break;  // int8
        case 0xd1: pos += 2; break;  // int16
        case 0xd2: pos += 4; break;  // int32
        case 0xd3: pos += 8; break;  // int64
        // bin8 / str8: next byte is the data length
        case 0xc4: case 0xd9:
            if (pos >= len) return false;
            pos += buf[pos] + 1;
            break;
        // bin16 / str16: next two bytes are big-endian length
        case 0xc5: case 0xda:
            if (pos + 1 >= len) return false;
            pos += 2 + ((uint16_t)buf[pos] << 8 | buf[pos + 1]);
            break;
        default: return false;  // unrecognised type
    }
    return pos <= len;
}

// Read the msgpack string/binary at pos into a std::string, advancing pos.
// Returns "" for nil or on error.
static std::string lxmf_read_string(const uint8_t* buf, size_t len, size_t& pos) {
    if (pos >= len) return "";
    uint8_t b = buf[pos++];

    size_t slen = 0;
    if      ((b & 0xe0) == 0xa0)      { slen = b & 0x1f; }          // fixstr
    else if (b == 0xd9 || b == 0xc4) {                               // str8 / bin8
        if (pos >= len) return "";
        slen = buf[pos++];
    }
    else if (b == 0xda || b == 0xc5) {                               // str16 / bin16
        if (pos + 1 >= len) return "";
        slen = (uint16_t)buf[pos] << 8 | buf[pos + 1];
        pos += 2;
    }
    else if (b == 0xc0) return "";  // nil
    else                return "";  // unexpected type

    if (pos + slen > len) return "";
    std::string result(reinterpret_cast<const char*>(buf + pos), slen);
    pos += slen;
    return result;
}

// Extract the content field from a raw LXMF packet.
// Returns "" if the packet is too short, malformed, or has an empty content.
static std::string parse_lxmf_content(const RNS::Bytes& data) {
    if (data.size() <= LXMF_SIG_LEN) return "";

    const uint8_t* buf = data.data();
    // Strip the trailing 64-byte Ed25519 signature — the msgpack ends before it
    size_t len = data.size() - LXMF_SIG_LEN;
    size_t pos = 0;

    // First byte must be 0x95: msgpack fixarray of exactly 5 elements
    if (buf[pos] != 0x95) return "";
    pos++;

    // Skip [0] msgid, [1] timestamp, [2] title
    if (!lxmf_skip(buf, len, pos)) return "";
    if (!lxmf_skip(buf, len, pos)) return "";
    if (!lxmf_skip(buf, len, pos)) return "";

    // Read [3] content — the actual text the farmer typed
    return lxmf_read_string(buf, len, pos);
}

// ── Packet callback ────────────────────────────────────────────────────────────
//
// This is the Reticulum equivalent of GatekeeperModule::handleReceived().
//
// In Meshtastic, handleReceived() was called by the mesh stack when a packet
// arrived on Channel 2 addressed to any node. In Reticulum, this callback is
// called only when a packet arrives addressed specifically to OUR Destination —
// the cryptographic addressing means filtering is automatic. We don't need to
// check the channel or ignore packets meant for others.
//
// Sideband sends LXMF messages — msgpack-encoded with a 64-byte signature.
// parse_lxmf_content() handles the unpacking. The command comparison below
// is unchanged from the original raw-packet version.

void onPacket(const RNS::Bytes& data, const RNS::Packet& packet) {
    std::string text = parse_lxmf_content(data);

    if (text.empty()) {
        INFO("Gatekeeper: malformed or empty LXMF message, ignoring");
        return;
    }

    INFOF("Gatekeeper: received '%s'", text.c_str());

    if (text == CMD_POWER_ON) {
        activateHeltec();
    } else if (text == CMD_POWER_OFF) {
        deactivateHeltec();
    } else {
        INFO("Gatekeeper: unknown command, ignoring");
    }
}

// ── Reticulum setup ────────────────────────────────────────────────────────────
//
// This runs once in setup(). It mirrors the structure of reticulum_setup()
// in the lora_announce example, adapted for the gatekeeper role.

void reticulum_setup() {
    INFO("Gatekeeper: starting Reticulum setup...");

    try {
        // 1. FILESYSTEM
        //    microStore's UniversalFileSystem adapter auto-detects LittleFS
        //    on nRF52. The filesystem stores two things persistently:
        //      - The Identity keypair (so address stays the same across reboots)
        //      - The learned path table (so routing works faster after restart)
        //    Without this, the node is effectively stateless and re-announces
        //    from scratch every boot.
        INFO("Gatekeeper: initializing filesystem...");
        microStore::FileSystem filesystem{microStore::Adapters::UniversalFileSystem()};
        filesystem.init();
        RNS::Utilities::OS::register_filesystem(filesystem);

        // 2. LORA INTERFACE
        //    LoRaInterface() constructs the RadioLib wrapper for the SX1262.
        //    Because we defined -DBOARD_RAK4631 in platformio.ini, the
        //    constructor picks the RAK4631's SPI pins and TCXO voltage.
        //
        //    MODE_FULL means this interface both sends AND receives AND
        //    forwards packets for other nodes (acts as a Reticulum relay).
        //    If you only want to receive commands and not relay traffic,
        //    change this to MODE_ACCESS_POINT.
        INFO("Gatekeeper: initializing LoRa interface...");
        lora_interface = new LoRaInterface();
        lora_interface.mode(RNS::Type::Interface::MODE_FULL);
        RNS::Transport::register_interface(lora_interface);
        lora_interface.start();

        // 3. RETICULUM STACK
        //    Reticulum() starts the transport layer. transport_enabled(true)
        //    tells it to route packets for other nodes, not just receive its own.
        INFO("Gatekeeper: starting Reticulum stack...");
        reticulum = RNS::Reticulum();
        reticulum.transport_enabled(true);
        reticulum.start();

        // 4. IDENTITY
        //    The keypair must be stable across reboots — the destination hash
        //    is derived from it, so if the keypair changes, the node gets a new
        //    address and Sideband loses track of it.
        //
        //    On first boot: RNS::Identity() generates a new 64-byte keypair.
        //    We save those bytes to LittleFS at KEY_FILE.
        //    On every subsequent boot: we load the saved bytes and restore the
        //    keypair via load_private_key(), keeping the same address forever.
        //
        //    KEY_SIZE = 64: Reticulum identities combine a 32-byte Ed25519
        //    seed (for signing) and a 32-byte X25519 key (for encryption).
        INFO("Gatekeeper: loading identity...");
        {
            const char*  KEY_FILE = "/gk_identity.bin";
            const size_t KEY_SIZE = 64;

            File keyFile(InternalFS);
            if (keyFile.open(KEY_FILE, FILE_O_READ)) {
                // Saved keypair found — restore it so the address stays the same
                uint8_t key_buf[KEY_SIZE] = {0};
                keyFile.read(key_buf, KEY_SIZE);
                keyFile.close();

                identity = RNS::Identity();
                RNS::Bytes prv;
                prv.append(key_buf, KEY_SIZE);
                identity.load_private_key(prv);
                INFO("Gatekeeper: identity loaded from storage");
            } else {
                // First boot — generate and save the keypair
                identity = RNS::Identity();
                RNS::Bytes prv = identity.get_private_key();

                if (prv.size() > 0) {
                    File saveFile(InternalFS);
                    if (saveFile.open(KEY_FILE, FILE_O_WRITE)) {
                        saveFile.write(prv.data(), prv.size());
                        saveFile.close();
                        INFO("Gatekeeper: new identity generated and saved");
                    } else {
                        INFO("Gatekeeper: WARNING — could not save identity to flash");
                    }
                }
            }
        }
        INFOF("Gatekeeper: identity hash: %s", identity.hash().toHex().c_str());

        // 5. DESTINATION
        //    A Destination is the named endpoint other nodes send packets to.
        //    RNS::Type::Destination::IN means it accepts incoming packets.
        //    SINGLE means one identity owns this address (vs GROUP destinations).
        //    "navamesh" + "gatekeeper" is the app namespace.
        //
        //    The full Reticulum address = hash(identity.public_key + "navamesh" + "gatekeeper")
        //    This is what shows up in Sideband's contact list.
        INFO("Gatekeeper: creating destination...");
        destination = RNS::Destination(
            identity,
            RNS::Type::Destination::IN,
            RNS::Type::Destination::SINGLE,
            APP_NAME,
            APP_ASPECT
        );

        // Register our packet callback. Every packet addressed to this
        // Destination will trigger onPacket(). This is the equivalent of
        // registering GatekeeperModule as a SinglePortModule in Meshtastic.
        destination.set_packet_callback(onPacket);
        destination.set_proof_strategy(RNS::Type::Destination::PROVE_ALL);

        INFOF("Gatekeeper: destination hash: %s",
              destination.hash().toHex().c_str());

        // 6. ANNOUNCE
        //    Sideband shows a display name when the announce includes msgpack
        //    app_data with a "name" key. We build this manually:
        //      0x81        fixmap, 1 key-value pair
        //      0xa4 "name" fixstr, 4 chars, key
        //      0xaN <name> fixstr, N chars, value (works for names < 32 chars)
        //    This is the minimal msgpack needed — no library required.
        INFO("Gatekeeper: announcing destination...");
        {
            const char*    name     = DISPLAY_NAME;
            const uint8_t  name_len = (uint8_t)strlen(name);
            uint8_t app_data_buf[32];
            uint8_t i = 0;
            // LXMF display name is msgpack bin8: 0xc4, length, raw UTF-8 bytes.
            // A dict {"name": ...} is NOT the right format — Sideband won't parse it.
            app_data_buf[i++] = 0xc4;              // bin8 marker
            app_data_buf[i++] = name_len;          // length byte
            memcpy(app_data_buf + i, name, name_len);
            i += name_len;
            RNS::Bytes app_data;
            app_data.append(app_data_buf, i);
            destination.announce(app_data);
        }

        INFO("Gatekeeper: setup complete. Listening for commands.");

    } catch (const std::exception& e) {
        ERRORF("Gatekeeper: exception in setup: %s", e.what());
    }
}

// ── Arduino entry points ───────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);

    // On nRF52, USB serial needs time to enumerate with the host.
    // We wait up to 5 seconds — if nothing connects, we continue anyway.
    // This is important for field deployment where no laptop is attached.
    uint32_t start = millis();
    while (!Serial && (millis() - start) < 5000) {
        delay(100);
    }

    INFO("Gatekeeper: booting...");

    // Force EN LOW immediately at boot.
    // Identical reasoning to GatekeeperModule's constructor:
    // if the board resets mid-session, IO1 could float HIGH briefly,
    // so we drive it LOW before anything else runs.
    pinMode(GATEKEEPER_EN_PIN, OUTPUT);
    digitalWrite(GATEKEEPER_EN_PIN, LOW);
    INFO("Gatekeeper: EN pin LOW (Heltec off)");

    reticulum_setup();
}

void loop() {
    // reticulum.loop() does two things:
    //   1. Calls LoRaInterface::loop() which polls the SX1262 for received
    //      packets and feeds them into the stack via handle_incoming()
    //   2. Processes any queued announces, path requests, and transport tasks
    //
    // When a packet arrives addressed to our Destination, the stack calls
    // onPacket() before returning from loop(). So the callback fires here,
    // inline, not from an interrupt.
    reticulum.loop();

    // Watchdog — identical logic to GatekeeperModule::runOnce().
    // If the Heltec is on and max on-time has elapsed, cut power.
    // We don't use a timer thread here — checking every loop() iteration
    // is fine because reticulum.loop() returns quickly when there's nothing
    // to do and the millis() check is cheap.
    if (_heltecOn) {
        uint32_t elapsed = millis() - _activatedAtMs;
        if (elapsed >= HELTEC_MAX_ON_MS) {
            INFO("Gatekeeper: max on-time reached, forcing Heltec off");
            deactivateHeltec();
        }
    }
}

// ── Serial output bridge ───────────────────────────────────────────────────────
//
// On nRF52 Arduino with nosys.specs, the C library's _write() is not provided.
// This stub routes printf/INFO/ERROR output through Arduino Serial.
// Without this, any call to printf or the LOG macros silently does nothing.
int _write(int file, char* ptr, int len) {
    int wrote = Serial.write(ptr, len);
    Serial.flush();
    return wrote;
}
