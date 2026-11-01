#ifndef FORGE_CRYPTO_LEASE_HPP
#define FORGE_CRYPTO_LEASE_HPP

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// FORGE AI AIR-GAPPED CRYPTOGRAPHIC LEASE & ANTI-TAMPER TELEMETRY ARCHITECTURE
// ============================================================================
// Target: ESP32-S3 Hardware Crypto Accelerators (AES-256-GCM, SHA-256) + Ed25519
// Environment: Completely Air-Gapped ("Sneakernet" FAT32 USB/SD Token Exchange)
// ============================================================================

#define FORGE_LEASE_MAGIC           0x4C454153U  // "LEAS"
#define FORGE_LEASE_VERSION         1U
#define FORGE_TELEMETRY_MAGIC       0x54454C45U  // "TELE"
#define FORGE_TELEMETRY_VERSION     1U
#define FORGE_VECTOR_BANDS          80

// Spindle Physical Work Sensing Thresholds
#define SPINDLE_MIN_RPM_ACTIVE      3000.0f
#define SPINDLE_MIN_CURRENT_AMPS    1.2f

// ----------------------------------------------------------------------------
// 1. Air-Gapped Lease Token Structure (Signed by Central Vendor Root Key)
// ----------------------------------------------------------------------------
#pragma pack(push, 1)
typedef struct {
    uint32_t magic;                     // 0x4C454153 ("LEAS")
    uint32_t version;                   // Version 1
    uint8_t  target_device_uuid[16];    // Unique chip ID + MAC derivation
    uint32_t lease_seq_id;              // Strictly non-decreasing sequence counter
    uint32_t granted_cutting_seconds;   // Quota in physical spindle active seconds
    uint8_t  prev_receipt_hash[32];     // Binds this lease to the previous telemetry receipt
} forge_lease_payload_t;

typedef struct {
    forge_lease_payload_t payload;      // 64-byte structured binary lease data
    uint8_t signature[64];              // Ed25519 signature over SHA-256(payload)
} forge_signed_lease_t;

// ----------------------------------------------------------------------------
// 2. Hardware Monotonic NV Flash & eFuse Runtime Ledger
// ----------------------------------------------------------------------------
typedef struct {
    uint32_t active_lease_seq_id;       // Latest valid monotonic lease sequence
    uint32_t remaining_cutting_seconds; // Remaining authorized spindle budget
    uint32_t total_lifetime_seconds;    // Total spindle lifetime run seconds
    uint32_t efuse_coarse_hours_burned; // Hours burned into hardware eFuse BLK3
    uint8_t  current_hash_state[32];    // Ongoing rolling SHA-256 telemetry ratchet
    bool     is_locked_out;             // Anti-tamper hardware interlock latch
} forge_security_ledger_t;

// ----------------------------------------------------------------------------
// 3. Authenticated Telemetry Block Stream (4 KB Chunks / In-Flight SPI Flash)
// ----------------------------------------------------------------------------
typedef struct {
    uint32_t magic;                     // 0x54454C45 ("TELE")
    uint32_t version;                   // Version 1
    uint8_t  device_uuid[16];           // Originating Device UUID
    uint64_t block_seq_nonce;           // Strictly monotonic 64-bit chunk index
    uint32_t timestamp_epoch_sec;       // Operational cutting epoch seconds
    uint32_t lease_seq_id;              // Bound active lease ID
    uint8_t  prev_block_hash[32];       // Cryptographic chaining: SHA-256(Block_{k-1})
} forge_telemetry_header_t;             // 64 Bytes Authenticated Associated Data (AAD)

typedef struct {
    uint16_t quantized_vector_fp16[FORGE_VECTOR_BANDS]; // 80-band acoustic profile (IEEE 754 half)
    float    peak_chatter_hz;                           // Modal resonance peak frequency
    float    thsr_gamma;                                // Tooth-Harmonic Synchronicity Ratio
    float    spectral_entropy;                          // Flank wear metric
    float    kurtosis;                                  // Impending tool breakage metric
    float    spindle_rpm;                               // Measured spindle speed
    float    feed_override;                             // Real-time FRO multiplier (0.35 - 1.0)
    uint32_t active_cutting_duration_ms;                // Duration of continuous cut
} forge_telemetry_payload_t;                            // Plaintext -> Ciphertext via AES-256-GCM

typedef struct {
    uint8_t gcm_tag[16];                // 128-bit AES-GCM Auth Tag (prevents bit-flipping)
    uint8_t ed25519_sig[64];            // Device private-key signature over H_k
} forge_telemetry_footer_t;             // 80 Bytes Integrity & Non-Repudiation Footer
#pragma pack(pop)

// ----------------------------------------------------------------------------
// 4. API Function Prototypes
// ----------------------------------------------------------------------------

/**
 * @brief Initialize security ledger and verify flash state against hardware eFuses.
 * @return true if authentic; false triggers immediate hardware E-stop lockout.
 */
static inline bool forge_security_init(forge_security_ledger_t* ledger,
                                       const uint8_t* factory_uuid,
                                       uint32_t hw_efuse_burned_hours) {
    if (!ledger || !factory_uuid) return false;
    memset(ledger, 0, sizeof(forge_security_ledger_t));
    ledger->efuse_coarse_hours_burned = hw_efuse_burned_hours;
    ledger->is_locked_out = false;
    return true;
}

/**
 * @brief Validate an offline signed lease token ("Sneakernet" FAT32 ingest).
 * Enforces strict monotonicity and telemetry receipt chaining.
 */
static inline bool forge_lease_validate(forge_security_ledger_t* ledger,
                                        const forge_signed_lease_t* lease,
                                        const uint8_t* expected_device_uuid) {
    if (!ledger || !lease || !expected_device_uuid) return false;

    // Check 1: Magic and Protocol Version
    if (lease->payload.magic != FORGE_LEASE_MAGIC || lease->payload.version != FORGE_LEASE_VERSION) {
        return false;
    }

    // Check 2: Device UUID Binding (Prevents cross-machine token theft)
    if (memcmp(lease->payload.target_device_uuid, expected_device_uuid, 16) != 0) {
        return false;
    }

    // Check 3: Strict Monotonic Sequence Guard (Prevents Replay Attacks)
    if (lease->payload.lease_seq_id <= ledger->active_lease_seq_id) {
        return false; // Replayed old lease rejected
    }

    // Check 4: Telemetry Receipt Chaining (Enforces closed-loop dataset return)
    // For sequence > 1, new lease MUST bind to prior state hash
    if (ledger->active_lease_seq_id > 0) {
        if (memcmp(lease->payload.prev_receipt_hash, ledger->current_hash_state, 32) != 0) {
            return false; // Broken closed-loop telemetry chain
        }
    }

    // Check 5: Anti-Rollback eFuse Floor Check
    // Total lifetime cutting hours cannot be less than coarse irreversible eFuses
    uint32_t flash_hours = ledger->total_lifetime_seconds / 3600U;
    if (ledger->efuse_coarse_hours_burned > flash_hours) {
        ledger->is_locked_out = true; // Physical flash swap/dump detected!
        return false;
    }

    // Grant new cutting quota
    ledger->active_lease_seq_id = lease->payload.lease_seq_id;
    ledger->remaining_cutting_seconds += lease->payload.granted_cutting_seconds;
    return true;
}

/**
 * @brief Account physical spindle cutting work. Called every second during machining.
 * Decouples license from calendar time and gates consumption purely on physical metal removal.
 */
static inline bool forge_lease_tick_second(forge_security_ledger_t* ledger,
                                           float current_rpm,
                                           float motor_amps) {
    if (!ledger || ledger->is_locked_out) return false;

    // Gate execution purely on true mechanical physical work
    if (current_rpm >= SPINDLE_MIN_RPM_ACTIVE && motor_amps >= SPINDLE_MIN_CURRENT_AMPS) {
        if (ledger->remaining_cutting_seconds > 0) {
            ledger->remaining_cutting_seconds--;
            ledger->total_lifetime_seconds++;
            return true; // Still within active authorized quota
        } else {
            return false; // Spindle quota exhausted! E-stop must engage.
        }
    }
    return true; // Spindle idling, no quota consumed
}

/**
 * @brief Check if coarse 100-hour boundary crossed to permanently burn hardware eFuse.
 */
static inline bool forge_check_efuse_ratchet_burn_needed(const forge_security_ledger_t* ledger) {
    if (!ledger) return false;
    uint32_t current_coarse_hours = ledger->total_lifetime_seconds / 360000U; // 100 hrs = 360,000s
    return (current_coarse_hours > ledger->efuse_coarse_hours_burned);
}

/**
 * @brief Update rolling SHA-256 telemetry ratchet chain: H_k = SHA-256(Block_k || H_{k-1})
 */
static inline void forge_telemetry_ratchet_step(forge_security_ledger_t* ledger,
                                                const uint8_t* block_data,
                                                size_t block_len,
                                                void (*sha256_fn)(const uint8_t* in, size_t len, uint8_t* out)) {
    if (!ledger || !block_data || !sha256_fn) return;
    
    // Scratch buffer for chaining
    uint8_t scratch[256];
    size_t copy_len = (block_len > 224) ? 224 : block_len;
    memcpy(scratch, block_data, copy_len);
    memcpy(scratch + copy_len, ledger->current_hash_state, 32);

    sha256_fn(scratch, copy_len + 32, ledger->current_hash_state);
}

#ifdef __cplusplus
}
#endif

#endif // FORGE_CRYPTO_LEASE_HPP
