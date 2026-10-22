//! Saved Wi-Fi credentials, kept in the first bytes of the `nvs` flash partition
//! (0x9000 with the default partition table). Re-flashing the app leaves it alone.
//!
//! Two formats are accepted there; whichever was written last wins:
//! - a small binary record (`PWF1`), written by the setup hotspot page, or
//! - a plain-text `wifi.txt` flashed directly with esptool (see `wifitxt.rs`).
//!
//! Neither is ESP-IDF NVS; this firmware doesn't use NVS for anything else.

use alloc::string::String;
use core::cell::RefCell;
use critical_section::Mutex;
use esp_bootloader_esp_idf::partitions::{self, DataPartitionSubType, PartitionType};
use esp_storage::FlashStorage;

const MAGIC: [u8; 4] = *b"PWF1";
const LEN: usize = 4 + 1 + 1 + 32 + 64 + 4;
/// How much of the partition to read when looking for a text file.
const TEXT_MAX: usize = 512;

static FLASH: Mutex<RefCell<Option<FlashStorage<'static>>>> = Mutex::new(RefCell::new(None));

pub struct Creds {
    pub ssid: String,
    pub pass: String,
}

pub fn init(flash: FlashStorage<'static>) {
    critical_section::with(|cs| FLASH.borrow_ref_mut(cs).replace(flash));
}

fn checksum(b: &[u8]) -> u32 {
    // FNV-1a
    b.iter().fold(0x811c9dc5u32, |h, &x| (h ^ x as u32).wrapping_mul(0x0100_0193))
}

/// Run `f` on the nvs partition. Flash access is short and blocking.
fn with_nvs<R>(f: impl FnOnce(&mut partitions::FlashRegion<'_, '_>) -> Option<R>) -> Option<R> {
    critical_section::with(|cs| {
        let mut guard = FLASH.borrow_ref_mut(cs);
        let flash = guard.as_mut()?;
        let mut pt_mem = [0u8; partitions::PARTITION_TABLE_MAX_LEN];
        let pt = partitions::read_partition_table(flash, &mut pt_mem).ok()?;
        let nvs = pt.find_partition(PartitionType::Data(DataPartitionSubType::Nvs)).ok()??;
        let mut region = nvs.as_flash_region(flash);
        f(&mut region)
    })
}

pub fn load() -> Option<Creds> {
    with_nvs(|r| {
        let mut b = [0u8; TEXT_MAX];
        r.read(0, &mut b).ok()?;
        if let Some(c) = parse_record(&b[..LEN]) {
            log::info!("Wi-Fi credentials: saved from setup page");
            return Some(c);
        }
        let (ssid, pass) = crate::wifitxt::parse(&b)?;
        log::info!("Wi-Fi credentials: from wifi.txt in flash");
        Some(Creds { ssid: String::from(ssid), pass: String::from(pass) })
    })
}

fn parse_record(b: &[u8]) -> Option<Creds> {
    if b[..4] != MAGIC || checksum(&b[..LEN - 4]) != u32::from_le_bytes(b[LEN - 4..].try_into().ok()?) {
        return None;
    }
    let (sl, pl) = (b[4] as usize, b[5] as usize);
    if sl == 0 || sl > 32 || pl > 64 {
        return None;
    }
    let ssid = String::from(core::str::from_utf8(&b[6..6 + sl]).ok()?);
    let pass = String::from(core::str::from_utf8(&b[38..38 + pl]).ok()?);
    Some(Creds { ssid, pass })
}

pub fn save(ssid: &str, pass: &str) -> bool {
    if ssid.is_empty() || ssid.len() > 32 || pass.len() > 64 {
        return false;
    }
    let mut b = [0u8; LEN];
    b[..4].copy_from_slice(&MAGIC);
    b[4] = ssid.len() as u8;
    b[5] = pass.len() as u8;
    b[6..6 + ssid.len()].copy_from_slice(ssid.as_bytes());
    b[38..38 + pass.len()].copy_from_slice(pass.as_bytes());
    let c = checksum(&b[..LEN - 4]);
    b[LEN - 4..].copy_from_slice(&c.to_le_bytes());
    with_nvs(|r| r.write(0, &b).ok()).is_some()
}

pub fn forget() -> bool {
    with_nvs(|r| r.write(0, &[0u8; LEN]).ok()).is_some()
}

// ---- bob calibration: ms of period change per mm of bob movement (+ = down).
// Kept in the 2nd 4 KB sector of the nvs partition, so Wi-Fi changes, /forget
// and flashing wifi.txt at 0x9000 leave it alone.

const CAL_OFF: u32 = 4096;
const CAL_MAGIC: [u8; 4] = *b"PCAL";

pub fn load_cal() -> Option<f32> {
    with_nvs(|r| {
        let mut b = [0u8; 12];
        r.read(CAL_OFF, &mut b).ok()?;
        if b[..4] != CAL_MAGIC || checksum(&b[..8]) != u32::from_le_bytes(b[8..12].try_into().ok()?) {
            return None;
        }
        let k = f32::from_le_bytes(b[4..8].try_into().ok()?);
        k.is_finite().then_some(k)
    })
}

/// Save `k` (ms per mm), or erase the calibration with `None`.
pub fn save_cal(k: Option<f32>) -> bool {
    let mut b = [0u8; 12];
    if let Some(k) = k {
        b[..4].copy_from_slice(&CAL_MAGIC);
        b[4..8].copy_from_slice(&k.to_le_bytes());
        let c = checksum(&b[..8]);
        b[8..12].copy_from_slice(&c.to_le_bytes());
    }
    with_nvs(|r| r.write(CAL_OFF, &b).ok()).is_some()
}
