//! Parse a plain-text Wi-Fi file written straight to flash with esptool:
//!
//! ```text
//! ssid=MyNetwork
//! pass=my password
//! ```
//!
//! - One `key=value` per line. Keys: `ssid`, and `pass` or `password` (any case).
//! - Blank lines and lines starting with `#` are ignored. Windows line endings and
//!   a UTF-8 BOM (Notepad) are fine. Spaces around the key and value are trimmed.
//! - The value is everything after the first `=`, so passwords may contain `=`.
//! - Leave `pass` out (or empty) for an open network.
//! - The text ends at the first 0x00 or 0xFF byte (erased flash reads 0xFF).
//!
//! Hardware-free so it can be unit-tested on a PC (see ../host-tests).

/// Returns `(ssid, pass)` borrowed from `raw`, or `None` if it isn't a usable file.
pub fn parse(raw: &[u8]) -> Option<(&str, &str)> {
    let end = raw.iter().position(|&b| b == 0x00 || b == 0xFF).unwrap_or(raw.len());
    let text = core::str::from_utf8(&raw[..end]).ok()?;
    let text = text.strip_prefix('\u{feff}').unwrap_or(text);

    let (mut ssid, mut pass) = (None, "");
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let Some((k, v)) = line.split_once('=') else { continue };
        let (k, v) = (k.trim(), v.trim());
        if k.eq_ignore_ascii_case("ssid") {
            ssid = Some(v);
        } else if k.eq_ignore_ascii_case("pass") || k.eq_ignore_ascii_case("password") {
            pass = v;
        }
    }
    let ssid = ssid?;
    if ssid.is_empty() || ssid.len() > 32 || pass.len() > 64 {
        return None;
    }
    Some((ssid, pass))
}
