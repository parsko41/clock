//! Minimal mDNS responder: answers "A" queries for `<host>.local` so the page is
//! reachable at http://pendulum.local. Pure packet logic, no networking here.

const TYPE_A: u16 = 1;
const TYPE_ANY: u16 = 255;

/// Read a (possibly compressed) DNS name starting at `pos` and compare it to
/// `host` + ".local", case-insensitively. Returns (matches, position after name).
fn name_matches(pkt: &[u8], mut pos: usize, host: &str) -> Option<(bool, usize)> {
    let want = [host.as_bytes(), b"local"];
    let mut label = 0usize;
    let mut ok = true;
    let mut end: Option<usize> = None;
    let mut hops = 0;
    loop {
        let len = *pkt.get(pos)? as usize;
        if len & 0xC0 == 0xC0 {
            let ptr = ((len & 0x3F) << 8) | *pkt.get(pos + 1)? as usize;
            end.get_or_insert(pos + 2);
            pos = ptr;
            hops += 1;
            if hops > 8 {
                return None;
            }
            continue;
        }
        if len == 0 {
            let after = end.unwrap_or(pos + 1);
            return Some((ok && label == want.len(), after));
        }
        let bytes = pkt.get(pos + 1..pos + 1 + len)?;
        if label >= want.len() || !bytes.eq_ignore_ascii_case(want[label]) {
            ok = false;
        }
        label += 1;
        pos += 1 + len;
    }
}

fn put_name(out: &mut [u8], mut p: usize, host: &str) -> Option<usize> {
    for l in [host.as_bytes(), b"local"] {
        *out.get_mut(p)? = l.len() as u8;
        out.get_mut(p + 1..p + 1 + l.len())?.copy_from_slice(l);
        p += 1 + l.len();
    }
    *out.get_mut(p)? = 0;
    Some(p + 1)
}

/// Build a response to `query` if it asks for `<host>.local`. `legacy` = the
/// query came from a port other than 5353 (one-shot resolver): reply unicast,
/// echoing the id and question. Returns the response length.
pub fn answer(query: &[u8], host: &str, ip: [u8; 4], legacy: bool, out: &mut [u8]) -> Option<usize> {
    if query.len() < 12 || query[2] & 0x80 != 0 {
        return None; // too short, or it's a response
    }
    let qd = u16::from_be_bytes([query[4], query[5]]);
    let mut pos = 12;
    let mut hit = false;
    for _ in 0..qd {
        let (m, after) = name_matches(query, pos, host)?;
        let qtype = u16::from_be_bytes([*query.get(after)?, *query.get(after + 1)?]);
        pos = after + 4;
        if m && (qtype == TYPE_A || qtype == TYPE_ANY) {
            hit = true;
        }
    }
    if !hit {
        return None;
    }

    out.get_mut(..12)?.fill(0);
    if legacy {
        out[0] = query[0];
        out[1] = query[1];
    }
    out[2] = 0x84; // response, authoritative
    let mut p = 12;
    if legacy {
        out[5] = 1; // one question
        p = put_name(out, p, host)?;
        out.get_mut(p..p + 4)?.copy_from_slice(&[0, 1, 0, 1]);
        p += 4;
    }
    out[7] = 1; // one answer
    p = put_name(out, p, host)?;
    let class: u16 = if legacy { 0x0001 } else { 0x8001 }; // IN, cache-flush for multicast
    let ttl: u32 = if legacy { 10 } else { 120 };
    let rr = out.get_mut(p..p + 14)?;
    rr[0..2].copy_from_slice(&TYPE_A.to_be_bytes());
    rr[2..4].copy_from_slice(&class.to_be_bytes());
    rr[4..8].copy_from_slice(&ttl.to_be_bytes());
    rr[8..10].copy_from_slice(&4u16.to_be_bytes());
    rr[10..14].copy_from_slice(&ip);
    Some(p + 14)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn query(name: &[&str], qtype: u16) -> alloc::vec::Vec<u8> {
        let mut q = alloc::vec![0x12, 0x34, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0];
        for l in name {
            q.push(l.len() as u8);
            q.extend_from_slice(l.as_bytes());
        }
        q.push(0);
        q.extend_from_slice(&qtype.to_be_bytes());
        q.extend_from_slice(&[0x80, 1]); // QU bit set, class IN
        q
    }

    #[test]
    fn answers_a_query() {
        let q = query(&["Pendulum", "LOCAL"], 1);
        let mut out = [0u8; 128];
        let n = answer(&q, "pendulum", [192, 168, 1, 50], false, &mut out).unwrap();
        assert_eq!(&out[n - 4..n], &[192, 168, 1, 50]);
        assert_eq!(out[7], 1);
        assert_eq!(out[0..2], [0, 0]);
        let n = answer(&q, "pendulum", [10, 0, 0, 9], true, &mut out).unwrap();
        assert_eq!(out[0..2], [0x12, 0x34]);
        assert_eq!(out[5], 1);
        assert_eq!(&out[n - 4..n], &[10, 0, 0, 9]);
    }

    #[test]
    fn ignores_others() {
        let mut out = [0u8; 128];
        assert!(answer(&query(&["printer", "local"], 1), "pendulum", [1, 2, 3, 4], false, &mut out).is_none());
        assert!(answer(&query(&["pendulum", "local"], 28), "pendulum", [1, 2, 3, 4], false, &mut out).is_none());
        assert!(answer(&query(&["pendulum", "local", "x"], 1), "pendulum", [1, 2, 3, 4], false, &mut out).is_none());
        assert!(answer(&[0u8; 5], "pendulum", [1, 2, 3, 4], false, &mut out).is_none());
    }
}
