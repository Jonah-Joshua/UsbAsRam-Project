use anyhow::{bail, Context, Result};

/// Parse sizes such as `4096`, `64K`, `64KiB`, `1M`, `2GiB`, or `1.5T`.
pub fn parse_size(input: &str) -> Result<u64> {
    let trimmed = input.trim();
    if trimmed.is_empty() {
        bail!("size is empty");
    }
    let split = trimmed
        .find(|character: char| {
            !(character.is_ascii_digit() || character == '.' || character == '_')
        })
        .unwrap_or(trimmed.len());
    let (number, suffix) = trimmed.split_at(split);
    let number: f64 = number
        .replace('_', "")
        .parse()
        .with_context(|| format!("invalid number in size {input:?}"))?;
    let multiplier = match suffix.trim().to_ascii_lowercase().as_str() {
        "" | "b" => 1.0,
        "k" | "kb" | "kib" => 1024.0,
        "m" | "mb" | "mib" => 1024.0 * 1024.0,
        "g" | "gb" | "gib" => 1024.0 * 1024.0 * 1024.0,
        "t" | "tb" | "tib" => 1024.0 * 1024.0 * 1024.0 * 1024.0,
        other => bail!("unknown size suffix {other:?}"),
    };
    let bytes = number * multiplier;
    if !bytes.is_finite() || bytes < 0.0 || bytes > u64::MAX as f64 {
        bail!("size {input:?} is out of range");
    }
    Ok(bytes as u64)
}

pub fn human_bytes(bytes: u64) -> String {
    const UNITS: [&str; 5] = ["B", "KiB", "MiB", "GiB", "TiB"];
    let mut value = bytes as f64;
    let mut unit = 0;
    while value >= 1024.0 && unit < UNITS.len() - 1 {
        value /= 1024.0;
        unit += 1;
    }
    format!("{value:.2} {}", UNITS[unit])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_common_sizes() {
        assert_eq!(parse_size("4096").unwrap(), 4096);
        assert_eq!(parse_size("4K").unwrap(), 4096);
        assert_eq!(parse_size("1MiB").unwrap(), 1024 * 1024);
        assert_eq!(parse_size("1.5M").unwrap(), 1024 * 1024 * 3 / 2);
        assert_eq!(parse_size("2 GiB").unwrap(), 2 * 1024 * 1024 * 1024);
    }

    #[test]
    fn rejects_bad_sizes() {
        assert!(parse_size("").is_err());
        assert!(parse_size("12wat").is_err());
        assert!(parse_size("-1").is_err());
    }
}
