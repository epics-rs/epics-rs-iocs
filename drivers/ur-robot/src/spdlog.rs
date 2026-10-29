//! spdlog's default logger, which every urRobot driver prints through
//! (`spdlog::info(...)` etc.): stdout, pattern `[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v`,
//! the level name coloured only on a terminal, threshold `info` unless
//! `SPDLOG_LEVEL` says otherwise (`spdlog::cfg::load_env_levels`).

use std::fmt;
use std::io::{IsTerminal, Write};
use std::sync::OnceLock;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Level {
    Trace,
    Debug,
    Info,
    Warn,
    Error,
    Critical,
    Off,
}

impl Level {
    /// `level::to_string_view`, the `%l` text.
    fn name(self) -> &'static str {
        match self {
            Level::Trace => "trace",
            Level::Debug => "debug",
            Level::Info => "info",
            Level::Warn => "warning",
            Level::Error => "error",
            Level::Critical => "critical",
            Level::Off => "off",
        }
    }

    /// `ansicolor_sink`'s per-level colour.
    fn color(self) -> &'static str {
        match self {
            Level::Trace => "\x1b[37m",
            Level::Debug => "\x1b[36m",
            Level::Info => "\x1b[32m",
            Level::Warn => "\x1b[33m\x1b[1m",
            Level::Error => "\x1b[31m\x1b[1m",
            Level::Critical => "\x1b[1m\x1b[41m",
            Level::Off => "",
        }
    }

    /// `level::from_str`, which also takes "warn" and "err".
    fn parse(s: &str) -> Option<Level> {
        Some(match s {
            "trace" => Level::Trace,
            "debug" => Level::Debug,
            "info" => Level::Info,
            "warning" | "warn" => Level::Warn,
            "error" | "err" => Level::Error,
            "critical" => Level::Critical,
            "off" => Level::Off,
            _ => return None,
        })
    }
}

/// `load_env_levels`: the bare entries of `SPDLOG_LEVEL` set the default
/// logger's level, the last one winning; `name=level` entries are other
/// loggers', and an unknown level name is skipped.
fn threshold_from(env: Option<&str>) -> Level {
    env.into_iter()
        .flat_map(|v| v.split(','))
        .filter(|entry| !entry.contains('='))
        .filter_map(|entry| Level::parse(&entry.trim().to_lowercase()))
        .next_back()
        .unwrap_or(Level::Info)
}

fn threshold() -> Level {
    static LEVEL: OnceLock<Level> = OnceLock::new();
    *LEVEL.get_or_init(|| threshold_from(std::env::var("SPDLOG_LEVEL").ok().as_deref()))
}

pub fn log(level: Level, msg: fmt::Arguments<'_>) {
    if level < threshold() || level == Level::Off {
        return;
    }
    let stamp = chrono::Local::now().format("%Y-%m-%d %H:%M:%S%.3f");
    let mut out = std::io::stdout().lock();
    let _ = if out.is_terminal() {
        writeln!(
            out,
            "[{stamp}] [{}{}\x1b[m] {msg}",
            level.color(),
            level.name()
        )
    } else {
        writeln!(out, "[{stamp}] [{}] {msg}", level.name())
    };
    let _ = out.flush();
}

/// A double as spdlog's bundled fmt 9.1 writes `{}` (`format.h:3238`):
/// Dragonbox's shortest round-trip digits, which are the digits Rust's `{:e}`
/// gives, laid out fixed for a decimal exponent in [-4, 16) and as
/// `d.ddde±XX` otherwise (`do_write_float`, `:2381`); `nan`/`inf` keep the
/// sign bit.
pub struct FmtDouble(pub f64);

impl fmt::Display for FmtDouble {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let v = self.0;
        let sign = if v.is_sign_negative() { "-" } else { "" };
        if !v.is_finite() {
            return write!(f, "{sign}{}", if v.is_nan() { "nan" } else { "inf" });
        }
        let shortest = format!("{:e}", v.abs());
        let (mantissa, exp) = shortest.split_once('e').expect("{:e} has an exponent");
        let exp: i32 = exp.parse().expect("{:e}'s exponent is an integer");
        let digits = mantissa.replace('.', "");
        let last = digits.len() as i32 - 1;
        f.write_str(sign)?;
        if !(-4..16).contains(&exp) {
            let (lead, rest) = digits.split_at(1);
            let point = if rest.is_empty() { "" } else { "." };
            let exp_sign = if exp < 0 { '-' } else { '+' };
            write!(f, "{lead}{point}{rest}e{exp_sign}{:02}", exp.unsigned_abs())
        } else if exp >= last {
            write!(f, "{digits}{}", "0".repeat((exp - last) as usize))
        } else if exp >= 0 {
            let (int, frac) = digits.split_at(exp as usize + 1);
            write!(f, "{int}.{frac}")
        } else {
            write!(f, "0.{}{digits}", "0".repeat((-exp - 1) as usize))
        }
    }
}

pub fn debug(msg: fmt::Arguments<'_>) {
    log(Level::Debug, msg);
}

pub fn info(msg: fmt::Arguments<'_>) {
    log(Level::Info, msg);
}

pub fn warn(msg: fmt::Arguments<'_>) {
    log(Level::Warn, msg);
}

pub fn error(msg: fmt::Arguments<'_>) {
    log(Level::Error, msg);
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Each pair is what `fmt::format("{}", x)` prints with the bundled fmt
    /// 9.1 headers.
    #[test]
    fn fmt_double_is_fmt_9_1_braces() {
        for (x, c) in [
            (0.0, "0"),
            (-0.0, "-0"),
            (1.0, "1"),
            (0.5, "0.5"),
            (0.1 + 0.2, "0.30000000000000004"),
            (100.0, "100"),
            (123.456, "123.456"),
            (1e15, "1000000000000000"),
            (1e16, "1e+16"),
            (1.5e16, "1.5e+16"),
            (9999999999999998.0, "9999999999999998"),
            (0.0001, "0.0001"),
            (0.00012345, "0.00012345"),
            (1e-5, "1e-05"),
            (1.5e-7, "1.5e-07"),
            (1e20, "1e+20"),
            (1e100, "1e+100"),
            (1e-100, "1e-100"),
            (5e-324, "5e-324"),
            (f64::MAX, "1.7976931348623157e+308"),
            (-2.5, "-2.5"),
            (1234567.0, "1234567"),
            (f64::NAN, "nan"),
            (-f64::NAN, "-nan"),
            (f64::INFINITY, "inf"),
            (f64::NEG_INFINITY, "-inf"),
        ] {
            assert_eq!(FmtDouble(x).to_string(), c, "{:?}", x.to_bits());
        }
    }

    #[test]
    fn spdlog_level_takes_the_bare_entries() {
        assert_eq!(threshold_from(None), Level::Info);
        assert_eq!(threshold_from(Some("debug")), Level::Debug);
        assert_eq!(threshold_from(Some("WARN")), Level::Warn);
        assert_eq!(threshold_from(Some("other=trace,err")), Level::Error);
        assert_eq!(threshold_from(Some("bogus")), Level::Info);
    }
}
