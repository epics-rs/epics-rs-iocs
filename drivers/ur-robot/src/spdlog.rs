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

    #[test]
    fn spdlog_level_takes_the_bare_entries() {
        assert_eq!(threshold_from(None), Level::Info);
        assert_eq!(threshold_from(Some("debug")), Level::Debug);
        assert_eq!(threshold_from(Some("WARN")), Level::Warn);
        assert_eq!(threshold_from(Some("other=trace,err")), Level::Error);
        assert_eq!(threshold_from(Some("bogus")), Level::Info);
    }
}
