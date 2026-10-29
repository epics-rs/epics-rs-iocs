//! The startup allowlist of Serval channel destinations (C
//! `destination_policy.cpp`).

/// C `permissiveDestinationAllowlist` (ADTimePix.cpp:52).
pub const PERMISSIVE: &str = "file:/*,tcp://*,http://*";

/// C `PolicyStatus`; `Ok` is the `Ok` arm of [`Policy::configure`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PolicyError {
    NotConfigured,
    InvalidEntry,
}

impl PolicyError {
    /// C `statusMessage`.
    pub fn message(self) -> &'static str {
        match self {
            Self::NotConfigured => "destination allowlist is not configured",
            Self::InvalidEntry => "destination allowlist contains an invalid entry",
        }
    }
}

#[derive(Debug, Clone)]
struct Entry {
    value: String,
    prefix: bool,
}

/// C `Policy`: exact entries, or prefixes written with one trailing `*`.
#[derive(Debug, Clone, Default)]
pub struct Policy {
    entries: Vec<Entry>,
    description: String,
}

/// C `isspace` in the "C" locale.
fn trim(s: &str) -> &str {
    s.trim_matches([' ', '\t', '\n', '\x0b', '\x0c', '\r'])
}

fn is_file(value: &str) -> bool {
    value.starts_with("file:/") && !value.starts_with("file://")
}

fn supported_scheme(value: &str) -> bool {
    is_file(value) || value.starts_with("tcp://") || value.starts_with("http://")
}

/// C `safeFileDestination`: no escapes, and no `.` or `..` component.
fn safe_file(value: &str) -> bool {
    !is_file(value)
        || (!value.contains(['%', '?', '#', '\\'])
            && !value[5..].split('/').any(|c| c == "." || c == ".."))
}

impl Policy {
    /// C `configure`: any bad entry leaves the policy empty.
    pub fn configure(&mut self, entries: &str) -> Result<(), PolicyError> {
        self.entries.clear();
        self.description = entries.to_string();
        if trim(entries).is_empty() {
            return Err(PolicyError::NotConfigured);
        }
        // std::getline yields no token after a final ','.
        let mut tokens: Vec<&str> = entries.split(',').collect();
        if tokens.last() == Some(&"") {
            tokens.pop();
        }
        let mut parsed = Vec::new();
        for token in tokens {
            let token = trim(token);
            let prefix = token.contains('*');
            if token.is_empty()
                || (prefix && token.find('*') != Some(token.len() - 1))
                || !supported_scheme(token)
            {
                return Err(PolicyError::InvalidEntry);
            }
            let value = token.strip_suffix('*').unwrap_or(token);
            if !safe_file(value) || (prefix && is_file(value) && !value.ends_with('/')) {
                return Err(PolicyError::InvalidEntry);
            }
            parsed.push(Entry {
                value: value.to_string(),
                prefix,
            });
        }
        if parsed.is_empty() {
            return Err(PolicyError::NotConfigured);
        }
        self.entries = parsed;
        Ok(())
    }

    /// C `allows`.
    pub fn allows(&self, destination: &str) -> bool {
        supported_scheme(destination)
            && safe_file(destination)
            && self.entries.iter().any(|e| {
                if e.prefix {
                    destination.starts_with(&e.value)
                } else {
                    destination == e.value
                }
            })
    }

    /// C `permissive`: all three scheme-wide prefixes are present.
    pub fn permissive(&self) -> bool {
        ["file:/", "tcp://", "http://"]
            .iter()
            .all(|s| self.entries.iter().any(|e| e.prefix && e.value == *s))
    }

    pub fn description(&self) -> &str {
        &self.description
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_default_allowlist_is_permissive() {
        let mut p = Policy::default();
        p.configure(PERMISSIVE).unwrap();
        assert!(p.permissive());
        assert!(p.allows("file:/data/raw"));
        assert!(p.allows("tcp://listen@localhost:8085"));
        assert!(!p.allows("file://data"));
        assert!(!p.allows("file:/data/../etc"));
        assert!(!p.allows("ftp://x"));
    }

    #[test]
    fn an_entry_is_exact_or_a_trailing_star_prefix() {
        let mut p = Policy::default();
        p.configure(" file:/data/* , tcp://localhost:8085,")
            .unwrap();
        assert!(!p.permissive());
        assert!(p.allows("file:/data/run1"));
        assert!(!p.allows("file:/database"));
        assert!(p.allows("tcp://localhost:8085"));
        assert!(!p.allows("tcp://localhost:8086"));
    }

    #[test]
    fn a_bad_entry_empties_the_policy() {
        let mut p = Policy::default();
        for bad in [
            "file:/a/*,,tcp://*",
            ",tcp://*",
            "tcp://*x*",
            "tcp://a*b",
            "ftp://*",
            "file:/a*",
            "file:/a/./b",
            "file:/a%20",
            "tcp://*, ",
        ] {
            assert_eq!(p.configure(bad), Err(PolicyError::InvalidEntry), "{bad:?}");
            assert!(!p.allows("tcp://x"));
        }
        assert_eq!(p.configure(" \t"), Err(PolicyError::NotConfigured));
        assert_eq!(p.configure(","), Err(PolicyError::InvalidEntry));
        assert_eq!(p.description(), ",");
    }
}
