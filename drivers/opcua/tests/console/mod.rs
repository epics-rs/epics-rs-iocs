//! The test process's stdout and stderr, captured into one file.

#![allow(clippy::disallowed_macros, reason = "tests may keep eprint!")]

/// Fds 1 and 2 into one file, so stdout and stderr lines keep the order they
/// were printed in.
pub struct Console {
    sink: tempfile::NamedTempFile,
    saved: Option<[i32; 2]>,
}

impl Console {
    pub fn capture() -> Console {
        let sink = tempfile::NamedTempFile::new().unwrap();
        // SAFETY: nextest runs each test in its own process, so fds 1 and 2
        // are this test's alone.
        let saved = [1, 2].map(|fd| unsafe {
            let saved = libc::dup(fd);
            libc::dup2(std::os::fd::AsRawFd::as_raw_fd(sink.as_file()), fd);
            saved
        });
        Console {
            sink,
            saved: Some(saved),
        }
    }

    fn restore(&mut self) {
        let Some(saved) = self.saved.take() else {
            return;
        };
        for (fd, saved) in [1, 2].into_iter().zip(saved) {
            // SAFETY: restoring what `capture` replaced.
            unsafe {
                libc::dup2(saved, fd);
                libc::close(saved);
            }
        }
    }

    /// Restore the fds; return every line printed meanwhile.
    pub fn lines(mut self) -> Vec<String> {
        self.restore();
        std::fs::read_to_string(self.sink.path())
            .unwrap()
            .lines()
            .map(str::to_string)
            .collect()
    }
}

/// A test that fails while capturing printed its panic message into the
/// sink; pass the sink on to the real stderr.
impl Drop for Console {
    fn drop(&mut self) {
        self.restore();
        if std::thread::panicking() {
            epics_rs::base::stderr_print!(
                "{}",
                std::fs::read_to_string(self.sink.path()).unwrap_or_default()
            );
        }
    }
}
