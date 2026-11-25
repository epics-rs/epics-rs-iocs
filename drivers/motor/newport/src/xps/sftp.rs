//! XPS-D trajectory upload, C `xpsSFTPUpload` (xpsSFTPUpload.cpp).
//!
//! C builds one of three bodies: libcurl when `HAVE_LIBCURL`, libssh2 when
//! `HAVE_LIBSSH2`, and otherwise a `curl` command line. The `sftp` feature is
//! the libssh2 build; without it the upload runs C's `curl` command. Both
//! answer C's status: 0 on success, the libssh2 body's -1, or the `curl`
//! command's `system()` status. Both upload the file `buildProfile` wrote
//! under `file_name` in the working directory.

use super::UPLOAD_TIMEOUT;

/// xpsSFTPUpload.cpp:183-192, C's build with neither library.
#[cfg(not(feature = "sftp"))]
pub fn xps_sftp_upload(
    ip_address: &str,
    trajectory_directory: &str,
    file_name: &str,
    user_name: &str,
    password: &str,
    verbose: bool,
) -> i32 {
    let (command, args) = curl_command(
        ip_address,
        trajectory_directory,
        file_name,
        user_name,
        password,
        verbose,
    );
    if verbose {
        println!("curl command={command}");
    }
    run("curl", &args)
}

/// C's command line, and the same words as arguments. C runs it through
/// `system()`, so a shell word in the password splits or expands it; here
/// each word goes to `curl` as it is. C leaves curl its 300 s to connect
/// and no bound on a stalled transfer; here both are [`UPLOAD_TIMEOUT`].
#[cfg(any(not(feature = "sftp"), test))]
fn curl_command(
    ip_address: &str,
    trajectory_directory: &str,
    file_name: &str,
    user_name: &str,
    password: &str,
    verbose: bool,
) -> (String, Vec<String>) {
    let verbose_option = if verbose { "--verbose" } else { "--silent" };
    let auth = format!("{user_name}:{password}");
    let url = format!("scp://{ip_address}{trajectory_directory}/");
    let bound = UPLOAD_TIMEOUT.as_secs().to_string();
    let bounds = [
        "--connect-timeout",
        &bound,
        "--speed-limit",
        "1",
        "--speed-time",
        &bound,
    ];
    let command = format!(
        "curl -k -u {auth} -T {file_name} {verbose_option} {}  {url}",
        bounds.join(" ")
    );
    let mut args = vec!["-k", "-u", &auth, "-T", file_name, verbose_option];
    args.extend(bounds);
    args.push(&url);
    (command, args.into_iter().map(String::from).collect())
}

/// `system()`'s answer: the wait status, or the shell's 127 when the
/// program cannot be run.
#[cfg(any(not(feature = "sftp"), all(test, unix)))]
fn run(program: &str, args: &[String]) -> i32 {
    match std::process::Command::new(program).args(args).status() {
        #[cfg(unix)]
        Ok(status) => std::os::unix::process::ExitStatusExt::into_raw(status),
        #[cfg(not(unix))]
        Ok(status) => status.code().unwrap_or(-1),
        Err(_) if cfg!(unix) => 127 << 8,
        Err(_) => 1,
    }
}

/// xpsSFTPUpload.cpp:47-181, C's `HAVE_LIBSSH2` build.
#[cfg(feature = "sftp")]
pub fn xps_sftp_upload(
    ip_address: &str,
    trajectory_directory: &str,
    file_name: &str,
    user_name: &str,
    password: &str,
    _verbose: bool,
) -> i32 {
    use std::io::Write;
    use std::net::{Ipv4Addr, SocketAddr, TcpStream};
    use std::path::Path;

    use ssh2::{ErrorCode, HashType, OpenFlags, OpenType, Session};

    let rc = |e: &ssh2::Error| match e.code() {
        ErrorCode::Session(rc) | ErrorCode::SFTP(rc) => rc,
    };
    // C appends the file name to the folder with no separator, naming
    // "/Admin/Public/TrajectoriesNAME".
    let remote_file = format!("{trajectory_directory}/{file_name}");

    let mut session = match Session::new() {
        Ok(session) => session,
        Err(e) => {
            epics_rs::base::stderr_println!("libssh2 initialization failed ({})", rc(&e));
            return -1;
        }
    };
    // C `inet_addr`: a dotted address only, no name lookup.
    let stream = ip_address.parse::<Ipv4Addr>().ok().and_then(|ip| {
        TcpStream::connect_timeout(&SocketAddr::from((ip, ssh_port())), UPLOAD_TIMEOUT).ok()
    });
    let Some(stream) = stream else {
        epics_rs::base::stderr_println!("failed to connect to {ip_address}");
        return -1;
    };
    let _ = stream.set_nodelay(true);
    session.set_blocking(true);
    // Every blocking libssh2 call below gives up after it.
    session.set_timeout(UPLOAD_TIMEOUT.as_millis() as u32);
    session.set_tcp_stream(stream);
    if let Err(e) = session.handshake() {
        epics_rs::base::stderr_println!("Failure establishing SSH session: {}", rc(&e));
        return -1;
    }
    let fingerprint = session.host_key_hash(HashType::Sha1).unwrap_or_default();
    let mut line = String::from("Fingerprint: ");
    for byte in fingerprint.iter().take(20) {
        line.push_str(&format!("{byte:02X} "));
    }
    epics_rs::base::stderr_println!("{line}");

    let status = (|| {
        if session.userauth_password(user_name, password).is_err() {
            epics_rs::base::stderr_println!("Authentication by password failed.");
            return -1;
        }
        let Ok(contents) = std::fs::read(file_name) else {
            epics_rs::base::stderr_println!("Can't open local file {file_name}");
            return -1;
        };
        let Ok(sftp) = session.sftp() else {
            epics_rs::base::stderr_println!("Unable to init SFTP session");
            return -1;
        };
        let Ok(mut remote) = sftp.open_mode(
            Path::new(&remote_file),
            OpenFlags::WRITE | OpenFlags::CREATE | OpenFlags::TRUNCATE,
            0o644,
            OpenType::File,
        ) else {
            epics_rs::base::stderr_println!("Unable to open remote file {remote_file} with SFTP");
            return -1;
        };
        // C answers 0 after a failed write.
        if remote.write_all(&contents).is_err() {
            return -1;
        }
        0
    })();
    let _ = session.disconnect(None, "Normal Shutdown", None);
    status
}

/// The XPS's SSH port.
#[cfg(all(feature = "sftp", not(test)))]
fn ssh_port() -> u16 {
    22
}

/// The port the libssh2 upload calls in a test, whose own server stands
/// in for the XPS's privileged 22.
#[cfg(all(feature = "sftp", test))]
static SSH_PORT: std::sync::atomic::AtomicU16 = std::sync::atomic::AtomicU16::new(22);

#[cfg(all(feature = "sftp", test))]
fn ssh_port() -> u16 {
    SSH_PORT.load(std::sync::atomic::Ordering::Relaxed)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(any(feature = "sftp", unix))]
    use crate::xps::ftp::tests::{silent_server, within};

    /// xpsSFTPUpload.cpp:187-188: C's command line, two spaces before the URL
    /// and the folder joined to the address with no slash, with curl's bounds
    /// on a stalled connect and transfer added.
    #[test]
    fn the_curl_command_is_cs_bounded() {
        let (command, args) = curl_command(
            "10.0.0.5",
            "/Admin/Public/Trajectories",
            "traj.trj",
            "Administrator",
            "pass word",
            false,
        );
        assert_eq!(
            command,
            "curl -k -u Administrator:pass word -T traj.trj --silent \
             --connect-timeout 1 --speed-limit 1 --speed-time 1  \
             scp://10.0.0.5/Admin/Public/Trajectories/"
        );
        assert_eq!(
            args,
            [
                "-k",
                "-u",
                "Administrator:pass word",
                "-T",
                "traj.trj",
                "--silent",
                "--connect-timeout",
                "1",
                "--speed-limit",
                "1",
                "--speed-time",
                "1",
                "scp://10.0.0.5/Admin/Public/Trajectories/"
            ]
        );
        let (_, args) = curl_command("h", "/d", "f", "u", "p", true);
        assert_eq!(args[5], "--verbose");
    }

    /// A server that takes the connection and never answers ends the curl
    /// upload in curl's timeout status 28, where C's waits curl's 300 s.
    #[cfg(all(unix, not(feature = "sftp")))]
    #[test]
    fn a_curl_upload_to_a_server_that_never_answers_times_out() {
        let file = std::env::temp_dir().join(format!("xps-sftp-{}.trj", std::process::id()));
        std::fs::write(&file, "0.5, 1, 2\n").unwrap();
        let (addr, _log) = silent_server(false);
        let name = file.to_str().unwrap().to_string();
        let status =
            within(move || xps_sftp_upload(&addr.to_string(), "/d", &name, "u", "p", false));
        let _ = std::fs::remove_file(&file);
        assert_eq!(status, Some(28 << 8));
    }

    /// A server that takes the connection and never answers fails the
    /// libssh2 upload, where C's waits in its handshake forever.
    #[cfg(feature = "sftp")]
    #[test]
    fn a_libssh2_upload_to_a_server_that_never_answers_fails() {
        let (addr, _log) = silent_server(false);
        SSH_PORT.store(addr.port(), std::sync::atomic::Ordering::Relaxed);
        let status = within(|| xps_sftp_upload("127.0.0.1", "/d", "f", "u", "p", false));
        assert_eq!(status, Some(-1));
    }

    /// `system()`'s status: the wait status, and 127 from the shell for a
    /// program that is not there.
    #[cfg(unix)]
    #[test]
    fn the_status_is_systems() {
        assert_eq!(run("true", &[]), 0);
        assert_eq!(run("false", &[]), 1 << 8);
        assert_eq!(run("/nonexistent/curl", &[]), 127 << 8);
    }
}
