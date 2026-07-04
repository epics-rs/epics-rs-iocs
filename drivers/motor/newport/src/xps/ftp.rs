//! FTP upload of PVT trajectory files to the XPS.
//!
//! C uploads the generated trajectory file into the controller's trajectory
//! directory before verifying/executing it (`XPSController::buildProfile`,
//! XPSController.cpp:709-747). Two transports there: plain FTP for XPS-C/Q, and
//! SFTP (`xpsSFTPUpload`) for XPS-D.
//!
//! This port implements the plain-FTP path with the blocking `suppaftp` client.
//! SFTP (XPS-D) needs an SSH transport `suppaftp` does not provide and is not
//! implemented — [`upload_trajectory`] returns an error naming it, so an XPS-D
//! caller fails loudly rather than silently skipping the upload.

use std::io::Cursor;

use suppaftp::FtpStream;

/// The XPS FTP control port.
const XPS_FTP_PORT: u16 = 21;

/// Upload `contents` as `file_name` into `directory` on the XPS at `host`,
/// logging in with `user`/`password` (plain FTP, binary mode).
///
/// `host` is the controller IP or hostname without a port. On any FTP step
/// failing, returns a message naming the step (mirrors C, which reports which
/// of connect/cd/store/disconnect failed).
pub fn upload_trajectory(
    host: &str,
    directory: &str,
    file_name: &str,
    contents: &str,
    user: &str,
    password: &str,
) -> Result<(), String> {
    let mut ftp = FtpStream::connect((host, XPS_FTP_PORT))
        .map_err(|e| format!("FTP connect to {host}:{XPS_FTP_PORT} failed: {e}"))?;
    ftp.login(user, password)
        .map_err(|e| format!("FTP login failed: {e}"))?;
    // XPS trajectory files are ASCII, but store binary so no CR/LF translation
    // mangles the line endings the controller parses.
    ftp.transfer_type(suppaftp::types::FileType::Binary)
        .map_err(|e| format!("FTP set binary mode failed: {e}"))?;
    ftp.cwd(directory)
        .map_err(|e| format!("FTP change dir to {directory} failed: {e}"))?;
    let mut reader = Cursor::new(contents.as_bytes());
    ftp.put_file(file_name, &mut reader)
        .map_err(|e| format!("FTP store {file_name} failed: {e}"))?;
    ftp.quit()
        .map_err(|e| format!("FTP disconnect failed: {e}"))?;
    Ok(())
}
