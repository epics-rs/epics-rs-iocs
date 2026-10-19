//! List all connected MCC DAQ devices and their UNIQUE_IDs.
//!
//! Usage:
//!   cargo run -p meascomp --bin list-devices

#![expect(
    clippy::disallowed_macros,
    reason = "single-threaded device-listing CLI: no second writer to interleave with"
)]

fn main() {
    match meascomp::device::discover_devices() {
        Ok(devs) if devs.is_empty() => {
            println!("No MCC DAQ devices found.");
        }
        Ok(devs) => {
            println!("Found {} device(s):", devs.len());
            println!(
                "{:<4} {:<30} {:<20} product_id",
                "#", "product_name", "unique_id"
            );
            for (i, (name, uid, pid)) in devs.iter().enumerate() {
                println!("{:<4} {:<30} {:<20} 0x{:04x}", i, name, uid, pid);
            }
        }
        Err(e) => {
            eprintln!("discover_devices failed: {e}");
            std::process::exit(1);
        }
    }
}
