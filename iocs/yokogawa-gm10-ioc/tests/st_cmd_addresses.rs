//! C names channel records only through `gm10_probe`'s substitutions file
//! (`gm10_probe.c:62-219`), so st.cmd's ADDRESS values follow its formats
//! and the PVs match a C IOC's.

#[test]
fn st_cmd_addresses_are_formatted_as_gm10_probe_formats_them() {
    let st_cmd = include_str!("../st.cmd");
    let addresses: Vec<&str> = st_cmd
        .lines()
        .filter(|l| !l.starts_with('#'))
        .filter_map(|l| l.split("ADDRESS=").nth(1))
        .map(|a| a.trim_end_matches(['"', ')']))
        .collect();
    assert!(!addresses.is_empty());
    for address in addresses {
        let digits = address.find(|c: char| c.is_ascii_digit()).unwrap();
        let (family, n) = address.split_at(digits);
        let n: u32 = n.parse().unwrap();
        let probe = match family {
            "" => format!("{n:04}"),
            "A" | "C" | "K" | "W" => format!("{family}{n:03}"),
            _ => panic!("{address}"),
        };
        assert_eq!(address, probe);
    }
}
