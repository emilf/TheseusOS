use std::{
    io::{Read, Write},
    os::unix::process::ExitStatusExt,
    path::{Path, PathBuf},
    process::{Command, ExitStatus, Stdio},
    thread,
};

use anyhow::{Context, Result, anyhow, bail};
use clap::{Parser, ValueEnum};
use serde::{Deserialize, Serialize};

#[derive(Parser, Debug)]
#[command(
    name = "theseus-qemu",
    version,
    about = "Build and run TheseusOS under QEMU with profiles"
)]
pub struct Cli {
    /// Default action: run QEMU with the provided args.
    #[command(flatten)]
    pub run: Args,

    #[command(subcommand)]
    pub cmd: Option<Cmd>,
}

#[derive(clap::Subcommand, Debug)]
pub enum Cmd {
    /// Print the resolved QEMU argv (no execution). Implies --dry.
    Print(Args),
    /// Emit a JSON artifact containing the resolved configuration + argv (implies --dry).
    Artifact(ArtifactArgs),
}

#[derive(Parser, Debug, Clone)]
pub struct ArtifactArgs {
    #[command(flatten)]
    pub args: Args,

    /// Where to write the JSON artifact
    #[arg(long, default_value = "build/qemu-argv.json")]
    pub out: PathBuf,
}

#[derive(Parser, Debug, Clone)]
pub struct Args {
    /// Do not require build artifacts to exist (OVMF vars, disk image, etc.).
    /// Useful for printing commands in locked-down environments.
    #[arg(long)]
    pub dry: bool,

    /// Build the project before running QEMU (equivalent to `make all`).
    /// Enabled by default for human convenience.
    #[arg(long, default_value_t = true)]
    pub build: bool,

    /// Skip building before running QEMU.
    #[arg(long)]
    pub no_build: bool,

    /// Profile (selects device set and defaults)
    #[arg(long, default_value = "default")]
    pub profile: Profile,

    /// Headless mode (no display)
    #[arg(long)]
    pub headless: bool,

    /// Add a timeout wrapper (seconds). Only applies to run mode.
    #[arg(long, default_value_t = 0)]
    pub timeout_secs: u64,

    /// Kernel success marker string. If present in QEMU output, exit code is forced to 0.
    #[arg(long, default_value = "Kernel environment test completed successfully")]
    pub success_marker: String,

    /// QEMU accelerator (e.g. kvm:tcg, tcg)
    #[arg(long, default_value = "kvm:tcg")]
    pub accel: String,

    /// QEMU irqchip mode (off|on|split)
    #[arg(long, default_value = "split")]
    pub irqchip: String,

    /// Enable QMP socket. If provided without a value, defaults to `/tmp/qemu-qmp.sock`.
    #[arg(long, num_args = 0..=1, default_missing_value = "/tmp/qemu-qmp.sock")]
    pub qmp: Option<String>,

    /// Enable HMP monitor socket. If provided without a value, defaults to `/tmp/theseus-hmp.sock`.
    #[arg(long, num_args = 0..=1, default_missing_value = "/tmp/theseus-hmp.sock")]
    pub hmp: Option<String>,

    /// Enable a PTY-backed QEMU monitor endpoint. If provided without a value, defaults to `/tmp/qemu-monitor`.
    #[arg(long, num_args = 0..=1, default_missing_value = "/tmp/qemu-monitor")]
    pub monitor_pty: Option<String>,

    /// Send isa-debugcon output to a PTY path. If provided without a value, defaults to `/tmp/qemu-debugcon`.
    #[arg(long, num_args = 0..=1, default_missing_value = "/tmp/qemu-debugcon")]
    pub debugcon_pty: Option<String>,

    /// Serial mode.
    #[arg(long, value_enum, default_value_t = SerialMode::Off)]
    pub serial: SerialMode,

    /// Serial endpoint for unix mode (e.g. /tmp/theseus-serial.sock)
    #[arg(long, default_value = "/tmp/qemu-serial")]
    pub serial_path: String,

    /// Enable the standard /tmp relays used for interactive debugging.
    ///
    /// This is a convenience flag equivalent to enabling:
    /// - --serial unix --serial-path /tmp/qemu-serial
    /// - --monitor-pty /tmp/qemu-monitor
    /// - --debugcon-pty /tmp/qemu-debugcon
    /// - --qmp /tmp/qemu-qmp.sock
    ///
    /// Explicit flags always win (e.g., passing your own --serial/stdio overrides this).
    #[arg(long)]
    pub relays: bool,

    /// Extra raw QEMU args appended verbatim
    #[arg(long)]
    pub extra: Vec<String>,
}

#[derive(ValueEnum, Debug, Clone, Copy, PartialEq, Eq)]
pub enum SerialMode {
    /// Do not expose a serial device (use debugcon instead)
    Off,
    /// Use stdio for serial
    Stdio,
    /// Expose a unix socket at --serial-path
    Unix,
    /// Use an existing PTY/tty at --serial-path (pairs well with socat PTY relays)
    /// Use an existing PTY at --serial-path by opening it as a file chardev.
    PseudoTtyFile,
}

#[derive(ValueEnum, Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Profile {
    /// Mirrors the current startQemu.sh defaults (q35 + nvme + xhci + virtio-gpu + virtio-net)
    Default,
    /// Minimal headless, no USB/GPU/NIC. Useful for bring-up.
    Min,
    /// USB keyboard profile (xhci + usb-kbd)
    UsbKbd,
}

#[derive(Debug, Serialize)]
pub struct Artifact {
    pub profile: Profile,
    pub headless: bool,
    pub argv: Vec<String>,
    pub cwd: PathBuf,
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    match cli.cmd {
        Some(Cmd::Print(mut args)) => {
            args.dry = true;
            let argv = build_qemu_argv(&args)?;
            println!("{}", shell_join(&argv));
            Ok(())
        }
        Some(Cmd::Artifact(artifact)) => {
            let mut args = artifact.args;
            args.dry = true;
            let argv = build_qemu_argv(&args)?;
            let cwd = std::env::current_dir()?;
            let doc = Artifact {
                profile: args.profile,
                headless: args.headless,
                argv: argv.clone(),
                cwd,
            };
            if let Some(parent) = artifact.out.parent() {
                std::fs::create_dir_all(parent).ok();
            }
            std::fs::write(&artifact.out, serde_json::to_string_pretty(&doc)?)
                .with_context(|| format!("write artifact {}", artifact.out.display()))?;
            println!("Wrote {}", artifact.out.display());
            Ok(())
        }
        None => {
            // Default behavior: run.
            let argv = build_qemu_argv(&cli.run)?;
            let status = run_qemu(&cli.run, &argv)?;
            std::process::exit(status.code().unwrap_or(1));
        }
    }
}

fn repo_root() -> Result<PathBuf> {
    // We live in tools/theseus-qemu; repo root is two parents up.
    let exe_dir = std::env::current_dir()?;
    // We prefer locating by walking upward until we find Cargo.toml with a workspace.
    for ancestor in exe_dir.ancestors() {
        let p = ancestor.join("Cargo.toml");
        if p.is_file() {
            let txt = std::fs::read_to_string(&p).unwrap_or_default();
            if txt.contains("[workspace]") {
                return Ok(ancestor.to_path_buf());
            }
        }
    }
    Err(anyhow!(
        "could not locate repo root (workspace Cargo.toml not found)"
    ))
}

fn apply_relay_defaults(args: &mut Args) {
    if !args.relays {
        return;
    }

    // Enable defaults only when the user hasn't explicitly set alternatives.
    if matches!(args.serial, SerialMode::Off) {
        // For the standard relay setup, QEMU should use the QEMU-side PTY.
        args.serial = SerialMode::PseudoTtyFile;
    }

    if args.monitor_pty.is_none() {
        args.monitor_pty = Some("/tmp/qemu-monitor".to_string());
    }

    if args.debugcon_pty.is_none() {
        args.debugcon_pty = Some("/tmp/qemu-debugcon".to_string());
    }

    if args.qmp.is_none() {
        args.qmp = Some("/tmp/qemu-qmp.sock".to_string());
    }
}

fn build_qemu_argv(args: &Args) -> Result<Vec<String>> {
    let mut args = args.clone();
    apply_relay_defaults(&mut args);

    let root = repo_root()?;

    // Paths matching startQemu.sh
    let ovmf_code = root.join("OVMF/OVMF_CODE.fd");
    let ovmf_vars = root.join("build/OVMF_VARS.fd");
    let disk_img = root.join("build/disk.img");

    // We intentionally do NOT auto-build here (runner should be usable even when build is blocked).
    if !args.dry {
        ensure_exists(&ovmf_code, "OVMF code")?;
        ensure_exists(&ovmf_vars, "OVMF vars")?;
        ensure_exists(&disk_img, "disk image")?;
    }

    let mut argv: Vec<String> = vec!["qemu-system-x86_64".into()];
    argv.extend([
        "-machine".into(),
        format!("q35,accel={},kernel-irqchip={}", args.accel, args.irqchip),
        "-cpu".into(),
        "max".into(),
        "-smp".into(),
        "4".into(),
        "-m".into(),
        "2G".into(),
    ]);

    // Firmware
    argv.extend([
        "-drive".into(),
        format!(
            "if=pflash,format=raw,readonly=on,file={}",
            ovmf_code.display()
        ),
        "-drive".into(),
        format!("if=pflash,format=raw,file={}", ovmf_vars.display()),
    ]);

    // Deterministic exit and debugcon
    argv.extend([
        "-device".into(),
        "isa-debug-exit,iobase=0xf4,iosize=0x04".into(),
        "-device".into(),
        "isa-debugcon,chardev=debugcon".into(),
    ]);

    if args.headless {
        argv.extend(["-display".into(), "none".into()]);
        if let Some(path) = &args.debugcon_pty {
            argv.extend(["-chardev".into(), format!("file,id=debugcon,path={}", path)]);
        } else {
            argv.extend(["-chardev".into(), "stdio,id=debugcon".into()]);
        }
        argv.extend(["-d".into(), "int,guest_errors,cpu_reset".into()]);
    } else {
        if let Some(path) = &args.debugcon_pty {
            argv.extend(["-chardev".into(), format!("file,id=debugcon,path={}", path)]);
        } else {
            argv.extend(["-chardev".into(), "file,id=debugcon,path=debug.log".into()]);
        }
        // default: provide a monitor on stdio if headed.
        argv.extend(["-monitor".into(), "stdio".into()]);
    }

    // Optional serial
    match args.serial {
        SerialMode::Off => {}
        SerialMode::Stdio => {
            argv.extend(["-serial".into(), "stdio".into()]);
        }
        SerialMode::Unix => {
            argv.extend([
                "-serial".into(),
                format!("unix:{},server,nowait", args.serial_path),
            ]);
        }
        SerialMode::PseudoTtyFile => {
            // QEMU does not always include a `tty` chardev backend, but the `file` backend
            // can open an existing PTY device node just fine.
            argv.extend([
                "-chardev".into(),
                format!("file,id=serial0,path={}", args.serial_path),
                "-serial".into(),
                "chardev:serial0".into(),
            ]);
        }
    }

    // Optional QMP/HMP / monitor PTY
    if let Some(qmp) = &args.qmp {
        argv.extend(["-qmp".into(), format!("unix:{},server,nowait", qmp)]);
    }

    // If monitor_pty is set, use it via a tty chardev. Otherwise fall back to HMP unix socket if configured.
    if let Some(mon) = &args.monitor_pty {
        // Use file backend so it can open an existing PTY device node.
        argv.extend([
            "-chardev".into(),
            format!("file,id=mon0,path={}", mon),
            "-monitor".into(),
            "chardev:mon0".into(),
        ]);
    } else if let Some(hmp) = &args.hmp {
        argv.extend(["-monitor".into(), format!("unix:{},server,nowait", hmp)]);
    }

    // Storage: match existing NVMe setup
    argv.extend([
        "-drive".into(),
        format!("if=none,id=nvme0,file={},format=raw", disk_img.display()),
        "-device".into(),
        "nvme,drive=nvme0,serial=deadbeef".into(),
    ]);

    // PCIe root ports
    argv.extend([
        "-device".into(),
        "pcie-root-port,id=rp0,slot=0,chassis=1".into(),
        "-device".into(),
        "pcie-root-port,id=rp1,slot=1,chassis=2".into(),
        "-device".into(),
        "pcie-root-port,id=rp2,slot=2,chassis=3".into(),
    ]);

    // Apply profile-specific devices
    match args.profile {
        Profile::Default => {
            argv.extend(["-device".into(), "virtio-gpu-pci,bus=rp0".into()]);
            argv.extend(["-device".into(), "qemu-xhci,id=xhci0".into()]);
            argv.extend(["-device".into(), "usb-kbd,bus=xhci0.0".into()]);
            argv.extend(["-device".into(), "usb-mouse,bus=xhci0.0".into()]);
            argv.extend([
                "-device".into(),
                "virtio-net-pci,id=nic0,bus=rp2".into(),
                "-nic".into(),
                "none".into(),
            ]);
        }
        Profile::Min => {
            // no gpu/usb/nic
            argv.extend(["-nic".into(), "none".into()]);
        }
        Profile::UsbKbd => {
            argv.extend(["-device".into(), "qemu-xhci,id=xhci0".into()]);
            argv.extend(["-device".into(), "usb-kbd,bus=xhci0.0".into()]);
            argv.extend(["-nic".into(), "none".into()]);
        }
    }

    argv.push("-no-reboot".into());

    // Extra args
    argv.extend(args.extra.iter().cloned());

    Ok(argv)
}

fn run_qemu(args: &Args, argv: &[String]) -> Result<ExitStatus> {
    // Normalize build flags: --no-build wins.
    let build = if args.no_build { false } else { args.build };

    if build {
        run_make_all()?;
    }

    // We run QEMU and capture its combined stdout/stderr to a temp file, while also printing.
    // This enables success-marker detection similar to startQemu.sh.
    let out_path =
        std::env::temp_dir().join(format!("theseus-qemu-output-{}.log", std::process::id()));

    let status = if args.timeout_secs > 0 {
        // Use `timeout` for simplicity (Linux). Keep output merged.
        let mut t = Command::new("timeout");
        t.arg("--foreground")
            .arg(format!("{}s", args.timeout_secs))
            .arg(&argv[0])
            .args(&argv[1..]);
        run_with_tee(t, &out_path).context("run qemu (timeout)")?
    } else {
        let mut cmd = Command::new(&argv[0]);
        cmd.args(&argv[1..]);
        run_with_tee(cmd, &out_path).context("run qemu")?
    };

    // Success marker override: if kernel prints it, force exit code 0.
    if output_contains(&out_path, &args.success_marker)? {
        eprintln!("✓ Detected success marker from kernel");
        return Ok(ExitStatus::from_raw(0));
    }

    // Match startQemu.sh messaging a bit.
    if let Some(code) = status.code() {
        if code == 1 {
            eprintln!("✓ QEMU exited gracefully from guest");
        } else if code == 124 {
            eprintln!("⚠ QEMU timed out after {}s", args.timeout_secs);
        }
    }

    Ok(status)
}

fn ensure_exists(path: &Path, what: &str) -> Result<()> {
    if !path.exists() {
        bail!(
            "Missing {} at {}. Build first (e.g. `make all`).",
            what,
            path.display()
        );
    }
    Ok(())
}

fn run_make_all() -> Result<()> {
    let root = repo_root()?;
    eprintln!("Building project (make all)...");
    let status = Command::new("make")
        .arg("all")
        .current_dir(root)
        .status()
        .context("run make all")?;
    if !status.success() {
        bail!("make all failed with status {status}");
    }
    Ok(())
}

fn run_with_tee(mut cmd: Command, out_path: &Path) -> Result<ExitStatus> {
    let mut child = cmd
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .context("spawn")?;

    let mut out_file = std::fs::File::create(out_path)
        .with_context(|| format!("create {}", out_path.display()))?;

    let mut stdout = child.stdout.take().context("take stdout")?;
    let mut stderr = child.stderr.take().context("take stderr")?;

    let t1 = thread::spawn(move || {
        let mut b = Vec::new();
        let _ = stdout.read_to_end(&mut b);
        b
    });

    let t2 = thread::spawn(move || {
        let mut b = Vec::new();
        let _ = stderr.read_to_end(&mut b);
        b
    });

    let status = child.wait().context("wait")?;

    let stdout_buf = t1.join().unwrap_or_default();
    let stderr_buf = t2.join().unwrap_or_default();

    // Write combined output to file and to our stdout/stderr.
    out_file.write_all(&stdout_buf).ok();
    out_file.write_all(&stderr_buf).ok();
    std::io::stdout().write_all(&stdout_buf).ok();
    std::io::stderr().write_all(&stderr_buf).ok();

    Ok(status)
}

fn output_contains(path: &Path, needle: &str) -> Result<bool> {
    let mut s = String::new();
    std::fs::File::open(path)
        .with_context(|| format!("open {}", path.display()))?
        .read_to_string(&mut s)
        .ok();
    Ok(s.contains(needle))
}

fn shell_join(argv: &[String]) -> String {
    argv.iter().map(shell_escape).collect::<Vec<_>>().join(" ")
}

fn shell_escape(s: &String) -> String {
    if s.chars()
        .all(|c| c.is_ascii_alphanumeric() || "-._/:,=+".contains(c))
    {
        return s.clone();
    }
    format!("'{}'", s.replace('\\', "\\\\").replace('\'', "'\\''"))
}
