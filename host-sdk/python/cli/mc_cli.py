#!/usr/bin/env python3
"""
mc_cli.py — Command-Line Interface for stepper-motion-core and 3D Scanner.
Usage examples:
  mc-cli status --mock
  mc-cli jog --axis A --deg 15 --mock
  mc-cli home --mock
  mc-cli scan --poses 56 --settle-ms 150 --out-dir ./scan_output --mock
  mc-cli estop --mock
  mc-cli reset --mock
"""

import os
import sys
import click

# Ensure parent directory is on sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from motion_client import ScannerController


@click.group()
@click.option("--port", default=None, help="Serial port (e.g. COM3 or /dev/ttyUSB0).")
@click.option("--baud", default=115200, type=int, help="Serial baud rate.")
@click.option("--mock", is_flag=True, default=False, help="Use software mock simulation (no hardware required).")
@click.pass_context
def cli(ctx, port, baud, mock):
    """stepper-motion-core CLI Tool."""
    ctx.ensure_object(dict)
    ctx.obj["port"] = port
    ctx.obj["baud"] = baud
    ctx.obj["mock"] = mock or (port is None)


def _get_scanner(ctx) -> ScannerController:
    scanner = ScannerController()
    scanner.connect(port=ctx.obj["port"], baudrate=ctx.obj["baud"], mock=ctx.obj["mock"])
    return scanner


@cli.command()
@click.pass_context
def status(ctx):
    """Query and display real-time motion and safety status."""
    scanner = _get_scanner(ctx)
    try:
        st = scanner.get_status()
        click.echo("=== System Status ===")
        click.echo(f"  State:        {st.get('state', 'unknown')}")
        click.echo(f"  E-STOP:       {st.get('estop_active', False)}")
        click.echo(f"  Moving:       {st.get('moving', False)}")
        pos = st.get("pos", {})
        click.echo(f"  Coordinates:  A={pos.get('A', 0.0):.2f}°, B={pos.get('B', 0.0):.2f}°, Z={pos.get('Z', 0.0):.2f}mm")
    finally:
        scanner.disconnect()


@cli.command()
@click.option("--axis", default="A", help="Axis to jog (A, B, or Z).")
@click.option("--deg", "--dist", "distance", default=10.0, type=float, help="Jog relative angle or distance.")
@click.option("--feedrate", default=0.0, type=float, help="Jog feedrate (0 for default).")
@click.pass_context
def jog(ctx, axis, distance, feedrate):
    """Jog an axis by relative distance or angle."""
    scanner = _get_scanner(ctx)
    try:
        res = scanner.jog(axis, distance, feedrate)
        click.echo(f"Jogging axis {axis} by {distance} -> {res}")
    finally:
        scanner.disconnect()


@cli.command()
@click.option("--axis", default=None, help="Axis to home (or omit for all axes).")
@click.pass_context
def home(ctx, axis):
    """Execute homing sequence on one or all axes."""
    scanner = _get_scanner(ctx)
    try:
        click.echo(f"Homing {'axis ' + axis if axis else 'all axes'}...")
        res = scanner.home([axis] if axis else None)
        click.echo(f"Homing completed: {res}")
    finally:
        scanner.disconnect()


@cli.command()
@click.option("--a", default=None, type=float, help="Target Azimuth angle (degrees).")
@click.option("--b", default=None, type=float, help="Target Elevation angle (degrees).")
@click.option("--z", default=None, type=float, help="Target Focus Z distance (mm).")
@click.option("--profile", default="scurve", type=click.Choice(["scurve", "trapezoidal"]), help="Velocity profile.")
@click.option("--feedrate", default=0.0, type=float, help="Speed limit.")
@click.pass_context
def move(ctx, a, b, z, profile, feedrate):
    """Absolute coordinated move across axes."""
    axes = {}
    if a is not None: axes["A"] = a
    if b is not None: axes["B"] = b
    if z is not None: axes["Z"] = z
    if not axes:
        click.echo("Error: At least one axis target (--a, --b, or --z) must be provided.")
        return

    scanner = _get_scanner(ctx)
    try:
        res = scanner.move_to(axes, profile=profile, feedrate=feedrate)
        click.echo(f"Move queued: {res}")
    finally:
        scanner.disconnect()


@cli.command()
@click.option("--poses", default=56, type=int, help="Number of Fibonacci sphere poses.")
@click.option("--settle-ms", default=150, type=int, help="Vibration settling time in ms.")
@click.option("--out-dir", default="./scan_output", help="Directory to save exported poses and point clouds.")
@click.pass_context
def scan(ctx, poses, settle_ms, out_dir):
    """Execute an automated 3D photogrammetry scan session and export metadata."""
    scanner = _get_scanner(ctx)
    try:
        click.echo(f"Starting 3D scan session: {poses} Fibonacci poses, {settle_ms}ms settle delay...")
        os.makedirs(out_dir, exist_ok=True)

        with click.progressbar(length=poses, label="Scanning poses") as bar:
            def _progress(frame_id, total, rec):
                bar.update(1)

            scanner.run_scan(n_poses=poses, settle_ms=settle_ms, progress_cb=_progress)

        click.echo(f"Scan complete! Exporting photogrammetry files to {out_dir}...")
        exported = scanner.exporter.export_all(out_dir)
        for fmt, path in exported.items():
            click.echo(f"  [+] {fmt:16s}: {path}")
    finally:
        scanner.disconnect()


@cli.command()
@click.option("--reason", default="CLI Emergency Stop", help="Reason for emergency stop.")
@click.pass_context
def estop(ctx, reason):
    """Emergency Stop: immediately halts all motion and enters locked safety state."""
    scanner = _get_scanner(ctx)
    try:
        res = scanner.emergency_stop(reason)
        click.echo(f"[EMERGENCY STOP TRIGGERED] {res}")
    finally:
        scanner.disconnect()


@cli.command()
@click.pass_context
def reset(ctx):
    """Reset system from E-STOP lock back to operational IDLE state."""
    scanner = _get_scanner(ctx)
    try:
        res = scanner.reset()
        click.echo(f"System reset: {res}")
    finally:
        scanner.disconnect()


if __name__ == "__main__":
    cli()
