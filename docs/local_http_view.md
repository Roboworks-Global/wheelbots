# Serving files over HTTP for browser preview

Quick way to view PDFs, images, log files, or the auto-generated `frames_*.pdf` from `view_frames` in a browser when the Orin is headless / accessed over SSH.

## Start

From the directory containing the files:

```bash
python3 -m http.server 8000 --bind 0.0.0.0
```

- `--bind 0.0.0.0` — listen on every interface, so other hosts on your LAN can reach it. Omit for `127.0.0.1` only.
- The final `8000` is the port; pick anything free.

## Open

- **Directory index:** `http://<orin-ip>:8000/`
- **Specific file:** `http://<orin-ip>:8000/<filename>`

`ip -brief addr show | grep -v DOWN` on the Orin tells you the current IPs. Typical WiFi lease example: `http://192.168.1.15:8000/`.

## Stop

`Ctrl-C` in the shell that started it, or `pkill -f "http.server 8000"`.

## Caveats

- **No authentication.** Anyone on the network segment can list and download every file in the served directory. Serve from a scratch dir that only contains what you want visible; don't run it from `~` or the repo root.
- **Firewall.** If your Orin has a firewall (unusual on JP7.2.1 but check), allow the port: `sudo ufw allow 8000/tcp`.
- **Kill it when done.** It's not a systemd service and won't come back after a reboot, but leaving it running on `0.0.0.0` is bad hygiene.

## Related

- `ros2 run web_video_server web_video_server` — separate service, listens on `:8080`, streams `sensor_msgs/Image` topics as MJPEG / H.264. Use for live camera preview; use `http.server` for static files.
