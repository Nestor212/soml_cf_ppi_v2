#!/usr/bin/env python3
"""
Subscribe to MQTT and print messages published by the PPI device.

Usage examples:
  python test_utils/led_client_gui/ppi_mqtt_monitor.py --host 192.168.1.51
  python test_utils/led_client_gui/ppi_mqtt_monitor.py --host 192.168.1.51 --topic "spBv1.0/#"
  python test_utils/led_client_gui/ppi_mqtt_monitor.py --host 192.168.1.51 --all

Install dependency:
  pip install paho-mqtt
"""

import argparse
import datetime as dt
import json
import re
import struct
import sys
import time

try:
    import paho.mqtt.client as mqtt
except Exception:
    print("Missing dependency: paho-mqtt")
    print("Install with: pip install paho-mqtt")
    sys.exit(1)


DEFAULT_TOPIC = "spBv1.0/#"
DEFAULT_NODE_PATTERN = r"PPI\\d+"
SPARKPLUG_VERSION = "spBv1.0"
DEFAULT_HOST_ID = "FCC"

# ─── Sparkplug B decoder ──────────────────────────────────────────────────────
# Alias → (metric_name, type_hint)  sourced from og_ppi_metrics.hpp + og_ppi.cpp
ALIAS_MAP: dict[int, tuple[str, str]] = {
    0:  ("bdSeq",                                    "UInt64"),
    1:  ("Node Control/Reboot",                      "Boolean"),
    2:  ("Node Control/Rebirth",                     "Boolean"),
    3:  ("Node Control/Next Server",                 "Boolean"),
    4:  ("Properties/Communications Version",        "Int64"),
    5:  ("Properties/Firmware Version",              "String"),
    6:  ("Properties/Unique Id",                     "UInt64"),
    7:  ("Properties/Communications Channel",        "Int32"),
    8:  ("Inputs/NTP Synchronized",                  "Boolean"),
    9:  ("Inputs/NTP Last Update Time",              "UInt64"),
    10: ("Inputs/NTP Server",                        "String"),
    11: ("Inputs/Panel Status",                      "String"),
    12: ("Inputs/Active Schedule",                   "DataSet"),
    13: ("Inputs/Active Schedule Start",             "UInt64"),
    14: ("Inputs/Command Counter",                   "UInt32"),
    15: ("Inputs/Command Index",                     "Int32"),
    16: ("Inputs/Command Sent NTP",                  "UInt64"),
    17: ("Inputs/Command Sent Micros",               "UInt32"),
    18: ("Inputs/Heater Output",                     "UInt32"),
    19: ("Inputs/Voltage Monitor",                   "UInt32"),
    20: ("Inputs/Local Current Monitor",             "UInt32"),
    21: ("Inputs/Remote Current Monitor",            "UInt32"),
    22: ("Inputs/Error Lamps",                       "UInt32"),
    23: ("Inputs/Cycle Period",                      "UInt32"),
    24: ("Inputs/Cycle NTP Offset",                  "Int32"),
    25: ("Outputs/Next Schedule",                    "DataSet"),
    26: ("Outputs/Next Schedule Start",              "UInt64"),
    27: ("Outputs/Voltage Sensor Down",              "UInt32"),
    28: ("Outputs/Local Current Sensor Down",        "UInt32"),
    29: ("Outputs/Remote Current Sensor Down",       "UInt32"),
}

SP_DATATYPES: dict[int, str] = {
    0: "Unknown", 1: "Int8", 2: "Int16", 3: "Int32", 4: "Int64",
    5: "UInt8", 6: "UInt16", 7: "UInt32", 8: "UInt64",
    9: "Float", 10: "Double", 11: "Boolean", 12: "String",
    13: "DateTime", 14: "Text", 15: "UUID", 16: "DataSet",
    17: "Bytes", 18: "File", 19: "Template",
}

# Timestamp-like aliases (UInt64 epoch ms) — format as datetime
_TS_ALIASES = {9, 13, 16, 26}
_HEATER_OUTPUT_ALIAS = 18  # format as relay bitmask


def _read_varint(data: bytes, pos: int) -> tuple:
    result, shift = 0, 0
    while pos < len(data):
        b = data[pos]; pos += 1
        result |= (b & 0x7F) << shift
        shift += 7
        if not (b & 0x80):
            break
    return result, pos


def _skip_field(data: bytes, pos: int, wire: int) -> int:
    if wire == 0:
        while pos < len(data) and (data[pos] & 0x80): pos += 1
        pos += 1
    elif wire == 1:
        pos += 8
    elif wire == 2:
        length, pos = _read_varint(data, pos)
        pos += length
    elif wire == 5:
        pos += 4
    return pos


def _decode_dataset(data: bytes) -> str:
    num_cols, col_names, num_rows = 0, [], 0
    pos = 0
    while pos < len(data):
        tag, pos = _read_varint(data, pos)
        field, wire = tag >> 3, tag & 0x7
        if wire == 0:
            val, pos = _read_varint(data, pos)
            if field == 1: num_cols = val
        elif wire == 2:
            length, pos = _read_varint(data, pos)
            chunk = data[pos:pos+length]
            if field == 3: col_names.append(chunk.decode("utf-8", errors="replace"))
            elif field == 4: num_rows += 1
            pos += length
        else:
            pos = _skip_field(data, pos, wire)
    return f"DataSet({num_cols} cols {col_names}, {num_rows} rows)"


def _decode_metric(data: bytes) -> dict:
    m: dict = {}
    pos = 0
    while pos < len(data):
        tag, pos = _read_varint(data, pos)
        field, wire = tag >> 3, tag & 0x7
        if wire == 0:
            val, pos = _read_varint(data, pos)
            if   field == 2:  m["alias"]      = val
            elif field == 3:  m["ts"]          = val
            elif field == 4:  m["datatype"]    = val
            elif field == 10: m["int_value"]   = val
            elif field == 11: m["long_value"]  = val
            elif field == 14: m["bool_value"]  = bool(val)
        elif wire == 1:
            if pos + 8 <= len(data):
                if field == 13: m["double_value"] = struct.unpack_from("<d", data, pos)[0]
            pos += 8
        elif wire == 2:
            length, pos = _read_varint(data, pos)
            chunk = data[pos:pos+length]
            if   field == 1:  m["name"]          = chunk.decode("utf-8", errors="replace")
            elif field == 15: m["str_value"]      = chunk.decode("utf-8", errors="replace")
            elif field == 16: m["bytes_value"]    = chunk
            elif field == 17: m["dataset_value"]  = _decode_dataset(chunk)
            pos += length
        elif wire == 5:
            if pos + 4 <= len(data):
                if field == 12: m["float_value"] = struct.unpack_from("<f", data, pos)[0]
            pos += 4
        else:
            break
    return m


def _fmt_epoch_ms(ms: int) -> str:
    try:
        return dt.datetime.fromtimestamp(ms / 1000.0).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    except Exception:
        return "?"


def _fmt_metric_value(m: dict) -> str:
    alias = m.get("alias")
    if "str_value"     in m: return repr(m["str_value"])
    if "bool_value"    in m: return str(m["bool_value"])
    if "float_value"   in m: return f"{m['float_value']:.6g}"
    if "double_value"  in m: return f"{m['double_value']:.10g}"
    if "bytes_value"   in m: return f"<bytes {len(m['bytes_value'])}B>"
    if "dataset_value" in m: return m["dataset_value"]
    if "int_value"     in m:
        v = m["int_value"]
        if alias == _HEATER_OUTPUT_ALIAS:
            on_bits = [i for i in range(30) if v & (1 << i)]
            return f"0x{v:08X}  heaters ON={on_bits if on_bits else 'none'}"
        return str(v)
    if "long_value"    in m:
        v = m["long_value"]
        if alias in _TS_ALIASES:
            return f"{v}  ({_fmt_epoch_ms(v)})"
        return str(v)
    return "<no value>"


def _is_sparkplug_topic(topic: str) -> bool:
    parts = topic.split("/")
    return (len(parts) >= 4 and parts[0] == SPARKPLUG_VERSION and parts[2] in (
        "NBIRTH", "NDATA", "NDEATH", "NCMD", "DBIRTH", "DDATA", "DDEATH", "DCMD"))


def decode_sparkplug(data: bytes) -> str:
    try:
        result: dict = {"ts": None, "seq": None, "metrics": []}
        pos = 0
        while pos < len(data):
            tag, pos = _read_varint(data, pos)
            field, wire = tag >> 3, tag & 0x7
            if wire == 0:
                val, pos = _read_varint(data, pos)
                if field == 1:   result["ts"]  = val
                elif field == 4: result["seq"] = val
            elif wire == 2:
                length, pos = _read_varint(data, pos)
                if field == 2:
                    result["metrics"].append(_decode_metric(data[pos:pos+length]))
                pos += length
            else:
                pos = _skip_field(data, pos, wire)

        lines = []
        ts, seq = result["ts"], result["seq"]
        if ts:
            lines.append(f"  seq={seq}  ts={ts}  ({_fmt_epoch_ms(ts)})")
        for m in result["metrics"]:
            alias = m.get("alias")
            name  = m.get("name") or (ALIAS_MAP.get(alias, (f"alias#{alias}", "?"))[0]
                                       if alias is not None else "?")
            dtype = SP_DATATYPES.get(m.get("datatype", 0), "?")
            lines.append(f"  {name}  [{dtype}]  = {_fmt_metric_value(m)}")
        return "\n".join(lines) if lines else "<empty SpB payload>"
    except Exception as exc:
        return f"<SpB decode error: {exc}>"


def now_str() -> str:
    return dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def payload_preview(payload: bytes, max_chars: int = 240) -> str:
    if not payload:
        return "<empty>"
    # Caller supplies topic so Sparkplug binary is decoded; fallback handled below.
    return "<binary>"


def should_print(topic: str, args: argparse.Namespace) -> bool:
    if args.all:
        return True

    # Match node token in topic, e.g. .../PPI1
    if re.search(args.node_regex, topic):
        return True

    return False


def host_state_topic(host_id: str) -> str:
    return f"{SPARKPLUG_VERSION}/STATE/{host_id}"


def publish_host_online(client: mqtt.Client, host_id: str, online: bool = True) -> None:
    topic = host_state_topic(host_id)
    epoch_ms = int(time.time() * 1000)
    payload = json.dumps({"online": online, "timestamp": epoch_ms})
    result = client.publish(topic, payload, qos=1, retain=True)
    state = "ONLINE" if online else "OFFLINE"
    ok = "queued" if result.rc == 0 else f"FAILED rc={result.rc}"
    print(f"[{now_str()}] [host-sim] STATE {state} -> {topic}  ({ok})  payload={payload}")


def on_connect(client: mqtt.Client, userdata, flags, reason_code, properties=None):
    args = userdata["args"]
    print(f"[{now_str()}] Connected rc={reason_code}; subscribing to: {args.topic}")
    userdata["connected"] = True
    if args.simulate_host:
        publish_host_online(client, args.host_id, online=True)
    client.subscribe(args.topic, qos=args.qos)


def on_disconnect(client: mqtt.Client, userdata, reason_code, properties=None):
    print(f"[{now_str()}] Disconnected rc={reason_code}")


def on_subscribe(client: mqtt.Client, userdata, mid, granted_qos_or_reason_codes, properties=None):
    codes = granted_qos_or_reason_codes
    # Detect any failed grants (paho v2 returns ReasonCode objects; v1 returns ints)
    failed = []
    for c in (codes if hasattr(codes, '__iter__') else [codes]):
        val = c.value if hasattr(c, 'value') else c
        if val >= 0x80:
            failed.append(str(c))
    if failed:
        print(f"[{now_str()}] WARNING: Subscription rejected by broker ACL: {failed}")
        print(f"[{now_str()}] Try a narrower --topic (e.g. spBv1.0/PS/# ) or add credentials with --username/--password")
    else:
        print(f"[{now_str()}] Subscription confirmed (mid={mid}, granted={granted_qos_or_reason_codes})")


def on_publish(client: mqtt.Client, userdata, mid, reason_code=None, properties=None):
    print(f"[{now_str()}] Publish confirmed (mid={mid})")

def on_message(client: mqtt.Client, userdata, msg: mqtt.MQTTMessage):
    args = userdata["args"]
    userdata["msg_count"] += 1
    if not should_print(msg.topic, args):
        return

    if _is_sparkplug_topic(msg.topic):
        preview = decode_sparkplug(msg.payload)
    else:
        # Text / JSON (STATE messages etc.)
        try:
            text = msg.payload.decode("utf-8").strip()
            if text.startswith("{") or text.startswith("["):
                text = json.dumps(json.loads(text), indent=2, sort_keys=True)
            preview = text if text else "<whitespace>"
        except Exception:
            hex_data = msg.payload.hex()
            if len(hex_data) > 240: hex_data = hex_data[:240] + " ..."
            preview = "<binary hex> " + hex_data

    print(
        f"\n[{now_str()}] topic={msg.topic} qos={msg.qos} retain={int(msg.retain)} bytes={len(msg.payload)}"
    )
    print(preview)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Monitor MQTT traffic and print messages published by the PPI device"
    )
    parser.add_argument("--host", required=True, help="MQTT broker host/IP")
    parser.add_argument("--port", type=int, default=1883, help="MQTT broker port")
    parser.add_argument("--topic", default=DEFAULT_TOPIC, help="Subscription topic filter")
    parser.add_argument("--qos", type=int, default=0, choices=[0, 1, 2], help="Subscription QoS")
    parser.add_argument("--client-id", default="ppi-monitor", help="MQTT client ID")
    parser.add_argument("--username", default="", help="MQTT username")
    parser.add_argument("--password", default="", help="MQTT password")
    parser.add_argument(
        "--node-regex",
        default=DEFAULT_NODE_PATTERN,
        help="Regex to identify this device in topic names (default: PPI\\d+)",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Print all subscribed messages (disable node-regex filtering)",
    )
    parser.add_argument(
        "--heartbeat",
        type=int,
        default=10,
        metavar="SECONDS",
        help="Print a 'still listening' line every N seconds when no messages arrive (0 = disable, default: 10)",
    )
    parser.add_argument(
        "--simulate-host",
        action="store_true",
        help="Act as the Sparkplug primary host: publish STATE online on connect and STATE offline on exit",
    )
    parser.add_argument(
        "--host-id",
        default=DEFAULT_HOST_ID,
        help=f"Sparkplug host application ID used with --simulate-host (default: {DEFAULT_HOST_ID})",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()

    # paho-mqtt v1/v2 compatibility.
    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=args.client_id)
    except Exception:
        client = mqtt.Client(client_id=args.client_id)

    if args.username:
        client.username_pw_set(args.username, args.password)

    userdata = {"args": args, "connected": False, "msg_count": 0}
    client.user_data_set(userdata)
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_subscribe = on_subscribe
    client.on_publish = on_publish
    client.on_message = on_message

    print(f"[{now_str()}] Connecting to MQTT broker at {args.host}:{args.port} ...")
    if args.simulate_host:
        print(f"[{now_str()}] [host-sim] ENABLED — will publish STATE online as host '{args.host_id}' on connect")
    else:
        print(f"[{now_str()}] [host-sim] DISABLED — pass --simulate-host to unlock device publishing")
    client.connect(args.host, args.port, keepalive=30)

    if args.heartbeat <= 0:
        # No heartbeat — use the simple blocking loop.
        try:
            client.loop_forever(retry_first_connection=True)
        except KeyboardInterrupt:
            print(f"\n[{now_str()}] Stopped by user")
        finally:
            if args.simulate_host and userdata["connected"]:
                publish_host_online(client, args.host_id, online=False)
            try:
                client.disconnect()
            except Exception:
                pass
        return 0

    # Heartbeat mode: drive the network loop ourselves so we can print a
    # periodic "still listening" line whenever no messages have arrived.
    client.loop_start()
    last_heartbeat = time.monotonic()
    last_count = 0
    try:
        while True:
            time.sleep(0.5)
            now = time.monotonic()
            if now - last_heartbeat >= args.heartbeat:
                current_count = userdata["msg_count"]
                new = current_count - last_count
                last_count = current_count
                status = f"{new} new" if new else "no messages"
                print(f"[{now_str()}] ... still listening ({status} in last {args.heartbeat}s, {current_count} total)")
                last_heartbeat = now
    except KeyboardInterrupt:
        print(f"\n[{now_str()}] Stopped by user")
    finally:
        if args.simulate_host and userdata["connected"]:
            publish_host_online(client, args.host_id, online=False)
            time.sleep(0.2)  # brief flush before stopping the network loop
        client.loop_stop()
        try:
            client.disconnect()
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
