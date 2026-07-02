#!/usr/bin/env python3
#cd ~/tof_test_ws
#python3 tools/maixsense_at_config.py --device /dev/ttyUSB0

"""Interactive AT configuration helper for Sipeed MaixSense A010.

This tool is intentionally conservative:
- it stops USB streaming before sending AT queries/commands;
- it lets the user leave each field blank to keep the current value;
- it saves every planned command set to JSON before applying it;
- it requires an explicit YES before changing BAUD.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import serial


QUERY_COMMANDS = {
    "FPS": "AT+FPS?",
    "BAUD": "AT+BAUD?",
    "UNIT": "AT+UNIT?",
    "BINN": "AT+BINN?",
    "DISP": "AT+DISP?",
}

CONFIG_FIELDS = [
    {
        "key": "FPS",
        "command": "AT+FPS={value}",
        "prompt": "FPS desejado",
        "hint": "ex.: 10 ou 15. Menor FPS reduz fluxo de dados.",
        "allowed": None,
        "dangerous": False,
    },
    {
        "key": "BINN",
        "command": "AT+BINN={value}",
        "prompt": "BINN desejado",
        "hint": "ex.: 1. Deixe vazio para manter.",
        "allowed": None,
        "dangerous": False,
    },
    {
        "key": "UNIT",
        "command": "AT+UNIT={value}",
        "prompt": "UNIT desejado",
        "hint": "muda quantizacao/distancia. Nao altere sem medir/calibrar.",
        "allowed": None,
        "dangerous": False,
    },
    {
        "key": "BAUD",
        "command": "AT+BAUD={value}",
        "prompt": "BAUD desejado",
        "hint": "perigoso: pode quebrar comunicacao se host e sensor divergirem.",
        "allowed": None,
        "dangerous": True,
    },
]


def open_serial(device: str, baudrate: int, timeout: float) -> serial.Serial:
    return serial.Serial(device, baudrate, timeout=timeout, write_timeout=1.0)


def read_available(ser: serial.Serial, wait: float, max_bytes: int = 8192) -> bytes:
    time.sleep(wait)
    chunks: List[bytes] = []
    deadline = time.monotonic() + wait
    while time.monotonic() < deadline:
        waiting = ser.in_waiting
        if waiting <= 0:
            time.sleep(0.05)
            continue
        chunks.append(ser.read(min(waiting, max_bytes)))
    waiting = ser.in_waiting
    if waiting > 0:
        chunks.append(ser.read(min(waiting, max_bytes)))
    return b"".join(chunks)


def send_at(
    ser: serial.Serial,
    command: str,
    wait: float = 0.8,
    stop_stream_first: bool = False,
) -> str:
    if stop_stream_first:
        ser.reset_input_buffer()
        ser.write(b"AT+DISP=1\r")
        time.sleep(0.8)
        _ = read_available(ser, 0.3)

    ser.reset_input_buffer()
    ser.write(command.encode("ascii") + b"\r")
    response = read_available(ser, wait)
    return response.decode("utf-8", errors="replace")


def query_current(ser: serial.Serial) -> Dict[str, str]:
    current: Dict[str, str] = {}
    print("\nParando stream USB antes de consultar parametros...")
    print("AT+DISP=1 ->", repr(send_at(ser, "AT+DISP=1", wait=0.8)))

    for key, command in QUERY_COMMANDS.items():
        response = send_at(ser, command, wait=0.8)
        current[key] = response.strip()
        print(f"{command} -> {repr(current[key])}")
    return current


def ask_value(field: dict, current: Dict[str, str]) -> Optional[str]:
    key = field["key"]
    print(f"\n{field['prompt']} ({field['hint']})")
    if current.get(key):
        print(f"Atual: {current[key]}")
    value = input("Novo valor [enter = manter]: ").strip()
    if not value:
        return None

    allowed = field.get("allowed")
    if allowed is not None and value not in allowed:
        raise ValueError(f"Valor invalido para {key}: {value}. Permitidos: {allowed}")

    if field.get("dangerous"):
        print("\nATENCAO: mudar BAUD pode fazer o sensor parar de responder nesta porta.")
        confirm = input("Digite YES para permitir alteracao de BAUD: ").strip()
        if confirm != "YES":
            print("BAUD ignorado.")
            return None

    return value


def collect_plan(current: Dict[str, str]) -> List[Tuple[str, str, str]]:
    plan: List[Tuple[str, str, str]] = []
    print("\nPreencha apenas o que deseja mudar.")
    for field in CONFIG_FIELDS:
        value = ask_value(field, current)
        if value is None:
            continue
        command = field["command"].format(value=value)
        plan.append((field["key"], value, command))

    print("\nComandos customizados opcionais.")
    print("Use somente se souber o comando AT correto. Enter vazio termina.")
    print("Exemplos:")
    print("  AT+BAUD?     consulta baud configurado")
    print("  AT+FPS?      consulta FPS configurado")
    print("  AT+FPS=10    altera FPS para 10")
    print("  AT+DISP=1    para stream USB")
    print("  AT+DISP=3    inicia stream USB")
    while True:
        custom = input("AT customizado, ex. AT+FPS? [enter = fim]: ").strip()
        if not custom:
            break
        if not custom.startswith("AT"):
            print("Ignorado: comando precisa comecar com AT.")
            continue
        plan.append(("CUSTOM", custom, custom))

    return plan


def save_plan(
    output_dir: Path,
    device: str,
    baudrate: int,
    current: Dict[str, str],
    plan: List[Tuple[str, str, str]],
) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = output_dir / f"maixsense_at_config_{stamp}.json"
    data = {
        "timestamp": stamp,
        "device": device,
        "host_baudrate": baudrate,
        "current": current,
        "commands": [
            {"key": key, "value": value, "command": command}
            for key, value, command in plan
        ],
    }
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n")
    return path


def apply_plan(ser: serial.Serial, plan: List[Tuple[str, str, str]]) -> Dict[str, str]:
    responses: Dict[str, str] = {}
    for key, _value, command in plan:
        print(f"\nEnviando {command}")
        response = send_at(ser, command, wait=1.0)
        responses[command] = response.strip()
        print("Resposta:", repr(responses[command]))
        if "ERROR" in response.upper():
            print("ERRO retornado pelo sensor. Parando aplicacao do plano.")
            break
        if key == "BAUD":
            print("BAUD alterado. Talvez seja necessario reabrir a serial com outro baudrate.")
            break
    return responses


def main() -> int:
    parser = argparse.ArgumentParser(description="Configurar MaixSense A010 via AT")
    parser.add_argument("--device", default="/dev/tof", help="porta serial")
    parser.add_argument("--baudrate", type=int, default=115200, help="baudrate do host")
    parser.add_argument("--timeout", type=float, default=1.0, help="timeout serial")
    parser.add_argument(
        "--output-dir",
        default="maixsense_at_configs",
        help="diretorio onde salvar JSON dos comandos",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="salva o plano, mas nao envia comandos",
    )
    args = parser.parse_args()

    print("MaixSense A010 AT config helper")
    print(f"Device: {args.device}")
    print(f"Host baudrate: {args.baudrate}")

    try:
        ser = open_serial(args.device, args.baudrate, args.timeout)
    except Exception as exc:
        print(f"Falha abrindo serial: {exc}", file=sys.stderr)
        return 2

    with ser:
        current = query_current(ser)
        plan = collect_plan(current)
        if not plan:
            print("Nenhuma alteracao selecionada.")
            return 0

        print("\nPlano de comandos:")
        for _key, _value, command in plan:
            print(f"  {command}")

        saved_path = save_plan(Path(args.output_dir), args.device, args.baudrate, current, plan)
        print(f"\nPlano salvo em: {saved_path}")

        if args.dry_run:
            print("dry-run ativo: comandos nao enviados.")
            return 0

        confirm = input("\nEnviar comandos ao sensor agora? [yes/NO]: ").strip().lower()
        if confirm != "yes":
            print("Cancelado. Nada foi enviado.")
            return 0

        responses = apply_plan(ser, plan)
        response_path = saved_path.with_suffix(".responses.json")
        response_path.write_text(json.dumps(responses, indent=2, ensure_ascii=False) + "\n")
        print(f"\nRespostas salvas em: {response_path}")

        print("\nReconsultando parametros principais:")
        _ = query_current(ser)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
