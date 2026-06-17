import can
import struct
import json
import ssl
import time
import pandas as pd
import paho.mqtt.client as mqtt

from datetime import datetime

# =========================================================
# CONFIG MQTT HIVEMQ
# =========================================================

MQTT_BROKER = "1caad7433f0748fdb74c5d4e9a427af1.s1.eu.hivemq.cloud"
MQTT_PORT = 8883

MQTT_USERNAME = "bms_user"
MQTT_PASSWORD = "Monpassword123"

MQTT_BASE_TOPIC = "bms"

# =========================================================
# CLIENT MQTT
# =========================================================

mqtt_client = mqtt.Client()

# login 
mqtt_client.username_pw_set(
    MQTT_USERNAME,
    MQTT_PASSWORD
)

# TLS obligatoire HiveMQ Cloud
mqtt_client.tls_set(
    tls_version=ssl.PROTOCOL_TLS
)

# reconnexion auto
mqtt_client.reconnect_delay_set(
    min_delay=1,
    max_delay=120
)

# =========================================================
# CALLBACKS MQTT
# =========================================================

def on_connect(client, userdata, flags, rc):

    if rc == 0:
        print("MQTT CONNECTED")
    else:
        print(f"MQTT ERROR : {rc}")

def on_disconnect(client, userdata, rc):

    print("MQTT DISCONNECTED")

mqtt_client.on_connect = on_connect
mqtt_client.on_disconnect = on_disconnect

# =========================================================
# CONNEXION MQTT
# =========================================================

while True:

    try:

        mqtt_client.connect(
            MQTT_BROKER,
            MQTT_PORT,
            60
        )

        mqtt_client.loop_start()

        break

    except Exception as e:

        print("Connexion MQTT impossible :", e)
        time.sleep(5)

# =========================================================
# PACKERS CAN
# =========================================================

list_packer = [
    ">hhhh",       # N0
    ">lhbb",       # N1
    ">ll",         # N2
    ">ll",         # N3
    ">hhbbbb",     # N4
    ">hbhbbb",     # N5
    ">bbbbbbbb",   # N6
    ">hbbbbbb"     # N7
]

# =========================================================
# OUTILS
# =========================================================

def string_like_byte_array(data, packer):

    byte_data = bytes(data)

    return struct.unpack(
        packer,
        byte_data
    )

# =========================================================
# DECODE CAN
# =========================================================

def decode_message(address, values):

    # =====================================================
    # N0
    # =====================================================

    if address == 0:

        df = pd.DataFrame(
            [values],
            columns=[
                'total_voltage_V',
                'current_in_A',
                'current_out_A',
                'current_bat_A'
            ]
        )

        df = df / 10

    # =====================================================
    # N1
    # =====================================================

    elif address == 1:

        df = pd.DataFrame(
            [values],
            columns=[
                'Energy_stored_Wh',
                'Battery_capacity_kWh',
                'SOC_percent',
                'unused'
            ]
        )

    # =====================================================
    # N2
    # =====================================================

    elif address == 2:

        df = pd.DataFrame(
            [values],
            columns=[
                'Energy_today_collected_kWh',
                'Energy_today_consummed_kWh'
            ]
        )

    # =====================================================
    # N3
    # =====================================================

    elif address == 3:

        df = pd.DataFrame(
            [values],
            columns=[
                'Total_energy_collected_kWh',
                'Total_energy_consummed_kWh'
            ]
        )

    # =====================================================
    # N4
    # =====================================================

    elif address == 4:

        df = pd.DataFrame(
            [values],
            columns=[
                'Cell_voltage_min_V',
                'Cell_voltage_max_V',
                'Cell_voltage_bypass_V',
                'unused1',
                'unused2',
                'unused3'
            ]
        )

        df['Cell_voltage_min_V'] /= 1000
        df['Cell_voltage_max_V'] /= 1000
        df['Cell_voltage_bypass_V'] /= 1000

    # =====================================================
    # N5
    # =====================================================

    elif address == 5:

        df = pd.DataFrame(
            [values],
            columns=[
                'Cell_voltage_lowest_V',
                'Low_nr',
                'Cell_voltage_highest_V',
                'High_nr',
                'Sbyte1',
                'Sbyte2'
            ]
        )

        df['Cell_voltage_lowest_V'] /= 1000
        df['Cell_voltage_highest_V'] /= 1000

    # =====================================================
    # N6
    # =====================================================

    elif address == 6:

        df = pd.DataFrame(
            [values],
            columns=[
                'Tmp_lowest_deg',
                'Low_nr',
                'Tmp_highest_deg',
                'High_nr',
                'Min_charg_temp_deg',
                'Min_dis_temp_deg',
                'Max_temp_deg',
                'unused'
            ]
        )

    # =====================================================
    # N7
    # =====================================================

    elif address == 7:

        df = pd.DataFrame(
            [values],
            columns=[
                'Current_Cell_voltage_V',
                'Curr_temp_deg',
                'Curr_nr',
                'Cell_cnt',
                'unused1',
                'Isolation_resistance_kOhm',
                'unused2'
            ]
        )

        df['Current_Cell_voltage_V'] /= 1000

    else:

        return None

    return df

# =========================================================
# MQTT PUBLISH
# =========================================================

def publish_mqtt(address, df):

    timestamp = datetime.now().isoformat()

    payload = df.iloc[0].to_dict()

    payload["timestamp"] = timestamp

    # =====================================================
    # CELLULES
    # =====================================================

    if address == 7:

        cell_number = int(payload["Curr_nr"])

        if cell_number < 1 or cell_number > 8:
            return

        topic = f"{MQTT_BASE_TOPIC}/cell/C{cell_number}"

    # =====================================================
    # N0 -> N6
    # =====================================================

    else:

        topic = f"{MQTT_BASE_TOPIC}/N{address}"

    # =====================================================
    # JSON
    # =====================================================

    json_payload = json.dumps(payload)

    # =====================================================
    # PUBLISH
    # =====================================================

    mqtt_client.publish(
        topic,
        json_payload,
        qos=1
    )

    print(f"[MQTT] {topic}")
    print(json_payload)

# =========================================================
# INITIALISATION CAN
# =========================================================

print("Connexion CAN...")

bus = can.interface.Bus(
    channel='can0',
    interface='socketcan'
)

print("CAN READY")

# =========================================================
# BOUCLE PRINCIPALE
# =========================================================

while True:

    try:

        # lecture CAN
        message = bus.recv()

        if message is None:
            continue

        address = int(
            message.arbitration_id
        )

        # seulement N0 -> N7
        if address < 0 or address > 7:
            continue

        # =================================================
        # PACKER
        # =================================================

        packer = list_packer[address]

        # =================================================
        # UNPACK
        # =================================================

        values = string_like_byte_array(
            message.data,
            packer
        )

        # =================================================
        # FILTRE CELLULES
        # =================================================

        if address == 7:

            cell_number = values[2]

            if cell_number < 1 or cell_number > 8:
                continue

        # =================================================
        # DECODE
        # =================================================

        df = decode_message(
            address,
            values
        )

        # =================================================
        # MQTT
        # =================================================

        if df is not None:

            publish_mqtt(
                address,
                df
            )

    except KeyboardInterrupt:

        print("STOP")

        mqtt_client.loop_stop()
        mqtt_client.disconnect()

        break

    except Exception as e:

        print("Erreur :", e)
        time.sleep(1)
        