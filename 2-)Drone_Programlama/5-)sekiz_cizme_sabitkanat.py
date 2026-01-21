import asyncio
import math
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)

async def run():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14541")

    print("Sabit kanatlı İHA bekleniyor...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Bağlantı başarılı!")
            break

    async for pos in drone.telemetry.position():
        base_lat = pos.latitude_deg
        base_lon = pos.longitude_deg
        break

    mission_items = []
    
    # --- AYARLAR ---
    radius = 200.0   
    altitude = 60.0  
    speed = 20.0     
    METRE_TO_DEG = 1.0 / 111111.0
    # ---------------

    # 1. SEKİZ ÇİZME GÖREVİ (Daha önce yaptığımız kısımlar)
    for side in [1, -1]:
        for i in range(0, 361, 30):
            angle_rad = math.radians(i)
            lat = base_lat + (radius * math.sin(angle_rad)) * METRE_TO_DEG
            lon = base_lon + (side * radius * (1 - math.cos(angle_rad))) * METRE_TO_DEG / math.cos(math.radians(base_lat))
            
            mission_items.append(MissionItem(
                lat, lon, altitude, speed, True,
                float('nan'), float('nan'), MissionItem.CameraAction.NONE,
                float('nan'), float('nan'), 20.0, float('nan'), float('nan'),
                MissionItem.VehicleAction.NONE
            ))

    # 2. LANDING PATTERN (İniş Deseni Başlangıcı)
    # Önce uçağı iniş irtifasına (40m) alçaltacak bir nokta (Loiter)
    # Bu noktayı kalkıştan 500m uzağa koyuyoruz ki alçalma mesafesi kalsın
    approach_lat = base_lat + (350 * METRE_TO_DEG)
    
    mission_items.append(MissionItem(
        approach_lat, base_lon, 40.0, speed, True,
        float('nan'), float('nan'), MissionItem.CameraAction.NONE,
        float('nan'), float('nan'), 20.0, float('nan'), float('nan'),
        MissionItem.VehicleAction.NONE
    ))

    # 3. FINAL LANDING POINT (Son İniş Noktası)
    # PX4'e görevin bittiğini ve burada teker koyması gerektiğini söyler
    mission_items.append(MissionItem(
        base_lat, base_lon, 0.0, speed, False,
        float('nan'), float('nan'), MissionItem.CameraAction.NONE,
        float('nan'), float('nan'), 35.0, float('nan'), float('nan'),
        MissionItem.VehicleAction.LAND
    ))

    mission_plan = MissionPlan(mission_items)
    
    print("-- Landing Pattern içeren görev yükleniyor...")
    await drone.mission.upload_mission(mission_plan)

    print("-- Arm ediliyor ve motorlar başlatılıyor...")
    await drone.action.arm()
    await drone.mission.start_mission()

    async for progress in drone.mission.mission_progress():
        print(f"Görev Durumu: {progress.current}/{progress.total}", end="\r")
        if progress.current == progress.total:
            print("\n[TAMAM] Uçak piste teker koydu.")
            break

if __name__ == "__main__":
    asyncio.run(run())