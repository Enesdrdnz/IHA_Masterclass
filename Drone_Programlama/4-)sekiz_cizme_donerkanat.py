import asyncio
import math
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)

async def run():
    drone = System()
    # Codespaces/SITL bağlantısı
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Drone bekleniyor...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Bağlantı başarılı!")
            break

    print("GPS konumu alınıyor...")
    async for pos in drone.telemetry.position():
        base_lat = pos.latitude_deg
        base_lon = pos.longitude_deg
        break

    mission_items = []
    radius = 40.0   
    altitude = 20.0 
    speed = 4.0     # Hızı biraz düşürmek akıcılığı artırır
    METRE_TO_DEG = 1.0 / 111111.0

    # Adım açısını 15 dereceye düşürdük (Nokta sayısı arttı)
    step_angle = 15 

    for side in [1, -1]:
        for i in range(0, 361, step_angle):
            angle_rad = math.radians(i)
            
            lat = base_lat + (radius * math.sin(angle_rad)) * METRE_TO_DEG
            lon = base_lon + (side * radius * (1 - math.cos(angle_rad))) * METRE_TO_DEG / math.cos(math.radians(base_lat))
            
            mission_items.append(MissionItem(
                lat,
                lon,
                altitude,
                speed,
                True,                          # is_fly_through: Noktada durma, içinden geç
                float('nan'),
                float('nan'),
                MissionItem.CameraAction.NONE,
                float('nan'),
                float('nan'),
                5.0,                           # acceptance_radius: 5 metreye yaklaşınca sıradaki noktaya yönel (Yumuşak dönüş)
                float('nan'),
                float('nan'),
                MissionItem.VehicleAction.NONE
            ))

    mission_plan = MissionPlan(mission_items)
    
    print(f"-- {len(mission_items)} adet waypoint yükleniyor...")
    await drone.mission.set_return_to_launch_after_mission(True)
    await drone.mission.upload_mission(mission_plan)

    print("-- Arm ediliyor ve görev başlıyor...")
    await drone.action.arm()
    await drone.mission.start_mission()

    async for progress in drone.mission.mission_progress():
        print(f"İlerleme: {progress.current}/{progress.total}", end="\r")
        if progress.current == progress.total:
            print("\n[TAMAM] Akıcı 8 görevi bitti.")
            break

if __name__ == "__main__":
    asyncio.run(run())