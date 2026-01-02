import asyncio
from mavsdk import System

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Drone bekleniyor...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Bağlantı kuruldu!")
            break

    # Sağlık kontrolü
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("GPS hazır.")
            break

    # Kalkış yüksekliğini önceden 50 metreye ayarlayalım
    await drone.action.set_takeoff_altitude(50.0)

    print("-- Arm ediliyor")
    await drone.action.arm()

    print("-- Kalkış yapılıyor (Hedef: 50m)")
    await drone.action.takeoff()

    # YÜKSEKLİK KONTROL DÖNGÜSÜ
    # İrtifayı sürekli dinleyip 50 metreye ulaşıp ulaşmadığını kontrol ediyoruz
    async for position in drone.telemetry.position():
        mevcut_irtifa = position.relative_altitude_m
        print(f"Şu anki yükseklik: {mevcut_irtifa:.2f} m", end="\r")

        if mevcut_irtifa >= 49.5:
            print(f"\n[HEDEF] 50 metreye ulaşıldı: {mevcut_irtifa:.2f} m")
            break

    print("5 saniye havada asılı kalınıyor...")
    await asyncio.sleep(5)

    print("-- İniş yapılıyor")
    await drone.action.land()

    # Motorlar durana kadar bekle
    async for is_armed in drone.telemetry.armed():
        if not is_armed:
            print("Drone indi ve motorlar kapatıldı.")
            break

if __name__ == "__main__":
    asyncio.run(run())