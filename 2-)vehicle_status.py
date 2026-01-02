import asyncio
from mavsdk import System

async def run():
    drone = System()
    # Simülasyona bağlan
    await drone.connect(system_address="udp://:14540")

    print("Drone bekleniyor...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone bağlandı!")
            break

    # Tüm telemetri görevlerini eş zamanlı (async) olarak başlatıyoruz
    print("--- Veriler Akıyor ---")
    await asyncio.gather(
        get_health(drone),
        get_position(drone),
        get_attitude(drone),
        get_velocity(drone),
        get_flight_mode(drone)
    )

async def get_health(drone):
    """Sağlık durumu: GPS fix, İvmeölçer, Jiroskop vb. uçuşa hazır mı?"""
    async for health in drone.telemetry.health():
        # Güncel sürümde 'is_global_position_ok' GPS durumunu en iyi temsil eden değişkendir
        gps_durumu = "OK" if health.is_global_position_ok else "BEKLENİYOR"
        hazirlik = "EVET" if health.is_armable else "HAYIR"
        
        print(f"[SAĞLIK] GPS: {gps_durumu} | Uçuşa Hazır mı: {hazirlik}")
async def get_position(drone):
    """Global GPS konumu ve İrtifa"""
    async for pos in drone.telemetry.position():
        print(f"[KONUM] Enlem: {pos.latitude_deg:.5f}, Boylam: {pos.longitude_deg:.5f}, "
              f"İrtifa (MSL): {pos.absolute_altitude_m:.2f}m")

async def get_attitude(drone):
    """Yaw, Pitch, Roll (Euler Açıları)"""
    async for attitude in drone.telemetry.attitude_euler():
        print(f"[YÖNELİM] Roll: {attitude.roll_deg:.1f}°, "
              f"Pitch: {attitude.pitch_deg:.1f}°, "
              f"Yaw: {attitude.yaw_deg:.1f}°")

async def get_velocity(drone):
    """Hız verisi (NED - North East Down formatında)"""
    async for vel in drone.telemetry.velocity_ned():
        toplam_hiz = (vel.north_m_s**2 + vel.east_m_s**2 + vel.down_m_s**2)**0.5
        print(f"[HIZ] Toplam Hız: {toplam_hiz:.2f} m/s")

async def get_flight_mode(drone):
    """Aktif uçuş modu (MANUAL, HOLD, TAKEOFF vb.)"""
    async for mode in drone.telemetry.flight_mode():
        print(f"[MOD] Mevcut Mod: {mode}")
        print(f"-----------------------------------")

if __name__ == "__main__":
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        print("\nProgram kullanıcı tarafından kapatıldı.")