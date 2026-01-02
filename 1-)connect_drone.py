import asyncio
from mavsdk import System

async def run():
    # Drone nesnesini oluştur
    drone = System()

    # Simülasyona bağlan (Varsayılan port 14540)
    # Eğer gerçek bir cihaz ise "serial:///dev/ttyUSB0:57600" gibi bir adres kullanılır
    print("Drone'a bağlanılıyor...")
    await drone.connect(system_address="udp://:14540")

    # Bağlantı durumunu kontrol et
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Bağlantı Başarılı! Cihaz ID: {state}")
            break # Bağlantı kurulduğu an döngüden çık

    print("İşlem tamamlandı, bağlantı aktif.")

if __name__ == "__main__":
    # Event loop'u başlat
    asyncio.run(run())